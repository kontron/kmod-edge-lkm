// SPDX-License-Identifier: GPL-2.0
/* TTTech EDGE/DE-IP Linux driver
 * Copyright(c) 2018 TTTech Computertechnik AG.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, see <http://www.gnu.org/licenses/>.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * support@tttech.com
 * TTTech Computertechnik AG, Schoenbrunnerstrasse 7, 1040 Vienna, Austria
 */

#include "linux/bitops.h"
#include "linux/etherdevice.h"
#include "linux/workqueue.h"

#include "edge_br_fdb.h"
#include "edge_port.h"
#include "edge_util.h"

struct smac_entry {
	u16 general;
	u8  mac[ETH_ALEN];
	u16 fwd_mask;
	u16 policed_pts;
	u16 policer;
	u16 vlan;
	u16 stream_no;    /* Future, for Stream Identification (802.1CB) */
};

struct edgx_brfdb {
	struct edgx_br      *parent;
	edgx_io_t           *iobase;
	u16                  nrows;
	unsigned int         sents;  /* Number of static entries */
	struct mutex         lock;   /* SMAC table lock */
};

#define DYNAMIC_UPDATE_MS       (1000)

#define SMAC_CTL_POS(row, col)  ((((col) & 0x3) << 12) | ((row) & 0xFFF))
#define SMAC_CTL_RD(row, col)   (BIT(15) | SMAC_CTL_POS(row, col))
#define SMAC_CTL_WR(row, col)   (BIT(15) | BIT(14) | SMAC_CTL_POS(row, col))
#define SMAC_CTL_BUSY(ctrl)     ((ctrl) & BIT(15))

#define SMAC_NCOLS              (4)   /* number of SMAC columns */
#define NCOLS_VLAN		(2)   /* First two columns for specific VLANs*/
#define COL_VLAN                (1)   /* VLAN-aware column */
#define COL_WILDCARD            (0)   /* VLAN-wildcard column */
#define VID_WILDCARD            (0)   /* switchdev wildcard ("for all VIDs") */

/* CRC40 lookup table for polynomial 0x0104c11db7. */
static const u64 crc40tbl[] = {
	0x000000000ull, 0x104c11db7ull, 0x209823b6eull, 0x30d4326d9ull,
	0x4130476dcull, 0x517c56b6bull, 0x61a864db2ull, 0x71e475005ull,
	0x82608edb8ull, 0x922c9f00full, 0xa2f8ad6d6ull, 0xb2b4bcb61ull,
	0xc350c9b64ull, 0xd31cd86d3ull, 0xe3c8ea00aull, 0xf384fbdbdull,
};

/* "Broadcast" MAC, terminating dynamic FDB traversal */
static const u8 mac_bcast[ETH_ALEN] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/* We do a 50:50 split on the columns.
 * Columns 0 and 1 are used for FDB-entries applicable to specific VLANs.
 * Columns 2 and  are used for FDB-entries applicable to all VLANs.
 */
static const u8 column[SMAC_NCOLS] = { COL_VLAN,     COL_VLAN,
				       COL_WILDCARD, COL_WILDCARD };

static inline bool edgx_brfdb_is_fdb_used(struct smac_entry *e)
{
	return ((e->general & BIT(15)) != 0);
}

static inline bool edgx_brfdb_is_sid_used(struct smac_entry *e)
{
	return ((e->general & BIT(6)) != 0);
}

static inline bool edgx_brfdb_is_smac_match(struct smac_entry *e,
					    const unsigned char *mac, u16 vid)
{
	return ((!ether_addr_cmp(mac, e->mac)) && (vid == e->vlan));
}

static inline bool edgx_brfdb_is_sid_fit(struct smac_entry *e,
					 const unsigned char *mac, u16 vid)
{
	if (!edgx_brfdb_is_sid_used(e))
		return false;
	return edgx_brfdb_is_smac_match(e, mac, vid);
}

static inline bool edgx_brfdb_is_fdb_fit(struct smac_entry *e,
					 const unsigned char *mac, u16 vid)
{
	if (!edgx_brfdb_is_fdb_used(e))
		return false;
	return edgx_brfdb_is_smac_match(e, mac, vid);
}

static inline bool edgx_brfdb_col_is_vid(u16 col, u16 vid)
{
	if (vid == VID_WILDCARD)
		return (column[col] == COL_WILDCARD);
	return (column[col] == COL_VLAN);
}

static void edgx_brfdb_calc_smac_rows(struct edgx_brfdb *brfdb,
				      const unsigned char *mac, u16 vid,
				      u16 *row)
{
	unsigned int i;
	u64 hash;
	u64 extra;
	u64 crc = ~((u64)0);

	for (i = 0; i < ETH_ALEN; i++) {
		crc = (crc << 4) ^
		      crc40tbl[(((u8)(crc >> 36) & 0xfu) ^ (mac[i] >> 4))];
		crc = (crc << 4) ^
		      crc40tbl[(((u8)(crc >> 36) & 0xfu) ^ (mac[i] & 0xfu))];
	}
	crc &= 0xffffffffffull;
	/* Split the 12-bit column hashes of CRC40 to a 64-bit value.
	 * Use 16 bits (not 12) for each column hash.
	 */
	extra = ((mac[3] ^ mac[4]) & 0x3) << 10;
	hash =   (extra | ((crc >>  0) & 0x3ff)) <<  0;
	hash |=  (extra | ((crc >> 10) & 0x3ff)) << 16;
	hash |=  (extra | ((crc >> 20) & 0x3ff)) << 32;
	hash |=  (extra | ((crc >> 30) & 0x3ff)) << 48;
	if (vid != VID_WILDCARD)
		/* XOR each column hash with VLAN ID. */
		hash ^= ((u64)vid << 48) |
			((u64)vid << 32) |
			((u64)vid << 16) |
			((u64)vid << 0);
	/* Extract and trucate rows */
	row[0] = hash & (brfdb->nrows - 1);
	row[1] = (hash >> 16) & (brfdb->nrows - 1);
	row[2] = (hash >> 32) & (brfdb->nrows - 1);
	row[3] = (hash >> 48) & (brfdb->nrows - 1);
}

static void edgx_brfdb_smac_get(struct edgx_brfdb *brfdb, u16 col, u16 row,
				struct smac_entry *e)
{
	u16 ctrl = SMAC_CTL_RD(row, col);

	edgx_wr16(brfdb->iobase, 0x220, ctrl);
	do {
		ctrl = edgx_rd16(brfdb->iobase, 0x220);
	} while (SMAC_CTL_BUSY(ctrl));

	e->general           = edgx_rd16(brfdb->iobase, 0x230);
	*((u16 *)&e->mac[0]) = edgx_rd16(brfdb->iobase, 0x232);
	*((u16 *)&e->mac[2]) = edgx_rd16(brfdb->iobase, 0x234);
	*((u16 *)&e->mac[4]) = edgx_rd16(brfdb->iobase, 0x236);
	e->fwd_mask          = edgx_rd16(brfdb->iobase, 0x238);
	e->policed_pts       = 0; /* unused */
	e->policer           = 0; /* unused */
	e->vlan              = edgx_rd16(brfdb->iobase, 0x23E);
	e->stream_no	     = edgx_rd16(brfdb->iobase, 0x240);
}

static void edgx_brfdb_smac_set(struct edgx_brfdb *brfdb, u16 col, u16 row,
				struct smac_entry *e, bool sid)
{
	u16 ctrl = SMAC_CTL_WR(row, col);

	edgx_wr16(brfdb->iobase, 0x230, e->general);
	edgx_wr16(brfdb->iobase, 0x232, *((u16 *)&e->mac[0]));
	edgx_wr16(brfdb->iobase, 0x234, *((u16 *)&e->mac[2]));
	edgx_wr16(brfdb->iobase, 0x236, *((u16 *)&e->mac[4]));
	edgx_wr16(brfdb->iobase, 0x23A, 0);
	edgx_wr16(brfdb->iobase, 0x23C, 0);
	edgx_wr16(brfdb->iobase, 0x23E, e->vlan);
	edgx_wr16(brfdb->iobase, 0x238, e->fwd_mask);

	if (sid)
		edgx_wr16(brfdb->iobase, 0x240, e->stream_no);

	edgx_wr16(brfdb->iobase, 0x220, ctrl);
	do {
		ctrl = edgx_rd16(brfdb->iobase, 0x220);
	} while (SMAC_CTL_BUSY(ctrl));
}

static inline void edgx_brfdb_set_general(struct smac_entry *e)
{
	/* If no ports are left in fwd-mask, set entry unused again */
	if (e->fwd_mask)
		e->general |= BIT(15);
	else
		e->general &= ~BIT(15);

	if (e->vlan == VID_WILDCARD)
		e->general &= ~BIT(12); /* clear VLAN matching */
	else
		e->general |= BIT(12);  /* set VLAN matching */
}

u16 edgx_brfdb_sid_get_max_str(struct edgx_brfdb *brfdb)
{
	u16 frer_streams =  edgx_br_get_generic(brfdb->parent, BR_GX_NSTREAMS);
	u32 fdb_streams = edgx_brfdb_sz(brfdb) / 2;
	/* Maximum supported streams is the minimum between number of supported
	 * streams according to register FRER_STREAMS value and number of
	 * entries in the first two columns in FDB (only first two columns
	 * support VLAN tagged frames)
	 */
	if (fdb_streams < (1 << frer_streams))
		return (u16)fdb_streams;
	else
		return 1 << frer_streams;
}

void edgx_brfdb_sid_get_smac(struct edgx_brfdb *brfdb, u8 col, u16 row,
			     u8 *addr, u16 *vid)
{
	struct smac_entry e;

	edgx_brfdb_smac_get(brfdb, col, row, &e);
	ether_addr_copy(addr, e.mac);
	*vid = e.vlan;
}

int edgx_brfdb_sid_add(struct edgx_brfdb *brfdb, struct edgx_sid *sid)
{
	u16 row[SMAC_NCOLS];
	struct smac_entry e[SMAC_NCOLS];
	unsigned int i;
	int ret = 0;

	edgx_brfdb_calc_smac_rows(brfdb, sid->addr, sid->vid, row);
	mutex_lock(&brfdb->lock);

	/* Get all entries on hash positions only columns 1 2 (VLAN aware)*/
	for (i = 0; i < NCOLS_VLAN; i++)
		edgx_brfdb_smac_get(brfdb, i, row[i], &e[i]);

	for (i = 0; i < NCOLS_VLAN; i++)
		if (edgx_brfdb_is_fdb_fit(&e[i], sid->addr, sid->vid)) {
			if (e[i].stream_no == sid->str_hdl) {
				/* Same entry in fdb, nothing to be done */
				goto out_done;
			}
		}
	/* Try to create new entry ... */
	for (i = 0; i < NCOLS_VLAN; i++) {
		if (!edgx_brfdb_is_sid_used(&e[i])) {
			/* we use this entry */
			ether_addr_copy(e[i].mac, sid->addr);
			e[i].vlan = sid->vid;
			e[i].general |= BIT(6) | BIT(12);
			e[i].general |= (sid->id_type) ? BIT(7) : 0;
			e[i].stream_no = sid->str_hdl;
			edgx_brfdb_smac_set(brfdb, i, row[i], &e[i], 1);
			brfdb->sents++;
			sid->valid_fdb = 1;

			goto out_done;
		}
	}
	/* No re-usable entry and no possible entry left, set return code */
	ret = -ENOMEM;

out_done:
	mutex_unlock(&brfdb->lock);
	return ret;

	return 0;
}

int edgx_brfdb_add(struct edgx_brfdb *brfdb,
		   struct switchdev_notifier_fdb_info *info,
		   ptid_t ptid)
{
	u16 row[SMAC_NCOLS];
	struct smac_entry entry[SMAC_NCOLS];
	unsigned int i;
	int r = 0;

	edgx_brfdb_calc_smac_rows(brfdb, info->addr, info->vid, row);

	/* RTNL lock is not sufficient, since SMAC table will also be used
	 *for Stream lookup in the future
	 */
	mutex_lock(&brfdb->lock);

	/* Get all entries on hash positions */
	for (i = 0; i < SMAC_NCOLS; i++)
		edgx_brfdb_smac_get(brfdb, i, row[i], &entry[i]);

	/* Try to add port to existing entry ... */
	for (i = 0; i < SMAC_NCOLS; i++)
		if (edgx_brfdb_is_fdb_fit(&entry[i], info->addr, info->vid)) {
			entry[i].fwd_mask |= BIT(ptid);
			edgx_brfdb_set_general(&entry[i]);
			edgx_brfdb_smac_set(brfdb, i, row[i], &entry[i], 0);
			goto out_done;
		}

	/* Try to create new entry ... */
	for (i = 0; i < SMAC_NCOLS; i++)
		if (!edgx_brfdb_is_fdb_used(&entry[i]) &&
		    edgx_brfdb_col_is_vid(i, info->vid)) {
			/* we use this entry */
			ether_addr_copy(entry[i].mac, info->addr);
			entry[i].fwd_mask    = BIT(ptid);
			entry[i].policed_pts = 0;
			entry[i].policer     = 0;
			entry[i].vlan        = info->vid;
			edgx_brfdb_set_general(&entry[i]);
			edgx_brfdb_smac_set(brfdb, i, row[i], &entry[i], 0);
			brfdb->sents++;
			goto out_done;
		}

	/* No re-usable entry and no possible entry left, set return code */
	r = -ENOMEM;

out_done:
	mutex_unlock(&brfdb->lock);
	return r;
}

int edgx_brfdb_sid_del(struct edgx_brfdb *brfdb, u8 *addr, u16 vid)
{
	struct smac_entry entry[NCOLS_VLAN];
	u16 row[SMAC_NCOLS];
	u8 i;

	edgx_brfdb_calc_smac_rows(brfdb, addr, vid, row);

	mutex_lock(&brfdb->lock);
	/* Get all entries on hash positions */
	for (i = 0; i < NCOLS_VLAN; i++)
		edgx_brfdb_smac_get(brfdb, i, row[i], &entry[i]);

	for (i = 0; i < NCOLS_VLAN; i++)
		if (edgx_brfdb_is_sid_fit(&entry[i], addr, vid)) {
			entry[i].general &=  ~BIT(6);
			edgx_brfdb_smac_set(brfdb, i, row[i], &entry[i], 1);
			break;
		}

	mutex_unlock(&brfdb->lock);

	return 0;
}

int edgx_brfdb_del(struct edgx_brfdb *brfdb,
		   struct switchdev_notifier_fdb_info *info,
		   ptid_t ptid)
{
	u16 row[SMAC_NCOLS];
	struct smac_entry entry[SMAC_NCOLS];
	unsigned int i;

	edgx_brfdb_calc_smac_rows(brfdb, info->addr, info->vid, row);

	/* RTNL lock is not sufficient, since SMAC table will also be used
	 *for Stream lookup in the future
	 */
	mutex_lock(&brfdb->lock);

	/* Get all entries on hash positions */
	for (i = 0; i < SMAC_NCOLS; i++)
		edgx_brfdb_smac_get(brfdb, i, row[i], &entry[i]);

	for (i = 0; i < SMAC_NCOLS; i++)
		if (edgx_brfdb_is_fdb_fit(&entry[i], info->addr, info->vid)) {
			entry[i].fwd_mask &= ~BIT(ptid);
			edgx_brfdb_set_general(&entry[i]);
			edgx_brfdb_smac_set(brfdb, i, row[i], &entry[i], 0);
			if (!entry[i].fwd_mask)
				brfdb->sents--;
			break;
		}

	mutex_unlock(&brfdb->lock);
	return 0;
}

int edgx_brfdb_dump(struct edgx_brfdb *brfdb, struct sk_buff *skb,
		    struct netlink_callback *ncb, struct net_device *dev,
		    edgx_brfdb_cb_t *cb, ptid_t ptid, int *idx)
{
	unsigned int row, col;
	struct smac_entry entry;
	u16 pt_mask = BIT(ptid);
	int stored_r = 0;

	/* For all rows ... */
	for (row = 0; row < brfdb->nrows; row++)
		/* ... and all columns. */
		for (col = 0; col < SMAC_NCOLS; col++) {
			mutex_lock(&brfdb->lock);
			edgx_brfdb_smac_get(brfdb, col, row, &entry);

			/* No need to match fdb->vid too; bridge does the
			 * filtering itself and always sets fdb->vid = 0
			 */
			if (entry.fwd_mask & pt_mask &&
			    edgx_brfdb_is_fdb_used(&entry)) {
				int r = 0;

				r = cb(skb, ncb, dev,
				       entry.mac, entry.vlan, idx);
				if (r)
					stored_r = r;
			}
			mutex_unlock(&brfdb->lock);
		}
	return stored_r;
}

unsigned int edgx_brfdb_sz(struct edgx_brfdb *brfdb)
{
	return (brfdb) ? brfdb->nrows * SMAC_NCOLS : 0;
}

unsigned int edgx_brfdb_nsmac(struct edgx_brfdb *brfdb)
{
	return (brfdb) ? brfdb->sents : 0;
}

int edgx_brfdb_init(struct edgx_br *br, edgx_io_t *iobase,
		    struct edgx_brfdb **brfdb)
{
	struct edgx_brfdb *fdb;

	if (!br || !brfdb)
		return -EINVAL;
	fdb = kzalloc(sizeof(*fdb), GFP_KERNEL);
	if (!fdb)
		return -ENOMEM;

	fdb->parent = br;
	fdb->iobase = iobase;
	fdb->nrows  = 1 << edgx_br_get_generic(br, BR_GX_SMAC_ROWS);
	fdb->sents  = 0;
	if (!fdb->nrows)
		goto out_rows;
	mutex_init(&fdb->lock);
	*brfdb = fdb;

	/* Initialize per-column VLAN-XOR setting */
	edgx_set16(iobase, 0x222, 1, 0, column[0]);
	edgx_set16(iobase, 0x222, 3, 2, column[1]);
	edgx_set16(iobase, 0x222, 5, 4, column[2]);
	edgx_set16(iobase, 0x222, 7, 6, column[3]);

	edgx_br_info(br, "Instantiating FDB (%u rows) ... done\n", fdb->nrows);
	return 0;

out_rows:
	kfree(fdb);
	return -ENODEV;
}

void edgx_brfdb_shutdown(struct edgx_brfdb *brfdb)
{
	if (brfdb) {
//		cancel_delayed_work_sync(&brfdb->dynamic);
		kfree(brfdb);
	}
}

void edgx_brfdb_flush(struct edgx_brfdb *brfdb, u8 fid)
{
	(void)brfdb;
	(void)fid;
}
