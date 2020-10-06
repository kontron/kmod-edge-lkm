
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

#include <linux/version.h>
#include <linux/etherdevice.h>
#include <linux/if_bridge.h>
#include <linux/netdevice.h>


#include "edge_ac.h"
#include "edge_com.h"
#include "edge_br_vlan.h"
#include "edge_br_fdb.h"
#include "edge_bridge.h"
#include "edge_link.h"
#include "edge_port.h"
#include "edge_preempt.h"
#include "edge_sched.h"
#include "edge_stat.h"
#include "edge_time.h"
#include "edge_util.h"
#include "edge_fqtss.h"
#include "edge_br_sid.h"
#include "edge_frer.h"

struct edgx_ptfillqu {
	/* Protect the control register for buffer fill level capture */
	spinlock_t   lock;
};

struct edgx_pt {
	struct edgx_br          *parent;

	struct net_device       *netdev;
	struct edgx_com_hdl     *hcom;
	struct edgx_link        *link;

	ptid_t                   ptid;

	/* From here it's specific to bridge ports */
	edgx_io_t               *iobase;

	struct edgx_time        *time;
	struct edgx_stat_hdl    *hstat;
	struct edgx_ptfillqu	fillqu;
	struct edgx_sched       *sched;
	struct edgx_preempt     *preempt;
	struct edgx_fqtss	*fqtss;
};

#define _PT_STP_FWD       0x0
#define _PT_STP_LEARN     0x1
#define _PT_STP_DISABLED  0x2
#define _MAX_PRIO_VAL	  (EDGX_BR_NUM_PCP - 1)
#define _TX8_DEF_VAL	  0x76543201
#define _PRIO_REGEN_LO	  0x16
#define _PRIO_REGEN_HI	  0x18
#define _QUEUE_TBL_LO	  0x20
#define _QUEUE_TBL_HI	  0x22
#define _TYPE_PRIO_REGEN  0xab
#define _TYPE_QUEUE_TBL   0xcd

#define  EDGX_PT_WDT_MS   2000U

#define _BRPT_FLAGS (BR_LEARNING | BR_LEARNING_SYNC | BR_FLOOD)

/* VID 4095 (0xFFF) is reserved for implementation use.
 * We don't need any further guard in, as Linux bridge code does not allow the
 * creation of this VLAN.
 */
#define _PVID_NONE      0xFFF

#define _STAT_PORT_BASE 0x200
#define _FL_CAPT	0xC0
#define _FILL_QU_BASE	0xC2

enum _stat_pt_idx {
	_STAT_RX_GOOD_OCTETS = 0,
	_STAT_RX_BAD_OCTETS,
	_STAT_RX_UC,
	_STAT_RX_BC,
	_STAT_RX_MC,
	_STAT_RX_UNDERSZ,
	_STAT_RX_FRAGS,
	_STAT_RX_OVERSZ,
	_STAT_RX_JABBER,
	_STAT_RX_ERR,
	_STAT_RX_CRC,
	_STAT_RX_FULLDROP,
	_STAT_RX_POLICED,  /* Unused */
	_STAT_TX_OCTETS,
	_STAT_TX_UC,
	_STAT_TX_BC,
	_STAT_TX_MC,
	_STAT_TX_PRIQDROP,
	_STAT_TX_EARLYDROP,
	_STAT_RX_64,
	_STAT_RX_65_127,
	_STAT_RX_128_255,
	_STAT_RX_256_511,
	_STAT_RX_512_1023,
	_STAT_RX_1024_1536,
	_STAT_MAX,         /* must be last element */
};

struct edgx_pt_ipo {
	u16 cfg0;
	u16 fwd;
	u16 mirror;
	u16 cfg1;
	u8  mac[ETH_ALEN];
};

#define _GEN_BASE(_iob)         ((_iob) + 0x0)
#define _IPO_BASE(_iob)         ((_iob) + 0x8000)
#define _IPO_RBASE(_iob, _rule) (_IPO_BASE(_iob) + (0x20 * (_rule)))
#define _FRAMESIZE_BASE(_iob, idx) (_GEN_BASE(_iob) + 0x40 + (0x02 * (idx)))
#define _FRAMESIZE_MASK		(0x7ff)
#define _FRAMESIZE_OVERHEAD	(10U)

#define _FIDCFG_REG(_ofs)       (0x80 + (_ofs))

static const struct edgx_statinfo _pt_statinfo = {
	.feat_id = EDGX_STAT_FEAT_PORT,
	.rate_ms = 3000, /* fastest counter in block overwraps every
			  * 4.2 seconds -> set to 3 seconds
			  */
	.base    = _STAT_PORT_BASE,
	.nwords  = _STAT_MAX,
};

struct edgx_switchdev_event_work {
	struct work_struct work;
	struct switchdev_notifier_fdb_info fdb_info;
	struct edgx_pt *pt;
	unsigned long event;
};

static inline bool is_bridged(struct edgx_pt *pt)
{
	return (pt->netdev->priv_flags & IFF_BRIDGE_PORT);
}

static inline struct edgx_pt *net2pt(struct net_device *netdev)
{
	return ((struct edgx_pt *)netdev_priv(netdev));
}

struct edgx_link *edgx_net2link(struct net_device *netdev)
{
	return (net2pt(netdev)) ? net2pt(netdev)->link : NULL;
}

static inline struct edgx_time *net2time(struct net_device *netdev)
{
	return (net2pt(netdev)) ? net2pt(netdev)->time : NULL;
}

static inline struct edgx_br *pt2br(struct edgx_pt *pt)
{
	return pt->parent;
}

static inline int edgx_pt_open(struct net_device *netdev)
{
	struct edgx_link *lnk = net2pt(netdev)->link;

	edgx_link_start(lnk);
	return 0;
}

static int edgx_pt_stop(struct net_device *netdev)
{
	struct edgx_link *lnk = net2pt(netdev)->link;

	edgx_link_stop(lnk);
	return 0;
}

static struct edgx_pt_ipo ipo_mgmt[] = {
	/* CFG0:   Enabled, Compare length = 44bit
	 *         new/preserve priority is set to traffic-class 0,
	 *         will be fixed in ipo_init() as it depends on #queues
	 * FWD:    left empty, will become mgmt-port (or all) upon ipo_init()
	 * MIRROR: left empty, will become mgmt-port (or all) upon ipo_init()
	 *                     may become dedicated mirror-port
	 * CFG1:   Compare MSB first, disable cut-through, DO use IPO-mark
	 */
	{0xB1, 0x0, 0x0, 0xD000, {0x01, 0x80, 0xC2, 0x00, 0x00, 0x00} },
	{0xB1, 0x0, 0x0, 0xD000, {0x01, 0x80, 0xC2, 0x00, 0x00, 0x10} },
	/* CFG0:   Enabled, Compare length = 48bit
	 * Used for preemption verification frames which have to be
	 * received on the corresponding bridge port, and are identified
	 * by MAC address 00:00:00:00:00:00
	 * Remaining items as described above.
	 */
	{0xC1, 0x0, 0x0, 0xD000, {0x0, 0x0, 0x0, 0x0, 0x0, 0x0} },
	/* Management entry for 01:80:C2:00:00:2X is only valid if MRP is
	 *  supported (see IEEE 802.1Q-2014, Sect. 10.5, item b.2)
	 */
};

/* CFG0:   Cmp-len = 48bit (full MAC), keep priority
 * FWD:    0 now, will be set to mgmt-port upon ipo_init()
 * MIRROR: 0 now, may become dedicated mirror-port
 * CFG1:   MSB-first compare, NO cut-through, NO IPO-mark
 */
static struct edgx_pt_ipo ipo_self = {0x40C1, 0x0, 0x0, 0xD000,
				      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };

/* CFG0:   Cmp-len = 0 (match all), keep priority
 * FWD:    0xFFFF - allow on all ports
 * MIRROR: 0 now, may become dedicated mirror-port
 * CFG1:   MSB-first compare, NO cut-through, NO IPO-mark
 */
static struct edgx_pt_ipo ipo_all =  {0x40C1, 0xFFFF, 0x0, 0xC000,
				      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };

/* Unused/Disabled IPO entry */
static struct edgx_pt_ipo ipo_null = {0x0, 0x0, 0x0, 0x0,
				      {0x00, 0x00, 0x00, 0x00, 0x00, 0x00} };

/* We use dedicated entries, so that we know what we need to update, e.g.,
 * self MAC or mirroring.
 */
#define _IPO_SELF_ENT       ARRAY_SIZE(ipo_mgmt)
#define _IPO_ALL_ENT        (_IPO_SELF_ENT + 1)
#define _IPO_NONE_ENT_START (_IPO_ALL_ENT + 1)
#define _IPO_NRULES         (16)

#define _IPO_REG_CMD	    0x0
#define _IPO_REG_CFG0       0x10
#define _IPO_REG_FWD        0x12
#define _IPO_REG_MIRR       0x14
#define _IPO_REG_CFG1       0x16
#define _IPO_REG_ETH_0      0x18
#define _IPO_REG_ETH_1      0x1A
#define _IPO_REG_ETH_2      0x1C

#define _IPO_CMD_CFG0	    (0x1)
#define _IPO_CMD_FWD        (0x2)
#define _IPO_CMD_MIRR       (0x3)
#define _IPO_CMD_CFG1       (0x4)
#define _IPO_CMD_ETH_0      (0x5)
#define _IPO_CMD_ETH_1      (0x6)
#define _IPO_CMD_ETH_2      (0x7)

#define _IPO_CMD_WRITE	    (1<<14)
#define _IPO_CMD_READ	    (0<<14)
#define _IPO_CMD_TRANSFER   (1<<15)


static inline void edgx_pt_ipo_set_mac(edgx_io_t *ipo_rbase, u8 *mac)
{
	u16 reg;

	edgx_set16(ipo_rbase, _IPO_REG_ETH_0, 7, 0, mac[0]);
	edgx_set16(ipo_rbase, _IPO_REG_ETH_0, 15, 8, mac[1]);
	edgx_set16(ipo_rbase, _IPO_REG_ETH_1, 7, 0, mac[2]);
	edgx_set16(ipo_rbase, _IPO_REG_ETH_1, 15, 8, mac[3]);
	edgx_set16(ipo_rbase, _IPO_REG_ETH_2, 7, 0, mac[4]);
	edgx_set16(ipo_rbase, _IPO_REG_ETH_2, 15, 8, mac[5]);
}

static inline void edgx_pt_ipo_init_single(edgx_io_t *ipo_rbase,
					   struct edgx_pt_ipo *ipo,
					   u8 entry)
{
	u16 reg;
	/* Read the value from registers */
	edgx_wr16(ipo_rbase, _IPO_REG_CMD,
		 (entry | _IPO_CMD_READ | _IPO_CMD_TRANSFER));
	/* Sleep and then wait until flags are cleared by HW. */
	usleep_range(300, 400);
	do {
		reg = edgx_get16(ipo_rbase, _IPO_REG_CMD, 15, 15);
	} while (reg);
	edgx_wr16(ipo_rbase, _IPO_REG_CFG0, ipo->cfg0);
	edgx_wr16(ipo_rbase, _IPO_REG_FWD,  ipo->fwd);
	edgx_wr16(ipo_rbase, _IPO_REG_MIRR, ipo->mirror);
	edgx_wr16(ipo_rbase, _IPO_REG_CFG1, ipo->cfg1);
	edgx_pt_ipo_set_mac(ipo_rbase, ipo->mac);
}

static inline u16 edgx_get_tc_mgmtraffic(struct edgx_br *br)
{
	/* If traffic class for management traffic isn't set via a correct
	 * value in module parameter mgmttc, the highest traffic class is
	 * used.
	 */
	return(min((unsigned int)edgx_br_get_generic(br, BR_GX_QUEUES) - 1,
			mgmttc));
}

static void edgx_pt_ipo_init(struct edgx_pt *pt, ptid_t mgmt_ptid)
{
	unsigned int entry;
	edgx_io_t *ipo_rbase;
	u16 reg;
	u16 mtc = edgx_get_tc_mgmtraffic(pt->parent);

	for (entry = 0; entry < ARRAY_SIZE(ipo_mgmt); entry++) {
		ipo_rbase = _IPO_BASE(pt->iobase);
		edgx_pt_ipo_init_single(ipo_rbase, &ipo_mgmt[entry], entry);
		/* Mgmt traffic uses highest traffic class by default */
		if (mtc > 4)
			edgx_set16(ipo_rbase, _IPO_REG_CFG0, 15, 15, mtc >> 2);
		edgx_set16(ipo_rbase, _IPO_REG_CFG0, 13, 12, (mtc & 0x3));

		/* Need to set FWD and MIRROR, so that mgmt frames also arrive
		 * when in blocking state
		 */
		edgx_wr16(ipo_rbase, _IPO_REG_FWD, BIT(mgmt_ptid));
		edgx_wr16(ipo_rbase, _IPO_REG_MIRR, BIT(mgmt_ptid));
		/* Write the values to IP using indirect access */
		edgx_wr16(ipo_rbase, _IPO_REG_CMD,
			  (entry | _IPO_CMD_WRITE | _IPO_CMD_TRANSFER));
		/* Sleep and wait until flags are cleared by HW */
		usleep_range(300, 400);
		do {
			reg = edgx_get16(ipo_rbase, _IPO_REG_CMD, 15, 15);
		} while (reg);
	}

	/* Setup individual(self)-MAC-rule for port */
	ipo_rbase = _IPO_BASE(pt->iobase);
	/* Read values for _IPO_SELF_ENT rule */
	edgx_wr16(ipo_rbase, _IPO_REG_CMD,
		  (_IPO_SELF_ENT |
		   _IPO_CMD_READ |
		   _IPO_CMD_TRANSFER));
	/* Sleep and wait until flags are cleared by HW */
	usleep_range(300, 400);
	do {
		reg = edgx_get16(ipo_rbase, _IPO_REG_CMD, 15, 15);
	} while (reg);

	edgx_pt_ipo_init_single(ipo_rbase, &ipo_self, _IPO_SELF_ENT);
	edgx_pt_ipo_set_mac(ipo_rbase, pt->netdev->dev_addr);
	/* Need to set FWD and MIRROR, so that mgmt frames also arrive
	 * when port is in blocking state
	 */
	edgx_wr16(ipo_rbase, _IPO_REG_FWD, BIT(mgmt_ptid));
	edgx_wr16(ipo_rbase, _IPO_REG_MIRR, BIT(mgmt_ptid));
	edgx_wr16(ipo_rbase, _IPO_REG_CMD,
		  (_IPO_SELF_ENT |
		   _IPO_CMD_WRITE |
		   _IPO_CMD_TRANSFER));
	/* Sleep and then wait until flags are cleared by HW */
	usleep_range(300, 400);
	do {
		reg = edgx_get16(ipo_rbase, _IPO_REG_CMD, 15, 15);
	} while (reg);

	/* Setup all-MAC rule for port */
	ipo_rbase = _IPO_BASE(pt->iobase);
	 /* Read values for _IPO_ALL_ENT rule */
	edgx_wr16(ipo_rbase, _IPO_REG_CMD,
		  (_IPO_ALL_ENT |
		   _IPO_CMD_READ |
		   _IPO_CMD_TRANSFER));
	/* Sleep and then wait until flags are cleared by HW */
	usleep_range(300, 400);
	do {
		reg = edgx_get16(ipo_rbase, _IPO_REG_CMD, 15, 15);
	} while (reg);

	edgx_pt_ipo_init_single(ipo_rbase, &ipo_all, _IPO_ALL_ENT);
	/* don't allow forwarding to self, no allowed for bridges! */
	edgx_set16(ipo_rbase, _IPO_REG_FWD, pt->ptid, pt->ptid, 0);
	edgx_wr16(ipo_rbase, _IPO_REG_CMD,
		  (_IPO_ALL_ENT |
		   _IPO_CMD_WRITE |
		   _IPO_CMD_TRANSFER));
	/* Sleep and then wait until flags are cleared by HW */
	usleep_range(300, 400);
	do {
		reg = edgx_get16(ipo_rbase, _IPO_REG_CMD, 15, 15);
	} while (reg);

	/* Clear out the rest using the none-rule */
	for (entry = _IPO_NONE_ENT_START; entry < _IPO_NRULES; entry++) {
		ipo_rbase = _IPO_BASE(pt->iobase);
		/* Read values for _IPO_ALL_ENT rule */
		edgx_wr16(ipo_rbase, _IPO_REG_CMD,
			  (entry | _IPO_CMD_READ | _IPO_CMD_TRANSFER));
		/* Sleep and then wait until flags are cleared by HW */
		usleep_range(300, 400);
		do {
			reg = edgx_get16(ipo_rbase, _IPO_REG_CMD, 15, 15);
		} while (reg);
		edgx_pt_ipo_init_single(ipo_rbase, &ipo_null, entry);
		/* Write values for _IPO_NONE rules */
		/* Read values for _IPO_ALL_ENT rule */
		edgx_wr16(ipo_rbase, _IPO_REG_CMD,
			  (entry | _IPO_CMD_READ | _IPO_CMD_TRANSFER));
		/* Sleep and then wait until flags are cleared by HW */
		usleep_range(300, 400);
		do {
			reg = edgx_get16(ipo_rbase, _IPO_REG_CMD, 15, 15);
		} while (reg);
	}
}

static ptid_t edgx_pt_ipo_get_mirror(struct edgx_pt *pt)
{
	u16 reg;
	ptid_t mgmt_ptid = edgx_com_get_mgmt_ptid(edgx_br_get_com(pt2br(pt)));
	edgx_io_t *ipo_rbase = _IPO_BASE(pt->iobase);

	edgx_wr16(ipo_rbase, _IPO_REG_CMD,
		 (0 | _IPO_CMD_READ | _IPO_CMD_TRANSFER));
	/* Sleep and then wait until flags are cleared by HW */
	usleep_range(300, 400);
	do {
		reg = edgx_get16(ipo_rbase, _IPO_REG_CMD, 15, 15);
	} while (reg);

	u16 mirr_cfg = edgx_rd16(ipo_rbase, _IPO_REG_MIRR);

	mirr_cfg = mirr_cfg ^ BIT(mgmt_ptid);

	return (mirr_cfg) ? ffs(mirr_cfg) - 1 : -1;
}

static int edgx_pt_ipo_set_mirror(struct edgx_pt *pt, ptid_t mirr_ptid)
{
	unsigned int i;
	ptid_t mgmt_ptid = edgx_com_get_mgmt_ptid(edgx_br_get_com(pt2br(pt)));
	u16 mirr_cfg = BIT(mgmt_ptid) | ((mirr_ptid < 0) ? 0 : BIT(mirr_ptid));
	u16 reg;

	if (mirr_ptid == edgx_pt_get_id(pt) || mirr_ptid == mgmt_ptid) {
		edgx_pt_err(pt, "Cannot mirror management port or to self.\n");
		return -EINVAL;
	}

	for (i = 0; i < _IPO_NRULES; i++) {
		edgx_io_t *ipo_rbase = _IPO_BASE(pt->iobase);

		edgx_wr16(ipo_rbase, _IPO_REG_CMD,
			  (i | _IPO_CMD_WRITE | _IPO_CMD_TRANSFER));
		/* Sleep and then wait until flags are cleared by HW */
		usleep_range(300, 400);
		do {
			reg = edgx_get16(ipo_rbase, _IPO_REG_CMD, 15, 15);
		} while (reg);
		edgx_wr16(ipo_rbase, _IPO_REG_MIRR, mirr_cfg);
		edgx_wr16(ipo_rbase, _IPO_REG_CMD,
			  (i | _IPO_CMD_WRITE | _IPO_CMD_TRANSFER));
		/* Sleep and then wait until flags are cleared by HW */
		usleep_range(300, 400);
		do {
			reg = edgx_get16(ipo_rbase, _IPO_REG_CMD, 15, 15);
		} while (reg);
	}
	return 0;
}

void edgx_pt_set_pvid(struct edgx_pt *pt, u16 vid)
{
	/* Set both registers; one is PVID, the other one also assigns priority-
	 * tagged frames to the PVID.
	 */
	edgx_set16(_GEN_BASE(pt->iobase), 0x10, 11, 0, vid);
	edgx_set16(_GEN_BASE(pt->iobase), 0x12, 11, 0, vid);
}

void edgx_pt_clear_pvid(struct edgx_pt *pt)
{
	edgx_pt_set_pvid(pt, _PVID_NONE);
}

u16 edgx_pt_get_pvid(struct edgx_pt *pt)
{
	return edgx_get16(_GEN_BASE(pt->iobase), 0x10, 11, 0);
}

static int edgx_pt_get_dflt_pcp(struct edgx_pt *pt)
{
	return edgx_get16(_GEN_BASE(pt->iobase), 0x10, 14, 12);
}

static int edgx_pt_set_dflt_pcp(struct edgx_pt *pt, unsigned int prio)
{
	if (prio > 7)
		return -EINVAL;
	edgx_set16(_GEN_BASE(pt->iobase), 0x10, 14, 12, prio);
	return 0;
}

static ssize_t flush_tree_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct edgx_pt *pt = edgx_dev2pt(dev);
	mstid_t mstid;

	if (kstrtou16(buf, 10, &mstid))
		return -EINVAL;

	if (edgx_br_vlan_flush_mstpt(edgx_br_get_vlan(pt2br(pt)),
				     mstid, pt->ptid))
		return -EINVAL;
	return count;
}

static ssize_t tree_port_state_read(struct file *filp, struct kobject *kobj,
				    struct bin_attribute *bin_attr, char *buf,
				    loff_t ofs, size_t count)
{
	struct edgx_pt *pt = edgx_dev2pt(kobj_to_dev(kobj));
	loff_t idx;
	u8 ptstate;

	if (edgx_sysfs_tbl_params(ofs, count, sizeof(unsigned int), &idx)  ||
	    edgx_br_vlan_get_mstpt_state(edgx_br_get_vlan(pt2br(pt)),
					 (mstid_t)idx, pt->ptid, &ptstate))
		return 0;

	*((unsigned int *)buf) = ptstate;
	return count;
}

static ssize_t tree_port_state_write(struct file *filp, struct kobject *kobj,
				     struct bin_attribute *bin_attr, char *buf,
				     loff_t ofs, size_t count)
{
	struct edgx_pt *pt = edgx_dev2pt(kobj_to_dev(kobj));
	loff_t idx;

	if (edgx_sysfs_tbl_params(ofs, count, sizeof(unsigned int), &idx) ||
	    edgx_br_vlan_set_mstpt_state(edgx_br_get_vlan(pt2br(pt)),
					 (mstid_t)idx, pt, *((u8 *)buf)))
		return -EINVAL;

	return count;
}

EDGX_DEV_ATTR_WO(flush_tree, "flushTree");
EDGX_BIN_ATTR_RW(tree_port_state, "treePortState",
		 EDGX_MAX_MSTID * sizeof(unsigned int));

static struct attribute *ieee8021_mstp_attrs[] = {
	&dev_attr_flush_tree.attr,
	NULL,
};

static struct bin_attribute *ieee8021_mstp_binattrs[] = {
	&bin_attr_tree_port_state,
	NULL,
};

static struct attribute_group ieee8021_mstp_group = {
	.name      = "ieee8021Mstp",
	.attrs     = ieee8021_mstp_attrs,
	.bin_attrs = ieee8021_mstp_binattrs,
};

static ssize_t num_tcs_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct edgx_pt *pt = edgx_dev2pt(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 edgx_br_get_generic(pt->parent, BR_GX_QUEUES));
}

static ssize_t dflt_usr_prio_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	struct edgx_pt *pt = edgx_dev2pt(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n", edgx_pt_get_dflt_pcp(pt));
}

static ssize_t dflt_usr_prio_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct edgx_pt *pt = edgx_dev2pt(dev);
	unsigned int prio;
	int r = kstrtouint(buf, 10, &prio);

	if (r)
		return r;
	return (edgx_pt_set_dflt_pcp(pt, prio)) ? -EINVAL : count;
}

static ssize_t external_show(struct device *dev,
			     struct device_attribute *attr,
			     char *buf)
{
	struct edgx_pt *pt = edgx_dev2pt(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 edgx_link_is_external(pt->link));
}

static ssize_t port_type_show(struct device *dev,
			      struct device_attribute *attr,
			      char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "2\n"); /* customerVlanPort */
}

EDGX_DEV_ATTR_RO(num_tcs,        "portNumTrafficClasses");
EDGX_DEV_ATTR_RW(dflt_usr_prio,  "portDefaultUserPriority");
EDGX_DEV_ATTR_RO(external,       "portExternal");

EDGX_DEV_FCT_ATTR_RO(aft, edgx_sysfs_one,   "portAcceptableFrameTypes");
EDGX_DEV_FCT_ATTR_RO(eif, edgx_sysfs_true,  "portEnableIngressFiltering");
EDGX_DEV_FCT_ATTR_RO(tag, edgx_sysfs_true,  "portTaggingCapable");
EDGX_DEV_FCT_ATTR_RO(cfgft, edgx_sysfs_false,
		     "portConfigurableAcceptableFrameTypes");
EDGX_DEV_FCT_ATTR_RO(inf, edgx_sysfs_true,  "portIngressFilteringCapable");
EDGX_DEV_FCT_ATTR_RO(cv,  edgx_sysfs_true,  "portTypeCapCustomerVlan");
EDGX_DEV_FCT_ATTR_RO(pn,  edgx_sysfs_false, "portTypeCapProviderNetwork");
EDGX_DEV_FCT_ATTR_RO(cn,  edgx_sysfs_false, "portTypeCapCustomerNetwork");
EDGX_DEV_FCT_ATTR_RO(ce,  edgx_sysfs_false, "portTypeCapCustomerEdge");
EDGX_DEV_FCT_ATTR_RO(cb,  edgx_sysfs_false, "portTypeCapCustomerBackbone");
EDGX_DEV_FCT_ATTR_RO(vi,  edgx_sysfs_false, "portTypeCapVirtualInstance");
EDGX_DEV_FCT_ATTR_RO(db,  edgx_sysfs_true,  "portTypeCapDBridge");
EDGX_DEV_FCT_ATTR_RO(rca, edgx_sysfs_false, "portTypeCapRemoteCustomerAccess");
EDGX_DEV_FCT_ATTR_RO(sf,  edgx_sysfs_false, "portTypeCapStationFacing");
EDGX_DEV_FCT_ATTR_RO(ua,  edgx_sysfs_false, "portTypeCapUplinkAccess");
EDGX_DEV_FCT_ATTR_RO(ur,  edgx_sysfs_false, "portTypeCapUplinkRelay");
EDGX_DEV_FCT_ATTR_RO(pt,  port_type,        "portType");

static void get_tc_prio_params(int idx, int type,  size_t *reg_ofs, int *bithi,
			       int *bitlo)
{
	if (idx < SZ_4) {
		if (type == _TYPE_PRIO_REGEN)
			*reg_ofs = _PRIO_REGEN_LO;
		else if (type == _TYPE_QUEUE_TBL)
			*reg_ofs = _QUEUE_TBL_LO;
		*bithi = idx * SZ_4 + 3;
		*bitlo = idx * SZ_4;
	} else {
		if (type == _TYPE_PRIO_REGEN)
			*reg_ofs = _PRIO_REGEN_HI;
		else if (type == _TYPE_QUEUE_TBL)
			*reg_ofs = _QUEUE_TBL_HI;

		*bithi = (idx - 4) * SZ_4 + 3;
		*bitlo = (idx - 4) * SZ_4;
	}
}

static ssize_t prio_regen_tbl_write(struct file *filp, struct kobject *kobj,
				    struct bin_attribute *bin_attr,
				    char *buf, loff_t ofs, size_t count)
{
	/* parameter buf contains value of priority
	 * parameter ofs contains value of PCP */
	loff_t idx = 0;
	struct edgx_pt *pt = edgx_dev2pt(kobj_to_dev(kobj));
	size_t reg_ofs;
	int bithi, bitlo;

	if (edgx_sysfs_tbl_params(ofs, count, sizeof(u8), &idx) ||
	    idx > _MAX_PRIO_VAL ||
	    ((u8 *)buf)[0] > _MAX_PRIO_VAL)
		return -EINVAL;

	get_tc_prio_params(idx, _TYPE_PRIO_REGEN, &reg_ofs, &bithi, &bitlo);
	edgx_set16(pt->iobase, reg_ofs, bithi, bitlo, ((u8 *)buf)[0]);
	printk("%s: PCP: %d, priority: %d\n", __func__, (u8)idx), ((u8 *)buf)[0];

	return count;
}

static ssize_t traffic_class_tbl_write(struct file *filp, struct kobject *kobj,
				       struct bin_attribute *bin_attr,
				       char *buf, loff_t ofs, size_t count)
{
	/* parameter buf contains value of traffic class
	 * parameter ofs contains value of priority*/
	loff_t idx = 0;
	struct edgx_pt *pt = edgx_dev2pt(kobj_to_dev(kobj));
	size_t reg_ofs;
	int bithi, bitlo;

	if (edgx_sysfs_tbl_params(ofs, count, sizeof(u8), &idx) ||
	    idx > _MAX_PRIO_VAL ||
	    ((u8 *)buf)[0] > (edgx_br_get_generic(edgx_pt_get_br(pt), BR_GX_QUEUES) - 1))
		return -EINVAL;

	get_tc_prio_params(idx, _TYPE_QUEUE_TBL, &reg_ofs, &bithi, &bitlo);
	edgx_set16(pt->iobase, reg_ofs, bithi, bitlo, ((u8 *)buf)[0]);
	printk("%s: priority: %d, traffic class: %d\n", __func__, (u8)idx, ((u8 *)buf)[0]);

	return count;
}

static ssize_t prio_regen_tbl_read(struct file *filp, struct kobject *kobj,
				   struct bin_attribute *bin_attr,
				   char *buf, loff_t ofs, size_t count)
{
	/* parameter buf contains value of priority
	 * parameter ofs contains value of PCP */
	loff_t idx = 0;
	struct edgx_pt *pt = edgx_dev2pt(kobj_to_dev(kobj));
	size_t reg_ofs;
	int bithi, bitlo;

	if (edgx_sysfs_tbl_params(ofs, count, sizeof(u8), &idx) ||
	    idx > _MAX_PRIO_VAL)
		return -EINVAL;

	get_tc_prio_params(idx, _TYPE_PRIO_REGEN, &reg_ofs, &bithi, &bitlo);
	((u8 *)buf)[0] = edgx_get16(pt->iobase, reg_ofs, bithi, bitlo);
	edgx_dbg("%s: PCP: %d, priority: %d, port: %s\n", __func__, (u8)idx,
			edgx_get16(pt->iobase, reg_ofs, bithi, bitlo), edgx_pt_get_name(pt));

	return count;
}

static ssize_t traffic_class_tbl_read(struct file *filp, struct kobject *kobj,
				      struct bin_attribute *bin_attr,
				      char *buf, loff_t ofs, size_t count)
{
	/* parameter buf contains value of traffic class
	 * parameter ofs contains value of priority*/
	loff_t idx = 0;
	struct edgx_pt *pt = edgx_dev2pt(kobj_to_dev(kobj));
	size_t reg_ofs;
	int bithi, bitlo;

	if (edgx_sysfs_tbl_params(ofs, count, sizeof(u8), &idx) ||
	    idx > _MAX_PRIO_VAL)
		return -EINVAL;

	get_tc_prio_params(idx, _TYPE_QUEUE_TBL, &reg_ofs, &bithi, &bitlo);
	((u8 *)buf)[0] = edgx_get16(pt->iobase, reg_ofs, bithi, bitlo);
	edgx_dbg("%s: priority: %d, traffic class: %d, port: %s\n", __func__, (u8)idx,
			edgx_get16(pt->iobase, reg_ofs, bithi, bitlo), edgx_pt_get_name(pt));

	return count;
}

EDGX_BIN_ATTR_RW(prio_regen_tbl,  "portUserPriorityRegenTable",
		EDGX_BR_NUM_PCP * sizeof(u8));
EDGX_BIN_ATTR_RW(traffic_class_tbl,  "portTrafficClassTable",
		EDGX_BR_NUM_PCP * sizeof(u8));

static struct attribute *ieee8021_bridge_attrs[] = {
	&dev_attr_num_tcs.attr,
	&dev_attr_dflt_usr_prio.attr,
	&dev_attr_external.attr,
	&dev_attr_aft.attr,
	&dev_attr_eif.attr,
	&dev_attr_tag.attr,
	&dev_attr_cfgft.attr,
	&dev_attr_inf.attr,
	&dev_attr_cv.attr,
	&dev_attr_pn.attr,
	&dev_attr_cn.attr,
	&dev_attr_ce.attr,
	&dev_attr_cb.attr,
	&dev_attr_vi.attr,
	&dev_attr_db.attr,
	&dev_attr_rca.attr,
	&dev_attr_sf.attr,
	&dev_attr_ua.attr,
	&dev_attr_ur.attr,
	&dev_attr_pt.attr,
	NULL,
};

static struct bin_attribute *ieee8021_bridge_binattrs[] = {
	&bin_attr_prio_regen_tbl,
	&bin_attr_traffic_class_tbl,
	NULL
};

static struct attribute_group ieee8021_bridge_group = {
	.name = "ieee8021Bridge",
	.attrs = ieee8021_bridge_attrs,
	.bin_attrs = ieee8021_bridge_binattrs,
};

static ssize_t mirror_port_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct edgx_pt *pt = edgx_dev2pt(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n",
			 edgx_pt_ipo_get_mirror(pt));
}

static ssize_t mirror_port_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct edgx_pt *pt = edgx_dev2pt(dev);
	ptid_t mirror_pt;
	int r;

	if (kstrtoint(buf, 10, &mirror_pt))
		return -EINVAL;

	r = edgx_pt_ipo_set_mirror(pt, mirror_pt);
	return (r) ? r : count;
}

static ssize_t cutthrough_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct edgx_pt *pt = edgx_dev2pt(dev);

	return scnprintf(buf, PAGE_SIZE, "%u\n",
			 edgx_get16(_GEN_BASE(pt->iobase), 0x0, 15, 15));
}

static ssize_t cutthrough_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct edgx_pt *pt = edgx_dev2pt(dev);
	bool en;

	if (kstrtobool(buf, &en))
		return -EINVAL;

	edgx_wr16(_GEN_BASE(pt->iobase), 0x1a, (en) ? 0xff : 0x0);
	edgx_set16(_GEN_BASE(pt->iobase), 0x0, 15, 15, (en) ? 1 : 0);
	return count;
}

EDGX_DEV_ATTR_RW(mirror_port, "mirrorPort");
EDGX_DEV_ATTR_RW(cutthrough, "cutThrough");

static struct attribute *edgex_ext_attrs[] = {
	&dev_attr_mirror_port.attr,
	&dev_attr_cutthrough.attr,
	NULL,
};

static struct attribute_group edgex_ext_group = {
	.name = "edgex-ext",
	.attrs = edgex_ext_attrs,
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 3, 0)
static int edgx_pt_attr_get(struct net_device *dev,
			    struct switchdev_attr *attr)
{
	struct edgx_pt *pt = net2pt(dev);
	int err = 0;

	switch (attr->id) {
	case SWITCHDEV_ATTR_ID_PORT_PARENT_ID:
		attr->u.ppid.id_len = sizeof(ETH_ALEN);
		memcpy(&attr->u.ppid.id, edgx_br_get_mac(pt2br(pt)),
		       attr->u.ppid.id_len);
		break;
	case SWITCHDEV_ATTR_ID_PORT_BRIDGE_FLAGS:
		attr->u.brport_flags = _BRPT_FLAGS;
		break;
	case SWITCHDEV_ATTR_ID_PORT_STP_STATE:
		err = edgx_br_vlan_get_mstpt_state(edgx_br_get_vlan(pt2br(pt)),
						   EDGX_CIST_MSTID, pt->ptid,
						   &attr->u.stp_state);
		break;
	case SWITCHDEV_ATTR_ID_BRIDGE_AGEING_TIME:
		edgx_pt_info(pt, "getting ageing time\n");
		attr->u.ageing_time = edgx_br_ageing_get(pt2br(pt));
		break;
	default:
		return -EOPNOTSUPP;
	}

	return err;
}
#endif

static int edgx_pt_attr_set(struct net_device *dev,
			    const struct switchdev_attr *attr,
			    struct switchdev_trans *trans)
{
	struct edgx_pt *pt = net2pt(dev);
	int err = 0;

	switch (attr->id) {
	case SWITCHDEV_ATTR_ID_PORT_STP_STATE:
		err = edgx_br_vlan_set_mstpt_state(edgx_br_get_vlan(pt2br(pt)),
						   EDGX_CIST_MSTID, pt,
						   attr->u.stp_state);
		break;
	case SWITCHDEV_ATTR_ID_BRIDGE_AGEING_TIME:
		err = edgx_br_ageing_set(pt2br(pt), attr->u.ageing_time);
		break;
	default:
		return -EOPNOTSUPP;
	}
	return err;
}

static int edgx_pt_obj_add(struct net_device *dev,
			   const struct switchdev_obj *obj,
			   struct switchdev_trans *trans)
{
	struct edgx_pt *pt = net2pt(dev);
	int             err  = 0;

	if (switchdev_trans_ph_prepare(trans))
		return 0;

	switch (obj->id) {
	case SWITCHDEV_OBJ_ID_PORT_VLAN:
		if (!is_bridged(pt)) {
			edgx_pt_err(pt, "Cannot add VID(s) to unbridged port\n");
			return -EINVAL;
		}
		err = edgx_br_vlan_add_pt(edgx_br_get_vlan(pt2br(pt)),
					  SWITCHDEV_OBJ_PORT_VLAN(obj), pt);
		break;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	case SWITCHDEV_OBJ_ID_PORT_MDB:
	case SWITCHDEV_OBJ_ID_HOST_MDB: {
		/* Ignore MDB objects to avoid kernel warning. */
		break;
	}
#endif
	default:
		err = -EOPNOTSUPP;
	}
	return err;
}

static int edgx_pt_obj_del(struct net_device *dev,
			   const struct switchdev_obj *obj)
{
	struct edgx_pt *pt = net2pt(dev);
	int             err  = 0;

	switch (obj->id) {
	case SWITCHDEV_OBJ_ID_PORT_VLAN:
		if (!is_bridged(pt)) {
			edgx_pt_err(pt, "Cannot delete VID from unbridged port\n");
			return -EINVAL;
		}
		err = edgx_br_vlan_del_pt(edgx_br_get_vlan(pt2br(pt)),
					  SWITCHDEV_OBJ_PORT_VLAN(obj), pt);
		break;
	default:
		err = -EOPNOTSUPP;
	}
	return err;
}

void edgx_pt_set_fid_fwd_state(struct edgx_pt *pt, fid_t fid, u8 ptstate)
{
	u16 hwstate;
	u64 ofs = fid;
	int bitlo;

	switch (ptstate) {
	case BR_STATE_FORWARDING:
		hwstate = _PT_STP_FWD;
		break;
	case BR_STATE_LEARNING:
		hwstate = _PT_STP_LEARN;
		break;
	case BR_STATE_LISTENING:
	case BR_STATE_DISABLED:
	case BR_STATE_BLOCKING:
		hwstate = _PT_STP_DISABLED;
		break;
	default:
		edgx_pt_warn(pt, "Unknown port state %u requested\n", ptstate);
		return;
	}

	bitlo = do_div(ofs, 8);
	ofs *= 2;
	bitlo *= 2;

	edgx_set16(pt->iobase, _FIDCFG_REG(ofs), bitlo + 1, bitlo, hwstate);
}

int edgx_pt_call_switchdev_notifiers(struct edgx_pt *pt, unsigned long type,
				     struct switchdev_notifier_info *info)
{
	int r;

	rtnl_lock();

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 3, 0)
	r = call_switchdev_notifiers(type, pt->netdev, info);
#else
	r = call_switchdev_notifiers(type, pt->netdev, info, NULL);
#endif

	rtnl_unlock();
	return r;
}

static netdev_tx_t edgx_pt_start_xmit(struct sk_buff *skb,
				      struct net_device *netdev)
{
	struct edgx_pt *pt = net2pt(netdev);

	return edgx_com_xmit(pt->hcom, skb, COM_FLAG_NONE);
}

void edgx_pt_rcv(struct edgx_pt *pt, struct sk_buff *skb, ptflags_t flags)
{
	struct ethhdr *h;

	skb->dev = pt->netdev;
	if (!skb->protocol)
		skb->protocol = eth_type_trans(skb, pt->netdev);
	h = eth_hdr(skb);

	/* ATTENTION:
	 * This is a workaround to prevent receiving management frames on the
	 * internal port, which should actually be sent out and received on the
	 * internal end station!
	 * Definitely has side-effects, but at least should prevent MSTP, the
	 * bridge and possibly other parts to get confused.
	 * (see FLEXDE-2279)
	 * eth_type_trans() shall be called before this workaround.
	 */
	if (!memcmp(pt->netdev->dev_addr, &h->h_source, pt->netdev->addr_len)) {
		dev_kfree_skb_any(skb);
		return;
	}

	if (flags == COM_FLAG_NONE) {
		/* no flags set, regular receive */
		netif_receive_skb(skb);
	} else {
		/* flags set, dispatch to subfunction */
		/* We don't support generic registration here;
		 * Currently this would be overkill as we just have Preempt
		 * and (maybe) MACSec support here
		 */
		if (flags & COM_FLAG_PREEMPT_MASK)
			edgx_preempt_rcv(pt->preempt, skb, flags);
	}
}

static int edgx_brpt_set_mac_addr(struct net_device *netdev, void *p)
{
	u16 reg;
	struct edgx_pt *pt = net2pt(netdev);
	struct sockaddr *addr = p;
	edgx_io_t *ipo_rbase = _IPO_BASE(pt->iobase);

	edgx_wr16(ipo_rbase, _IPO_REG_CMD,
		  (_IPO_SELF_ENT |
		   _IPO_CMD_READ |
		   _IPO_CMD_TRANSFER));
	/* Sleep and then wait until flags are cleared again by chip. */
	usleep_range(300, 400);
	do {
		reg = edgx_get16(ipo_rbase, _IPO_REG_CMD, 15, 15);
	} while (reg);

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	edgx_pt_ipo_set_mac(ipo_rbase, addr->sa_data);
	edgx_wr16(ipo_rbase, _IPO_REG_CMD,
		  (_IPO_SELF_ENT |
		   _IPO_CMD_WRITE |
		   _IPO_CMD_TRANSFER));
	/* Sleep and then wait until flags are cleared again by chip. */
	usleep_range(300, 400);
	do {
		reg = edgx_get16(ipo_rbase, _IPO_REG_CMD, 15, 15);
	} while (reg);
	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);

	return 0;
}

static int edgx_eppt_set_mac_addr(struct net_device *netdev, void *p)
{
	struct sockaddr *addr = p;

	if (!is_valid_ether_addr(addr->sa_data))
		return -EADDRNOTAVAIL;

	/* Endpoint is virtual, so we just copy in the new address */
	memcpy(netdev->dev_addr, addr->sa_data, netdev->addr_len);

	return 0;
}

static int edgx_pt_do_ioctl(struct net_device *netdev, struct ifreq *ifr,
			    int cmd)
{
	struct edgx_pt *pt = net2pt(netdev);

	switch (cmd) {
	case SIOCSHWTSTAMP: {
		edgx_info("pt_do_ioctl SIOCSHWTSTAMP ...\n");
		return edgx_com_hwts_set(pt->hcom, ifr);
	}
	case SIOCGHWTSTAMP:{
		edgx_info("pt_do_ioctl SIOCGHWTSTAMP ...\n");
		return edgx_com_hwts_get(pt->hcom, ifr);
	}
	default:
		return edgx_link_ioctl(netdev, ifr, cmd);
	}
	return -EOPNOTSUPP;
}

static void edgx_pt_tx_timeout(struct net_device *netdev)
{
	struct edgx_pt *pt = net2pt(netdev);

	edgx_com_tx_timeout(pt->hcom, netdev);
}

static void
edgx_brpt_get_stats64(struct net_device *netdev,
		      struct rtnl_link_stats64 *storage)
{
	struct edgx_pt *pt = net2pt(netdev);

	edgx_stat_update(pt->hstat);

	storage->multicast  = edgx_stat_get(pt->hstat, _STAT_RX_MC) +
			      edgx_stat_get(pt->hstat, _STAT_RX_BC);

	storage->rx_packets = edgx_stat_get(pt->hstat, _STAT_RX_UC) +
			      storage->multicast;

	storage->tx_packets = edgx_stat_get(pt->hstat, _STAT_TX_UC) +
			      edgx_stat_get(pt->hstat, _STAT_TX_BC) +
			      edgx_stat_get(pt->hstat, _STAT_TX_MC);

	storage->rx_bytes   = edgx_stat_get(pt->hstat, _STAT_RX_GOOD_OCTETS);
	storage->tx_bytes   = edgx_stat_get(pt->hstat, _STAT_TX_OCTETS);

	storage->rx_crc_errors = edgx_stat_get(pt->hstat, _STAT_RX_CRC);

	/* We don't count RX_FRAGMENTS as length error, because it's actually
	 * the combination of length and CRC error
	 */
	storage->rx_length_errors =
				edgx_stat_get(pt->hstat, _STAT_RX_OVERSZ) +
				edgx_stat_get(pt->hstat, _STAT_RX_UNDERSZ);

	storage->rx_errors = edgx_stat_get(pt->hstat, _STAT_RX_FRAGS)    +
			     edgx_stat_get(pt->hstat, _STAT_RX_JABBER)   +
			     edgx_stat_get(pt->hstat, _STAT_RX_ERR)      +
			     edgx_stat_get(pt->hstat, _STAT_RX_FULLDROP) +
			     storage->rx_crc_errors                        +
			     storage->rx_length_errors;

	storage->tx_errors = (edgx_stat_get(pt->hstat, _STAT_TX_PRIQDROP) +
			      edgx_stat_get(pt->hstat, _STAT_TX_EARLYDROP)
			     );
}

static int edgx_pt_get_phys_name(struct net_device *netdev,
				 char *name, size_t len)
{
	struct edgx_pt *pt = net2pt(netdev);
	ptid_t ptid = edgx_pt_get_id(pt);
	int ret;

	if (PT_IS_BRP_ID(ptid))
		ptid++;

	ret = snprintf(name, len, "p%u", ptid);

	return (ret < 0 || ret >= len) ? -ENOBUFS : 0;
}

static void
edgx_eppt_get_stats64(struct net_device *netdev,
		      struct rtnl_link_stats64 *storage)
{
	struct edgx_pt *pt = net2pt(netdev);
	struct edgx_pt *mgmt_pt = edgx_com_get_mgmt_pt(pt->hcom);
	struct rtnl_link_stats64 mgmt_stats;

	if (!mgmt_pt)
		return;

	/* Just get stats from management port, and inverse them */
	edgx_brpt_get_stats64(mgmt_pt->netdev, &mgmt_stats);

	storage->tx_packets = mgmt_stats.rx_packets;
	storage->rx_packets = mgmt_stats.tx_packets;
	storage->tx_bytes   = mgmt_stats.rx_bytes;
	storage->rx_bytes   = mgmt_stats.tx_bytes;

	/* We assume that we don't experience errors on internal link */
	storage->rx_crc_errors = 0;
	storage->rx_length_errors = 0;
	storage->tx_errors = 0;
}

void edgx_pt_get_drvinfo(struct net_device *netdev,
			 struct ethtool_drvinfo *info)
{
	struct edgx_pt *pt = net2pt(netdev);
	u32 svn = edgx_br_get_version(pt->parent);

	strlcpy(info->driver,  EDGX_SW_CORE_NAME, sizeof(info->driver));
	strlcpy(info->version, EDGX_SW_CORE_VERSION, sizeof(info->version));
	strlcat(info->version, EDGX_GIT, sizeof(info->version));
	snprintf(info->fw_version, sizeof(info->fw_version), "SVN %u", svn);
}

int edgx_pt_set_wol(struct net_device *netdev, struct ethtool_wolinfo *wol)
{
	return -EOPNOTSUPP;
}

void edgx_pt_get_wol(struct net_device *netdev, struct ethtool_wolinfo *wol)
{
	wol->supported = 0;
	wol->wolopts = 0;
}

int edgx_pt_get_ts_info(struct net_device *netdev, struct ethtool_ts_info *info)
{
	struct edgx_pt *pt = net2pt(netdev);

	info->phc_index = edgx_tm_get_phc(pt->time, 0);
	if (info->phc_index < 0)
		/* port is not managed by time interface - use defaults */
		return ethtool_op_get_ts_info(netdev, info);

	info->so_timestamping = SOF_TIMESTAMPING_TX_HARDWARE |
				SOF_TIMESTAMPING_RX_HARDWARE |
				SOF_TIMESTAMPING_RAW_HARDWARE;
	info->tx_types = (1u << HWTSTAMP_TX_OFF) |
			 (1u << HWTSTAMP_TX_ON);
	info->rx_filters = (1u << HWTSTAMP_FILTER_NONE) |
			   (1u << HWTSTAMP_FILTER_PTP_V2_L2_EVENT);

	info->phc_index = edgx_tm_get_phc(pt->time, 0);

#if defined(ETHTOOL_STSCLK) || defined(ETHTOOL_SWRKCLK)
	info->phc_index_2nd   = edgx_tm_get_phc(pt->time, 1);
	info->phc_timestamper = edgx_tm_get_ts_phc(pt->time);
	info->phc_worker      = edgx_tm_get_wrk_phc(pt->time);
#endif

	return 0;
}

static inline u32 edgx_pt_get_link_state(struct net_device *netdev)
{
	return edgx_link_get_state(edgx_net2link(netdev));
}

static int edgx_netdev_event(struct notifier_block *unused,
			     unsigned long event, void *ptr);

static void
edgx_fdb_offload_notify(struct edgx_pt *pt,
			struct switchdev_notifier_fdb_info *recv_info)
{
	struct switchdev_notifier_fdb_info info;

	info.addr = recv_info->addr;
	info.vid = recv_info->vid;

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 3, 0)
	call_switchdev_notifiers(SWITCHDEV_FDB_OFFLOADED,
				 pt->netdev, &info.info);
#else
	info.offloaded = true;
	call_switchdev_notifiers(SWITCHDEV_FDB_OFFLOADED,
				 pt->netdev, &info.info, NULL);
#endif
}

int edgx_pt_fdb_add(struct ndmsg *ndm, struct nlattr *tb[],
		    struct net_device *netdev,
		    const unsigned char *addr, u16 vid,
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 3, 0)
		    u16 flags)
#else
		    u16 flags,
		    struct netlink_ext_ack *extack)
#endif
{
	int err;
	struct edgx_pt *pt = net2pt(netdev);
	struct switchdev_notifier_fdb_info fdb_info = {
		.addr = addr,
		.vid = vid,
	};

	err = edgx_brfdb_add(edgx_br_get_fdb(pt2br(pt)), &fdb_info, pt->ptid);
	if (err) {
		edgx_pt_err(pt, "FDB add failed err=%d\n", err);
		return err;
	}
	return 0;
}

int edgx_pt_fdb_del(struct ndmsg *ndm, struct nlattr *tb[],
		    struct net_device *netdev,
		    const unsigned char *addr, u16 vid)
{
	int err;
	struct edgx_pt *pt = net2pt(netdev);
	struct switchdev_notifier_fdb_info fdb_info = {
		.addr = addr,
		.vid = vid,
	};

	err = edgx_brfdb_del(edgx_br_get_fdb(pt2br(pt)), &fdb_info, pt->ptid);
	if (err) {
		edgx_pt_err(pt, "FDB delete failed err=%d\n", err);
		return err;
	}
	return 0;
}

static int edgx_pt_fdb_dump_cb(struct sk_buff *skb, struct netlink_callback *cb,
			       struct net_device *dev,
			       const unsigned char *addr, u16 vid, int *idx)
{
	u32 portid = NETLINK_CB(cb->skb).portid;
	u32 seq = cb->nlh->nlmsg_seq;
	struct nlmsghdr *nlh;
	struct ndmsg *ndm;

	if (*idx < cb->args[2])
		goto skip;

	nlh = nlmsg_put(skb, portid, seq, RTM_NEWNEIGH,
			sizeof(*ndm), NLM_F_MULTI);
	if (!nlh)
		return -EMSGSIZE;

	ndm = nlmsg_data(nlh);
	ndm->ndm_family  = AF_BRIDGE;
	ndm->ndm_pad1    = 0;
	ndm->ndm_pad2    = 0;
	ndm->ndm_flags   = NTF_SELF;
	ndm->ndm_type    = 0;
	ndm->ndm_ifindex = dev->ifindex;
	ndm->ndm_state   = NUD_REACHABLE | NUD_NOARP;

	if (nla_put(skb, NDA_LLADDR, ETH_ALEN, addr))
		goto nla_put_failure;

	if (vid && nla_put_u16(skb, NDA_VLAN, vid))
		goto nla_put_failure;

	nlmsg_end(skb, nlh);

skip:
	(*idx)++;
	return 0;
nla_put_failure:
	nlmsg_cancel(skb, nlh);
	return -EMSGSIZE;
}

int edgx_pt_fdb_dump(struct sk_buff *skb, struct netlink_callback *ncb,
		     struct net_device *dev,
		     struct net_device *filter_dev, int *idx)
{
	struct edgx_pt *pt = net2pt(dev);
	int err;

	err = edgx_brfdb_dump(edgx_br_get_fdb(pt2br(pt)), skb, ncb, dev,
			      edgx_pt_fdb_dump_cb, pt->ptid, idx);
	return err;
}

static struct notifier_block edgx_netdev_nb __read_mostly = {
	.notifier_call = edgx_netdev_event,
};

int edgx_tc_setup(struct net_device *netdev, enum tc_setup_type type,
		   void *type_data)
{
	struct edgx_pt *pt = net2pt(netdev);
	struct edgx_com *com = edgx_br_get_com(pt->parent);
	struct tc_mqprio_qopt *mqprio = type_data;
	u8 num_tc, num_rx_queues, num_tx_queues;
	int i;

	edgx_pt_warn(pt, "edgx_tc_setup type:%d\n", type);

	if ((type != TC_SETUP_QDISC_MQPRIO) ||
	    (!edgx_multiqueue_support_get(com, &num_tx_queues, &num_rx_queues)))
		return -EOPNOTSUPP;

	mqprio->hw = TC_MQPRIO_HW_OFFLOAD_TCS;
	num_tc = mqprio->num_tc;

	edgx_pt_warn(pt, "edgx_tc_setup num_tc:%d\n", num_tc);

	/* This is called by reset */
	if (!num_tc) {
		netdev_reset_tc(netdev);
		netdev_set_num_tc(netdev, num_tc);
		return 0;
	}

	/* Check if we have enough BD rings available to accommodate all TCs */
	if (num_tc > num_tx_queues) {
		netdev_err(netdev, "Max %d traffic classes supported\n",
			   num_tx_queues);
		return -EINVAL;
	}


	/* Do not change rx queues */
	netdev_set_num_tc(netdev, num_tc);

	// TODO: netdev_set_prio_tc_map and netdev_get_prio_tc_map for stats and q selection?
	// netdev_set_prio_tc_map is done by the tc mqprio cmd so maybe do it at start

	/* Each TC is associated with one netdev queue */
	// TODO use offset and count from tc mqprio command?
	for (i = 0; i < num_tc; i++)
		netdev_set_tc_queue(netdev, i, 1, i);

	return 0;
}

static u16 edgx_select_ep_queue(struct net_device *netdev, struct sk_buff *skb,
			      struct net_device *sb_dev)
{
	struct edgx_pt *pt;
	int txq;

	edgx_dbg("edgx_select_ep_q\n");

	if (sb_dev) {
		u8 tc = netdev_get_prio_tc_map(netdev, skb->priority);
		struct net_device *vdev = sb_dev;

		txq = vdev->tc_to_txq[tc].offset;
		txq += reciprocal_scale(skb_get_hash(skb),
					vdev->tc_to_txq[tc].count);

		return txq;
	} else
		return netdev_pick_tx(netdev, skb, NULL) % netdev->real_num_tx_queues;
}

static u16 edgx_select_brport_queue(struct net_device *netdev, struct sk_buff *skb,
			      struct net_device *sb_dev)
{
	struct edgx_pt *pt = net2pt(netdev);

	edgx_dbg("edgx_select_brport_q port:%s\n", edgx_pt_get_name(pt));

	return edgx_get_tc_mgmtraffic(pt->parent);
}


#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
static int edgx_pt_get_pt_parent_id(struct net_device *dev,
				    struct netdev_phys_item_id *ppid)
{
	struct edgx_pt *pt = net2pt(dev);

	ppid->id_len = sizeof(ETH_ALEN);
	memcpy(&ppid->id, edgx_br_get_mac(pt2br(pt)), ppid->id_len);

	return 0;
}
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 3, 0)
static const struct switchdev_ops edgx_brpt_switchdev_ops = {
	.switchdev_port_attr_get = edgx_pt_attr_get,
	.switchdev_port_attr_set = edgx_pt_attr_set,
	.switchdev_port_obj_add  = edgx_pt_obj_add,
	.switchdev_port_obj_del  = edgx_pt_obj_del,
};
#endif

static const struct net_device_ops edgx_brpt_netdev_ops = {
	.ndo_open               = edgx_pt_open,
	.ndo_stop               = edgx_pt_stop,
	.ndo_start_xmit         = edgx_pt_start_xmit,
	.ndo_set_mac_address    = edgx_brpt_set_mac_addr,
	.ndo_do_ioctl           = edgx_pt_do_ioctl,
	.ndo_get_stats64        = edgx_brpt_get_stats64,
	.ndo_get_phys_port_name = edgx_pt_get_phys_name,
	.ndo_tx_timeout         = edgx_pt_tx_timeout,
	.ndo_fdb_add            = edgx_pt_fdb_add,
	.ndo_fdb_del            = edgx_pt_fdb_del,
	.ndo_fdb_dump           = edgx_pt_fdb_dump,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	.ndo_get_port_parent_id	= edgx_pt_get_pt_parent_id,
#endif
	.ndo_select_queue       = edgx_select_brport_queue,
};

static const struct net_device_ops edgx_eppt_netdev_ops = {
	.ndo_open               = edgx_pt_open,
	.ndo_stop               = edgx_pt_stop,
	.ndo_start_xmit         = edgx_pt_start_xmit,
	.ndo_set_mac_address    = edgx_eppt_set_mac_addr,
	.ndo_do_ioctl           = edgx_pt_do_ioctl,
	.ndo_get_stats64        = edgx_eppt_get_stats64,
	.ndo_tx_timeout         = edgx_pt_tx_timeout,
	.ndo_setup_tc           = edgx_tc_setup,
	.ndo_select_queue       = edgx_select_ep_queue,
};

static const struct ethtool_ops edgx_eppt_ethtool_ops = {
	.get_drvinfo        = edgx_pt_get_drvinfo,
	.get_wol            = edgx_pt_get_wol,
	.set_wol            = edgx_pt_set_wol,
	.get_link           = edgx_pt_get_link_state,
	.get_link_ksettings = edgx_link_get_ksettings,
	.set_link_ksettings = edgx_link_set_ksettings,
};

static const struct ethtool_ops edgx_brpt_ethtool_ops = {
	.get_drvinfo        = edgx_pt_get_drvinfo,
	.get_wol            = edgx_pt_get_wol,
	.set_wol            = edgx_pt_set_wol,
	.nway_reset         = edgx_link_nway_reset,
	.get_link           = edgx_pt_get_link_state,
	.get_ts_info        = edgx_pt_get_ts_info,
	.get_link_ksettings = edgx_link_get_ksettings,
	.set_link_ksettings = edgx_link_set_ksettings,
	.get_ethtool_stats  = edgx_pt_stats,
	.get_strings        = edgx_pt_strings,
	.get_sset_count     = edgx_pt_sset_count,
};

static int edgx_pt_upper_event(struct edgx_pt *pt, unsigned long event,
			       void *ptr)
{
	struct netdev_notifier_changeupper_info *info  = ptr;
	struct net_device                       *upper = info->upper_dev;
	int err = NOTIFY_DONE;

	if (!netif_is_bridge_master(upper))
		return NOTIFY_DONE;

	if (info->linking) {
		err = edgx_br_pt_join(pt2br(pt), upper);
		err = notifier_from_errno(err);
	} else {
		edgx_br_pt_leave(pt2br(pt));
		edgx_br_vlan_purge_pt(edgx_br_get_vlan(pt2br(pt)), pt);
		err = NOTIFY_OK;
	}
	return err;
}

static int edgx_netdev_event(struct notifier_block *unused,
			     unsigned long event, void *ptr)
{
	struct net_device *netdev = netdev_notifier_info_to_dev(ptr);
	struct edgx_pt  *pt   = net2pt(netdev);

	if (netdev->netdev_ops != &edgx_brpt_netdev_ops)
		return NOTIFY_DONE; /* not one of ours */

	switch (event) {
	case NETDEV_CHANGEUPPER:
		return edgx_pt_upper_event(pt, event, ptr);
	default:
		break;
	}
	return NOTIFY_DONE;
}

static void edgx_switchdev_event_work_fn(struct work_struct *work)
{
	struct edgx_switchdev_event_work *wk =
		container_of(work, struct edgx_switchdev_event_work, work);
	struct edgx_pt *pt = wk->pt;
	struct switchdev_notifier_fdb_info *fdb_info;
	int err;

	rtnl_lock();
	switch (wk->event) {
	case SWITCHDEV_FDB_ADD_TO_DEVICE:
		fdb_info = &wk->fdb_info;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
		if (!fdb_info->added_by_user)
			break;
#endif
		if (!is_bridged(pt)) {
			edgx_pt_err(pt, "Cannot add FDB entry to unbridged port\n");
			break;
		}
		err = edgx_brfdb_add(edgx_br_get_fdb(pt2br(pt)),
				     fdb_info, pt->ptid);
		if (err) {
			edgx_pt_err(pt, "FDB add failed err=%d\n", err);
			break;
		}
		edgx_fdb_offload_notify(pt, fdb_info);
		break;
	case SWITCHDEV_FDB_DEL_TO_DEVICE:
		fdb_info = &wk->fdb_info;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
		if (!fdb_info->added_by_user)
			break;
#endif
		if (!is_bridged(pt)) {
			edgx_pt_err(pt, "Cannot delete FDB emtry from unbridged port\n");
			break;
		}
		err = edgx_brfdb_del(edgx_br_get_fdb(pt2br(pt)),
				     fdb_info, pt->ptid);
		if (err)
			edgx_pt_err(pt, "FDB delete failed err=%d\n", err);
		break;
	}
	rtnl_unlock();

	kfree(wk->fdb_info.addr);
	kfree(wk);
	dev_put(pt->netdev);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
static int
edgx_switchdev_pt_attr_set_event(struct net_device *netdev,
		struct switchdev_notifier_port_attr_info *port_attr_info)
{
	int err;

	err = edgx_pt_attr_set(netdev, port_attr_info->attr,
			       port_attr_info->trans);

	port_attr_info->handled = true;
	return notifier_from_errno(err);
}

static int
edgx_switchdev_pt_obj_event(unsigned long event, struct net_device *netdev,
			struct switchdev_notifier_port_obj_info *port_obj_info)
{
	int err = -EOPNOTSUPP;

	switch (event) {
	case SWITCHDEV_PORT_OBJ_ADD:
		err = edgx_pt_obj_add(netdev, port_obj_info->obj,
				      port_obj_info->trans);
		break;
	case SWITCHDEV_PORT_OBJ_DEL:
		err = edgx_pt_obj_del(netdev, port_obj_info->obj);
		break;
	}

	port_obj_info->handled = true;
	return notifier_from_errno(err);
}

static int edgx_switchdev_blocking_event(struct notifier_block *unused,
					 unsigned long event, void *ptr)
{
	struct net_device *dev = switchdev_notifier_info_to_dev(ptr);

	if (dev->netdev_ops != &edgx_brpt_netdev_ops)
		return NOTIFY_DONE; /* not one of ours */

	switch (event) {
	case SWITCHDEV_PORT_OBJ_ADD:
	case SWITCHDEV_PORT_OBJ_DEL:
		return edgx_switchdev_pt_obj_event(event, dev, ptr);
	case SWITCHDEV_PORT_ATTR_SET:
		return edgx_switchdev_pt_attr_set_event(dev, ptr);
	}

	return NOTIFY_DONE;
}
#endif

static int edgx_switchdev_event(struct notifier_block *unused,
				unsigned long event, void *ptr)
{
	struct net_device *netdev = switchdev_notifier_info_to_dev(ptr);
	struct edgx_pt *pt;
	struct edgx_br *br;
	struct edgx_switchdev_event_work *wk;
	struct switchdev_notifier_fdb_info *fdb_info = ptr;

	if (netdev->netdev_ops != &edgx_brpt_netdev_ops)
		return NOTIFY_DONE; /* not one of ours */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	if (event == SWITCHDEV_PORT_ATTR_SET)
		return edgx_switchdev_pt_attr_set_event(netdev, ptr);
#endif

	pt = net2pt(netdev);
	br = edgx_pt_get_br(pt);

	wk = kzalloc(sizeof(*wk), GFP_ATOMIC);
	if (WARN_ON(!wk))
		return NOTIFY_BAD;

	INIT_WORK(&wk->work, edgx_switchdev_event_work_fn);
	wk->pt = pt;
	wk->event = event;

	switch (event) {
	case SWITCHDEV_FDB_ADD_TO_DEVICE: /* fall through */
	case SWITCHDEV_FDB_DEL_TO_DEVICE:
		memcpy(&wk->fdb_info, ptr,
		       sizeof(wk->fdb_info));
		wk->fdb_info.addr = kzalloc(ETH_ALEN, GFP_ATOMIC);
		if (unlikely(!wk->fdb_info.addr)) {
			kfree(wk);
			return NOTIFY_BAD;
		}
		ether_addr_copy((u8 *)wk->fdb_info.addr, fdb_info->addr);
		/* Take a reference on the rocker device */
		dev_hold(netdev);
		break;
	default:
		kfree(wk);
		return NOTIFY_DONE;
	}

	queue_work(edgx_br_get_owq(br), &wk->work);
	return NOTIFY_DONE;
}

static struct notifier_block edgx_switchdev_nb __read_mostly = {
	.notifier_call = edgx_switchdev_event,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
static struct notifier_block edgx_switchdev_blocking_nb = {
	.notifier_call = edgx_switchdev_blocking_event,
};
#endif

ptid_t edgx_pt_get_id(struct edgx_pt *pt)
{
	return (pt) ? pt->ptid : PT_INV_ID;
}

struct edgx_time *edgx_pt_get_time(struct edgx_pt *pt)
{
	return (pt) ? pt->time : NULL;
}

const char *edgx_pt_get_name(struct edgx_pt *pt)
{
	return (pt) ? pt->netdev->name : "(unnamed port)";
}

struct edgx_br *edgx_pt_get_br(struct edgx_pt *pt)
{
	return (pt) ? pt2br(pt) : NULL;
}

struct edgx_link *edgx_pt_get_link(struct edgx_pt *pt)
{
	return (pt) ? pt->link : NULL;
}

struct net_device *edgx_pt_get_netdev(struct edgx_pt *pt)
{
	return (pt) ? pt->netdev : NULL;
}

int edgx_pt_get_speed(struct edgx_pt *pt)
{
	if (!pt)
		return SPEED_UNKNOWN;
	return edgx_link_get_speed(pt->link);
}

u32 edgx_pt_get_speed_caps(struct edgx_pt *pt)
{
	u32 s = SUPPORTED_10baseT_Full | SUPPORTED_100baseT_Full;

	if (!pt)
		return 0;
	if (edgx_br_get_generic(pt->parent, BR_GX_GIGABIT))
		s |= SUPPORTED_1000baseT_Full;

	return s;
}

static edgx_pt_link_cb _link_cb[] = {
	/* TODO: Put callback functions to port submodules here that require
	 * link change notifications
	 */
	edgx_preempt_link_chng,
	NULL,
};

static void edgx_pt_update_speed(struct edgx_pt *pt, int speed)
{
	switch (speed) {
	case SPEED_10:
		/* Set MII/10 Mbit/s */
		edgx_set16(_GEN_BASE(pt->iobase), 0x0, 5, 4, 0x0);
		edgx_set16(_GEN_BASE(pt->iobase), 0x0, 9, 8, 0x3);
		break;
	case SPEED_100:
		/* Set MII/100 Mbit/s */
		edgx_set16(_GEN_BASE(pt->iobase), 0x0, 5, 4, 0x0);
		edgx_set16(_GEN_BASE(pt->iobase), 0x0, 9, 8, 0x2);
		break;
	case SPEED_1000:
		/* Set MII/1000 Mbit/s */
		edgx_set16(_GEN_BASE(pt->iobase), 0x0, 5, 4, 0x2);
		edgx_set16(_GEN_BASE(pt->iobase), 0x0, 9, 8, 0x1);
		break;
	default:
		edgx_pt_err(pt, "Invalid link speed: %u\n",
			    edgx_link_get_speed(pt->link));
		break;
	}
}

void edgx_pt_link_change(struct net_device *netdev)
{
	struct edgx_pt *pt = net2pt(netdev);
	edgx_pt_link_cb *cb;
	int lnk_state = edgx_link_get_state(pt->link);
	int lnk_speed = edgx_link_get_speed(pt->link);

	if (lnk_state) {
		edgx_pt_update_speed(pt, lnk_speed);
		edgx_link_update_speed(pt->link, lnk_speed);
		edgx_pt_info(pt, "Link is UP at %u Mbps\n", lnk_speed);
	} else {
		edgx_pt_info(pt, "Link is DOWN\n");
	}

	/* Notify all receivers about link change */
	for (cb = &_link_cb[0]; *cb; cb++)
		(*cb)(pt, lnk_state, lnk_speed);
}

ktime_t _edgx_pt_get_dly(struct edgx_pt *pt, size_t ofs_base, bool max)
{
	size_t ofs = ofs_base;
	unsigned int mii_cycle_ns;
	u16 v;

	switch (edgx_pt_get_speed(pt)) {
	case SPEED_10:
		mii_cycle_ns = 400;
		break;
	case SPEED_100:
		mii_cycle_ns = 40;
		ofs          += BR_FEAT_DLY_SPD_STEP;
		break;
	case SPEED_1000:
		mii_cycle_ns = 8;
		ofs          += (2 * BR_FEAT_DLY_SPD_STEP);
		break;
	default:
		edgx_pt_warn(pt, "Internal delay unknown (set to 0ns) due to unknown port speed.\n");
		return ktime_set(0, 0);
	}

	ofs += (BR_FEAT_DLY_MAX_OFS * max);
	v = edgx_br_get_feature(pt->parent, ofs);

	/* Calculation for the framesize-independent part is identical for G2O
	 * and I2G. However, the manual states for I2G the formula for the
	 * combined (dependent and independent) delay.
	 */
	return ktime_set(0, (edgx_br_get_cycle_ns(pt->parent) *
			     (v & 0xFF) + mii_cycle_ns * ((v >> 8) & 0xFF)));
}

ktime_t edgx_pt_get_g2omax(struct edgx_pt *pt)
{
	return _edgx_pt_get_dly(pt, BR_FEAT_G2O_BASE, true);
}

ktime_t edgx_pt_get_g2omin(struct edgx_pt *pt)
{
	return _edgx_pt_get_dly(pt, BR_FEAT_G2O_BASE, false);
}

ktime_t edgx_pt_get_i2gmax(struct edgx_pt *pt)
{
	return _edgx_pt_get_dly(pt, BR_FEAT_I2G_BASE, true);
}

ktime_t edgx_pt_get_i2gmin(struct edgx_pt *pt)
{
	return _edgx_pt_get_dly(pt, BR_FEAT_I2G_BASE, false);
}

static void edgx_pt_set_mgmt(struct edgx_pt *pt, bool is_mgmt)
{
	edgx_set16(_GEN_BASE(pt->iobase), 0, 3, 2, is_mgmt);
}

struct edgx_sched *edgx_pt_get_sched(struct edgx_pt *pt)
{
	return (pt) ? pt->sched : NULL;
}

void edgx_pt_add_sysfs(struct edgx_pt *pt, struct attribute_group *grp)
{
	int err = sysfs_create_group(&pt->netdev->dev.kobj, grp);

	if (err)
		edgx_pt_warn(pt, "Can't create sysfs group %s\n", grp->name);
}

void edgx_pt_rem_sysfs(struct edgx_pt *pt, struct attribute_group *grp)
{
	if (pt && grp)
		sysfs_remove_group(&pt->netdev->dev.kobj, grp);
}

struct edgx_pt *edgx_dev2pt(struct device *dev)
{
	struct net_device *netdev = container_of(dev, struct net_device, dev);

	return net2pt(netdev);
}

ptid_t edgx_dev2ptid(struct device *dev)
{
	struct edgx_pt *pt = edgx_dev2pt(dev);

	return (pt) ? pt->ptid : PT_INV_ID;
}

struct edgx_fqtss *edgx_pt_get_fqtss(struct edgx_pt *pt)
{
	return (pt) ? pt->fqtss : NULL;
}

struct edgx_preempt *edgx_pt_get_preempt(struct edgx_pt *pt)
{
	return (pt) ? pt->preempt : NULL;
}

static struct edgx_pt *_edgx_pt_init(struct edgx_br  *br,
				     struct edgx_pt **ppt,
				     ptid_t           ptid)
{
	struct edgx_pt  *pt   = NULL;
	struct net_device *netdev = alloc_etherdev(sizeof(**ppt));
	u8 mac[ETH_ALEN];

	if (!netdev)
		return NULL;

	pt   = netdev_priv(netdev);
	*ppt = netdev_priv(netdev);
	memset(pt, 0, sizeof(*pt));

	/* can do multicast */
	netdev->flags    |= IFF_MULTICAST;
	/* prevent dead-loop bug? */
	netdev->features |= NETIF_F_LLTX;
	/* don't change network namespace? */
	netdev->features |= NETIF_F_NETNS_LOCAL;

	pt->ptid    = ptid;
	pt->parent = br;
	pt->netdev = netdev;

	ether_addr_copy(mac, edgx_br_get_mac(br));
	mac[ETH_ALEN - 1] = (u8)pt->ptid;
	ether_addr_copy(netdev->dev_addr, mac);

	if (PT_IS_EP_ID(ptid))
		snprintf(pt->netdev->name, IFNAMSIZ, "sw%uep",
			 edgx_br_get_id(br));
	else
		snprintf(pt->netdev->name, IFNAMSIZ, "sw%up%d",
			 edgx_br_get_id(br), ptid + 1);

	pt->hcom = edgx_com_reg_pt(edgx_br_get_com(br), pt);
	if (!pt->hcom) {
		free_netdev(netdev);
		return NULL;
	}
	return pt;
}

static void _edgx_pt_clear(struct edgx_pt *pt)
{
	edgx_com_unreg_pt(pt->hcom);
	free_netdev(pt->netdev);
}

static inline int _edgx_pt_activate(struct edgx_pt *pt)
{
	int r;

	ether_setup(pt->netdev);
	SET_NETDEV_DEV(pt->netdev, edgx_br_get_dev(pt2br(pt)));
	register_netdev(pt->netdev);

	r = edgx_link_init(pt, &pt->link);
	if (r)
		unregister_netdev(pt->netdev);
	return r;
}

static inline void _edgx_pt_deactivate(struct edgx_pt *pt)
{
	edgx_link_shutdown(pt->link);
	unregister_netdev(pt->netdev);
}

static void edgx_netdev_ep_setup(struct net_device *netdev)
{
	//ether_setup(netdev);
	// TODO: extra mq setups?
	netdev->netdev_ops  = &edgx_eppt_netdev_ops;
	netdev->ethtool_ops = &edgx_eppt_ethtool_ops;
	netdev->priv_flags |= IFF_DONT_BRIDGE;
}

int edgx_init_epport(struct edgx_br *br, struct edgx_pt **ppt)
{
	struct edgx_pt *pt = NULL;
	struct edgx_com *com = edgx_br_get_com(br);
	u8	num_rx_queues, num_tx_queues;
	struct net_device *netdev = NULL;

	if (edgx_multiqueue_support_get(com, &num_tx_queues, &num_rx_queues)) {
		u8 mac[ETH_ALEN];
		char port_name[IFNAMSIZ];

		snprintf(port_name, IFNAMSIZ, "sw%uep",
			 edgx_br_get_id(br));
		netdev = alloc_netdev_mqs(sizeof(struct edgx_pt), port_name,
				 NET_NAME_UNKNOWN, edgx_netdev_ep_setup,
				 num_tx_queues, num_rx_queues);

		if (!netdev)
			goto err_port;

		pt = netdev_priv(netdev);
		*ppt = netdev_priv(netdev);
		memset(pt, 0, sizeof(*pt));


		/* can do multicast */
		netdev->flags    |= IFF_MULTICAST;
		/* prevent dead-loop bug? */
		netdev->features |= NETIF_F_LLTX;
		/* don't change network namespace? */
		netdev->features |= NETIF_F_NETNS_LOCAL;

		pt->ptid    = PT_EP_ID;
		pt->parent = br;
		pt->netdev = netdev;

		ether_addr_copy(mac, edgx_br_get_mac(br));
		mac[ETH_ALEN - 1] = (u8)pt->ptid;
		ether_addr_copy(netdev->dev_addr, mac);

		pt->hcom = edgx_com_reg_pt(edgx_br_get_com(br), pt);
		if (!pt->hcom) {
			free_netdev(netdev);
			return NULL;
		}
		// TODO: netdev_set_prio_tc_map to default ones??
		netif_set_real_num_tx_queues(netdev, num_tx_queues);
		netif_set_real_num_rx_queues(netdev, num_rx_queues);
		// TODO set q len properly with define EDGX_DMA_DESC_CNT
		netdev->tx_queue_len = (256 - 1) * 8; // Max queues x Desc number
	} else {
		pt = _edgx_pt_init(br, ppt, PT_EP_ID);

		if (!pt)
			goto err_port;

		pt->netdev->netdev_ops  = &edgx_eppt_netdev_ops;
		pt->netdev->ethtool_ops = &edgx_eppt_ethtool_ops;
		pt->netdev->priv_flags |= IFF_DONT_BRIDGE;
	}

	edgx_pt_info(pt, "Setup Endpoint Port ...\n");
	if (_edgx_pt_activate(pt))
		goto err_port;

	return 0;

err_port:
	edgx_br_err(br, "Setup Endpoint Port failed!\n");
	return -ENOMEM;
}

int edgx_probe_brports(struct edgx_br *br, struct edgx_pt **ppt)
{
	const struct edgx_ifdesc *ifd;
	struct edgx_ifdesc        pifd;
	ptid_t                    ptid;
	ptid_t                    mgmt_ptid;
	const struct edgx_ifreq   ifreq = { .id = AC_PORT_ID, .v_maj = 2 };

	if (!br || !ppt)
		return -EINVAL;
	ifd = edgx_ac_get_if(&ifreq);
	if (!ifd)
		return -ENODEV;

	edgx_br_info(br, "Traffic class for management traffic: %u\n",
			edgx_get_tc_mgmtraffic(br));

	mgmt_ptid = edgx_com_get_mgmt_ptid(edgx_br_get_com(br));

	edgx_ac_for_each_ifpt(ptid, ifd, &pifd) {
		struct edgx_pt *pt;
		struct attribute **attr;

		pt = _edgx_pt_init(br, ppt, ptid);
		if (!pt)
			goto err_port;

		pt->iobase = pifd.iobase;
		pt->netdev->netdev_ops     = &edgx_brpt_netdev_ops;
		pt->netdev->ethtool_ops    = &edgx_brpt_ethtool_ops;
#if LINUX_VERSION_CODE < KERNEL_VERSION(5, 3, 0)
		pt->netdev->switchdev_ops  = &edgx_brpt_switchdev_ops;
#endif
		pt->netdev->watchdog_timeo = msecs_to_jiffies(EDGX_PT_WDT_MS);

		edgx_pt_info(pt, "Setup Bridge Port %d %s...\n", ptid,
			     (ptid == mgmt_ptid) ? "(mgmt)" : " ");

		if (_edgx_pt_activate(pt)) {
			edgx_pt_set_mgmt(pt, false);
			_edgx_pt_clear(pt);
			goto err_port;
		}

		edgx_pt_set_mgmt(pt, ptid == mgmt_ptid);
		edgx_pt_ipo_init(pt, mgmt_ptid);

		pt->time  = edgx_br_get_pt_time(br, ptid);
		pt->hstat = edgx_stat_alloc_hdl(edgx_br_get_pt_stat(br, ptid),
						ptid, &_pt_statinfo);

		if (edgx_br_get_generic(edgx_pt_get_br(pt), BR_GX_QUEUES) ==
				EDGX_BR_MAX_TC)
			edgx_wr32(pt->iobase, _QUEUE_TBL_LO, _TX8_DEF_VAL);

		edgx_wr16(_GEN_BASE(pt->iobase), 0x10, BIT(15));
		edgx_set16(_GEN_BASE(pt->iobase), 0x0, 1, 0, _PT_STP_FWD);

		edgx_pt_clear_pvid(pt);

		edgx_pt_add_sysfs(pt, &ieee8021_mstp_group);
		edgx_pt_add_sysfs(pt, &ieee8021_bridge_group);
		/* Take over common capability attributes from bridge */
		for (attr = &ieee8021_brpt_common[0]; *attr; attr++)
			sysfs_add_file_to_group(&pt->netdev->dev.kobj, *attr,
						ieee8021_bridge_group.name);
		edgx_pt_add_sysfs(pt, &edgex_ext_group);

		if (!edgx_probe_fqtss(pt, pt->iobase, &pt->fqtss))
			edgx_pt_info(pt, "   Adding FQTSS ...\n");
		if (!edgx_probe_sched(pt, edgx_br_get_sched_com(pt->parent),
				      &pt->sched))
			edgx_pt_info(pt, "   Adding Scheduled Traffic ...\n");
		if (!edgx_probe_preempt(pt, pt->hcom, pt->iobase, &pt->preempt))
			edgx_pt_info(pt, "   Adding Frame Preemption ...");
		/* Set initial speed for FES, based on link's default */
		edgx_pt_update_speed(pt, edgx_link_get_speed(pt->link));
		goto next_port;

err_port:
		edgx_br_err(br, "Setup Bridge Port %d failed!\n", ptid);
		*ppt = NULL;
next_port:
		ppt++;
	}

	// TODO: Check return values
	register_netdevice_notifier(&edgx_netdev_nb);
	register_switchdev_notifier(&edgx_switchdev_nb);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	register_switchdev_blocking_notifier(&edgx_switchdev_blocking_nb);
#endif
	return 0;
}

void edgx_shutdown_brports(struct edgx_pt **ppt)
{
	int i;

	if (!ppt)
		return;

	// TODO: Check return values
#if LINUX_VERSION_CODE >= KERNEL_VERSION(5, 3, 0)
	unregister_switchdev_blocking_notifier(&edgx_switchdev_blocking_nb);
#endif
	unregister_switchdev_notifier(&edgx_switchdev_nb);
	unregister_netdevice_notifier(&edgx_netdev_nb);
	for (i = 0; i < EDGX_BR_MAX_PORTS; i++, ppt++)
		if (*ppt) {
			edgx_pt_rem_sysfs(*ppt, &ieee8021_mstp_group);
			edgx_shutdown_preempt((*ppt)->preempt);
			edgx_shutdown_sched((*ppt)->sched);
			edgx_shutdown_fqtss((*ppt)->fqtss);
			_edgx_pt_deactivate(*ppt);
			edgx_stat_free_hdl((*ppt)->hstat);
			edgx_pt_set_mgmt((*ppt), false);
			edgx_pt_info((*ppt), "Shutdown Bridge Port %d ...\n",
				     (*ppt)->ptid);
			_edgx_pt_clear(*ppt);
		}
}

void edgx_shutdown_epport(struct edgx_pt *pt)
{
	if (pt) {
		_edgx_pt_deactivate(pt);
		edgx_pt_info(pt, "Shutdown Endpoint Port ...\n");
		_edgx_pt_clear(pt);
	}
}

u32 edgx_pt_get_framesize(struct edgx_pt *pt, int idx)
{
	return (edgx_rd16(_FRAMESIZE_BASE(pt->iobase, idx), 0)
		& _FRAMESIZE_MASK) - _FRAMESIZE_OVERHEAD;
}

void  edgx_pt_set_framesize(struct edgx_pt *pt, int idx, u32 val)
{
	edgx_wr16(_FRAMESIZE_BASE(pt->iobase, idx), 0,
		  (val + _FRAMESIZE_OVERHEAD) & _FRAMESIZE_MASK);
}

enum _queue_fill_pt_idx {
	_FL_SAMPLE_CNT = _STAT_MAX,
	_FL_Q0_MIN,
	_FL_Q1_MIN,
	_FL_Q2_MIN,
	_FL_Q3_MIN,
	_FL_Q4_MIN,
	_FL_Q5_MIN,
	_FL_Q6_MIN,
	_FL_Q7_MIN,
	_FL_Q0_MAX,
	_FL_Q1_MAX,
	_FL_Q2_MAX,
	_FL_Q3_MAX,
	_FL_Q4_MAX,
	_FL_Q5_MAX,
	_FL_Q6_MAX,
	_FL_Q7_MAX,
	_QUEUE_FILL_MAX,         /* must be last element */
};

static const char
edgx_statsc_strings[_QUEUE_FILL_MAX][ETH_GSTRING_LEN] = {
	[_STAT_RX_GOOD_OCTETS] =	"RX GOOD OCTETS",
	[_STAT_RX_BAD_OCTETS] =		"RX BAD OCTETS",
	[_STAT_RX_UC] =			"RX UNICAST",
	[_STAT_RX_BC] =			"RX BROADCAST",
	[_STAT_RX_MC] =			"RX MULTICAST",
	[_STAT_RX_UNDERSZ] =		"RX UNDERSIZE",
	[_STAT_RX_FRAGS] =		"RX FRAGMENTS",
	[_STAT_RX_OVERSZ] =		"RX OVERSIZE",
	[_STAT_RX_JABBER] =		"RX JABBER",
	[_STAT_RX_ERR] =		"RX ERR",
	[_STAT_RX_CRC] =		"RX CRC",
	[_STAT_RX_FULLDROP] =		"FULL DROP",
	[_STAT_RX_POLICED] =		"RX POLICED",
	[_STAT_TX_OCTETS] =		"TX OCTETS",
	[_STAT_TX_UC] =			"TX UNICAST",
	[_STAT_TX_BC] =			"TX BROADCAST",
	[_STAT_TX_MC] =			"TX MULTICAST",
	[_STAT_TX_PRIQDROP] =		"Q DROP",
	[_STAT_TX_EARLYDROP] =		"EARLY DROP",
	[_STAT_RX_64] =			"RX_64",
	[_STAT_RX_65_127] =		"RX_65_127",
	[_STAT_RX_128_255] =		"RX_128_255",
	[_STAT_RX_256_511] =		"RX_256_511",
	[_STAT_RX_512_1023] =		"RX_512_1023",
	[_STAT_RX_1024_1536] =		"RX_1024_1536",
	[_FL_SAMPLE_CNT] =	"FL_SAMPLE_CNT", // begin queue fill values
	[_FL_Q0_MIN] =		"FL_Q0_MIN",
	[_FL_Q1_MIN] =		"FL_Q1_MIN",
	[_FL_Q2_MIN] =		"FL_Q2_MIN",
	[_FL_Q3_MIN] =		"FL_Q3_MIN",
	[_FL_Q4_MIN] =		"FL_Q4_MIN",
	[_FL_Q5_MIN] =		"FL_Q5_MIN",
	[_FL_Q6_MIN] =		"FL_Q6_MIN",
	[_FL_Q7_MIN] =		"FL_Q7_MIN",
	[_FL_Q0_MAX] =		"FL_Q0_MAX",
	[_FL_Q1_MAX] =		"FL_Q1_MAX",
	[_FL_Q2_MAX] =		"FL_Q2_MAX",
	[_FL_Q3_MAX] =		"FL_Q3_MAX",
	[_FL_Q4_MAX] =		"FL_Q4_MAX",
	[_FL_Q5_MAX] =		"FL_Q5_MAX",
	[_FL_Q6_MAX] =		"FL_Q6_MAX",
	[_FL_Q7_MAX] =		"FL_Q7_MAX",
};

void edgx_pt_stats(struct net_device *netdev,
		   struct ethtool_stats *cmd, u64 *pt_num)
{
	u64 stat_values[_QUEUE_FILL_MAX];
	struct edgx_pt *pt = net2pt(netdev);
	int i, num_queues;
	edgx_io_t *cnt_base;
	u64 dma_ports;

	/* For getting statistic values with command
	 * ethtool -S following sequence is used by ethtool:
	 * 1) with request get_sset_count (edgx_pt_sset_count) the
	 *    number of statistic values is asked
	 * 2) with get_strings (edgx_pt_strings) the names of the statistic
	 *    values are asked
	 * 3) with get_ethtool_stats (edgx_pt_stats) the statistic
	 *    values are asked
	 * ethtool uses get_sset_count also to set "supports-statistics" when
	 * executing the command ethtool -i. (set to "yes" if
	 * return value > 0).
	 */
	edgx_stat_update(pt->hstat);

	for (i = 0; i < _STAT_MAX; i++)
		stat_values[i] = edgx_stat_get(pt->hstat, i);

	/* Write queue fill values after statistic values into the table
	 * First check if port is DMA, in that case queue fill values are zero
	 * If port is not DMA, set the fill level control register and
	 * when this is reset to zero, read fill level data
	 */
	dma_ports = edgx_br_get_generic(pt->parent, BR_GX_DMA_PORTS);
	if (test_bit(pt->ptid, (const unsigned long *)&dma_ports)) {
		for (i = _STAT_MAX; i < _QUEUE_FILL_MAX; i++)
			stat_values[i] = 0;
	} else {
		/* set lock*/
		spin_lock_bh(&pt->fillqu.lock);

		/* set register FL_CAPT */
		edgx_wr16(pt->iobase, _FL_CAPT, 1);
		/* wait until register FL_CAPT is cleared*/
		while (edgx_rd16(pt->iobase, _FL_CAPT) != 0)
			udelay(5);
		/* read values of complete fill capature and overwrite queue
		 * values afterwards if number of queues < EDGX_BR_MAX_TC
		 */
		cnt_base = pt->iobase + _FL_CAPT + 2;

		/* read _FL_SAMPLE_CNT and min/max values for all queues */
		for (i = 0; i < (_QUEUE_FILL_MAX - _STAT_MAX);
				i++, cnt_base += 2)
			stat_values[_STAT_MAX + i] = edgx_rd16(cnt_base, 0);

		/* read number of queues in actual system */
		num_queues = edgx_br_get_generic(pt->parent, BR_GX_QUEUES);
		/* overwrite stat_values with zero values for non existing
		 * queues
		 */
		for (i = 0; i < EDGX_BR_MAX_TC; i++) {
			if (i >= num_queues) {
				stat_values[_FL_Q0_MIN + i] = 0;
				stat_values[_FL_Q0_MAX + i] = 0;
			}
		}

		/* clear lock */
		spin_unlock_bh(&pt->fillqu.lock);
	}

	memcpy(pt_num, stat_values, sizeof(stat_values));
}

void edgx_pt_strings(struct net_device *netdev, u32 stringset, u8 *ptr_char)
{
	if (stringset == ETH_SS_STATS)
		memcpy(ptr_char, edgx_statsc_strings,
		       sizeof(edgx_statsc_strings));
}

int edgx_pt_sset_count(struct net_device *netdev, int stringset_id)
{
	if (stringset_id == ETH_SS_STATS)
		return _QUEUE_FILL_MAX;
	else
		return 0;
}
