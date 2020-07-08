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

#include <linux/interrupt.h>
#include "edge_ac.h"
#include "edge_util.h"
#include "edge_time.h"
#include "edge_com_ts.h"

/* #define EDGX_TS_DBG */

#if defined(EDGX_TS_DBG)
#define ts_dbg	edgx_info
#else
#define ts_dbg(args...)
#endif

#define EDGX_COM_TS_PORT_OFS	 (0xC000)
#define EDGX_COM_TS_CTRL_TX	 (0x400)
#define EDGX_COM_TS_CTRL_RX	 (0x402)
#define EDGX_COM_TS_HDR_LEN	 (16U)     /* NOTE: error in the IP manual */
#define EDGX_COM_TS_S_LO	 (0x0004)
#define EDGX_COM_TS_NS_HI	 (0x0002)
#define EDGX_COM_TS_NS_LO	 (0x0000)
#define EDGX_COM_TS_HDR		 (0x000E)
#define EDGX_COM_TS_OFS		 (0x0080)
#define EDGX_COM_TS_RX_OFS	 (0x0200)
#define EDGX_COM_TS_TX_OFS	 (0x0000)
#define EDGX_COM_TS_SEC_MASK	 (0x0f)
#define EDGX_COM_TS_NSEC_MASK	 (0x3fffffff)
#define EDGX_COM_TS_INT_MASK_VAL (EDGX_BR_INT_MASK_TX_TS)
#define EDGX_COM_TS_INT_MASK_TX	 (BIT(0))

#define edgx_work_to_ts(_work) container_of(_work, struct edgx_com_ts, work_tx)

static const u8 *edgx_com_get_ptp_hdr(struct sk_buff *skb)
{
	const struct ethhdr *eth = (struct ethhdr *)skb_mac_header(skb);
	const u8 *ptp_hdr;

	if (eth->h_proto != htons(ETH_P_1588))
		return NULL;

	ptp_hdr = (u8 *)(eth + 1);
	if ((ptp_hdr[0] & 0x0f) > 3)
		return NULL;

	return ptp_hdr;
}

static void edgx_com_ts_read(edgx_io_t *ts_base, struct timespec64 *cur_time,
			     ktime_t *timestamp, u16 *hdr, u16 hdr_size)
{
	int i;
	u64 sec, full_sec;
	u32 nsec;
	struct timespec64 hwts;

	for (i = 0; i < hdr_size; i++) {
		hdr[i] = __cpu_to_le16(edgx_rd16(ts_base,
						 EDGX_COM_TS_HDR +
						 (i * 2)));
	}

	sec = edgx_rd16(ts_base, EDGX_COM_TS_S_LO);
	nsec = edgx_rd16(ts_base, EDGX_COM_TS_NS_HI) << 16;
	nsec |= edgx_rd16(ts_base, EDGX_COM_TS_NS_LO);
	/*
	 * Get full seconds for bitwidth-limited timestamper seconds.
	 * There is no way to know amount of time passed between timestamper
	 * and current time from PTP clock, assume max. 1 s has passed.
	 */
	full_sec = cur_time->tv_sec;
	sec &= EDGX_COM_TS_SEC_MASK;
	if ((full_sec & 0x1ull) != (sec & 0x1ull))
		full_sec--;
	if ((full_sec & EDGX_COM_TS_SEC_MASK) != sec)
		return;

	nsec &= EDGX_COM_TS_NSEC_MASK;
	set_normalized_timespec64(&hwts, full_sec, (long)nsec);
	*timestamp = timespec64_to_ktime(hwts);
}

static void edgx_com_ts_read_rx(struct edgx_com_pts *pts,
				struct timespec64 *cur_time, u8 pos,
				ktime_t *timestamp, u16 *hdr, u16 hdr_size)
{
	edgx_io_t *ts_base;

	ts_base = pts->iobase + EDGX_COM_TS_RX_OFS +
		  (pos * EDGX_COM_TS_OFS);

	edgx_com_ts_read(ts_base, cur_time, timestamp, hdr, hdr_size);
}

static void edgx_com_ts_read_tx(struct edgx_com_pts *pts,
				struct timespec64 *cur_time, u8 pos,
				ktime_t *timestamp, u16 *hdr, u16 hdr_size)
{
	edgx_io_t *ts_base;

	ts_base = pts->iobase + EDGX_COM_TS_TX_OFS +
		  (pos * EDGX_COM_TS_OFS);

	edgx_com_ts_read(ts_base, cur_time, timestamp, hdr, hdr_size);
}

static void edgx_com_ts_find_tx(struct edgx_com_pts *pts,
				struct timespec64 *cur_time)
{
	int i;
	const u8 *hdr;
	unsigned long flags = 0;
	struct skb_shared_hwtstamps hwts;
	struct sk_buff *found_skb = NULL;
	u16 ts_hdr[EDGX_COM_TS_HDR_LEN];
	u16 ts_ctrl = edgx_rd16(pts->iobase, EDGX_COM_TS_CTRL_TX);

	while (!(ts_ctrl & BIT(pts->tx_ts_pos))) {
		ts_ctrl |= BIT(pts->tx_ts_pos);
		edgx_com_ts_read_tx(pts, cur_time, pts->tx_ts_pos,
				    &hwts.hwtstamp,
				    ts_hdr, EDGX_COM_TS_HDR_LEN);

		edgx_wr16(pts->iobase, EDGX_COM_TS_CTRL_TX,
			  BIT(pts->tx_ts_pos));
		pts->tx_ts_pos = (pts->tx_ts_pos + 1) % EDGX_COM_TS_CTRL_CNT;

		found_skb = NULL;
		spin_lock_irqsave(&pts->lock, flags);
		for (i = 0; i < EDGX_COM_TS_CTRL_CNT; i++) {
			if (!pts->tx_queue[i])
				continue;
			hdr = edgx_com_get_ptp_hdr(pts->tx_queue[i]);
			if (!memcmp(ts_hdr, hdr, EDGX_COM_TS_HDR_LEN)) {
				found_skb = pts->tx_queue[i];
				pts->tx_queue[i] = NULL;
				break;
			}
		}
		spin_unlock_irqrestore(&pts->lock, flags);

		if (found_skb) {
			skb_shinfo(found_skb)->tx_flags |=
					    SKBTX_HW_TSTAMP | SKBTX_IN_PROGRESS;
			ts_dbg("ts_find_tx: ts=%lld, pos=%d\n",
			       hwts.hwtstamp.tv64, pts->tx_ts_pos);
			edgx_com_txts_dispatch(found_skb, &hwts);
		}
	}
}

static void edgx_com_ts_find_rx(struct edgx_com_pts *pts, const u16 *hdr,
				struct timespec64 *cur_time, ktime_t *time)
{
	u16 ts_hdr[EDGX_COM_TS_HDR_LEN];
	ktime_t tmp_time;
	u16 ts_ctrl = edgx_rd16(pts->iobase, EDGX_COM_TS_CTRL_RX);

	while (!(ts_ctrl & BIT(pts->rx_ts_pos))) {
		ts_ctrl |= BIT(pts->rx_ts_pos);
		edgx_com_ts_read_rx(pts, cur_time, pts->rx_ts_pos, &tmp_time,
				    ts_hdr, EDGX_COM_TS_HDR_LEN);

		edgx_wr16(pts->iobase, EDGX_COM_TS_CTRL_RX,
			  BIT(pts->rx_ts_pos));
		pts->rx_ts_pos = (pts->rx_ts_pos + 1) % EDGX_COM_TS_CTRL_CNT;

		if (!memcmp(ts_hdr, hdr, EDGX_COM_TS_HDR_LEN)) {
			*time = tmp_time;
			ts_dbg("ts_find_rx: ts=%lld, pos=%d\n",
			       time->tv64, pts->rx_ts_pos);
			break;
		}
	}
}

int edgx_com_ts_cfg_get(struct edgx_com_ts *ts, ptcom_t ptcom,
			struct ifreq *ifr)
{
	struct edgx_com_pts *pts = ts->pts[ffs(ptcom) - 1];

	if (!pts)
		return -EINVAL;

	return copy_to_user(ifr->ifr_data, &pts->hwts_cfg,
			    sizeof(pts->hwts_cfg)) ? -EFAULT : 0;
}

/* Set per port HW timestamp configuration.
 *  NOTE: There is no possibility to set per port timestamp in the IP.
 *  It is a property of the switch. Therefore it is not possible to switch
 *  between L2 and L4 from the port interface without affecting other ports.
 */
int edgx_com_ts_cfg_set(struct edgx_com_ts *ts, ptcom_t ptcom,
			struct ifreq *ifr)
{
	struct edgx_com_pts *pts = ts->pts[ffs(ptcom) - 1];
	struct hwtstamp_config cfg;

	if (!pts)
		return -EINVAL;

	if (copy_from_user(&cfg, ifr->ifr_data, sizeof(cfg)))
		return -EFAULT;

	if (cfg.flags)
		return -EINVAL;

	switch (cfg.tx_type) {
	case HWTSTAMP_TX_OFF:
	case HWTSTAMP_TX_ON:
		pts->hwts_cfg.tx_type = cfg.tx_type;
		break;
	default:
		return -ERANGE;
	}

	switch (cfg.rx_filter) {
	case HWTSTAMP_FILTER_NONE:
		pts->hwts_cfg.rx_filter = cfg.rx_filter;
		break;
	/* Only V2 L2 is supported now. L4 requires special IPO rules */
	case HWTSTAMP_FILTER_PTP_V2_L2_SYNC:
	case HWTSTAMP_FILTER_PTP_V2_L2_DELAY_REQ:
		/* With V2 type filters which specify a Sync or Delay Request,
		 * Path Delay Request/Response messages are also time stamped
		 * by hardware so notify the caller the requested packets plus
		 * some others are time stamped.
		 */
		cfg.rx_filter = HWTSTAMP_FILTER_SOME;
		pts->hwts_cfg.rx_filter = HWTSTAMP_FILTER_PTP_V2_EVENT;
		break;
	case HWTSTAMP_FILTER_PTP_V2_L2_EVENT:
		pts->hwts_cfg.rx_filter = cfg.rx_filter;
		break;
	default:
		return -ERANGE;
	}

	return copy_to_user(ifr->ifr_data, &cfg, sizeof(cfg)) ?
			    -EFAULT : 0;
}

struct sk_buff *edgx_com_ts_xmit(struct edgx_com_ts *ts,
				 struct sk_buff *skb, ptcom_t ptcom)
{
	const u8            *ptp_hdr;
	struct sk_buff      *skb_backup = NULL;
	unsigned long        flags = 0;
	struct edgx_com_pts *pts = ts->pts[ffs(ptcom) - 1];

	if (!pts)
		return skb;

	ptp_hdr = edgx_com_get_ptp_hdr(skb);
	if (!ptp_hdr)
		return skb;

	skb_backup = skb_clone(skb, GFP_ATOMIC);
	if (!skb_backup) {
		edgx_err("%s(): Cannot clone skb!\n", __func__);
		return skb;
	}

	spin_lock_irqsave(&pts->lock, flags);
	if (pts->tx_queue[pts->tx_q_pos]) {
		dev_kfree_skb_any(pts->tx_queue[pts->tx_q_pos]);
		// TODO: increment lost timestamp stat
	}
	pts->tx_queue[pts->tx_q_pos] = skb;
	pts->tx_q_pos = (pts->tx_q_pos + 1) % EDGX_COM_TS_CTRL_CNT;
	spin_unlock_irqrestore(&pts->lock, flags);

	skb_shinfo(skb)->tx_flags |= SKBTX_IN_PROGRESS;
	skb_tx_timestamp(skb);
	skb_shinfo(skb)->tx_flags &= ~SKBTX_HW_TSTAMP;
	skb_shinfo(skb)->tx_flags &= ~SKBTX_IN_PROGRESS;
	ts_dbg("ts_xmit: queued. pos=%d\n", pts->tx_q_pos);
	return skb_backup;
}

static void edgx_com_ts_work_tx(struct work_struct *work)
{
	ptid_t ptid;
	struct timespec64 cur_time;
	struct edgx_time *time;
	struct edgx_com_pts *pts;
	struct edgx_com_ts *ts = edgx_work_to_ts(work);

	edgx_br_clr_int_stat(ts->parent, EDGX_COM_TS_INT_MASK_TX);

	for (ptid = 0; ptid < EDGX_BR_MAX_PORTS; ptid++) {
		pts = ts->pts[ptid];
		if (pts) {
			time = edgx_br_get_pt_time(ts->parent, ptid);
			edgx_tm_get_ts_time(time, &cur_time);
			edgx_com_ts_find_tx(pts, &cur_time);
		}
	}
	edgx_br_set_int_mask_irqsave(ts->parent, EDGX_COM_TS_INT_MASK_TX);
}

u16 edgx_com_ts_isr(struct edgx_com_ts *ts, u16 intmask, u16 intstat)
{
	if (!(intmask & EDGX_COM_TS_INT_MASK_TX))
		return intmask;

	if (!(intstat & EDGX_COM_TS_INT_MASK_TX))
		return intmask;

	ts_dbg("COM TS: Isr.\n");

	/* Disable TX interrupt until they are handled */
	intmask &= ~EDGX_COM_TS_INT_MASK_TX;

	queue_work(ts->wq_tx, &ts->work_tx);

	return intmask;
}

void edgx_com_ts_rx(struct edgx_com_ts *ts, struct sk_buff *skb, ptcom_t ptcom)
{
	u16                          ptid = ffs(ptcom) - 1;
	struct edgx_com_pts         *pts = ts->pts[ptid];
	struct skb_shared_hwtstamps *hwts;
	const u8                    *ptp_hdr;
	struct timespec64            cur_time;
	struct edgx_time            *time;

	hwts = skb_hwtstamps(skb);
	hwts->hwtstamp = ktime_set(0, 0);

	if (!pts)
		return;

	ptp_hdr = edgx_com_get_ptp_hdr(skb);
	if (!ptp_hdr)
		return;

	time = edgx_br_get_pt_time(ts->parent, ptid);
	edgx_tm_get_ts_time(time, &cur_time);
	edgx_com_ts_find_rx(pts, (const u16 *)ptp_hdr,
			    &cur_time, &hwts->hwtstamp);
}

int edgx_com_ts_init(struct edgx_com_ts *ts, const char *drv_name, int irq,
		     struct edgx_br *br)
{
	int                       ret, i;
	const struct edgx_ifdesc *ifd;
	struct edgx_ifdesc        pifd;
	ptid_t                    ptid;
	const struct edgx_ifreq   ifreq = { .id = AC_PORT_TS_ID, .v_maj = 1 };
	struct edgx_com_pts      *pts;

	ifd = edgx_ac_get_if(&ifreq);
	if (!ifd)
		return -ENODEV;

	ts->irq = irq;
	ts->parent = br;

	INIT_WORK(&ts->work_tx, &edgx_com_ts_work_tx);
	ts->wq_tx = alloc_workqueue(drv_name,
				    WQ_HIGHPRI | WQ_MEM_RECLAIM, 0);
	if (!ts->wq_tx) {
		edgx_err("COM TS: alloc_workqueue failed!\n");
		ret = -ENOMEM;
		goto out_err_alloc_work;
	}

	edgx_ac_for_each_ifpt(ptid, ifd, &pifd) {
		pts = kzalloc(sizeof(*pts), GFP_KERNEL);
		if (!ts) {
			edgx_err("COM TS: allocation failed!\n");
			ret = -ENOMEM;
			goto out_err_alloc;
		}
		spin_lock_init(&pts->lock);
		pts->iobase = pifd.iobase;
		ts->pts[ptid] = pts;
		edgx_wr16(pts->iobase, EDGX_COM_TS_CTRL_RX, 0xffff);
		edgx_wr16(pts->iobase, EDGX_COM_TS_CTRL_TX, 0xffff);
	}
	edgx_br_set_int_mask_irqsave(br, EDGX_COM_TS_INT_MASK_TX);

	return 0;

out_err_alloc:
	for (i = 0; i < EDGX_BR_MAX_PORTS; i++)
		kfree(ts->pts[i]);
	destroy_workqueue(ts->wq_tx);
out_err_alloc_work:
	return ret;
}

void edgx_com_ts_shutdown(struct edgx_com_ts *ts)
{
	int i, j;
	struct edgx_com_pts *pts;
	unsigned long flags = 0;

	destroy_workqueue(ts->wq_tx);

	for (i = 0; i < EDGX_BR_MAX_PORTS; i++) {
		pts = ts->pts[i];
		if (pts) {
			spin_lock_irqsave(&pts->lock, flags);
			for (j = 0; j < EDGX_COM_TS_CTRL_CNT; j++)
				if (pts->tx_queue[j])
					dev_kfree_skb_any(pts->tx_queue[j]);
			spin_unlock_irqrestore(&pts->lock, flags);
			kfree(pts);
		}
	}
}
