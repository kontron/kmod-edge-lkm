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

#include <linux/netdevice.h>
#include <linux/rtnetlink.h>
#include <linux/dma-mapping.h>
#include <linux/dma-direction.h>
#include <linux/kfifo.h>
#include <linux/interrupt.h>

#include "edge_ac.h"
#include "edge_com_intern.h"
#include "edge_util.h"
#include "edge_com_ts.h"

/* #define EDGX_COM_DMA_DBG */

#if defined(EDGX_COM_DMA_DBG)
#define com_dma_dbg	pr_info
#else
#define com_dma_dbg(args...)
#endif

#define EDGX_DMA_QUEUE_CNT		(1U)
#define DMA_ITF_OFFS			(0xA000)
#define EDGX_DMA_CFG			(0x0)
#define EDGX_DMA_DESC_P(idx)		(0x020 + ((idx) * 2))
#define EDGX_DMA_RING_CFG0(idx)		(0x100 + ((idx) * 8))
#define EDGX_DMA_RING_CFG1(idx)		(EDGX_DMA_RING_CFG0(idx) + 2)
#define EDGX_DMA_TX_IDX(idx)		(idx)
#define EDGX_DMA_RX_IDX(idx)		((idx) + 8)

#if defined(EDGX_COM_DMA_DBG)
#define EDGX_DMA_DESC_CNT		(4U)
#define EDGX_DMA_DESC_CNT_MASK		(1U)	/* 4 descriptors */
#else
#define EDGX_DMA_DESC_CNT		(256U)
#define EDGX_DMA_DESC_CNT_MASK		(7U)	/* 256 descriptors */
#endif
#define EDGX_DMA_TX_INT_MASK		(BIT(4))
#define EDGX_DMA_RX_INT_MASK		(BIT(5))
#define EDGX_DMA_MAX_LEN		(0x7ff)
#define EDGX_DMA_WEIGHT			(EDGX_DMA_DESC_CNT)

struct edgx_com_dma_desc {
	u16 cfg;
	u16 dummy_1;
	u16 mgmt;
	u16 dummy_2;
	u16 p_data_l;
	u16 p_data_h;
	u16 dummy_3;
	u16 dummy_4;
	u16 ts_ns_l;
	u16 ts_ns_h;
	u16 ts_s_l;
	u16 dummy_5;
	u16 dummy_6;
	u16 dummy_7;
	u16 dummy_8;
	u16 dummy_9;
} __packed;

#define edgx_dma_get_len(pdesc) (le16_to_cpu((pdesc)->cfg) & 0x7ff)
#define edgx_dma_get_lerr(pdesc) (le16_to_cpu((pdesc)->cfg) & BIT(13))
#define edgx_dma_get_err(pdesc) (le16_to_cpu((pdesc)->cfg) & BIT(14))
#define edgx_dma_get_init(pdesc) (le16_to_cpu((pdesc)->cfg) & BIT(15))
#define edgx_dma_get_port(pdesc) (le16_to_cpu((pdesc)->mgmt) & 0xfff)
#define edgx_dma_get_flags(pdesc) (le16_to_cpu((pdesc)->mgmt) >> 8)

static inline u32 edgx_dma_get_pdata(struct edgx_com_dma_desc *pdesc)
{
	return (u32)le16_to_cpu(pdesc->p_data_l) |
	       ((u32)le16_to_cpu(pdesc->p_data_h) << 16);
}

static inline void edgx_dma_clr_lerr(struct edgx_com_dma_desc *pdesc)
{
	pdesc->cfg = __cpu_to_le16(le16_to_cpu(pdesc->cfg) & (~BIT(13)));
}

static inline void edgx_dma_clr_err(struct edgx_com_dma_desc *pdesc)
{
	pdesc->cfg = __cpu_to_le16(le16_to_cpu(pdesc->cfg) & (~BIT(14)));
}

static inline void edgx_dma_set_port(struct edgx_com_dma_desc *pdesc,
				     u16 port_mask, u16 flags)
{
	pdesc->mgmt = __cpu_to_le16((flags << 8) | (port_mask & 0xfff));
}

static inline void edgx_dma_set_init(struct edgx_com_dma_desc *pdesc)
{
	pdesc->cfg = __cpu_to_le16(le16_to_cpu(pdesc->cfg) | BIT(15));
}

static inline void edgx_dma_set_len(struct edgx_com_dma_desc *pdesc,
				    unsigned int len)
{
	pdesc->cfg = __cpu_to_le16((le16_to_cpu(pdesc->cfg) & (~0x7ff)) |
		     (len & 0x7ff));
}

static inline void edgx_dma_set_pdata(struct edgx_com_dma_desc *pdesc,
				      u32 pdata)
{
	pdesc->p_data_l = __cpu_to_le16(pdata & 0xffff);
	pdesc->p_data_h = __cpu_to_le16(pdata >> 16);
}

struct edgx_dma_queue {
	spinlock_t		  lock;		/* Sync. access to dma queue */
	int			  idx;
	enum dma_data_direction   dir;
	edgx_io_t		 *cfg_base;
	struct edgx_com_dma_desc *desc;
	dma_addr_t		  desc_bus_addr;
	int			  enq_pos;
	int			  deq_pos;
	int			  cnt;
	struct sk_buff		**skb;
	DECLARE_KFIFO(pending_devs, struct net_device *, EDGX_BR_MAX_PORTS);
};

struct edgx_com_dma {
	struct edgx_com		 com;
	edgx_io_t		*dma_base;
	struct edgx_dma_queue	 tx_queue[EDGX_DMA_QUEUE_CNT];
	struct edgx_dma_queue	 rx_queue[EDGX_DMA_QUEUE_CNT];
	struct edgx_com_ts	 ts;
	struct net_device	 napi_dev;
	struct napi_struct	 napi;
	int			 irq;
};

#define edgx_com_to_dma(pcom)   container_of(pcom, struct edgx_com_dma, com)
#define edgx_napi_to_dma(pnapi) container_of(pnapi, struct edgx_com_dma, napi)
#define edgx_work_to_dma(pwork) container_of(pwork, struct edgx_com_dma, \
					     work_tx)

static void edgx_dma_dump(struct edgx_dma_queue *q, int i, const char *str)
{
#if defined(EDGX_COM_DMA_DBG)
#if (0)
	for (i = 0; i < EDGX_DMA_DESC_CNT; i++)
#endif
		if (q->skb[i]) {
			u8 *d = (u8 *)q->skb[i]->data;

			com_dma_dbg("%s Q%.2d D%.2d: cfg=0x%.4x mgt=0x%.4x dL=0x%.4x dH=0x%.4x %pM %pM %*phN %*phN\n",
				    str,
				    q->idx, i, q->desc[i].cfg, q->desc[i].mgmt,
				    q->desc[i].p_data_h, q->desc[i].p_data_l,
				    &d[0], &d[6], 4, &d[12], 4, &d[16]);
		} else {
			com_dma_dbg("D%.2d: cfg=0x%.4x mgt=0x%.4x dL=0x%.4x dH=0x%.4x\n",
				    i, q->desc[i].cfg, q->desc[i].mgmt,
				    q->desc[i].p_data_h, q->desc[i].p_data_l);
		}
#endif
}

static int edgx_dma_queue_init(struct edgx_dma_queue *q, struct device *dev,
			       edgx_io_t *dma_base, int idx,
			       enum dma_data_direction dir)
{
	q->cfg_base = dma_base;
	q->idx = idx;
	q->dir = dir;
	spin_lock_init(&q->lock);

	/* Allocate descriptors */
	q->desc = dma_alloc_coherent(dev,
				     EDGX_DMA_DESC_CNT *
				     sizeof(struct edgx_com_dma_desc),
				     &q->desc_bus_addr, GFP_KERNEL);

	if (!q->desc)
		return -ENOMEM;

	q->skb = kcalloc(EDGX_DMA_DESC_CNT, sizeof(*q->skb), GFP_KERNEL);

	if (!q->skb)
		goto out_skb;

	INIT_KFIFO(q->pending_devs);
	edgx_wr16(dma_base, EDGX_DMA_DESC_P(idx), EDGX_DMA_DESC_CNT - 1);
	edgx_wr16(dma_base, EDGX_DMA_RING_CFG0(idx),
		  EDGX_DMA_DESC_CNT_MASK | (q->desc_bus_addr & 0xffe0));
	edgx_wr16(dma_base, EDGX_DMA_RING_CFG1(idx), q->desc_bus_addr >> 16);

	com_dma_dbg("DMA: queue_init: desc=%p, bus_addr=0x%x\n",
		    q->desc, q->desc_bus_addr);
	return 0;

out_skb:
	dma_free_coherent(dev, EDGX_DMA_DESC_CNT * sizeof(*q->desc),
			  q->desc, q->desc_bus_addr);
	return -ENOMEM;
}

static void edgx_dma_queue_release(struct edgx_dma_queue *q, struct device *dev)
{
	int i;

	// TODO: make sure that all pending requests are processed first

	q->cnt = 0;
	for (i = 0; i < EDGX_DMA_DESC_CNT; i++) {
		if (!q->skb[i])
			continue;

		if (q->dir == DMA_FROM_DEVICE)
			dma_unmap_single(dev, edgx_dma_get_pdata(&q->desc[i]),
					 EDGX_DMA_MAX_LEN, q->dir);
		else
			dma_unmap_single(dev, edgx_dma_get_pdata(&q->desc[i]),
					 q->skb[i]->len, q->dir);

		dev_kfree_skb_any(q->skb[i]);
	}

	kfree(q->skb);
	dma_free_coherent(dev, EDGX_DMA_DESC_CNT * sizeof(*q->desc),
			  q->desc, q->desc_bus_addr);
}

static int edgx_dma_enq(struct edgx_dma_queue *q, struct device *dev,
			struct sk_buff *skb, ptcom_t port_mask,
			ptflags_t flags, struct edgx_com_ts *ts)
{
	int ret = -ENOSPC;
	struct edgx_com_dma_desc *desc;
	dma_addr_t phys_addr;
	int old_pos;
	unsigned int len = (q->dir == DMA_TO_DEVICE) ?
			   skb->len : EDGX_DMA_MAX_LEN;

	 /* Due to an IP limitation, keep one empty descriptor in the queue. */
	if (q->cnt < (EDGX_DMA_DESC_CNT - 1)) {
		desc = &q->desc[q->enq_pos];

		if (edgx_dma_get_init(desc)) {
			edgx_err("COM: DMA queuing error!\n");
			return -ENOMEM;
		}

		if (ts)
			skb = edgx_com_ts_xmit(ts, skb, port_mask);

		phys_addr = dma_map_single(dev, skb->data, len, q->dir);

		if (dma_mapping_error(dev, phys_addr)) {
			edgx_err("COM: DMA mapping error!\n");
			return -ENOMEM;
		}

		com_dma_dbg("DMA-enq: Q%d, pos=%d, cnt=%d, len=%d, port_mask=0x%x, flags=0x%x\n",
			    q->idx, q->enq_pos, q->cnt, len, port_mask, flags);

		q->skb[q->enq_pos] = skb;
		old_pos = q->enq_pos;
		q->enq_pos = (q->enq_pos + 1) % EDGX_DMA_DESC_CNT;
		q->cnt++;

		edgx_dma_set_len(desc, len);
		edgx_dma_set_port(desc, port_mask, flags);
		edgx_dma_set_pdata(desc, phys_addr);
		edgx_dma_set_init(desc);
		edgx_dma_dump(q, old_pos, "EN:");
		dma_wmb();
		edgx_wr16(q->cfg_base, EDGX_DMA_DESC_P(q->idx), (u16)old_pos);

		/* Return ok only if there is still place in the DMA ring. */
		if (q->cnt < (EDGX_DMA_DESC_CNT - 1))
			ret = 0;
	} else {
		com_dma_dbg("DMA-enq: Q%d, queue full. enq_pos=%d, cnt=%d, len=%d\n",
			    q->idx, q->enq_pos, q->cnt, len);
	}

	return ret;
}

static struct sk_buff *edgx_dma_deq(struct edgx_dma_queue *q,
				    struct device *dev,
				    u64 *error_cnt, ptcom_t *port_mask,
				    ptflags_t *flags)
{
	struct edgx_com_dma_desc *desc;
	struct sk_buff *skb;
	unsigned long irq_flags;

	spin_lock_irqsave(&q->lock, irq_flags);

	desc = &q->desc[q->deq_pos];
	skb = q->skb[q->deq_pos];

	if (!q->cnt || edgx_dma_get_init(desc)) {
		spin_unlock_irqrestore(&q->lock, irq_flags);
		return NULL;
	}

	if (error_cnt) {
		if (edgx_dma_get_lerr(desc)) {
			edgx_err("COM: DMA length error!\n");
			(*error_cnt)++;
			edgx_dma_clr_lerr(desc);
		}

		if (edgx_dma_get_err(desc)) {
			edgx_err("COM: DMA bus error!\n");
			(*error_cnt)++;
			edgx_dma_clr_err(desc);
		}
	}

	if (q->dir == DMA_FROM_DEVICE) {
		skb_put(skb, edgx_dma_get_len(desc));
		*port_mask = edgx_dma_get_port(desc);
		*flags = edgx_dma_get_flags(desc);
		dma_unmap_single(dev, edgx_dma_get_pdata(desc),
				 EDGX_DMA_MAX_LEN, q->dir);
	} else {
		dma_unmap_single(dev, edgx_dma_get_pdata(desc),
				 skb->len, q->dir);
	}

	com_dma_dbg("DMA-deq: Q%d, pos=%d, cnt=%d, pmask=0x%x, flags=0x%x, len=%d, skb_len=%d\n",
		    q->idx, q->deq_pos, q->cnt, edgx_dma_get_port(desc),
		    edgx_dma_get_flags(desc), edgx_dma_get_len(desc), skb->len);

	edgx_dma_dump(q, q->deq_pos, "DE:");

	q->skb[q->deq_pos] = NULL;
	q->deq_pos = (q->deq_pos + 1) % EDGX_DMA_DESC_CNT;
	q->cnt--;

	spin_unlock_irqrestore(&q->lock, irq_flags);
	return skb;
}

static inline int edgx_dma_enq_tx(struct edgx_dma_queue *q,
				  struct device *dev, struct sk_buff *skb,
				  ptcom_t port_mask, ptflags_t flags,
				  struct edgx_com_ts *ts)
{
	int ret;
	unsigned int skb_len_dif;
	unsigned long irq_flags;

	if (skb->len < ETH_ZLEN) {
		skb_len_dif = ETH_ZLEN - skb->len;

		ret = skb_pad(skb, skb_len_dif);
		if (ret)
			return ret;

		memset(skb_put(skb, skb_len_dif), 0, skb_len_dif);
	}

	spin_lock_irqsave(&q->lock, irq_flags);
	ret = edgx_dma_enq(q, dev, skb, port_mask, flags, ts);
	if (ret) {
		com_dma_dbg("DMA enq Tx: stop netdev %s\n", skb->dev->name);
		kfifo_put(&q->pending_devs, skb->dev);
		netif_stop_queue(skb->dev);
	}
	spin_unlock_irqrestore(&q->lock, irq_flags);

	return ret;
}

static int edgx_dma_enq_rx(struct edgx_dma_queue *q, struct device *dev)
{
	int ret = -ENOMEM;
	unsigned long irq_flags;
	struct sk_buff *skb = dev_alloc_skb(EDGX_DMA_MAX_LEN);

	if (!skb)
		return -ENOMEM;

	spin_lock_irqsave(&q->lock, irq_flags);
	ret = edgx_dma_enq(q, dev, skb, 0, 0, NULL);
	spin_unlock_irqrestore(&q->lock, irq_flags);

	if (ret == -ENOMEM)
		dev_kfree_skb_any(skb);

	return ret;
}

static inline struct sk_buff *edgx_dma_deq_tx(struct edgx_dma_queue *q,
					      struct device *dev,
					      u64 *error_cnt)
{
	struct net_device *pend_dev;
	struct sk_buff *skb;

	skb = edgx_dma_deq(q, dev, error_cnt, NULL, NULL);

	if (skb && kfifo_get(&q->pending_devs, &pend_dev)) {
		com_dma_dbg("DMA deq Tx: wake netdev %s\n", pend_dev->name);
		netif_wake_queue(pend_dev);
	}
	return skb;
}

static inline struct sk_buff *edgx_dma_deq_rx(struct edgx_dma_queue *q,
					      struct device *dev,
					      u64 *error_cnt,
					      ptcom_t *port_mask,
					      ptflags_t *flags)
{
	return edgx_dma_deq(q, dev, error_cnt, port_mask, flags);
}

static void edgx_dma_process_tx(struct edgx_com_dma *dma)
{
	struct sk_buff *skb = NULL;
	u64 err_cnt = 0;
	struct device *dev = edgx_br_get_dev(dma->com.parent);

	do {
		skb = edgx_dma_deq_tx(&dma->tx_queue[0], dev, &err_cnt);

		if (skb) {
			/* TODO: Add DMA error counters? */
			dev_kfree_skb_any(skb);
		}
	} while (skb);
}

static int edgx_dma_process_rx(struct edgx_com_dma *dma, int budget)
{
	int ret, i;
	ptcom_t ptcom;
	ptflags_t flags;
	u64 err_cnt = 0;
	struct sk_buff *skb;
	struct device *dev = edgx_br_get_dev(dma->com.parent);

	for (i = 0; i < budget; i++) {
		skb = edgx_dma_deq_rx(&dma->rx_queue[0], dev, &err_cnt,
				      &ptcom, &flags);

		if (!skb)
			break;

		/* TODO: Add DMA error counters? */
		skb_reset_mac_header(skb);
		edgx_com_ts_rx(&dma->ts, skb, ptcom);
		edgx_com_rx_dispatch(&dma->com, skb, ptcom, flags);

		ret = edgx_dma_enq_rx(&dma->rx_queue[0], dev);

		if (ret && (ret != -ENOSPC))
			edgx_err("COM DMA: RX enqueue error!\n");
	}
	return i;
}

static int edgx_com_dma_xmit(struct edgx_com *com, struct sk_buff *skb,
			     ptcom_t ptcom, ptflags_t flags)
{
	int  ret;
	struct device *dev = edgx_br_get_dev(com->parent);
	struct edgx_com_dma *dma = edgx_com_to_dma(com);

	ret = edgx_dma_enq_tx(&dma->tx_queue[0], dev, skb, ptcom, flags,
			      &dma->ts);

	if (ret == -ENOSPC)
		com_dma_dbg("COM DMA: xmit: TX queue full. (%s)\n",
			    skb->dev->name);

	if (ret == -ENOMEM) {
		com_dma_dbg("COM DMA: xmit: TX queue busy. (%s)\n",
			    skb->dev->name);
		return NETDEV_TX_BUSY;
	}
	return NETDEV_TX_OK;
}

static int edgx_dma_poll(struct napi_struct *napi, int budget)
{
	int used;
	struct edgx_com_dma *dma = edgx_napi_to_dma(napi);

	edgx_br_clr_int_stat(dma->com.parent,
			     EDGX_DMA_RX_INT_MASK | EDGX_DMA_TX_INT_MASK);

	com_dma_dbg("COM DMA: Poll Start.\n");
	edgx_dma_process_tx(dma);
	used = edgx_dma_process_rx(dma, budget);

	com_dma_dbg("COM DMA: Poll budget=%i, used=%i.\n", budget, used);

	if (used < budget) {
		napi_complete(&dma->napi);
		edgx_br_set_int_mask(dma->com.parent, EDGX_DMA_RX_INT_MASK |
				     EDGX_DMA_TX_INT_MASK);
		com_dma_dbg("COM DMA: Poll complete.\n");
	}
	return used;
}

static irqreturn_t edgx_com_dma_isr(int irq, void *device)
{
	u16 intmask, intstat;
	struct edgx_com_dma *dma = (struct edgx_com_dma *)device;

	intmask = edgx_br_get_int_mask(dma->com.parent);
	if (!(intmask & (EDGX_DMA_TX_INT_MASK | EDGX_DMA_RX_INT_MASK)))
		return IRQ_NONE;

	intstat = edgx_br_get_int_stat(dma->com.parent);
	if (!(intstat & (EDGX_DMA_TX_INT_MASK | EDGX_DMA_RX_INT_MASK)))
		return IRQ_NONE;

	com_dma_dbg("COM DMA: ISR m0x%x s0x%x.\n", intmask, intstat);

	edgx_br_clr_int_mask(dma->com.parent, EDGX_DMA_RX_INT_MASK |
			     EDGX_DMA_TX_INT_MASK);
	napi_schedule(&dma->napi);

	return IRQ_HANDLED;
}

void edgx_com_dma_tx_timeout(struct edgx_com *com, struct net_device *netdev)
{
	struct edgx_com_dma *dma = edgx_com_to_dma(com);

	edgx_warn("COM: DMA TX timeout! (%s)\n", netdev->name);
	edgx_dma_process_tx(dma);
	netif_wake_queue(netdev);
}

static int edgx_com_dma_ts_cfg_set(struct edgx_com *com, ptcom_t ptcom,
				   struct ifreq *ifr)
{
	struct edgx_com_dma *dma = edgx_com_to_dma(com);

	return edgx_com_ts_cfg_set(&dma->ts, ptcom, ifr);
}

static int edgx_com_dma_ts_cfg_get(struct edgx_com *com, ptcom_t ptcom,
				   struct ifreq *ifr)
{
	struct edgx_com_dma *dma = edgx_com_to_dma(com);

	return edgx_com_ts_cfg_get(&dma->ts, ptcom, ifr);
}

static void edgx_com_dma_shutdown(struct edgx_com *com)
{
	int i;
	struct device *dev = edgx_br_get_dev(com->parent);
	struct edgx_com_dma *dma = edgx_com_to_dma(com);

	edgx_wr16(dma->dma_base, EDGX_DMA_CFG, 0);
	free_irq(dma->irq, dma);
	napi_disable(&dma->napi);
	netif_napi_del(&dma->napi);

	for (i = 0; i < EDGX_DMA_QUEUE_CNT; i++) {
		edgx_dma_queue_release(&dma->tx_queue[i], dev);
		edgx_dma_queue_release(&dma->rx_queue[i], dev);
	}
	edgx_com_ts_shutdown(&dma->ts);
	kfree(dma);
}

static struct edgx_com_ops edgx_com_dma_ops = {
	.shutdown	= edgx_com_dma_shutdown,
	.xmit		= edgx_com_dma_xmit,
	.hwts_set	= edgx_com_dma_ts_cfg_set,
	.hwts_get	= edgx_com_dma_ts_cfg_get,
	.tx_timeout	= edgx_com_dma_tx_timeout,
};

static int edgx_com_dma_init(struct edgx_com_dma *dma, struct edgx_br *br,
			     edgx_io_t *dma_base, const char *drv_name,
			     int irq, ptid_t dma_pt)
{
	int i, ret;
	struct device *dev = edgx_br_get_dev(br);

	edgx_com_init(&dma->com, br, &edgx_com_dma_ops, dma_pt);

	dma->dma_base = dma_base;
	dma->irq = irq;

	ret = request_irq(irq, &edgx_com_dma_isr, IRQF_SHARED,
			  drv_name, dma);
	if (ret) {
		edgx_err("COM DMA: request_irq failed! ret=%d, irq=%d\n",
			 ret, irq);
		goto out_err_init_irq;
	}

	ret = init_dummy_netdev(&dma->napi_dev);
	if (ret) {
		edgx_err("COM DMA: cannot init dummy netdev! ret=%d\n", ret);
		goto out_err_init_dummy;
	}

	netif_napi_add(&dma->napi_dev, &dma->napi,
		       edgx_dma_poll, EDGX_DMA_WEIGHT);

	for (i = 0; !ret && (i < EDGX_DMA_QUEUE_CNT); i++)
		ret = edgx_dma_queue_init(&dma->tx_queue[i], dev,
					  dma->dma_base, EDGX_DMA_TX_IDX(i),
					  DMA_TO_DEVICE);

	for (i = 0; !ret && (i < EDGX_DMA_QUEUE_CNT); i++)
		ret = edgx_dma_queue_init(&dma->rx_queue[i], dev,
					  dma->dma_base, EDGX_DMA_RX_IDX(i),
					  DMA_FROM_DEVICE);

	for (i = 0; !ret && (i < EDGX_DMA_QUEUE_CNT); i++)
		while (!ret)
			ret = edgx_dma_enq_rx(&dma->rx_queue[i], dev);

	if (ret != -ENOSPC) {
		edgx_err("COM DMA: Queue initialization failed! ret=%d\n",
			 ret);
		goto out_err_init_queues;
	}

	ret = edgx_com_ts_init(&dma->ts, drv_name, irq, br);
	if (ret) {
		edgx_err("COM DMA: Cannot initialize timestamping! ret=%d\n",
			 ret);
		goto out_err_init_queues;
	}

	napi_enable(&dma->napi);
	edgx_br_clr_int_stat(br, EDGX_DMA_TX_INT_MASK | EDGX_DMA_RX_INT_MASK);
	edgx_br_set_int_mask(br, EDGX_DMA_TX_INT_MASK | EDGX_DMA_RX_INT_MASK);
	edgx_wr16(dma->dma_base, EDGX_DMA_CFG, 0x03);
	return 0;

out_err_init_queues:
	for (i = 0; i < EDGX_DMA_QUEUE_CNT; i++) {
		edgx_dma_queue_release(&dma->tx_queue[i], dev);
		edgx_dma_queue_release(&dma->rx_queue[i], dev);
	}
	netif_napi_del(&dma->napi);
out_err_init_dummy:
	free_irq(dma->irq, dma);
out_err_init_irq:
	return ret;
}

int edgx_com_dma_probe(struct edgx_br *br, const char *drv_name, int irq,
		       struct edgx_com **com)
{
	int ret;
	struct edgx_ifdesc ptifd;
	ptid_t dma_pt;
	struct edgx_com_dma *dma;
	struct device *dev = edgx_br_get_dev(br);
	const struct edgx_ifreq   ifreq = { .id = AC_PORT_ID, .v_maj = 1 };

	if (dma_set_mask(dev, 0xffffffe0)) {
		edgx_br_warn(br, "COM DMA not supported!\n");
		return -ENODEV;
	}

	dma_pt = ffs(edgx_br_get_generic(br, BR_GX_DMA_PORTS)) - 1;

	if (!edgx_ac_get_ptif(&ifreq, dma_pt, &ptifd))
		return -ENODEV;

	dma = kzalloc(sizeof(*dma), GFP_KERNEL);
	if (!dma)
		return -ENOMEM;

	ret = edgx_com_dma_init(dma, br, ptifd.iobase + DMA_ITF_OFFS,
				drv_name, irq, dma_pt);
	if (ret) {
		kfree(dma);
		return ret;
	}

	*com = &dma->com;
	edgx_br_info(br, "Setup COM DMA subsystem ... done\n");
	return 0;
}
