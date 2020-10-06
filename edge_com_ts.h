/* SPDX-License-Identifier: GPL-2.0 */
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

#ifndef _CORE_EDGE_COM_TS_H
#define _CORE_EDGE_COM_TS_H

#include "edge_com_intern.h"

#define EDGX_COM_TS_CTRL_CNT	(4U)

struct edgx_com_pts {
	struct hwtstamp_config	 hwts_cfg;
	edgx_io_t		*iobase;
	u8			 rx_ts_pos;
	struct sk_buff		*tx_queue[EDGX_COM_TS_CTRL_CNT];
	u8			 tx_q_pos;
	u8			 tx_ts_pos;
	spinlock_t		 lock;		/* Sync. access to tx_queue */
};

struct edgx_com_ts {
	struct edgx_br		*parent;
	struct edgx_com_pts	*pts[EDGX_BR_MAX_PORTS];
	int			 irq;
	struct workqueue_struct	*wq_tx;
	struct work_struct	 work_tx;
};

int edgx_com_ts_init(struct edgx_com_ts *ts, const char *drv_name, int irq,
		     struct edgx_br *br);
void edgx_com_ts_shutdown(struct edgx_com_ts *ts);
int edgx_com_ts_cfg_get(struct edgx_com_ts *ts, ptcom_t ptcom,
			struct ifreq *ifr);
int edgx_com_ts_cfg_set(struct edgx_com_ts *ts, ptcom_t ptcom,
			struct ifreq *ifr);
struct sk_buff *edgx_com_ts_xmit(struct edgx_com_ts *ts,
				 struct sk_buff *skb, ptcom_t ptcom);
void edgx_com_ts_rx(struct edgx_com_ts *ts,
		    struct sk_buff *skb, ptcom_t ptcom);

#endif /* _CORE_EDGE_COM_TS_H */
