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

#ifndef _EDGE_COM_INTERN_H
#define _EDGE_COM_INTERN_H

#include "edge_com.h"

/* Communication flags */
#define COM_TRAILER_LEN   (2)
#define COM_FLAGS_IDX     (0)
#define COM_PORT_IDX      (1)

/* TODO shouldn't we use the already defined ptvec_t??? */
typedef u16 ptcom_t;

struct edgx_com_ops {
	void (*shutdown)(struct edgx_com *com);
	int  (*xmit)(struct edgx_com *com, struct sk_buff *skb, ptcom_t ptcom,
		     ptflags_t flags);
	int  (*hwts_set)(struct edgx_com *com, ptcom_t ptcom,
			 struct ifreq *ifr);
	int  (*hwts_get)(struct edgx_com *com, ptcom_t ptcom,
			 struct ifreq *ifr);
	void (*tx_timeout)(struct edgx_com *com, struct net_device *netdev);
};

struct edgx_com {
	struct edgx_br		*parent;
	pid_t			 mgmt_ptid;
	struct edgx_com_ops	*ops;
};

void edgx_com_rx_dispatch(struct edgx_com *com, struct sk_buff *skb,
			  ptcom_t ptcom, ptflags_t flags);

int edgx_com_dma_probe(struct edgx_br *br, const char *drv_name,
		       int irq, struct edgx_com **com);
int edgx_com_xmii_probe(struct edgx_br *br, int irq, const char *ifname,
			const char *drv_name, struct edgx_com **com);
void edgx_com_init(struct edgx_com *com, struct edgx_br *br,
		   struct edgx_com_ops *ops, pid_t ptid);
#endif
