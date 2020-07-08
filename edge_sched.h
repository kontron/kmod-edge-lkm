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

#ifndef _EDGE_SCHED_H
#define _EDGE_SCHED_H

#include "edge_port.h"

#define EDGX_SCHED_MAX_QUEUES	(8U)

/** Per queue transmission rate defined as
 * CycleTime / (Sum of all control list intervals for the given queue)
 */
struct edgx_sched_tr_rate {
	u64 num;
	u64 denom;
};

struct edgx_sched;

struct edgx_sched_com;

int edgx_sched_com_probe(struct edgx_br *br, int irq, const char *drv_name,
			 struct edgx_sched_com **psched);

void edgx_sched_com_shutdown(struct edgx_sched_com *sched_com);

int edgx_probe_sched(struct edgx_pt *pt, struct edgx_sched_com *sched_com,
		     struct edgx_sched **psched);

void edgx_shutdown_sched(struct edgx_sched *sched);

int edgx_sched_get_trans_rate(struct edgx_sched *sched,
			      unsigned int queue_idx,
			      struct edgx_sched_tr_rate *tr_rate);

int edgx_sched_get_trans_rate_lock(struct edgx_sched *sched,
			      unsigned int queue_idx,
			      struct edgx_sched_tr_rate *tr_rate);

void edgx_sched_clr_int_mask(struct edgx_sched_com *sc);
void edgx_sched_set_int_mask(struct edgx_sched_com *sc, u16 mask);
u16 edgx_sched_get_int_mask(struct edgx_sched_com *sc);
u16 edgx_sched_get_int_stat(struct edgx_sched_com *sc);
u16 edgx_sched_isr(struct edgx_sched_com *sc, u16 intmask, u16 intstat);

#endif /* _EDGE_SCHED_H */
