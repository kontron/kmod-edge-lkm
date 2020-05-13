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

#ifndef _EDGE_BR_VLAN_H
#define _EDGE_BR_VLAN_H

#include <net/switchdev.h>
#include "edge_bridge.h"
#include "edge_port.h"

struct edgx_br_vlan;

int  edgx_br_init_vlan(struct edgx_br *br, edgx_io_t *iobase,
		       struct edgx_br_vlan **brvlan);
void edgx_br_shutdown_vlan(struct edgx_br_vlan *brvlan);

int edgx_br_vlan_add_pt(struct edgx_br_vlan *brvlan,
			struct switchdev_obj_port_vlan *v, struct edgx_pt *pt);
int edgx_br_vlan_del_pt(struct edgx_br_vlan *brvlan,
			struct switchdev_obj_port_vlan *v, struct edgx_pt *pt);
int edgx_br_vlan_purge_pt(struct edgx_br_vlan *brvlan, struct edgx_pt *pt);

int edgx_br_vlan_dump_pt(struct edgx_br_vlan *brvlan,
			 struct switchdev_obj_port_vlan *vlan,
			 switchdev_obj_dump_cb_t *cb, struct edgx_pt *pt);

int edgx_br_vlan_flush_mstpt(struct edgx_br_vlan *brvlan, mstid_t mstid,
			     ptid_t ptid);
int edgx_br_vlan_get_mstpt_state(struct edgx_br_vlan *brvlan, mstid_t mstid,
				 ptid_t ptid, u8 *ptstate);
int edgx_br_vlan_set_mstpt_state(struct edgx_br_vlan *brvlan, mstid_t mstid,
				 struct edgx_pt *pt, u8 ptstate);

#endif /* _EDGE_BR_VLAN_H */
