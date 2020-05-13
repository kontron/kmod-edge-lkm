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

#ifndef _EDGE_SCHED_HW_H
#define _EDGE_SCHED_HW_H

#include <linux/interrupt.h>
#include "edge_defines.h"

/* #define EDGX_SCHED_DBG */

#if defined(EDGX_SCHED_DBG)
#define sched_dbg	pr_info
#else
#define sched_dbg(args...)
#endif

/* FSC HW ITF address definitions */

#define EDGX_SCHED_PT_RES_LEN		(0x1000)
#define EDGX_SCHED_PT_RES_OFFS		(0x10000)
#define EDGX_SCHED_OF_GEN_REGS		(0x4000)
#define EDGX_SCHED_OF_GATE_CNT		((EDGX_SCHED_OF_GEN_REGS) + 0x002)
#define EDGX_SCHED_GATE_CNT_MASK	(0x7f)
#define EDGX_SCHED_OF_ROW_CNT		((EDGX_SCHED_OF_GEN_REGS) + 0x004)
#define EDGX_SCHED_ROW_CNT_MASK		(0x0f)
#define EDGX_SCHED_OF_CLK_FRQ		((EDGX_SCHED_OF_GEN_REGS) + 0x006)
#define EDGX_SCHED_CLK_FRQ_MASK		(0xff)
#define EDGX_SCHED_TAB_0		(0x800)
#define EDGX_SCHED_TAB_1		(0x900)
#define EDGX_SCHED_TBL_OFS	       ((EDGX_SCHED_TAB_1) - (EDGX_SCHED_TAB_0))
#define EDGX_SCHED_TBL_GEN		((EDGX_SCHED_TAB_0) + 0x0)
#define EDGX_SCHED_CAN_USE_MASK		(0x0001)
#define EDGX_SCHED_IN_USE_MASK		(0x0002)
#define EDGX_SCHED_START_TIME_NSEC	((EDGX_SCHED_TAB_0) + 0x014)
#define EDGX_SCHED_START_TIME_SEC	((EDGX_SCHED_TAB_0) + 0x018)
#define EDGX_SCHED_CYC_TIME_NSEC	((EDGX_SCHED_TAB_0) + 0x024)
#define EDGX_SCHED_CYC_TIME_SUBNS	((EDGX_SCHED_TAB_0) + 0x020)
#define EDGX_SCHED_CYC_TS_SEC		((EDGX_SCHED_TAB_0) + 0x038)
#define EDGX_SCHED_CYC_TS_NS_H		((EDGX_SCHED_TAB_0) + 0x036)
#define EDGX_SCHED_CYC_TS_NS_L		((EDGX_SCHED_TAB_0) + 0x034)
#define EDGX_SCHED_CYCLE_CNT		((EDGX_SCHED_TAB_0) + 0x040)
#define EDGX_SCHED_LAST_CYC		((EDGX_SCHED_TAB_0) + 0x044)
#define EDGX_SCHED_START_TIME_SEC_MASK	(0x00ff)
#define EDGX_SCHED_STOP_LAST		(0x0100)
#define EDGX_SCHED_ROW_AC_CMD0		(0x1000)
#define EDGX_SCHED_ROW_AC_CMD1		(0x1002)
#define EDGX_SCHED_ROW_AC_CMD1_MASK	(0x3ff)
#define EDGX_SCHED_ROW_DATA_OUT0	(0x1010)
#define EDGX_SCHED_ROW_DATA_CYCLES	(0x1018)
#define EDGX_SCHED_CMD_SCHED_MASK	(0x000f)
#define EDGX_SCHED_CMD_TAB_MASK		(0x0001)
#define EDGX_SCHED_CMD_TAB_MASK_SHIFT	(8)
#define EDGX_SCHED_AC_WRITE		BIT(14)
#define EDGX_SCHED_AC_TRANSFER		BIT(15)
#define EDGX_SCHED_AC_ERR		BIT(13)
#define EDGX_SCHED_EME_DIS_CTRL		(0x020)
#define EDGX_SCHED_EME_DIS_ON		(0x0001)
#define EDGX_SCHED_EME_DIS_OFF		(0x0000)
#define EDGX_SCHED_EME_STAT_DEF		(0xffff)
#define EDGX_SCHED_EME_DIS_CTRL_R_MASK	(0x0002)
#define EDGX_SCHED_EME_DIS_STAT0	(0x030)
#define EDGX_SCHED_EME_DIS_STAT1	(0x032)
#define EDGX_SCHED_EME_DIS_STAT2	(0x034)
#define EDGX_SCHED_EME_DIS_STAT3	(0x036)
#define EDGX_SCHED_SCH_GEN		(0x000)
#define EDGX_SCHED_DC_SPD		(0x002)
#define EDGX_SCHED_HW_DC_SPD_125MHZ	(0x4 << 6)
#define EDGX_SCHED_HW_DC_SPD_100MHZ	(0x5 << 6)
#define EDGX_SCHED_MAX_DELAY		(64U)
#define EDGX_SCHED_MIN_ADVANCE		(32U)
#define EDGX_SCHED_TIME_ADJ_DC		(1540U)
#define EDGX_SCHED_INTERVAL_MAX		(0xffff)
#define EDGX_SCHED_INTERVAL_MIN		(8U)
#define EDGX_SCHED_HW_DC_1000FULL	(1540 << 4)
#define EDGX_SCHED_HW_DC_100FULL	((1540 << 4) | 0x001)
#define EDGX_SCHED_HW_DC_10FULL		((1540 << 4) | 0x002)
#define EDGX_SCHED_UPDATE_TS_MASK	(0x8000)
#define EDGX_SCHED_HW_MAX_CT_NS		(999999999U)
#define EDGX_SCHED_INT_MASK		(0x1100)
#define EDGX_SCHED_INT_STAT		(0x1102)
#define EDGX_SCHED_INT_MSKVAL		(0x01)
#define EDGX_SCHED_DEF_MAX_SDU		(1504U)

/** Number of HW scheduling tables */
#define EDGX_SCHED_HW_TAB_CNT		(2U)

/* Maximum possible number of schedule table rows */
#define EDGX_SCHED_HW_MAX_ROWS		(1024U)

u16 edgx_sched_get_hw_gate_cnt(edgx_io_t *base);
u16 edgx_sched_get_hw_row_cnt(edgx_io_t *base);
u16 edgx_sched_get_hw_clk_frq(edgx_io_t *base);
bool edgx_sched_hw_is_pending(edgx_io_t *base_pt);
void edgx_sched_hw_cancel_pending(edgx_io_t *base_pt);
int edgx_sched_hw_get_free_tab(edgx_io_t *base_pt);
int edgx_sched_hw_get_used_tab(edgx_io_t *base_pt);
void edgx_sched_hw_set_pending(edgx_io_t *base_pt,
			       int tab_idx,
			       const struct timespec64 *base_time,
			       u32 cycle_nsec,
			       u32 cycle_subnsec,
			       u16 down_cnt_speed);
void edgx_sched_hw_get_cc_time(edgx_io_t *base_pt, int tab_idx,
			       struct timespec64 *time);
int edgx_sched_hw_write_entry(edgx_io_t *base, int sched_idx, int hw_tab_idx,
			      int row_idx, u16 gate_states,
			      u16 time_interval_clk);
int edgx_sched_hw_read_entry(edgx_io_t *base, int sched_idx, int hw_tab_idx,
			     int row_idx, u16 *gate_states,
			     u16 *interval_clk);
void edgx_sched_hw_disable(edgx_io_t *base_pt, u8 gate_states);
void edgx_sched_hw_enable(edgx_io_t *base_pt, u8 gate_states);
int edgx_sched_hw_init(edgx_io_t *base_pt, u16 ns_per_clk, u8 gate_states);
void edgx_sched_hw_dump(edgx_io_t *base, edgx_io_t *base_pt,
			int sched_idx, int tab_idx);
u16 edgx_sched_hw_get_cur_cycle(edgx_io_t *base_pt, int tab_idx,
				struct timespec64 *cur_c_time);
void edgx_sched_hw_set_last_cycle(edgx_io_t *base_pt, int tab_idx,
				  u16 last_cycle);

#endif /* _EDGE_SCHED_HW_H */
