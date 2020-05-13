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

#include "edge_util.h"
#include "edge_sched_hw.h"

/** Lock of the common shared registers across all schedulers */
static struct mutex com_itf_lock;

/** Get the number of gates per schedule table entry from FSC HW ITF. */
u16 edgx_sched_get_hw_gate_cnt(edgx_io_t *base)
{
	u16	val;

	val = edgx_rd16(base, EDGX_SCHED_OF_GATE_CNT);
	val &= EDGX_SCHED_GATE_CNT_MASK;

	return val;
}

/** Get the number of rows per table from FSC HW ITF. */
u16 edgx_sched_get_hw_row_cnt(edgx_io_t *base)
{
	u16	val;

	val = edgx_rd16(base, EDGX_SCHED_OF_ROW_CNT);
	val = val & EDGX_SCHED_ROW_CNT_MASK;

	return (1 << val);
}

int edgx_sched_hw_init(edgx_io_t *base_pt, u16 ns_per_clk, u8 gate_states)
{
	u16 dc_speed;

	mutex_init(&com_itf_lock);
	edgx_wr16(base_pt, EDGX_SCHED_EME_DIS_STAT0, (u16)gate_states);
	edgx_wr16(base_pt, EDGX_SCHED_EME_DIS_STAT1, 0);
	edgx_wr16(base_pt, EDGX_SCHED_EME_DIS_STAT2, 0);
	edgx_wr16(base_pt, EDGX_SCHED_EME_DIS_STAT3, 0);
	edgx_wr16(base_pt, EDGX_SCHED_EME_DIS_CTRL, EDGX_SCHED_EME_DIS_ON);

	if (ns_per_clk == 8U)
		dc_speed = EDGX_SCHED_HW_DC_SPD_125MHZ;
	else if (ns_per_clk == 10U)
		dc_speed = EDGX_SCHED_HW_DC_SPD_100MHZ;
	else
		return -EINVAL;

	edgx_wr16(base_pt, EDGX_SCHED_DC_SPD, dc_speed);

	return 0;
}

/** Get the HW clock frequency */
u16 edgx_sched_get_hw_clk_frq(edgx_io_t *base)
{
	u16 val;

	val = edgx_rd16(base, EDGX_SCHED_OF_CLK_FRQ);
	val &= EDGX_SCHED_CLK_FRQ_MASK;

	return val;
}

/** Check if a new configuration is pending */
bool edgx_sched_hw_is_pending(edgx_io_t *base_pt)
{
	u16 pending_0 = edgx_rd16(base_pt, EDGX_SCHED_TBL_GEN) &
			EDGX_SCHED_CAN_USE_MASK;
	u16 pending_1 = edgx_rd16(base_pt,
				  EDGX_SCHED_TBL_GEN + EDGX_SCHED_TBL_OFS) &
			EDGX_SCHED_CAN_USE_MASK;

	return (pending_0 || pending_1);
}

/** Cancel a pending configuration */
void edgx_sched_hw_cancel_pending(edgx_io_t *base_pt)
{
	u16 tbl_gen = edgx_rd16(base_pt, EDGX_SCHED_TBL_GEN);

	edgx_wr16(base_pt, EDGX_SCHED_TBL_GEN,
		  tbl_gen & ~EDGX_SCHED_CAN_USE_MASK);

	tbl_gen = edgx_rd16(base_pt, EDGX_SCHED_TBL_GEN + EDGX_SCHED_TBL_OFS);
	edgx_wr16(base_pt, EDGX_SCHED_TBL_GEN + EDGX_SCHED_TBL_OFS,
		  tbl_gen & ~EDGX_SCHED_CAN_USE_MASK);
}

/** Get the first free table index */
int edgx_sched_hw_get_free_tab(edgx_io_t *base_pt)
{
	u16 in_use = edgx_rd16(base_pt, EDGX_SCHED_TBL_GEN) &
			       EDGX_SCHED_IN_USE_MASK;
	if (!in_use)
		return 0;

	in_use = edgx_rd16(base_pt, EDGX_SCHED_TBL_GEN + EDGX_SCHED_TBL_OFS) &
			   EDGX_SCHED_IN_USE_MASK;
	if (!in_use)
		return 1;

	return -EINVAL;
}

/** Get the first used table index */
int edgx_sched_hw_get_used_tab(edgx_io_t *base_pt)
{
	u16 in_use = edgx_rd16(base_pt, EDGX_SCHED_TBL_GEN) &
			       EDGX_SCHED_IN_USE_MASK;
	if (in_use)
		return 0;

	in_use = edgx_rd16(base_pt, EDGX_SCHED_TBL_GEN + EDGX_SCHED_TBL_OFS) &
			   EDGX_SCHED_IN_USE_MASK;
	if (in_use)
		return 1;

	return -EINVAL;
}

/** Submit a new configuration */
void edgx_sched_hw_set_pending(edgx_io_t *base_pt,
			       int tab_idx,
			       const struct timespec64 *base_time,
			       u32 cycle_nsec,
			       u32 cycle_subnsec,
			       u16 down_cnt_speed)
{
	u16 tbl_gen;

	sched_dbg("SET PENDING: Cycle time: nsec=%u, subnsec=%u\n",
		  cycle_nsec, cycle_subnsec);
	sched_dbg("SET PENDING: Start time: sec=%u, nsec=%u\n",
		  (u16)base_time->tv_sec & EDGX_SCHED_START_TIME_SEC_MASK,
		  (u32)base_time->tv_nsec);

	edgx_wr32(base_pt, EDGX_SCHED_CYC_TIME_NSEC +
			   (tab_idx * EDGX_SCHED_TBL_OFS),
		  cycle_nsec);
	edgx_wr32(base_pt, EDGX_SCHED_CYC_TIME_SUBNS +
			   (tab_idx * EDGX_SCHED_TBL_OFS),
		  cycle_subnsec);

	edgx_wr16(base_pt,
		  EDGX_SCHED_START_TIME_SEC + (tab_idx * EDGX_SCHED_TBL_OFS),
		 (u16)base_time->tv_sec & EDGX_SCHED_START_TIME_SEC_MASK);
	edgx_wr32(base_pt,
		  EDGX_SCHED_START_TIME_NSEC + (tab_idx * EDGX_SCHED_TBL_OFS),
		 (u32)base_time->tv_nsec);
	edgx_wr16(base_pt,
		  EDGX_SCHED_CYCLE_CNT + (tab_idx * EDGX_SCHED_TBL_OFS), 0);

	edgx_wr16(base_pt, EDGX_SCHED_SCH_GEN, down_cnt_speed);

	tbl_gen = edgx_rd16(base_pt,
			    EDGX_SCHED_TBL_GEN +
			    (tab_idx * EDGX_SCHED_TBL_OFS));
	edgx_wr16(base_pt, EDGX_SCHED_TBL_GEN + (tab_idx * EDGX_SCHED_TBL_OFS),
		  (tbl_gen | EDGX_SCHED_CAN_USE_MASK) & ~EDGX_SCHED_STOP_LAST);
}

void edgx_sched_hw_get_cc_time(edgx_io_t *base_pt, int tab_idx,
			       struct timespec64 *time)
{
	u16 start_sec;
	u32 start_nsec;

	start_sec = edgx_rd16(base_pt,
			      EDGX_SCHED_START_TIME_SEC +
			      (tab_idx * EDGX_SCHED_TBL_OFS));
	start_nsec = edgx_rd32(base_pt,
			       EDGX_SCHED_START_TIME_NSEC +
			       (tab_idx * EDGX_SCHED_TBL_OFS));

	time->tv_sec = (time64_t)start_sec;
	time->tv_nsec = (long)start_nsec;
}

int edgx_sched_hw_transfer_wait(edgx_io_t *base)
{
	u16 cmd;
	u16 timeout = 500;

	do {
		cmd = edgx_rd16(base, EDGX_SCHED_ROW_AC_CMD0);
		cpu_relax();

		if ((timeout-- == 0U) || (cmd & EDGX_SCHED_AC_ERR))
			return -EBUSY;

	} while (cmd & EDGX_SCHED_AC_TRANSFER);

	return 0;
}

int edgx_sched_hw_write_entry(edgx_io_t *base, int sched_idx, int hw_tab_idx,
			      int row_idx, u16 gate_states,
			      u16 time_interval_clk)
{
	int ret;
	u16 cmd = 0;

	mutex_lock(&com_itf_lock);
	edgx_wr16(base, EDGX_SCHED_ROW_AC_CMD1,
		  row_idx & EDGX_SCHED_ROW_AC_CMD1_MASK);
	edgx_wr16(base, EDGX_SCHED_ROW_DATA_OUT0, gate_states);
	edgx_wr16(base, EDGX_SCHED_ROW_DATA_CYCLES, time_interval_clk);

	cmd = (sched_idx & EDGX_SCHED_CMD_SCHED_MASK) |
	      ((hw_tab_idx & EDGX_SCHED_CMD_TAB_MASK)
	       << EDGX_SCHED_CMD_TAB_MASK_SHIFT) |
	      EDGX_SCHED_AC_WRITE | EDGX_SCHED_AC_TRANSFER;
	edgx_wr16(base, EDGX_SCHED_ROW_AC_CMD0, cmd);
	ret = edgx_sched_hw_transfer_wait(base);

	mutex_unlock(&com_itf_lock);
	return ret;
}

int edgx_sched_hw_read_entry(edgx_io_t *base, int sched_idx, int hw_tab_idx,
			     int row_idx, u16 *gate_states,
			     u16 *interval_clk)
{
	int ret;
	u16 cmd = 0;

	mutex_lock(&com_itf_lock);
	edgx_wr16(base, EDGX_SCHED_ROW_AC_CMD1,
		  row_idx & EDGX_SCHED_ROW_AC_CMD1_MASK);

	cmd = (sched_idx & EDGX_SCHED_CMD_SCHED_MASK) |
	      ((hw_tab_idx & EDGX_SCHED_CMD_TAB_MASK)
	       << EDGX_SCHED_CMD_TAB_MASK_SHIFT) |
	      EDGX_SCHED_AC_TRANSFER;
	edgx_wr16(base, EDGX_SCHED_ROW_AC_CMD0, cmd);

	ret = edgx_sched_hw_transfer_wait(base);
	if (!ret) {
		*gate_states = edgx_rd16(base, EDGX_SCHED_ROW_DATA_OUT0);
		*interval_clk = edgx_rd16(base, EDGX_SCHED_ROW_DATA_CYCLES);
	}
	mutex_unlock(&com_itf_lock);
	return ret;
}

void edgx_sched_hw_disable(edgx_io_t *base_pt, u8 gate_states)
{
	edgx_wr16(base_pt, EDGX_SCHED_EME_DIS_STAT0, (u16)gate_states);
	edgx_wr16(base_pt, EDGX_SCHED_EME_DIS_CTRL, EDGX_SCHED_EME_DIS_ON);
}

void edgx_sched_hw_enable(edgx_io_t *base_pt, u8 gate_states)
{
	edgx_wr16(base_pt, EDGX_SCHED_EME_DIS_STAT0, (u16)gate_states);
	edgx_wr16(base_pt, EDGX_SCHED_EME_DIS_CTRL, EDGX_SCHED_EME_DIS_OFF);
}

bool edgx_sched_hw_is_enabled(edgx_io_t *base_pt)
{
	u16 eme_dis = edgx_rd16(base_pt, EDGX_SCHED_EME_DIS_CTRL);

	return (eme_dis & EDGX_SCHED_EME_DIS_CTRL_R_MASK) ? false : true;
}

u16 edgx_sched_hw_get_cur_cycle(edgx_io_t *base_pt, int tab_idx,
				struct timespec64 *cur_c_time)
{
	u16 tbl_gen;
	u16 cyc_cnt;

	tbl_gen = edgx_rd16(base_pt,
			    EDGX_SCHED_TBL_GEN +
			    (tab_idx * EDGX_SCHED_TBL_OFS));
	edgx_wr16(base_pt, EDGX_SCHED_TBL_GEN + (tab_idx * EDGX_SCHED_TBL_OFS),
		  tbl_gen | EDGX_SCHED_UPDATE_TS_MASK);

	cur_c_time->tv_sec = edgx_rd16(base_pt,
				       EDGX_SCHED_CYC_TS_SEC +
				       (tab_idx * EDGX_SCHED_TBL_OFS));

	cur_c_time->tv_nsec = edgx_rd16(base_pt,
					EDGX_SCHED_CYC_TS_NS_H +
					(tab_idx * EDGX_SCHED_TBL_OFS));

	cur_c_time->tv_nsec <<= 16;
	cur_c_time->tv_nsec |= edgx_rd16(base_pt,
					 EDGX_SCHED_CYC_TS_NS_L +
					 (tab_idx * EDGX_SCHED_TBL_OFS));

	cyc_cnt = edgx_rd16(base_pt,
			    EDGX_SCHED_CYCLE_CNT +
			    (tab_idx * EDGX_SCHED_TBL_OFS));

	return cyc_cnt;
}

void edgx_sched_hw_set_last_cycle(edgx_io_t *base_pt, int tab_idx,
				  u16 last_cycle)
{
	u16 tbl_gen;

	edgx_wr16(base_pt, EDGX_SCHED_LAST_CYC + (tab_idx * EDGX_SCHED_TBL_OFS),
		  last_cycle);

	tbl_gen = edgx_rd16(base_pt,
			    EDGX_SCHED_TBL_GEN +
			    (tab_idx * EDGX_SCHED_TBL_OFS));
	edgx_wr16(base_pt, EDGX_SCHED_TBL_GEN + (tab_idx * EDGX_SCHED_TBL_OFS),
		  tbl_gen | EDGX_SCHED_STOP_LAST);
}

#if defined(EDGX_SCHED_DBG)
void edgx_sched_hw_dump(edgx_io_t *base, edgx_io_t *base_pt,
			int sched_idx, int tab_idx)
{
	u16 start_sec;
	u32 start_nsec;
	u16 tbl_gen;
	int i, ret;
	u16 states;
	u16 clk;
	u32 cyc_nsec, cyc_subnsec;
	u16 eme_dis;

	eme_dis = edgx_rd16(base_pt, EDGX_SCHED_EME_DIS_CTRL);

	start_sec = edgx_rd16(base_pt,
			      EDGX_SCHED_START_TIME_SEC +
			      (tab_idx * EDGX_SCHED_TBL_OFS));
	start_nsec = edgx_rd32(base_pt,
			       EDGX_SCHED_START_TIME_NSEC +
			       (tab_idx * EDGX_SCHED_TBL_OFS));
	tbl_gen = edgx_rd16(base_pt,
			    EDGX_SCHED_TBL_GEN +
			    (tab_idx * EDGX_SCHED_TBL_OFS));

	cyc_nsec = edgx_rd32(base_pt, EDGX_SCHED_CYC_TIME_NSEC +
			     (tab_idx * EDGX_SCHED_TBL_OFS));
	cyc_subnsec = edgx_rd32(base_pt, EDGX_SCHED_CYC_TIME_SUBNS +
				(tab_idx * EDGX_SCHED_TBL_OFS));

	sched_dbg("SCHED: tbl_gen=0x%x, start_sec=%u, start_nsec=%u\n",
		  tbl_gen, start_sec, start_nsec);
	sched_dbg("SCHED: cyc_nsec=%u, cyc_subnsec=%u\n",
		  cyc_nsec, cyc_subnsec);
	sched_dbg("SCHED: EMEDIS=0x%x\n", eme_dis);

	for (i = 0; i < 30; i++) {
		ret = edgx_sched_hw_read_entry(base, sched_idx,
					       tab_idx, i, &states, &clk);
		sched_dbg("SCHED: TAB0, row%d ret=%d state=0x%x, clk=%u\n",
			  i, ret, states, clk);
	}
}
#endif
