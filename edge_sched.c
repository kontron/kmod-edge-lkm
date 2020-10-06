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

#include "edge_ac.h"
#include "edge_port.h"
#include "edge_sched.h"
#include "edge_stat.h"
#include "edge_time.h"
#include "edge_bridge.h"
#include "edge_sched_hw.h"
#include "edge_fqtss.h"
#include "edge_util.h"
#include "edge_link.h"

#define EDGX_SCHED_DEF_GATE_STATES	(0xff)
#define EDGX_SCHED_OP_SET_GSTATES	(0x00)
#define EDGX_SCHED_OP_HOLD_MAC		(0x01)
#define EDGX_SCHED_OP_REL_MAC		(0x02)

/** Scheduler state machine states */
enum edgx_sched_state {
	EDGX_SCHED_ST_IDLE = 0,	/* Idle state, no pending schedule */
	EDGX_SCHED_ST_PENDING,	/* New schedule submitted and pending in HW */
	EDGX_SCHED_ST_PENDING_DELAYED/* New schedule pending in SW
				      * and waiting to be submitted to HW
				      */
};

/** Schedule table entry */
struct edgx_sched_tab_entry {
	u32 time_interval;
	u8 operation_name;
	u8 gate_states;
	u16 padding;
} __packed;

/** Schedule table */
struct edgx_sched_table {
	struct edgx_sched_tab_entry *entries;	    /* Table rows */
	u32			     entry_cnt;     /* Number of entries */
	u32			     list_len;	    /* Number of used entries */
	struct timespec64	     base_time;	    /* Base time */
	u32			     cycle_time_num;/* Numerator */
	u32			     cycle_time_denom;  /* Denominator */
	u32			     cycle_time_ext;    /* Cycle extension */
	u8			     gate_states;       /* Initial gate states*/
};

/** Qbv Scheduler */
struct edgx_sched {
	struct edgx_pt		*parent;
	struct edgx_sched_com   *com;
	struct edgx_stat_hdl	*hstat;

	edgx_io_t		*iobase_pt;  /* Port specific HW base address */
	int			sched_idx;   /* FSC scheduler number */

	struct edgx_sched_table	admin_tab;   /* The admin schedule table */
	/* The operational schedule table mirrors of the HW schedule tables */
	struct edgx_sched_table	op_tabs[EDGX_SCHED_HW_TAB_CNT];

	bool			gate_enabled;      /* Gate enabled */
	bool			config_change;     /* Config change requested */
	struct timespec64	conf_change_time;  /* Config change time */
	struct timespec64	start_time;        /* Schedule start time */
	enum edgx_sched_state	state;		   /* Scheduler state */
	struct delayed_work	hnd_delayed;	   /* Pending delayed work */
	u64			conf_change_err;
	int			link_mode;
	u16			down_cnt_speed;	  /* Down-counter speed value */
	/* Sum of intervals in the admin entries */
	u32			interval_sum;
	struct mutex		lock;		  /* Protect edgx_sched */
	struct edgx_sched_tr_rate tr_rate[EDGX_SCHED_MAX_QUEUES];
};

enum _stat_st_idx {
	_STAT_TX_OVERRUN = 0,
	_STAT_MAX,
};

static const struct edgx_statinfo _st_statinfo = {
	.feat_id = EDGX_STAT_FEAT_ST,
	/* Worst case, there can be one TX_OVERFLOW in every interval of the
	 * gate control list. Since the lower limit of the interval as of
	 * the IP implementation is 64 clock (= 250ns@255MHz, where 255 MHz
	 * is the highest frequency possible on FSC), i.e., the 32bit delta-
	 * counter wraps after
	 *   2^32 * 250 nanoseconds (approx 18 minutes).
	 * So we'll run the capture counter update every 15 minutes to have
	 * a 'nice' number.
	 *
	 * Note that this is the real worst case, where it is assumed that
	 * every interval produces a TX_OVERRUN, which is extremely unlikely.
	 */
	.rate_ms = 900000, /* = 15 minutes */
	.base    = 0x2C0,
	.nwords  = _STAT_MAX,
};

/** Qbv scheduler common part */
struct edgx_sched_com {
	struct edgx_br		 *parent;
	const struct edgx_ifdesc *ifd_com;
	edgx_io_t		 *iobase;	  /* Common HW base address */
	struct edgx_sched	 *sched_list[EDGX_BR_MAX_PORTS];
	u16			  ns_per_clk;	  /* ns per clock tick */
	u32			  max_entry_cnt; /* Maximum ctrl entry count */
	u32			  max_cyc_time_ns;/*Maximum cycle time*/
	struct edgx_br_irq	 *irq;
	struct work_struct	  work_isr;
	struct workqueue_struct	 *wq_isr;
	u8			  nr_queues;
	u8			  gate_st_msk;
};

static void edgx_sched_stm_cct_handler(struct edgx_sched *sched);

static inline struct edgx_sched *edgx_dev2sched(struct device *dev)
{
	return edgx_pt_get_sched(edgx_dev2pt(dev));
}

static inline u16 edgx_sched_get_hold(struct edgx_sched *sched)
{
	return BIT(sched->com->nr_queues);
}

static void edgx_sched_isr_work(struct work_struct *work)
{
	struct edgx_sched_com *sc = container_of(work, struct edgx_sched_com,
						 work_isr);
	int i;
	u16 intstat = edgx_rd16(sc->iobase, EDGX_SCHED_INT_STAT);

	edgx_wr16(sc->iobase, EDGX_SCHED_INT_STAT, ~intstat);
	sched_dbg("SCHED ISR work.\n");
	for (i = 0; i < EDGX_BR_MAX_PORTS; i++) {
		if (sc->sched_list[i])
			edgx_sched_stm_cct_handler(sc->sched_list[i]);
	}
	if (sc->irq->trig == EDGX_IRQ_LEVEL_TRIG)
		edgx_wr16(sc->iobase, EDGX_SCHED_INT_MASK,
			  EDGX_SCHED_INT_MSKVAL);
}

static irqreturn_t edgx_sched_isr(int irq, void *device)
{
	struct edgx_sched_com *sc = (struct edgx_sched_com *)device;
	u16 mask;
	u16 stat;

	if (sc->irq->shared) {
		mask = edgx_rd16(sc->iobase, EDGX_SCHED_INT_MASK);
		if (!mask)
			return IRQ_NONE;

		stat = edgx_rd16(sc->iobase, EDGX_SCHED_INT_STAT);
		if (!(mask & stat))
			return IRQ_NONE;
	}

	if (sc->irq->trig == EDGX_IRQ_LEVEL_TRIG)
		edgx_wr16(sc->iobase, EDGX_SCHED_INT_MASK, 0);
	queue_work(sc->wq_isr, &sc->work_isr);

	return IRQ_HANDLED;
}

/** Initialize common the scheduler part. */
int edgx_sched_com_probe(struct edgx_br *br, struct edgx_br_irq *irq,
			 const char *drv_name,
			 struct edgx_sched_com **psc)
{
	const struct edgx_ifreq ifreq = { .id = AC_SCHED_ID, .v_maj = 1 };
	const struct edgx_ifdesc *ifd_com = edgx_ac_get_if(&ifreq);
	u16 clk_frq;
	int ret = 0;

	if (!br || !psc)
		return -EINVAL;

	if (!ifd_com)
		return -ENODEV;

	*psc = kzalloc(sizeof(**psc), GFP_KERNEL);
	if (!(*psc)) {
		edgx_br_err(br, "Cannot allocate Common Scheduled Traffic\n");
		return -ENOMEM;
	}

	(*psc)->parent = br;
	(*psc)->ifd_com = ifd_com;
	(*psc)->iobase = (*psc)->ifd_com->iobase;
	(*psc)->max_entry_cnt = edgx_sched_get_hw_row_cnt((*psc)->iobase);
	(*psc)->nr_queues = edgx_br_get_generic(br, BR_GX_QUEUES);
	(*psc)->gate_st_msk = (u8)(BIT((*psc)->nr_queues) - 1);
	/* The last row is reserved for 0 interval entries */
	(*psc)->max_entry_cnt -= 1U;
	clk_frq = edgx_sched_get_hw_clk_frq((*psc)->iobase);
	(*psc)->ns_per_clk = 1000U / clk_frq;
	(*psc)->max_cyc_time_ns = min(EDGX_SCHED_HW_MAX_CT_NS,
				      (*psc)->max_entry_cnt *
				      (u32)((*psc)->ns_per_clk) *
				      EDGX_SCHED_INTERVAL_MAX);

	(*psc)->irq = irq;
	INIT_WORK(&(*psc)->work_isr, &edgx_sched_isr_work);
	(*psc)->wq_isr = alloc_workqueue(drv_name,
					   WQ_HIGHPRI | WQ_MEM_RECLAIM, 0);
	if (!(*psc)->wq_isr) {
		pr_err("%s(): alloc_workqueue failed!\n", __func__);
		return -ENOMEM;
	}

	if (irq->shared)
		ret = request_irq(irq->irq_vec[0], &edgx_sched_isr, IRQF_SHARED,
				  drv_name, *psc);
	else
		ret = request_irq(irq->irq_vec[EDGX_IRQ_NR_SCHED_TAB],
				  &edgx_sched_isr, IRQF_SHARED, drv_name, *psc);

	if (ret) {
		pr_err("%s(): request_irq failed! ret=%d, irq=%d\n",
		       __func__, ret, irq->shared ?
		       irq->irq_vec[0] : irq->irq_vec[EDGX_IRQ_NR_SCHED_TAB]);
		destroy_workqueue((*psc)->wq_isr);
		return ret;
	}

	edgx_wr16((*psc)->iobase, EDGX_SCHED_INT_STAT, 0);
	edgx_wr16((*psc)->iobase, EDGX_SCHED_INT_MASK, EDGX_SCHED_INT_MSKVAL);

	sched_dbg("FSC cfg: max_entry_cnt=%d, ns_per_clk=%d\n",
		  (*psc)->max_entry_cnt, (*psc)->ns_per_clk);
	return ret;
}

void edgx_sched_com_shutdown(struct edgx_sched_com *sched_com)
{
	if (sched_com) {
		if (sched_com->irq->shared)
			free_irq(sched_com->irq->irq_vec[0], sched_com);
		else
			free_irq(sched_com->irq->irq_vec[EDGX_IRQ_NR_SCHED_TAB],
				 sched_com);

		cancel_work_sync(&sched_com->work_isr);
		edgx_wr16(sched_com->iobase, EDGX_SCHED_INT_STAT, 0);
		edgx_wr16(sched_com->iobase, EDGX_SCHED_INT_MASK, 0);
		destroy_workqueue(sched_com->wq_isr);
		kfree(sched_com);
	}
}

#if defined(EDGX_SCHED_DBG)
static void edgx_sched_dump(struct edgx_sched *sched)
{
	int i;

	sched_dbg("Port Id: %d\n", edgx_pt_get_id(sched->parent));
	sched_dbg("sched_idx: %d\n", sched->sched_idx);
	sched_dbg("iobase_com: 0x%x; iobase_pt: 0x%x\n",
		  (u32)sched->com->iobase, (u32)sched->iobase_pt);
	sched_dbg("max_entry_cnt: %u\n", sched->com->max_entry_cnt);
	sched_dbg("gate_enabled: %d\n", sched->gate_enabled);
	sched_dbg("config_change: %d\n", sched->config_change);
	sched_dbg("conf_change_time: sec=%lli, nsec=%li\n",
		  sched->conf_change_time.tv_sec,
		  sched->conf_change_time.tv_nsec);
	sched_dbg("start_time: sec=%lli, nsec=%li\n",
		  sched->start_time.tv_sec, sched->start_time.tv_nsec);
	sched_dbg("state: %d\n", sched->state);
	sched_dbg("ns_per_clk: %d\n", sched->com->ns_per_clk);
	sched_dbg("conf_change_err: %llu\n", sched->conf_change_err);
	sched_dbg("link_mode: %d\n", sched->link_mode);
	sched_dbg("interval_sum: %d\n", sched->interval_sum);
	sched_dbg("max_cyc_time_ns: %d\n", sched->com->max_cyc_time_ns);

	for (i = 0; i < sched->com->nr_queues; i++) {
		sched_dbg("tr_rate[%d]: %llu, %llu\n", i,
			  sched->tr_rate[i].num,
			  sched->tr_rate[i].denom);
	}
}

static void edgx_sched_dump_tab(struct edgx_sched_table *tab, const char *label)
{
	int i;

	sched_dbg("TABLE %s\n", label);
	sched_dbg("entry_cnt: %d\n", tab->entry_cnt);
	sched_dbg("list_len: %d\n", tab->list_len);
	sched_dbg("gate_states: 0x%x\n", tab->gate_states);
	sched_dbg("cycle_time_num: %d\n", tab->cycle_time_num);
	sched_dbg("cycle_time_denom: %d\n", tab->cycle_time_denom);
	sched_dbg("cycle_time_ext: %d\n", tab->cycle_time_ext);
	sched_dbg("base_time: sec=%lli, nsec=%li\n", tab->base_time.tv_sec,
		  tab->base_time.tv_nsec);

	if (!tab->entries)
		return;

	sched_dbg("Entries:\n");
	for (i = 0; i < tab->entry_cnt; i++) {
		sched_dbg("op=0x%x\tstate=0x%x\tint=%d\n",
			  tab->entries[i].operation_name,
			  tab->entries[i].gate_states,
			  tab->entries[i].time_interval);
	}
}
#endif

static void edgx_sched_rational_to_nsec(u32 sec_num,
					u32 sec_denom,
					u64 *nsec,
					u32 *subnsec
					)
{
	u32 cycle_sec;
	u64 num = sec_num;
	u64 rem = do_div(num, sec_denom);

	cycle_sec = num;
	num = (u64)rem * NSEC_PER_SEC;
	rem = do_div(num, sec_denom);
	*nsec = num + (cycle_sec * NSEC_PER_SEC);

	num = (u64)rem << 32;
	rem = do_div(num, sec_denom);
	*subnsec = num;
}

static void edgx_sched_nsec_to_rational(u64 nsec,
					u32 subnsec,
					u32 *sec_num,
					u32 *sec_denom
					)
{
	//TODO Add a real conversion algorithm.
	*sec_num = nsec;
	*sec_denom = NSEC_PER_SEC;
}

static void edgx_sched_calc_trans_rate(struct edgx_sched *sched)
{
	int queue, i;
	u64 sum, idle_time, cyc_time_nsec, sum_total = 0;
	struct edgx_sched_table	*tab = &sched->admin_tab;
	u32 num, denom, cyc_time_subnsec;

	edgx_sched_rational_to_nsec(sched->admin_tab.cycle_time_num,
				    sched->admin_tab.cycle_time_denom,
				    &cyc_time_nsec,
				    &cyc_time_subnsec);

	for (queue = 0; queue < sched->com->nr_queues; queue++) {
		sum = 0;
		sum_total = 0;
		for (i = 0; i < tab->list_len; i++) {
			sum_total += tab->entries[i].time_interval;

			if (tab->entries[i].gate_states & BIT(queue))
				sum += tab->entries[i].time_interval;
		}

		idle_time = cyc_time_nsec - sum_total;

		/* The gate remains in the last state -> add idle time */
		if (tab->entries[tab->list_len - 1].gate_states & BIT(queue))
			sum += idle_time;

		if (sum) {
			edgx_sched_nsec_to_rational(sum, 0, &num, &denom);
			sched->tr_rate[queue].num = (u64)denom *
						    (u64)tab->cycle_time_num;
			sched->tr_rate[queue].denom = (u64)num *
						      (u64)
						      tab->cycle_time_denom;
		} else {
			sched->tr_rate[queue].num = 0;
			sched->tr_rate[queue].denom = 1;
		}
	}
}

static void edgx_sched_notify(struct edgx_sched *sched)
{
	struct edgx_fqtss *fqtss = edgx_pt_get_fqtss(sched->parent);

	sched_dbg("Notify\n");
	edgx_fqtss_sched_change(fqtss, sched);
}

/* NOTE: Deviation from the standard:
 * The control interval is limited to 16-bit cycles.
 * Larger intervals are truncated.
 */
static ssize_t edgx_sched_adjust_admin_tab(struct edgx_sched *sched)
{
	int i;
	u32 cycles;
	struct edgx_sched_table	*tab = &sched->admin_tab;

	sched->interval_sum = 0;

	for (i = 0; i < tab->entry_cnt; i++) {
		tab->entries[i].gate_states &= sched->com->gate_st_msk;
		cycles = tab->entries[i].time_interval / sched->com->ns_per_clk;
		if ((cycles < EDGX_SCHED_INTERVAL_MIN) ||
		    (cycles > EDGX_SCHED_INTERVAL_MAX)) {
			sched->conf_change_err++;
			edgx_pt_err(sched->parent,
				    "Control List Interval out of range!\n");
			return -EINVAL;
		}
		sched->interval_sum += tab->entries[i].time_interval;
	}
	return 0;
}

static int edgx_sched_check_cc_params(struct edgx_sched *sched)
{
	if ((!sched->admin_tab.entries) ||
	    (sched->admin_tab.entry_cnt == 0) ||
	    (sched->admin_tab.cycle_time_num == 0) ||
	    (sched->admin_tab.list_len == 0) ||
	    (sched->admin_tab.entry_cnt < sched->admin_tab.list_len))
		return -EFAULT;

	sched->link_mode = edgx_pt_get_speed(sched->parent);

	if (sched->link_mode == SPEED_UNKNOWN) {
		edgx_pt_warn(sched->parent, "Link speed unknown!\n");
		return -EIO;
	}
	return 0;
}

/** Adjust time by FSC down-counter delay for given link speed */
static void edgx_sched_adjust_time(struct edgx_sched *sched,
				   const struct timespec64 *in_time,
				   struct timespec64 *out_time,
				   bool increase)
{
	u32 adj = EDGX_SCHED_TIME_ADJ_DC;
	struct edgx_link *lnk = edgx_pt_get_link(sched->parent);
	ktime_t link_dly = edgx_link_get_tx_delay(lnk);
	ktime_t g2o_min = edgx_pt_get_g2omin(sched->parent);
	s64 diff;

	switch (sched->link_mode) {
	case SPEED_1000:
		sched->down_cnt_speed = EDGX_SCHED_HW_DC_1000FULL;
		break;
	case SPEED_100:
		adj *= 10U;
		sched->down_cnt_speed = EDGX_SCHED_HW_DC_100FULL;
		break;
	case SPEED_10:
		adj *= 100U;
		sched->down_cnt_speed = EDGX_SCHED_HW_DC_10FULL;
		break;
	default:
		sched->down_cnt_speed = EDGX_SCHED_HW_DC_1000FULL;
		edgx_pt_err(sched->parent, "Unknown link speed during down counter delay calculation, assuming 1GBps");
		break;
	}

	adj *= sched->com->ns_per_clk;
	if (sched->com->ns_per_clk == 10U)
		adj = (adj * 4U) / 5U;	/* Divide by 1.25 */
	else if (sched->com->ns_per_clk == 5U)
		adj = (adj * 8U) / 5U;  /* 200Mhz */


	adj += 10U * sched->com->ns_per_clk;	/* Additional fixed delay */
	diff = adj + ktime_to_ns(g2o_min) + ktime_to_ns(link_dly);

	if (increase)
		set_normalized_timespec64(out_time, in_time->tv_sec,
					  in_time->tv_nsec + diff);
	else
		set_normalized_timespec64(out_time, in_time->tv_sec,
					  in_time->tv_nsec - diff);
}

/*  Calculate the number of cycles between two points in time.
 *
 *  Algorithm taken from deipce_fsc_hw.c:
 * Basically N = count = delta / (numerator/denominator) [round up].
 * That could overflow with 64 bit arithmetic if nanoseconds were used
 * directly. Calculate N separately for seconds and nanoseconds,
 * taking remainders into account, and round up the result. So
 *
 * N = count = (delta * denominator) / numerator =
 * (delta_sec * denominator) / numerator +
 * (delta_nsec * denominator) / (numerator * NSEC_PER_SEC)
 *
 * With integer division returning quotient as whole number and remainder,
 * this can be written as
 *
 * count = count_sec + count_sec_rem / numerator +
 * (count_nsec + count_nsec_rem / numerator) / NSEC_PER_SEC
 * = count_sec +
 * (NSEC_PER_SEC * count_sec_rem + count_nsec * numerator + count_nsec_rem) /
 * (NSEC_PER_SEC * numerator)
 *
 * That could still overflow, but only when start_time is very much
 * (in the order of 2^32 seconds, i.e. about 136 years) behind min_time.
 */
static u64 edgx_sched_diff_to_cycles(const struct timespec64 *begin_time,
				     const struct timespec64 *end_time,
				     u32 cycle_time_num,
				     u32 cycle_time_denom,
				     bool round_up,
				     u64 diff_sec_mask)
{
	struct timespec64 delta = timespec64_sub(*end_time, *begin_time);

	u64 count;
	u64 count_sec;
	u32 count_sec_rem;
	u64 count_nsec;
	u32 count_nsec_rem;
	u64 count_res;

	delta.tv_sec &= diff_sec_mask;

	count_sec = delta.tv_sec * cycle_time_denom;
	count_sec_rem = do_div(count_sec, cycle_time_num);

	count_nsec = (u64)delta.tv_nsec * cycle_time_denom;
	count_nsec_rem = do_div(count_nsec, cycle_time_num);

	count_res = (u64)count_sec_rem * NSEC_PER_SEC +
			count_nsec * cycle_time_num + count_nsec_rem;
	if (round_up)
		count_res += (u64)cycle_time_num * NSEC_PER_SEC - 1;
	do_div(count_res, cycle_time_num);
	do_div(count_res, NSEC_PER_SEC);

	count = count_sec + count_res;

	return count;
}

static void edgx_sched_calc_future_time(const struct timespec64 *in_time,
					const struct timespec64 *cur_time,
					struct timespec64 *out_time,
					u32 cycle_time_num,
					u32 cycle_time_denom)
{
	u64 count;
	u64 num;
	u32 rem;
	struct timespec64 advance;

	count = edgx_sched_diff_to_cycles(in_time, cur_time, cycle_time_num,
					  cycle_time_denom, true, ~((u64)0));

	/* Add calculated number of cycle times to out_time */
	num = count * cycle_time_num;
	rem = do_div(num, cycle_time_denom);
	advance.tv_sec = num;

	num = (u64)rem * NSEC_PER_SEC;
	do_div(num, cycle_time_denom);
	advance.tv_nsec = num;

	sched_dbg("EDGX_SCHED: %s() start0 %lli.%09li cycle_time %u/%u\n",
		  __func__, in_time->tv_sec, in_time->tv_nsec,
		  cycle_time_num, cycle_time_denom);
	sched_dbg("EDGX_SCHED: %s() count %llu advance %lli.%09li\n",
		  __func__, count, advance.tv_sec,
		  advance.tv_nsec);

	*out_time = timespec64_add(*in_time, advance);
}

static void edgx_sched_calc_cc_time(struct edgx_sched *sched,
				    time64_t *delay_sec,
				    bool *time_in_past)
{
	struct timespec64 cur_time;
	time64_t diff;
	struct edgx_time *time = edgx_pt_get_time(sched->parent);

	*time_in_past = false;
	*delay_sec = 0;
	edgx_tm_get_wrk_time(time, &cur_time);

	/* If base time in the past */
	if ((sched->admin_tab.base_time.tv_sec < cur_time.tv_sec) ||
	    ((sched->admin_tab.base_time.tv_sec == cur_time.tv_sec) &&
	    (sched->admin_tab.base_time.tv_nsec <= cur_time.tv_nsec))) {
		edgx_sched_calc_future_time(&sched->admin_tab.base_time,
					    &cur_time,
					   &sched->start_time,
					   sched->admin_tab.cycle_time_num,
					   sched->admin_tab.cycle_time_denom);
		*time_in_past = true;
	} else {
		sched->start_time = sched->admin_tab.base_time;
	}

	sched->conf_change_time = sched->start_time;
	edgx_sched_adjust_time(sched, &sched->start_time,
			       &sched->start_time, false);

	diff = sched->start_time.tv_sec - cur_time.tv_sec;

	if (diff > EDGX_SCHED_MAX_DELAY) {
		if ((diff - EDGX_SCHED_MIN_ADVANCE) < EDGX_SCHED_MAX_DELAY)
			*delay_sec = diff - EDGX_SCHED_MIN_ADVANCE;
		else
			*delay_sec = EDGX_SCHED_MAX_DELAY;
	}
}

/* NOTE: Deviation from the standard:
 * The cycle time extension is simulated by stopping the last cycle.
 * The value is not taken into account. All non-zero values will cause
 * stopping the last cycle.
 * The current implementation supports maximum cycle time extension of 1 cycle.
 */
void edgx_sched_handle_cyc_time_ext(struct edgx_sched *sched)
{
	int tab_idx = edgx_sched_hw_get_used_tab(sched->iobase_pt);
	u16 cur_cycle, last_cycle;
	u64 cycle_inc;
	struct timespec64 cur_c_time;
	struct timespec64 start_time_reduced;
	struct edgx_sched_table *tab;
	struct timespec64 ts;
	struct edgx_time *time = edgx_pt_get_time(sched->parent);

	if (tab_idx < 0)
		return;

	tab = &sched->op_tabs[tab_idx];
	if (tab->cycle_time_ext == 0)
		return;

	cur_cycle = edgx_sched_hw_get_cur_cycle(sched->iobase_pt, tab_idx,
						&cur_c_time);
	edgx_tm_get_wrk_time(time, &ts);
	sched_dbg("current_time: %lld.%lu\n", ts.tv_sec, ts.tv_nsec);

	/* cur_c_time has a HW limited range. Start time range must be reduced
	 * accordingly. This works only if start time is in the near future.
	 */
	start_time_reduced = sched->start_time;
	start_time_reduced.tv_sec &= EDGX_SCHED_START_TIME_SEC_MASK;

	cycle_inc = edgx_sched_diff_to_cycles(&cur_c_time,
					      &start_time_reduced,
					      tab->cycle_time_num,
					      tab->cycle_time_denom,
					      false,
					      EDGX_SCHED_START_TIME_SEC_MASK);
	last_cycle = cur_cycle + (u16)cycle_inc;

	edgx_sched_hw_set_last_cycle(sched->iobase_pt, tab_idx, last_cycle);

	sched_dbg("cur_c_time.tv_sec=%lld, tv_nsec=%lu\n",
		  cur_c_time.tv_sec, cur_c_time.tv_nsec);
	sched_dbg("start_time.tv_sec=%lld, tv_nsec=%lu\n",
		  sched->start_time.tv_sec, sched->start_time.tv_nsec);
	sched_dbg("start_time_reduced.tv_sec=%llu, tv_nsec=%lu\n",
		  start_time_reduced.tv_sec, start_time_reduced.tv_nsec);
	sched_dbg("cur_cycle=%u, cycle_inc=%llu, last_cycle=%u\n",
		  cur_cycle, cycle_inc, last_cycle);
}

static int edgx_sched_config_change(struct edgx_sched *sched)
{
	int tb, i, ret = 0;
	struct edgx_sched_table *tab;
	struct edgx_sched_tab_entry *entries;
	u64 ct_nsec;
	u32 ct_subnsec;
	u16 hold = 0;

	tb = edgx_sched_hw_get_free_tab(sched->iobase_pt);
	tab = &sched->op_tabs[tb];
	memcpy(tab, &sched->admin_tab, sizeof(*tab));
	entries = tab->entries;

	for (i = 0; !ret && (i < tab->list_len); i++) {
		if (entries[i].operation_name == EDGX_SCHED_OP_HOLD_MAC)
			hold = edgx_sched_get_hold(sched);
		else if (entries[i].operation_name == EDGX_SCHED_OP_REL_MAC)
			hold = 0;

		ret = edgx_sched_hw_write_entry(sched->com->iobase,
						sched->sched_idx, tb, i,
						(u16)entries[i].gate_states
						| hold,
						entries[i].time_interval /
						sched->com->ns_per_clk);
	}

	/* Add a 0 interval entry at the end. FSC will stop on this entry */
	if (!ret)
		ret = edgx_sched_hw_write_entry(sched->com->iobase,
						sched->sched_idx, tb, i,
						entries[i - 1].gate_states,
						0);

	for (i = 0; hold && !ret && (i < tab->list_len) &&
	     entries[i].operation_name != EDGX_SCHED_OP_HOLD_MAC &&
	     entries[i].operation_name != EDGX_SCHED_OP_REL_MAC; i++) {
		ret = edgx_sched_hw_write_entry(sched->com->iobase,
						sched->sched_idx, tb, i,
						(u16)entries[i].gate_states
						| hold,
						entries[i].time_interval /
						sched->com->ns_per_clk);
	}

	if (!ret) {
		edgx_sched_handle_cyc_time_ext(sched);
		/* NOTE: Deviation from the standard - cycle time extension.
		 * If config change is applied too late, the IP will adjust
		 * the start time and calculation of the last cycle will
		 * be wrong. Therefore, the last cycle should be calculated
		 * after edgx_sched_hw_set_pending based on the start time
		 * that is adjusted by IP.
		 * The correct sequence would be:
		 * 1. remember the table currently in use,
		 * 2. call edgx_sched_hw_set_pending,
		 * 3. wait till the IP adjusts the start time,
		 * 4. call edgx_sched_fix_cc_time to get the new start time,
		 * 5. call edgx_sched_handle_cyc_time_ext, but use
		 * the remembered table in use.
		 *
		 * The current IP does not provide a possibility to check when
		 * the start time adjustment is finished. Therefore step 3
		 * cannot be done. We stay with the current approach.
		 */
		edgx_sched_rational_to_nsec(sched->op_tabs[tb].cycle_time_num,
					    sched->op_tabs[tb].cycle_time_denom,
					    &ct_nsec,
					    &ct_subnsec);
		edgx_sched_hw_set_pending(sched->iobase_pt, tb,
					  &sched->start_time,
					  (u32)ct_nsec,
					  ct_subnsec,
					  sched->down_cnt_speed);
	}
	tab->entries = NULL;
	return ret;
}

/* Check if HW adjusted the start time. Adjust CCT accordingly.*/
static void edgx_sched_fix_cc_time(struct edgx_sched *sched)
{
	struct timespec64 hw_start_time;
	struct timespec64 start_time_reduced;
	struct timespec64 delta;

	int tab_idx = edgx_sched_hw_get_used_tab(sched->iobase_pt);

	if (tab_idx < 0)
		return;

	edgx_sched_hw_get_cc_time(sched->iobase_pt, tab_idx, &hw_start_time);
	/* hw_cct has a HW limited range. Start time range must be reduced. */
	start_time_reduced = sched->start_time;
	start_time_reduced.tv_sec &= EDGX_SCHED_START_TIME_SEC_MASK;

	delta = timespec64_sub(hw_start_time, start_time_reduced);

	if ((delta.tv_sec > 0) || (delta.tv_nsec > 0)) {
		sched->conf_change_err++;
		sched->start_time = timespec64_add(sched->start_time, delta);
		edgx_sched_adjust_time(sched, &sched->start_time,
				       &sched->conf_change_time, true);
		edgx_pt_warn(sched->parent, "Start time adjusted by HW!\n");
	}
}

/******************************************************************************
 * State Machine Request Handlers
 *****************************************************************************/
static void edgx_sched_stm_cct_handler(struct edgx_sched *sched)
{
	mutex_lock(&sched->lock);
	if ((sched->state == EDGX_SCHED_ST_PENDING) &&
	    (!edgx_sched_hw_is_pending(sched->iobase_pt))) {
		edgx_sched_fix_cc_time(sched);
		sched->state = EDGX_SCHED_ST_IDLE;
		edgx_sched_notify(sched);
	}
	mutex_unlock(&sched->lock);
}

/* NOTE: Deviation from the standard:
 * The Qbv List Config state machine allows to change administrative
 * parameters during a pending configuration.
 * The driver ensures this behavior only up to the point in time, when
 * the administrative values are copied to the pending operational
 * table (EDGX_SCHED_ST_PENDING_DELAYED).
 * Change of administrative values after that point in time
 * (EDGX_SCHED_ST_PENDING) have no effect on the already submitted
 * pending config change.
 */
static int edgx_sched_stm_cfg_change(struct edgx_sched *sched)
{
	int ret = 0;
	bool time_in_past;
	unsigned long jif;
	time64_t delay_sec;

	if (sched->state == EDGX_SCHED_ST_PENDING)
		edgx_sched_hw_cancel_pending(sched->iobase_pt);

	if (sched->state == EDGX_SCHED_ST_PENDING_DELAYED)
		cancel_delayed_work_sync(&sched->hnd_delayed);

	sched->state = EDGX_SCHED_ST_IDLE;

	if (!sched->gate_enabled)
		return ret;

	ret = edgx_sched_check_cc_params(sched);
	if (ret) {
		sched->config_change = false;
		return ret;
	}

	edgx_sched_calc_cc_time(sched, &delay_sec, &time_in_past);
	if (time_in_past) {
		sched->conf_change_err++;
		edgx_pt_warn(sched->parent, "AdminBaseTime in the past!\n");
	}

	if (delay_sec) {
		sched->state = EDGX_SCHED_ST_PENDING_DELAYED;
		jif = msecs_to_jiffies(delay_sec * MSEC_PER_SEC);
		schedule_delayed_work(&sched->hnd_delayed, jif);
		sched_dbg("SCHED: Delayed work started\n delay_sec=%llu\n",
			  delay_sec);
	} else {
		ret = edgx_sched_config_change(sched);
		if (!ret) {
			sched->state = EDGX_SCHED_ST_PENDING;
			edgx_sched_calc_trans_rate(sched);
		}
	}
	sched->config_change = false;
	return ret;
}

static int edgx_sched_stm_gate_enable(struct edgx_sched *sched)
{
	int ret = 0;
	int tab_idx;

	if (sched->state == EDGX_SCHED_ST_IDLE) {
		edgx_sched_hw_enable(sched->iobase_pt,
				     sched->admin_tab.gate_states);
		/* Notify only if there is a running schedule */
		tab_idx = edgx_sched_hw_get_used_tab(sched->iobase_pt);
		if (tab_idx >= 0)
			edgx_sched_notify(sched);

		if (sched->config_change)
			ret = edgx_sched_stm_cfg_change(sched);
	}
	return ret;
}

/* NOTE: Deviation from the standard:
 * Gate enable/disable is simulated by the MUX. The scheduler itself
 * is not disabled. After re-enabling the MUX, the schedule continues
 * at an arbitrary point in the control list.
 */
static void edgx_sched_stm_gate_disable(struct edgx_sched *sched)
{
	if (sched->state == EDGX_SCHED_ST_PENDING)
		edgx_sched_hw_cancel_pending(sched->iobase_pt);
	else if (EDGX_SCHED_ST_PENDING_DELAYED)
		cancel_delayed_work_sync(&sched->hnd_delayed);

	edgx_sched_hw_disable(sched->iobase_pt, sched->admin_tab.gate_states);
	sched->state = EDGX_SCHED_ST_IDLE;
	edgx_sched_notify(sched);
}

static void edgx_sched_stm_pending_delayed(struct work_struct *work)
{
	int ret;
	struct edgx_sched *sched =
			container_of(work, struct edgx_sched, hnd_delayed.work);

	sched_dbg("Pending_delayed work.\n");
	mutex_lock(&sched->lock);
	sched->state = EDGX_SCHED_ST_IDLE;
	ret = edgx_sched_stm_cfg_change(sched);
	mutex_unlock(&sched->lock);
}

/** Initialize the scheduler. */
static int edgx_sched_init(struct edgx_sched *sched, struct edgx_pt *pt,
			   struct edgx_sched_com *sched_com,
			   edgx_io_t *iobase_pt)
{
	int i;

	sched->parent = pt;
	sched->com = sched_com;
	sched->iobase_pt = iobase_pt;
	sched->sched_idx = edgx_pt_get_id(pt);

	mutex_init(&sched->lock);
	sched->link_mode = SPEED_UNKNOWN;
	sched->down_cnt_speed = EDGX_SCHED_HW_DC_1000FULL;
	sched->admin_tab.gate_states = EDGX_SCHED_DEF_GATE_STATES &
				       sched_com->gate_st_msk;
	sched->admin_tab.cycle_time_denom = 1;

	for (i = 0; i < sched->com->nr_queues; i++) {
		sched->tr_rate[i].num = 1;
		sched->tr_rate[i].denom = 1;
		edgx_pt_set_framesize(sched->parent, i, EDGX_SCHED_DEF_MAX_SDU);
	}
	INIT_DELAYED_WORK(&sched->hnd_delayed, edgx_sched_stm_pending_delayed);

	return edgx_sched_hw_init(iobase_pt, sched->com->ns_per_clk,
				  EDGX_SCHED_DEF_GATE_STATES &
				  sched_com->gate_st_msk);
}

/** Get FSC HW port specific interface. */
static const
struct edgx_ifdesc *edgx_sched_if2ptif(const struct edgx_ifdesc *ifdesc,
				       ptid_t pt,
				       struct edgx_ifdesc *ptifdesc)
{
	if (!ifdesc || !ptifdesc || (!(ifdesc->ptmap & BIT(pt))))
		return NULL;

	ptifdesc->id      = ifdesc->id;
	ptifdesc->ver     = ifdesc->ver;
	ptifdesc->len     = EDGX_SCHED_PT_RES_LEN;
	ptifdesc->iobase  = ifdesc->iobase + EDGX_SCHED_PT_RES_OFFS +
			    (pt * EDGX_SCHED_PT_RES_LEN);
	ptifdesc->ptmap   = BIT(pt);

	return ptifdesc;
}

static ssize_t edgx_sched_strtotimespec64(const char *buf,
					  size_t count,
					  struct timespec64 *time)
{
	char str[64], str_nsec[10];
	char *token, *str_p = str;
	s64 sec;
	long nsec = 0;
	ssize_t max_size = min_t(ssize_t, 63U, count);

	strncpy(str, buf, max_size);
	str[max_size] = '\0';
	token = strsep(&str_p, ".");
	if (!token)
		return -EINVAL;

	if (kstrtos64(token, 10, &sec))
		return -EINVAL;

	token = strsep(&str_p, "\n\r\t ");
	if (token) {
		if (strlen(token) > 9)
			return -EINVAL;
		/* Add trailing zeros to nsec part */
		scnprintf(str_nsec, 10, "%s%.*d", token,
			  (int)(9 - strlen(token)), 0);
		if (kstrtol(str_nsec, 10, &nsec))
			return -EINVAL;
	}
	time->tv_sec = sec;
	time->tv_nsec = nsec;
	return count;
}

/******************************************************************************
 * User Interface
 *****************************************************************************/

int edgx_sched_get_trans_rate(struct edgx_sched *sched,
			      unsigned int queue_idx,
			      struct edgx_sched_tr_rate *tr_rate)
{
	if (!sched->gate_enabled) {
		tr_rate->num = 1;
		tr_rate->denom = 1;
		return 0;
	}
	tr_rate->num = sched->tr_rate[queue_idx].num;
	tr_rate->denom = sched->tr_rate[queue_idx].denom;
	return 0;
}

int edgx_sched_get_trans_rate_lock(struct edgx_sched *sched,
			      unsigned int queue_idx,
			      struct edgx_sched_tr_rate *tr_rate)
{
	if (!sched || !tr_rate)
		return -EINVAL;

	if (queue_idx >= sched->com->nr_queues)
		return -EINVAL;

	mutex_lock(&sched->lock);
	edgx_sched_get_trans_rate(sched, queue_idx, tr_rate);
	mutex_unlock(&sched->lock);
	return 0;
}

static ssize_t current_time_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct timespec64 ts;
	struct edgx_time *time = edgx_pt_get_time(edgx_dev2pt(dev));

	edgx_tm_get_wrk_time(time, &ts);
	return scnprintf(buf, PAGE_SIZE, "%lli.%09li\n",
			 (long long)ts.tv_sec, ts.tv_nsec);
}

static ssize_t gate_enabled_show(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	ret = scnprintf(buf, PAGE_SIZE, "%u\n", sched->gate_enabled);
	mutex_unlock(&sched->lock);

	return ret;
}

static ssize_t gate_enabled_store(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf,
				  size_t count)
{
	int ret = 0;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);

	if (kstrtobool(buf, &sched->gate_enabled))
		ret = -EINVAL;
	else if (sched->gate_enabled)
		ret = edgx_sched_stm_gate_enable(sched);
	else
		edgx_sched_stm_gate_disable(sched);

	mutex_unlock(&sched->lock);

	return ret ? ret : count;
}

static ssize_t admin_gate_states_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	ret = scnprintf(buf, PAGE_SIZE, "0x%X\n", sched->admin_tab.gate_states);
	mutex_unlock(&sched->lock);

	return ret;
}

static ssize_t admin_gate_states_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t count)
{
	ssize_t ret = -EINVAL;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	if (!kstrtou8(buf, 0, &sched->admin_tab.gate_states))
		ret = count;
	sched->admin_tab.gate_states &= sched->com->gate_st_msk;
	mutex_unlock(&sched->lock);

	return ret;
}

static ssize_t max_sdu_tab_read(struct file *filp, struct kobject *kobj,
				struct bin_attribute *bin_attr,
				char *buf, loff_t ofs, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct edgx_sched *sched = edgx_dev2sched(dev);
	loff_t idx = 0;

	if (edgx_sysfs_tbl_params(ofs, count, sizeof(u32), &idx) ||
	    idx >= sched->com->nr_queues)
		return -EINVAL;

	/* NOTE: FLEXDE-2329: Standard Deviation:
	 * FRAMESIZEx registers does not account for VLAN- and R-TAG
	 */
	*((u32 *)buf) = edgx_pt_get_framesize(sched->parent, idx);

#if defined(EDGX_SCHED_DBG)
	mutex_lock(&sched->lock);
	edgx_sched_dump(sched);
	edgx_sched_dump_tab(&sched->admin_tab, "Admin");
	edgx_sched_dump_tab(&sched->op_tabs[0], "Oper 0");
	edgx_sched_hw_dump(sched->com->iobase, sched->iobase_pt,
			   sched->sched_idx, 0);
	edgx_sched_dump_tab(&sched->op_tabs[1], "Oper 1");
	edgx_sched_hw_dump(sched->com->iobase, sched->iobase_pt,
			   sched->sched_idx, 1);
	mutex_unlock(&sched->lock);
#endif
	return count;
}

static ssize_t max_sdu_tab_write(struct file *filp, struct kobject *kobj,
				 struct bin_attribute *bin_attr,
				 char *buf, loff_t ofs, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct edgx_sched *sched = edgx_dev2sched(dev);
	loff_t idx = 0;
	u32 val = *((u32 *)buf);

	if (edgx_sysfs_tbl_params(ofs, count, sizeof(u32), &idx) ||
	    idx >= sched->com->nr_queues)
		return -EINVAL;

	if (val > EDGX_SCHED_DEF_MAX_SDU)
		return -ERANGE;

	/* NOTE: FLEXDE-2329: Standard Deviation:
	 * FRAMESIZEx registers does not account for VLAN- and R-TAG
	 */
	edgx_pt_set_framesize(sched->parent, idx, val);
	return count;
}

static ssize_t overrun_sdu_tab_read(struct file *filp, struct kobject *kobj,
				    struct bin_attribute *bin_attr,
				    char *buf, loff_t ofs, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct edgx_sched *sched = edgx_dev2sched(dev);
	loff_t idx = 0;

	if (edgx_sysfs_tbl_params(ofs, count, sizeof(statw_t), &idx) ||
	    idx >= sched->com->nr_queues)
		return -EINVAL;

	/* NOTE: FLEXDE-451: Standard Deviation:
	 * The IP provides only one counters per port,
	 * the standard defines one per queue, so 8 per port.
	 */
	*((statw_t *)buf) = edgx_stat_upget(sched->hstat, 0);
	return count;
}

static ssize_t admin_ctrl_list_len_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	ret = scnprintf(buf, PAGE_SIZE, "%u\n", sched->admin_tab.list_len);
	mutex_unlock(&sched->lock);

	return ret;
}

static ssize_t admin_ctrl_list_len_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf,
					 size_t count)
{
	u32 list_len;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	if (kstrtou32(buf, 10, &list_len))
		return -EINVAL;

	if (list_len > sched->com->max_entry_cnt)
		return -ERANGE;

	mutex_lock(&sched->lock);
	sched->admin_tab.list_len = list_len;
	mutex_unlock(&sched->lock);

	return count;
}

static ssize_t oper_ctrl_list_len_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	ssize_t ret = -ENOENT;
	int row_cnt;
	struct edgx_sched *sched = edgx_dev2sched(dev);
	int op_tab_idx;

	mutex_lock(&sched->lock);
	op_tab_idx = edgx_sched_hw_get_used_tab(sched->iobase_pt);

	if (op_tab_idx >= 0) {
		row_cnt = sched->op_tabs[op_tab_idx].list_len;
		ret = scnprintf(buf, PAGE_SIZE, "%u\n", row_cnt);
	}
	mutex_unlock(&sched->lock);

	return ret;
}

static ssize_t admin_ctrl_list_read(struct file *filp, struct kobject *kobj,
				    struct bin_attribute *bin_attr,
				    char *buf, loff_t ofs, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct edgx_sched *sched = edgx_dev2sched(dev);
	struct edgx_sched_tab_entry *ent = (struct edgx_sched_tab_entry *)buf;
	struct edgx_sched_tab_entry undef_entry = {0, 0xff, 0, 0};
	loff_t idx, nmax;
	size_t nelems;
	int i, ret = 0;

	mutex_lock(&sched->lock);

	if (!sched->admin_tab.entry_cnt)
		ret = -ENOENT;

	if (!ret)
		ret = edgx_sysfs_list_params(ofs, count,
					     sizeof(*sched->admin_tab.entries),
					     &idx, &nelems);
	if (!ret) {
		nmax = min((u32)(idx + nelems), sched->admin_tab.entry_cnt);
		for (i = 0; idx < nmax; i++, idx++)
			ent[i] = sched->admin_tab.entries[idx];

		for (; i < nelems; i++)
			ent[i] = undef_entry;
		ret = i * sizeof(struct edgx_sched_tab_entry);
	}
	mutex_unlock(&sched->lock);

	return ret;
}

static ssize_t edgx_sched_set_cl_entry(struct edgx_sched *sched,
				       loff_t ofs, size_t count,
				       struct edgx_sched_tab_entry *entry)
{
	struct edgx_sched_tab_entry *new_entries;
	loff_t idx;
	size_t nelems;
	int i;
	ssize_t ret;

	if (edgx_sysfs_list_params(ofs, count,
				   sizeof(*sched->admin_tab.entries),
				   &idx, &nelems))
		return -EINVAL;

	if ((idx + nelems) > sched->com->max_entry_cnt)
		return -EFBIG;

	/* Clear table on entry[0] access */
	if (idx == 0) {
		sched->admin_tab.entry_cnt = 0;
		kfree(sched->admin_tab.entries);
		sched->admin_tab.entries = 0;
	}

	/* Enforce consecutive write */
	if (idx != sched->admin_tab.entry_cnt)
		return -EINVAL;

	new_entries = krealloc(sched->admin_tab.entries,
			       (idx + nelems) *
			       sizeof(struct edgx_sched_tab_entry),
			       GFP_KERNEL);
	if (!new_entries)
		return -ENOMEM;

	sched->admin_tab.entries = new_entries;
	sched->admin_tab.entry_cnt = idx + nelems;
	for (i = 0; i < nelems; i++, idx++)
		sched->admin_tab.entries[idx] = entry[i];

	ret = edgx_sched_adjust_admin_tab(sched);

	return ret ? ret : (nelems * sizeof(struct edgx_sched_tab_entry));
}

static ssize_t admin_ctrl_list_write(struct file *filp, struct kobject *kobj,
				     struct bin_attribute *bin_attr,
				     char *buf, loff_t ofs, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct edgx_sched *sched = edgx_dev2sched(dev);
	ssize_t ret;

	mutex_lock(&sched->lock);
	ret = edgx_sched_set_cl_entry(sched, ofs, count,
				      (struct edgx_sched_tab_entry *)buf);
	mutex_unlock(&sched->lock);
	return ret;
}

static ssize_t edgx_sched_get_op_entry(struct edgx_sched *sched,
				       loff_t ofs, size_t count,
				       struct edgx_sched_tab_entry *entry)
{
	u16 int_clk;
	int tab_idx = edgx_sched_hw_get_used_tab(sched->iobase_pt);
	struct edgx_sched_tab_entry undef_entry = {0, 0xff, 0, 0};
	loff_t idx, nmax;
	size_t nelems;
	int i;
	u16 hw_gate_states, new_hold, hold = 0;

	if ((tab_idx < 0) || (!sched->op_tabs[tab_idx].list_len))
		return -ENOENT;

	if (edgx_sysfs_list_params(ofs, count,
				   sizeof(struct edgx_sched_tab_entry),
				   &idx, &nelems))
		return -EINVAL;

	nmax = min((u32)(idx + nelems), sched->op_tabs[tab_idx].list_len);
	for (i = 0; idx < nmax; i++, idx++) {
		if (edgx_sched_hw_read_entry(sched->com->iobase,
					     sched->sched_idx, tab_idx, idx,
					     &hw_gate_states,
					     &int_clk))
			return -EBUSY;
		entry[i].time_interval = (u32)int_clk * sched->com->ns_per_clk;
		entry[i].gate_states = hw_gate_states & sched->com->gate_st_msk;
		new_hold = hw_gate_states & edgx_sched_get_hold(sched);
		if (!hold && new_hold)
			entry[i].operation_name = EDGX_SCHED_OP_HOLD_MAC;
		else if (hold && !new_hold)
			entry[i].operation_name = EDGX_SCHED_OP_REL_MAC;
		else
			entry[i].operation_name = EDGX_SCHED_OP_SET_GSTATES;
		hold = new_hold;
	}
	for (; i < nelems; i++)
		entry[i] = undef_entry;

	return i * sizeof(struct edgx_sched_tab_entry);
}

static ssize_t oper_ctrl_list_read(struct file *filp, struct kobject *kobj,
				   struct bin_attribute *bin_attr,
				   char *buf, loff_t ofs, size_t count)
{
	struct device *dev = kobj_to_dev(kobj);
	struct edgx_sched *sched = edgx_dev2sched(dev);
	ssize_t ret;

	mutex_lock(&sched->lock);
	ret = edgx_sched_get_op_entry(sched, ofs, count,
				      (struct edgx_sched_tab_entry *)buf);
	mutex_unlock(&sched->lock);

	return ret;
}

static ssize_t admin_cycle_time_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	ret = scnprintf(buf, PAGE_SIZE, "%u/%u\n",
			sched->admin_tab.cycle_time_num,
			sched->admin_tab.cycle_time_denom);
	mutex_unlock(&sched->lock);

	return ret;
}

static ssize_t admin_cycle_time_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf,
				      size_t count)
{
	int ret;
	u32 num, denom;
	u64 ct_nsec;
	u32 ct_subnsec;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	ret = sscanf(buf, "%u/%u", &num, &denom);

	if (ret == 2U) {
		if (denom) {
			edgx_sched_rational_to_nsec(num, denom,
						    &ct_nsec, &ct_subnsec);

			if (ct_nsec <= sched->com->max_cyc_time_ns) {
				sched->admin_tab.cycle_time_num = num;
				sched->admin_tab.cycle_time_denom = denom;
				ret = count;
			} else {
				sched->conf_change_err++;
				ret = -ERANGE;
			}
		} else {
			ret = -EINVAL;
		}
	}

	mutex_unlock(&sched->lock);
	return (ssize_t)ret;
}

static ssize_t oper_cycle_time_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	ssize_t ret = -ENOENT;
	struct edgx_sched *sched = edgx_dev2sched(dev);
	int tab_idx;

	mutex_lock(&sched->lock);
	tab_idx = edgx_sched_hw_get_used_tab(sched->iobase_pt);

	if (tab_idx >= 0)
		ret = scnprintf(buf, PAGE_SIZE, "%u/%u\n",
				sched->op_tabs[tab_idx].cycle_time_num,
				sched->op_tabs[tab_idx].cycle_time_denom);

	mutex_unlock(&sched->lock);
	return ret;
}

static ssize_t admin_cycle_time_ext_show(struct device *dev,
					 struct device_attribute *attr,
					 char *buf)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	ret = scnprintf(buf, PAGE_SIZE, "%u\n",
			sched->admin_tab.cycle_time_ext);
	mutex_unlock(&sched->lock);

	return ret;
}

static ssize_t admin_cycle_time_ext_store(struct device *dev,
					  struct device_attribute *attr,
					  const char *buf,
					  size_t count)
{
	ssize_t ret = -EINVAL;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	if (!kstrtou32(buf, 10, &sched->admin_tab.cycle_time_ext))
		ret = count;
	mutex_unlock(&sched->lock);

	return ret;
}

static ssize_t oper_cycle_time_ext_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t ret = -ENOENT;
	struct edgx_sched *sched = edgx_dev2sched(dev);
	int op_tab_idx;

	mutex_lock(&sched->lock);
	op_tab_idx = edgx_sched_hw_get_used_tab(sched->iobase_pt);

	if (op_tab_idx >= 0)
		ret = scnprintf(buf, PAGE_SIZE, "%u\n",
				sched->op_tabs[op_tab_idx].cycle_time_ext);

	mutex_unlock(&sched->lock);
	return ret;
}

static ssize_t admin_base_time_show(struct device *dev,
				    struct device_attribute *attr,
				    char *buf)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	ret = scnprintf(buf, PAGE_SIZE, "%lli.%09li\n",
			(long long)sched->admin_tab.base_time.tv_sec,
			sched->admin_tab.base_time.tv_nsec);

	mutex_unlock(&sched->lock);
	return ret;
}

static ssize_t admin_base_time_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf,
				     size_t count)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	ret = edgx_sched_strtotimespec64(buf, count,
					 &sched->admin_tab.base_time);

	mutex_unlock(&sched->lock);
	return ret;
}

static ssize_t oper_base_time_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	ssize_t ret = -ENOENT;
	struct edgx_sched *sched = edgx_dev2sched(dev);
	int tab_idx;
	struct edgx_sched_table *op;

	mutex_lock(&sched->lock);
	tab_idx = edgx_sched_hw_get_used_tab(sched->iobase_pt);

	if (tab_idx >= 0) {
		op = &sched->op_tabs[tab_idx];
		ret = scnprintf(buf, PAGE_SIZE, "%lli.%09li\n",
				(long long)op->base_time.tv_sec,
				op->base_time.tv_nsec);
	}
	mutex_unlock(&sched->lock);
	return ret;
}

static ssize_t config_change_show(struct device *dev,
				  struct device_attribute *attr,
				  char *buf)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	ret = scnprintf(buf, PAGE_SIZE, "%u\n", sched->config_change);
	mutex_unlock(&sched->lock);

	return ret;
}

static ssize_t config_change_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf,
				   size_t count)
{
	int ret = -EINVAL;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	if (!kstrtobool(buf, &sched->config_change))
		ret = edgx_sched_stm_cfg_change(sched);

	mutex_unlock(&sched->lock);
	return ret ? ret : count;
}

static ssize_t config_change_time_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	ret = scnprintf(buf, PAGE_SIZE, "%lli.%09li\n",
			(long long)sched->conf_change_time.tv_sec,
			sched->conf_change_time.tv_nsec);
	mutex_unlock(&sched->lock);
	return ret;
}

static ssize_t tick_granul_show(struct device *dev,
				struct device_attribute *attr,
				char *buf)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);

	sched->link_mode = edgx_pt_get_speed(sched->parent);

	/* Report in 0.1ns units: */
	switch (sched->link_mode) {
	case SPEED_10:
		ret = scnprintf(buf, PAGE_SIZE, "10000000\n");
		break;
	case SPEED_100:
		ret = scnprintf(buf, PAGE_SIZE, "1000000\n");
		break;
	case SPEED_1000:
		ret = scnprintf(buf, PAGE_SIZE, "100000\n");
		break;
	default:
		ret = scnprintf(buf, PAGE_SIZE, "UNKNOWN\n");
	}

	mutex_unlock(&sched->lock);
	return ret;
}

static ssize_t config_pending_show(struct device *dev,
				   struct device_attribute *attr,
				   char *buf)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	ret = scnprintf(buf, PAGE_SIZE, "%u\n", sched->state);
	mutex_unlock(&sched->lock);
	return ret;
}

static ssize_t config_change_error_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	ret = scnprintf(buf, PAGE_SIZE, "%llu\n", sched->conf_change_err);
	mutex_unlock(&sched->lock);

	return ret;
}

static ssize_t supported_list_max_show(struct device *dev,
				       struct device_attribute *attr,
				       char *buf)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);

	mutex_lock(&sched->lock);
	ret = scnprintf(buf, PAGE_SIZE, "%u\n", sched->com->max_entry_cnt);
	mutex_unlock(&sched->lock);
	return ret;
}

static ssize_t supported_cyc_max_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);
	u32 sec_num, sec_denom;

	mutex_lock(&sched->lock);
	edgx_sched_nsec_to_rational(sched->com->max_cyc_time_ns, 0,
				    &sec_num, &sec_denom);
	ret = scnprintf(buf, PAGE_SIZE, "%u/%u\n", sec_num, sec_denom);
	mutex_unlock(&sched->lock);

	return ret;
}

static ssize_t supported_int_max_show(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	ssize_t ret;
	struct edgx_sched *sched = edgx_dev2sched(dev);
	u32 max_int = (u32)sched->com->ns_per_clk * EDGX_SCHED_INTERVAL_MAX;

	mutex_lock(&sched->lock);
	ret = scnprintf(buf, PAGE_SIZE, "%u\n", max_int);
	mutex_unlock(&sched->lock);

	return ret;
}

EDGX_DEV_ATTR_RW(gate_enabled,         "GateEnabled");
EDGX_DEV_ATTR_RW(admin_gate_states,    "AdminGateStates");
EDGX_DEV_ATTR_RW(admin_ctrl_list_len,  "AdminControlListLength");
EDGX_DEV_ATTR_RO(oper_ctrl_list_len,   "OperControlListLength");
EDGX_DEV_ATTR_RW(admin_cycle_time,     "AdminCycleTime");
EDGX_DEV_ATTR_RO(oper_cycle_time,      "OperCycleTime");
EDGX_DEV_ATTR_RW(admin_cycle_time_ext, "AdminCycleTimeExtension");
EDGX_DEV_ATTR_RO(oper_cycle_time_ext,  "OperCycleTimeExtension");
EDGX_DEV_ATTR_RW(admin_base_time,      "AdminBaseTime");
EDGX_DEV_ATTR_RO(oper_base_time,       "OperBaseTime");
EDGX_DEV_ATTR_RW(config_change,        "ConfigChange");
EDGX_DEV_ATTR_RO(config_change_time,   "ConfigChangeTime");
EDGX_DEV_ATTR_RO(tick_granul,          "TickGranularity");
EDGX_DEV_ATTR_RO(current_time,         "CurrentTime");
EDGX_DEV_ATTR_RO(config_pending,       "ConfigPending");
EDGX_DEV_ATTR_RO(config_change_error,  "ConfigChangeError");
EDGX_DEV_ATTR_RO(supported_list_max,   "SupportedListMax");
EDGX_DEV_ATTR_RO(supported_cyc_max,    "SupportedCycleMax");
EDGX_DEV_ATTR_RO(supported_int_max,    "SupportedIntervalMax");
EDGX_BIN_ATTR_RW(admin_ctrl_list,      "AdminControlList",
		 EDGX_SCHED_HW_MAX_ROWS * sizeof(struct edgx_sched_tab_entry));
EDGX_BIN_ATTR_RO(oper_ctrl_list,       "OperControlList",
		 EDGX_SCHED_HW_MAX_ROWS * sizeof(struct edgx_sched_tab_entry));
EDGX_BIN_ATTR_RW(max_sdu_tab,          "queueMaxSDUTable",
		 EDGX_SCHED_MAX_QUEUES * sizeof(u32));
EDGX_BIN_ATTR_RO(overrun_sdu_tab,      "transmissionOverrunTable",
		 EDGX_SCHED_MAX_QUEUES * sizeof(statw_t));

static struct attribute_group ieee8021_st_group = {
	.name = "ieee8021ST",
	.attrs = (struct attribute*[]){
		&dev_attr_gate_enabled.attr,
		&dev_attr_admin_gate_states.attr,
		&dev_attr_admin_ctrl_list_len.attr,
		&dev_attr_oper_ctrl_list_len.attr,
		&dev_attr_admin_cycle_time.attr,
		&dev_attr_oper_cycle_time.attr,
		&dev_attr_admin_cycle_time_ext.attr,
		&dev_attr_oper_cycle_time_ext.attr,
		&dev_attr_admin_base_time.attr,
		&dev_attr_oper_base_time.attr,
		&dev_attr_config_change.attr,
		&dev_attr_config_change_time.attr,
		&dev_attr_tick_granul.attr,
		&dev_attr_current_time.attr,
		&dev_attr_config_pending.attr,
		&dev_attr_config_change_error.attr,
		&dev_attr_supported_list_max.attr,
		&dev_attr_supported_cyc_max.attr,
		&dev_attr_supported_int_max.attr,
		NULL
	},
	.bin_attrs = (struct bin_attribute*[]){
		&bin_attr_admin_ctrl_list,
		&bin_attr_oper_ctrl_list,
		&bin_attr_max_sdu_tab,
		&bin_attr_overrun_sdu_tab,
		NULL
	}
};

/* NOTE: The current FSC HW ITF is not completely separated on port basis.
 * There is a common part, a generic part and a port specific part.
 * Therefore edgx_ac_get_ptif() cannot be used.
 */

/* NOTE: Bug in AC: On IP 4.7 the AC block returns FSC resource size 0x10000.
 * The real resource size however is 0x15000 for 4 (+1) ports.
 */

/* NOTE: Missing from the Manual: On IP 4.7 the port number matches
 * the scheduler number. On older IPs, the first available scheduler
 * matches the first available port, that supports scheduling.
 */

int edgx_probe_sched(struct edgx_pt *pt, struct edgx_sched_com *sched_com,
		     struct edgx_sched **psched)
{
	const struct edgx_ifdesc *ifd_pt;
	struct edgx_ifdesc        ptifd;
	ptid_t                    ptid = edgx_pt_get_id(pt);
	struct edgx_stat         *sm = edgx_br_get_pt_stat(edgx_pt_get_br(pt),
							   ptid);
	struct edgx_time *time = edgx_pt_get_time(pt);

	if (!pt || !sched_com || !psched || !time)
		return -EINVAL;

	/* Get the port specific part of the HW ITF */
	ifd_pt = edgx_sched_if2ptif(sched_com->ifd_com, ptid, &ptifd);
	if (!ifd_pt)
		return -ENODEV;

	*psched = kzalloc(sizeof(**psched), GFP_KERNEL);
	if (!(*psched)) {
		edgx_pt_err(pt, "Cannot allocate Scheduled Traffic\n");
		return -ENOMEM;
	}

	if (edgx_sched_init(*psched, pt, sched_com, ifd_pt->iobase))
		goto out_sched_init;

	(*psched)->hstat = edgx_stat_alloc_hdl(sm, ptid, &_st_statinfo);

	edgx_pt_add_sysfs(pt, &ieee8021_st_group);
	sched_com->sched_list[ptid] = *psched;

	return 0;

out_sched_init:
	kfree(*psched);
	return -ENODEV;
}

void edgx_shutdown_sched(struct edgx_sched *sched)
{
	if (sched) {
		edgx_pt_rem_sysfs(sched->parent, &ieee8021_st_group);
		edgx_stat_free_hdl(sched->hstat);
		edgx_sched_hw_disable(sched->iobase_pt,
				      sched->admin_tab.gate_states);
		kfree(sched->admin_tab.entries);
		kfree(sched);
	}
}
