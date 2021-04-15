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

#include "edge_psfp.h"
#include "edge_util.h"
#include "edge_ac.h"
#include "edge_defines.h"

#define FILTER_CMD 0x500
#define FILTER_TBL_0 0x510
#define FILTER_TBL_1 0x512

#define MAX_FRAME_SIZE 0x7FF
#define DROP_ALL_FRAME_SIZE 59

struct edgx_psfp {
	struct edgx_br *parent;
	edgx_io_t *iobase;
	struct mutex lock;
	int droppcp;
};

static ssize_t pcpdrop_store(struct device *dev,
			     struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct edgx_br *br = edgx_dev2br(dev);
	struct edgx_psfp *psfp = edgx_br_get_psfp(br);
	int pcp;
	uint16_t transfer;
	int i;

	if (kstrtoint(buf, 0, &pcp))
		return -EINVAL;
	if (pcp >= EDGX_BR_NUM_PCP)
		return -EINVAL;

	mutex_lock(&psfp->lock);

	/* first switch off dropping in all PCP's - only one PCP can be
	 * switched on at the same time
	 */
	edgx_wr16(psfp->iobase, FILTER_TBL_0, MAX_FRAME_SIZE);
	edgx_wr16(psfp->iobase, FILTER_TBL_1, 0x0);
	for (i = 0; i < EDGX_BR_NUM_PCP; i++) {
		edgx_wr16(psfp->iobase, FILTER_CMD, 0xC000 | (i << 11));
		do {
			transfer = edgx_get16(psfp->iobase, FILTER_CMD, 15, 15);
		} while (transfer);
	}

	/* when pcp has a value 0 <= pcp < EDGX_BR_NUM_PCP then dropping has
	 * to be switched on for this pcp
	 */
	if (pcp >= 0) {
		edgx_wr16(psfp->iobase, FILTER_TBL_0,
				0xC000 | DROP_ALL_FRAME_SIZE);
		edgx_wr16(psfp->iobase, FILTER_TBL_1, 0x0);
		edgx_wr16(psfp->iobase, FILTER_CMD, 0xC000 | (pcp << 11));
		do {
			transfer = edgx_get16(psfp->iobase, FILTER_CMD, 15, 15);
		} while (transfer);
	}

	/* Store dropped PCP, so that we don't need to read back from HW.
	 * A negative value indicates that dropping is switched of - no
	 * pcp is dropped.
	 */
	psfp->droppcp = pcp;

	mutex_unlock(&psfp->lock);

	return count;
}

static ssize_t pcpdrop_show(struct device *dev,
			    struct device_attribute *attr,
			    char *buf)
{
	struct edgx_psfp *psfp = edgx_br_get_psfp(edgx_dev2br(dev));
	int ret = 0;

	mutex_lock(&psfp->lock);
	ret = scnprintf(buf, PAGE_SIZE, "%d\n", psfp->droppcp);
	mutex_unlock(&psfp->lock);

	return ret;
}

EDGX_DEV_ATTR_RW(pcpdrop, "_pcpdrop");

static struct attribute *ieee8021_psfp_attrs[] = {
	&dev_attr_pcpdrop.attr,
	NULL,
};

static struct attribute_group ieee8021_psfp_group = {
	.name = "ieee8021PSFP",
	.attrs = ieee8021_psfp_attrs,
};

int  edgx_probe_psfp(struct edgx_br *br, struct edgx_psfp **ppsfp)
{
	const struct edgx_ifdesc *ifd;
	const struct edgx_ifreq   ifreq = { .id = AC_PSFP_ID, .v_maj = 0 };
	struct edgx_psfp         *psfp;

	if (!br || !ppsfp)
		return -EINVAL;

	ifd = edgx_ac_get_if(&ifreq);
	if (!ifd)
		return -ENODEV;

	psfp = kzalloc(sizeof(**ppsfp), GFP_KERNEL);
	if (!psfp)
		return -ENOMEM;

	psfp->parent = br;
	psfp->iobase = ifd->iobase;
	psfp->droppcp = -1;

	mutex_init(&psfp->lock);

	if (edgx_br_sysfs_add(br, &ieee8021_psfp_group))
		goto out_sysfs;

	edgx_br_info(br, "Setup Per Stream Filtering and Policing ... done");
	*ppsfp = psfp;
	return 0;

out_sysfs:
	edgx_br_err(br, "Setup Per Stream Filtering and Policing ... failed");
	edgx_shutdown_psfp(*ppsfp);
	return -ENODEV;
}

void edgx_shutdown_psfp(struct edgx_psfp *psfp)
{
	kfree(psfp);
}
