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

#include <linux/kernel.h>
#include <linux/mdio.h>
#include <linux/phy.h>
#include <linux/of_mdio.h>
#include "edge_mdio.h"

#define REG_ADR		0x10
#define REG_WRDATA	0x20
#define REG_CMD		0x30
#define REG_RDDATA	0x40

#define PHY_ADDR_MAGIC	0x21
#define CMD_WRITE	1
#define CMD_READ	2

struct edgx_mdio {
	struct list_head  entry;
	struct mii_bus   *mii;
	void             *base;
	struct mutex      lock; /* MDIO read/write access lock */
	unsigned int      id;
};

static int __mdio_write(struct mii_bus *bus, int addr, u16 val)
{
	struct edgx_mdio *mdio = bus->priv;

	iowrite32(addr, mdio->base + REG_ADR);
	iowrite32(val, mdio->base + REG_WRDATA);
	iowrite32(CMD_WRITE, mdio->base + REG_CMD);
	while (ioread32(mdio->base + REG_CMD))
		cpu_relax();

	return 0;
}

static int __mdio_read(struct mii_bus *bus, int addr)
{
	struct edgx_mdio *mdio = bus->priv;

	iowrite32(addr, mdio->base + REG_ADR);
	iowrite32(CMD_READ, mdio->base + REG_CMD);
	while (ioread32(mdio->base + REG_CMD))
		cpu_relax();
	return ioread32(mdio->base + REG_RDDATA) & 0xffff;
}

static int __mdio_set_phy_addr(struct mii_bus *bus, int phy_id)
{
	return __mdio_write(bus, PHY_ADDR_MAGIC, phy_id);
}

static int edgx_mdio_read(struct mii_bus *bus, int addr, int regnum)
{
	struct edgx_mdio *mdio = bus->priv;
	int ret;

	mutex_lock(&mdio->lock);
	__mdio_set_phy_addr(bus, addr);
	ret = __mdio_read(bus, regnum);
	mutex_unlock(&mdio->lock);

	return ret;
}

int edgx_mdio_write(struct mii_bus *bus, int addr, int regnum, u16 val)
{
	struct edgx_mdio *mdio = bus->priv;
	int ret;

	mutex_lock(&mdio->lock);
	__mdio_set_phy_addr(bus, addr);
	ret = __mdio_write(bus, regnum, val);
	mutex_unlock(&mdio->lock);

	return ret;
}

int edgx_mdio_probe_one(unsigned int mdio_id, struct device *dev, void *base,
			struct edgx_mdio **mdio_ret)
{
	int ret = -ENOMEM;
	struct edgx_mdio *mdio = kzalloc(sizeof(*mdio), GFP_KERNEL);
	struct device_node *np = dev->of_node;

	if (!mdio)
		return -ENOMEM;

	mdio->mii = mdiobus_alloc();
	if (!mdio->mii)
		goto out_bus;

	mdio->id   = mdio_id;
	mdio->base = base;
	mutex_init(&mdio->lock);

	mdio->mii->phy_mask = 0;
	mdio->mii->read     = &edgx_mdio_read;
	mdio->mii->write    = &edgx_mdio_write;
	mdio->mii->parent   = dev;
	mdio->mii->priv     = mdio;
	mdio->mii->name     = "edgx_mdio";
	snprintf(mdio->mii->id, MII_BUS_ID_SIZE, "%s-%x",
		 mdio->mii->name, mdio_id + 1);

	if (np)
		/*
		 * try devicetree based phy registration to provide possible
		 * devicetree configuration data to the phys
		 */
		ret = of_mdiobus_register(mdio->mii, np);
	else
		ret = mdiobus_register(mdio->mii);

	if (ret)
		goto out_reg;

	*mdio_ret = mdio;
	return 0;

out_reg:
	mdiobus_free(mdio->mii);
out_bus:
	kfree(mdio);
	return ret;
}

void edgx_mdio_shutdown(struct edgx_mdio *mdio)
{
	if (!mdio)
		return;

	mdiobus_unregister(mdio->mii);
	mdiobus_free(mdio->mii);
	kfree(mdio);
}

void *edgx_mdio_get_base(struct edgx_mdio *mdio)
{
	if (!mdio)
		return NULL;

	return mdio->base;
}

const char *edgx_mdio_get_id(struct edgx_mdio *mdio)
{
	return mdio->mii->id;
}
