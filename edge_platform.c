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

#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include "edge_bridge.h"
#include "edge_mdio.h"
#include "altera_pio.h"

#define PFX "EDGX-PLATFORM: "

static const struct of_device_id flx_pio_dt_ids[] = {
	{ .compatible = "flx,pio" },
	{ /* sentinel */ },
};

static const struct of_device_id edgx_sw_dt_ids[] = {
	{ .compatible = "ttt,deip-sw",
	},
	{ .compatible = "ttt,es",
	},
	{ /* sentinel */ },
};

static const struct of_device_id edgx_mdio_dt_ids[] = {
	{ .compatible = "flx,eth-mdio",
	},
	{ /* sentinel */ },
};

static int bridge_id;
static int mdio_id;
static int pio_id;

static int flx_pio_pfm_probe(struct platform_device *pdev);
static int flx_pio_pfm_remove(struct platform_device *pdev);
static int edgx_sw_pfm_probe(struct platform_device *pdev);
static int edgx_sw_pfm_remove(struct platform_device *pdev);
static int edgx_mdio_pfm_probe(struct platform_device *pdev);
static int edgx_mdio_pfm_remove(struct platform_device *pdev);

static struct platform_driver flx_pio_pfm_driver = {
	.driver = {
		.name = "flx-pio",
		.of_match_table = flx_pio_dt_ids,
	},
	.probe = &flx_pio_pfm_probe,
	.remove = &flx_pio_pfm_remove,
};

static struct platform_driver edgx_sw_pfm_driver = {
	.driver = {
		.name = "edgx-pfm",
		.of_match_table = edgx_sw_dt_ids,
	},
	.probe  = edgx_sw_pfm_probe,
	.remove = edgx_sw_pfm_remove,
};

static struct platform_driver edgx_mdio_pfm_driver = {
	.driver = {
		.name = "edgx-mdio",
		.of_match_table = edgx_mdio_dt_ids,
	},
	.probe  = edgx_mdio_pfm_probe,
	.remove = edgx_mdio_pfm_remove,
};

static int edgx_get_mem_res(struct platform_device *pdev, void **base)
{
	struct resource *mem;

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "No I/O memory defined on dev '%s'\n",
			dev_name(&pdev->dev));
		return -ENODEV;
	}

	*base = ioremap_nocache(mem->start, resource_size(mem));
	if (!*base) {
		dev_err(&pdev->dev,
			"I/O remapping failed for dev '%s' 0x%x/0x%x\n",
			dev_name(&pdev->dev), mem->start, resource_size(mem));
		return -ENODEV;
	}

	dev_info(&pdev->dev, "MEM-resource '%s' 0x%x/0x%x -> 0x%p\n",
		 mem->name, mem->start, resource_size(mem), *base);
	return 0;
}

static int flx_pio_pfm_probe(struct platform_device *pdev)
{
	int ret;
	void *base;
	struct flx_pio_dev_priv *pio;

	ret = edgx_get_mem_res(pdev, &base);
	if (ret)
		return ret;

	ret = flx_pio_probe_one(pio_id, &pdev->dev, base, &pio);
	if (ret) {
		iounmap(base);
		return ret;
	}
	dev_set_drvdata(&pdev->dev, pio);
	pio_id++;
	return 0;
}

static int flx_pio_pfm_remove(struct platform_device *pdev)
{
	void *base;
	struct flx_pio_dev_priv *pio;

	pio = dev_get_drvdata(&pdev->dev);
	if (!pio)
		return -ENODEV;

	base = flx_pio_get_base(pio);
	if (!base)
		return -ENODEV;

	flx_pio_shutdown(pio);
	iounmap(base);
	dev_info(&pdev->dev, "Removed device '%s'\n",
		 dev_name(&pdev->dev));

	return 0;
}

static int edgx_sw_pfm_probe(struct platform_device *pdev)
{
	int ret;
	struct resource *irq;
	int irq_nr = -1;
	void *base;
	struct edgx_br *br;

	ret = edgx_get_mem_res(pdev, &base);
	if (ret)
		return ret;

	irq = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
	if (irq) {
		irq_nr = irq->start;
		dev_info(&pdev->dev, "IRQ-resource  0x%p %d\n", irq, irq_nr);
	}

	ret = edgx_br_probe_one(bridge_id, &pdev->dev, base, irq_nr, &br);
	if (ret) {
		iounmap(base);
		return ret;
	}
	dev_set_drvdata(&pdev->dev, br);
	bridge_id++;
	return 0;
}

static int edgx_sw_pfm_remove(struct platform_device *pdev)
{
	void *base;
	struct edgx_br *br;

	br = dev_get_drvdata(&pdev->dev);
	if (!br)
		return -ENODEV;

	base = edgx_br_get_base(br);
	if (!base)
		return -ENODEV;

	edgx_br_shutdown(br);
	iounmap(base);
	dev_info(&pdev->dev, "Removed device '%s'\n",
		 dev_name(&pdev->dev));

	return 0;
}

static int edgx_mdio_pfm_probe(struct platform_device *pdev)
{
	int ret;
	void *base;
	struct edgx_mdio *mdio;

	ret = edgx_get_mem_res(pdev, &base);
	if (ret)
		return ret;

	ret = edgx_mdio_probe_one(mdio_id, &pdev->dev, base, &mdio);
	if (ret) {
		iounmap(base);
		return ret;
	}
	dev_set_drvdata(&pdev->dev, mdio);
	mdio_id++;
	return 0;
}

static int edgx_mdio_pfm_remove(struct platform_device *pdev)
{
	void *base;
	struct edgx_mdio *mdio;

	mdio = dev_get_drvdata(&pdev->dev);
	if (!mdio)
		return -ENODEV;

	base = edgx_mdio_get_base(mdio);
	if (!base)
		return -ENODEV;

	edgx_mdio_shutdown(mdio);
	iounmap(base);
	dev_info(&pdev->dev, "Removed device '%s'\n",
		 dev_name(&pdev->dev));

	return 0;
}

static int __init edgx_pfm_init(void)
{
	int ret;

	pr_info(PFX "Registering pio platform device driver\n");
	ret = platform_driver_register(&flx_pio_pfm_driver);
	if (ret)
		return ret;

	pr_info(PFX "Registering mdio platform device driver\n");
	ret = platform_driver_register(&edgx_mdio_pfm_driver);
	if (ret)
		goto out_pfm_init_mdio;

	pr_info(PFX "Registering switch platform device driver\n");
	ret = platform_driver_register(&edgx_sw_pfm_driver);
	if (ret)
		goto out_pfm_init_sw;

	return ret;

out_pfm_init_sw:
	platform_driver_unregister(&edgx_mdio_pfm_driver);
out_pfm_init_mdio:
	platform_driver_unregister(&flx_pio_pfm_driver);
	return ret;
}

static void __exit edgx_pfm_exit(void)
{
	pr_info(PFX "Unregistering sw platform device driver\n");
	platform_driver_unregister(&edgx_sw_pfm_driver);
	pr_info(PFX "Unregistering mdio platform device driver\n");
	platform_driver_unregister(&edgx_mdio_pfm_driver);
	pr_info(PFX "Unregistering pio platform device driver\n");
	platform_driver_unregister(&flx_pio_pfm_driver);
}

module_init(edgx_pfm_init);
module_exit(edgx_pfm_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("TTTech Industrial Automation AG <support@tttech-industrial.com>");
MODULE_DESCRIPTION("EDGE Platform Host Interface Driver");
MODULE_VERSION(EDGX_SW_CORE_VERSION);

