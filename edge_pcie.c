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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <linux/phy.h>
#include "edge_bridge.h"
#include "edge_port.h"
#include "edge_link.h"
#include "edge_mdio.h"
#include "altera_pio.h"
#include "tsnic_vpd.h"

#define PFX			"EDGX-PCIe: "
#define DRV_NAME		"edgx-pcie"
#define EDGX_PCI_BR_BAR		0
#define EDGX_PCI_BR_OFFS	0x0000000
#define EDGX_PCI_BR_SIZE	0x1ffffff
#define EDGX_PCI_MDIO_BAR	0
#define EDGX_PCI_MDIO_OFFS	0x2000000
#define EDGX_PCI_MDIO_SIZE	0x3ff
#define EDGX_PCI_PIO_BAR	2
#define EDGX_PCI_PIO_OFFS	0xf0f00
#define EDGX_PCI_PIO_SIZE	0x1f

struct edgx_pci_drv {
	struct edgx_br *br;
	struct edgx_mdio *mdio;
	struct flx_pio_dev_priv *pio;
	struct edgx_br_irq irq;
	void *i2c_base;
	struct vpd *vpd;
};

static const struct pci_device_id edgx_pci_ids[] = {
	{ PCI_DEVICE(0x1059, 0xa100) },
	{}
};

static int bridge_id;
static int mdio_id;
static int pio_id;

static int edgx_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id);
static void edgx_pci_remove(struct pci_dev *pdev);

static struct pci_driver edgx_pci_driver = {
	.name = "edgx-pci",
	.id_table = edgx_pci_ids,
	.probe = &edgx_pci_probe,
	.remove = &edgx_pci_remove,
};

static int edgx_pci_get_irq(struct pci_dev *pdev, struct edgx_br_irq *irq)
{
	int ret, i;
	u8 *cra_base;

	/* Avalon-MM to PCI Express Interrupt Status Enable Register setting */
	cra_base = pci_iomap_range(pdev, 0, 0, 0x3fff);
	if (!cra_base) {
		dev_err(&pdev->dev, "Cannot map CRA device memory.\n");
		return -ENOMEM;
	}
	*((u16 *)&cra_base[0x50]) = 0x2;
	pci_iounmap(pdev, cra_base);

	ret = pci_alloc_irq_vectors(pdev, EDGX_IRQ_CNT, EDGX_IRQ_CNT,
				    PCI_IRQ_MSI | PCI_IRQ_MSIX);
	if (ret < 0)
		return ret;

	if (ret != EDGX_IRQ_CNT) {
		dev_err(&pdev->dev, "Invalid IRQ count = %d!\n", ret);
		pci_free_irq_vectors(pdev);
		return -ENODEV;
	}

	irq->shared = false;
	irq->trig = EDGX_IRQ_EDGE_TRIG;

	for (i = 0; i < ret; i++) {
		irq->irq_vec[i] = pci_irq_vector(pdev, i);
		if (irq->irq_vec[i] < 0) {
			dev_err(&pdev->dev, "IRQ vector %d failed!\n", i);
			pci_free_irq_vectors(pdev);
			return ret;
		}
		dev_info(&pdev->dev, "IRQ-vector %d: %d\n", i, irq->irq_vec[i]);
	}

	return 0;
}

static int edgx_pci_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	int ret, i;
	void *br_base;
	void *mdio_base;
	void *pio_base;
	struct edgx_pci_drv *pci_drv;
	const char *mdio_bus_id;
	char mdio_bus_id_pt[MII_BUS_ID_SIZE + 4];
	struct edgx_pt *pt;
	struct edgx_link *lnk;
	void *i2c_base;
	char asset[TSNIC_SNO_LEN + 1];

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "Cannot enable PCI device.\n");
		goto probe_out_enable;
	}

	pci_set_master(pdev);
	pr_info("pci_try_set_mwi: %d\n", pci_try_set_mwi(pdev));

#if 0
	if (!(pci_resource_flags(pdev, 0) & IORESOURCE_MEM)) {
		dev_err(&pdev->dev, "Invalid BAR0 resource flags.\n");
		ret = -ENODEV;
		goto probe_out_res_flag;
	}

	if (!(pci_resource_flags(pdev, 1) & IORESOURCE_MEM)) {
		dev_err(&pdev->dev, "Invalid BAR1 resource flags.\n");
		ret = -ENODEV;
		goto probe_out_res_flag;
	}

	if (!(pci_resource_flags(pdev, 2) & IORESOURCE_MEM)) {
		dev_err(&pdev->dev, "Invalid BAR2 resource flags.\n");
		ret = -ENODEV;
		goto probe_out_res_flag;
	}
#endif

	ret = pci_request_regions(pdev, DRV_NAME);
	if (ret) {
		dev_err(&pdev->dev, "Cannot get PCI resource.\n");
		goto probe_out_res_flag;
	}

	br_base = pci_iomap_range(pdev, EDGX_PCI_BR_BAR,
				  EDGX_PCI_BR_OFFS, EDGX_PCI_BR_SIZE);
	if (!br_base) {
		dev_err(&pdev->dev, "Cannot map bridge device memory.\n");
		ret = -ENOMEM;
		goto probe_out_iomap_br;
	}

	mdio_base = pci_iomap_range(pdev, EDGX_PCI_MDIO_BAR,
				    EDGX_PCI_MDIO_OFFS, EDGX_PCI_MDIO_SIZE);
	if (!mdio_base) {
		dev_err(&pdev->dev, "Cannot map MDIO device memory.\n");
		ret = -ENOMEM;
		goto probe_out_iomap_mdio;
	}

	if (0 && IS_ENABLED(CONFIG_GPIOLIB)) {
		pio_base = pci_iomap_range(pdev, EDGX_PCI_PIO_BAR,
					   EDGX_PCI_PIO_OFFS, EDGX_PCI_PIO_SIZE);
		if (!pio_base) {
			dev_err(&pdev->dev, "Cannot map PIO device memory.\n");
			ret = -ENOMEM;
			goto probe_out_iomap_pio;
		}
	}

	i2c_base = pci_iomap(pdev, 5, 0);
	if (!i2c_base) {
		dev_err(&pdev->dev, "Cannot map I2C device memory.\n");
		ret = -ENOMEM;
		goto probe_out_iomap_i2c;
	}

	pci_drv = kzalloc(sizeof(*pci_drv), GFP_KERNEL);
	if (!pci_drv) {
		ret = -ENOMEM;
		goto probe_out_drv_alloc;
	}
	pci_drv->i2c_base = i2c_base;

	pci_drv->vpd = tsnic_vpd_init(i2c_base);
	if (IS_ERR(pci_drv->vpd)) {
		ret = PTR_ERR(pci_drv->vpd);
		goto vpd_init_out;
	}

	ret = edgx_pci_get_irq(pdev, &pci_drv->irq);
	if (ret)
		goto probe_out_irq;

	if (0 && IS_ENABLED(CONFIG_GPIOLIB)) {
		ret = flx_pio_probe_one(pio_id, &pdev->dev, pio_base,
					&pci_drv->pio);
		if (ret)
			goto probe_out_pio;
		pio_id++;
	}

	ret = edgx_mdio_probe_one(mdio_id, &pdev->dev, mdio_base,
				  &pci_drv->mdio);
	if (ret)
		goto probe_out_mdio;
	mdio_id++;

	ret = edgx_br_probe_one(bridge_id, &pdev->dev, br_base, &pci_drv->irq,
				&pci_drv->br, pci_drv->vpd);
	if (ret)
		goto probe_out_bridge;
	bridge_id++;

	dev_set_drvdata(&pdev->dev, pci_drv);

	mdio_bus_id = edgx_mdio_get_id(pci_drv->mdio);
	for (i = 0; i < 3; i++) {
		pt =  edgx_br_get_brpt(pci_drv->br, i + 1);
		if (!pt) {
			dev_err(&pdev->dev, "Cannot get port %d.\n", i + 1);
			goto probe_out_link;
		}

		lnk = edgx_pt_get_link(pt);
		if (!pt) {
			dev_err(&pdev->dev, "Cannot get port %d link.\n",
				i + 1);
			goto probe_out_link;
		}

		snprintf(mdio_bus_id_pt, MII_BUS_ID_SIZE, "%s:%02d",
			 mdio_bus_id, i);

		if (edgx_link_set_mdiobus(lnk, mdio_bus_id_pt)) {
			dev_err(&pdev->dev, "Cannot set port %d link.\n",
				i + 1);
			goto probe_out_link;
		}

		edgx_link_set_delays(lnk, 6258, 679, 153, 2178, 362, 235);
	}

	tsnic_vpd_asset_tag(pci_drv->vpd, asset, sizeof(asset));
    dev_info(&pdev->dev, "Serial number - %s.\n", asset);

	return ret;

probe_out_link:
	edgx_br_shutdown(pci_drv->br);
	bridge_id--;
probe_out_bridge:
	edgx_mdio_shutdown(pci_drv->mdio);
	mdio_id--;
probe_out_mdio:
	if (0 && IS_ENABLED(CONFIG_GPIOLIB)) {
		flx_pio_shutdown(pci_drv->pio);
		pio_id--;
	}
probe_out_pio:
	pci_free_irq_vectors(pdev);
probe_out_irq:
	kfree(pci_drv);
vpd_init_out:
probe_out_drv_alloc:
	pci_iounmap(pdev, i2c_base);
probe_out_iomap_i2c:
	if (0 && IS_ENABLED(CONFIG_GPIOLIB))
		pci_iounmap(pdev, pio_base);
probe_out_iomap_pio:
	pci_iounmap(pdev, mdio_base);
probe_out_iomap_mdio:
	pci_iounmap(pdev, br_base);
probe_out_iomap_br:
	pci_release_regions(pdev);
probe_out_res_flag:
	pci_disable_device(pdev);
probe_out_enable:
	return ret;
}

static void edgx_pci_remove(struct pci_dev *pdev)
{
	struct edgx_pci_drv *pci_drv = dev_get_drvdata(&pdev->dev);
	void *br_base;
	void *mdio_base;
	void *pio_base;

	if (!pci_drv)
		return;

	br_base = edgx_br_get_base(pci_drv->br);
	mdio_base = edgx_mdio_get_base(pci_drv->mdio);
	if (0 && IS_ENABLED(CONFIG_GPIOLIB))
		pio_base = flx_pio_get_base(pci_drv->pio);

	edgx_br_shutdown(pci_drv->br);
	edgx_mdio_shutdown(pci_drv->mdio);
	if (0 && IS_ENABLED(CONFIG_GPIOLIB))
		flx_pio_shutdown(pci_drv->pio);
	tsnic_vpd_free(pci_drv->vpd);
	pci_free_irq_vectors(pdev);
	kfree(pci_drv);
	if (0 && IS_ENABLED(CONFIG_GPIOLIB))
		pci_iounmap(pdev, pio_base);
	pci_iounmap(pdev, mdio_base);
	pci_iounmap(pdev, br_base);
	pci_iounmap(pdev, pci_drv->i2c_base);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

static int __init edgx_pcie_module_init(void)
{
	return pci_register_driver(&edgx_pci_driver);
}

static void __exit edgx_pcie_module_exit(void)
{
	pci_unregister_driver(&edgx_pci_driver);
}

module_init(edgx_pcie_module_init);
module_exit(edgx_pcie_module_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("TTTech Industrial Automation AG <support@tttech-industrial.com>");
MODULE_DESCRIPTION("EDGE PCIe Host Interface Driver");
MODULE_DEVICE_TABLE(pci, edgx_pci_ids);
