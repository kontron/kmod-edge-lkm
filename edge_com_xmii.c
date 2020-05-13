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

#include <linux/netdevice.h>
#include <linux/rtnetlink.h>
#include <linux/version.h>

#include "edge_ac.h"
#include "edge_com_intern.h"
#include "edge_util.h"
#include "edge_com_ts.h"

struct edgx_com_xmii {
	struct edgx_com		 com;
	struct net_device	*net;
	struct edgx_com_ts	 ts;
};

#define edgx_com_to_xmii(_com) container_of(_com, struct edgx_com_xmii, com)

static netdev_tx_t edgx_com_xmii_xmit(struct edgx_com *com, struct sk_buff *skb,
				      ptcom_t ptcom, ptflags_t flags)
{
	u8                   *skb_trailer;
	netdev_tx_t           ret = NETDEV_TX_OK;
	struct edgx_com_xmii *xmii = edgx_com_to_xmii(com);

	if (skb->len < ETH_ZLEN) {
		unsigned int skb_len_dif = ETH_ZLEN - skb->len;

		ret = skb_pad(skb, skb_len_dif + COM_TRAILER_LEN);
		if (ret)
			return ret;
		memset(skb_put(skb, skb_len_dif), 0, skb_len_dif);
	} else {
		ret = skb_pad(skb, COM_TRAILER_LEN);
		if (ret)
			return ret;
	}

	skb = edgx_com_ts_xmit(&xmii->ts, skb, ptcom);

	skb_trailer = skb_put(skb, COM_TRAILER_LEN);
	skb_trailer[COM_PORT_IDX]  = ptcom;
	skb_trailer[COM_FLAGS_IDX] = flags;
	skb->dev = xmii->net;

	dev_queue_xmit(skb);

	return NETDEV_TX_OK;
}

static rx_handler_result_t edgx_com_xmii_rx(struct sk_buff **pskb)
{
	struct sk_buff       *skb = *pskb;
	struct net_device    *netdev = skb->dev;
	struct edgx_com_xmii *xmii = rcu_dereference(netdev->rx_handler_data);
	u8                   *skb_trailer;
	ptcom_t               ptcom;
	ptflags_t             flags;

	if (unlikely(skb->pkt_type == PACKET_LOOPBACK))
		return RX_HANDLER_PASS;
	skb = skb_share_check(skb, GFP_ATOMIC);
	if (!skb)
		return RX_HANDLER_CONSUMED;

	/* get management trailer ... */
	skb_trailer = skb_tail_pointer(skb) - COM_TRAILER_LEN;
	ptcom = skb_trailer[COM_PORT_IDX];
	flags = skb_trailer[COM_FLAGS_IDX];
	/* ... and remove it from skb */
	skb_trim(skb, skb->len - COM_TRAILER_LEN);

	/* newer kernels emit HW checksum failure without this. */
	if (skb->ip_summed == CHECKSUM_COMPLETE)
		skb->ip_summed = CHECKSUM_NONE;

	edgx_com_ts_rx(&xmii->ts, skb, ptcom);

	edgx_com_rx_dispatch(&xmii->com, skb, ptcom, flags);

	return RX_HANDLER_CONSUMED;
}

static int edgx_com_xmii_ts_cfg_get(struct edgx_com *com, ptcom_t ptcom,
				    struct ifreq *ifr)
{
	struct edgx_com_xmii *xmii = edgx_com_to_xmii(com);

	return edgx_com_ts_cfg_get(&xmii->ts, ptcom, ifr);
}

static int edgx_com_xmii_ts_cfg_set(struct edgx_com *com, ptcom_t ptcom,
				    struct ifreq *ifr)
{
	struct edgx_com_xmii *xmii = edgx_com_to_xmii(com);

	return edgx_com_ts_cfg_set(&xmii->ts, ptcom, ifr);
}

static void edgx_com_xmii_shutdown(struct edgx_com *com)
{
	struct edgx_com_xmii *xmii = edgx_com_to_xmii(com);

	rtnl_lock();
	dev_close(xmii->net);
	dev_set_promiscuity(xmii->net, -1);
	netdev_rx_handler_unregister(xmii->net);
	rtnl_unlock();
	edgx_com_ts_shutdown(&xmii->ts);
	kfree(xmii);
}

static struct edgx_com_ops edgx_com_xmii_ops = {
	.shutdown	= edgx_com_xmii_shutdown,
	.xmit		= edgx_com_xmii_xmit,
	.hwts_set	= edgx_com_xmii_ts_cfg_set,
	.hwts_get	= edgx_com_xmii_ts_cfg_get,
};

int edgx_com_xmii_init(struct edgx_com_xmii *xmii, struct edgx_br *br,
		       const char *drv_name, int irq,
		       ptid_t mgmt_pt, struct net_device *net)
{
	int ret;

	edgx_com_init(&xmii->com, br, &edgx_com_xmii_ops, mgmt_pt);

	xmii->net = net;

	ret = edgx_com_ts_init(&xmii->ts, drv_name, irq, br);
	if (ret < 0) {
		edgx_err("COM XMII: Cannot initialize port timestamping\n");
		return ret;
	}

	rtnl_lock();
	netdev_rx_handler_register(net, edgx_com_xmii_rx, xmii);
	dev_set_promiscuity(net, 1);

#if LINUX_VERSION_CODE < KERNEL_VERSION(5,3,0)
	dev_open(net);
#else
	dev_open(net, NULL);
#endif
	rtnl_unlock();
	return 0;
}

int edgx_com_xmii_probe(struct edgx_br *br, int irq, const char *ifname,
			const char *drv_name, struct edgx_com **com)
{
	struct edgx_com_xmii	*xmii;
	struct net_device	*net;
	char			*ptstr;
	int			 ret;
	ptid_t			 mgmt_pt;

	ptstr = strchr(ifname, ':');
	if (!ptstr) {
		edgx_err("COM XMII: Invalid format for 'netif' load parameter\n");
		return -EINVAL;
	}
	*(ptstr++) = '\0';
	if (kstrtoint(ptstr, 10, &mgmt_pt)) {
		edgx_err("COM XMII: Unable to parse management port\n");
		return -EINVAL;
	}

	if (mgmt_pt >= edgx_br_get_num_ports(br)) {
		edgx_err("COM XMII: Invalid port number for management port\n");
		return -ENODEV;
	}

	net = dev_get_by_name(&init_net, ifname);
	if (!net) {
		edgx_err("COM XMII: Cannot connect to interface '%s'\n",
			 ifname);
		return -ENODEV;
	}

	xmii = kzalloc(sizeof(*xmii), GFP_KERNEL);
	if (!xmii)
		return -ENOMEM;

	ret = edgx_com_xmii_init(xmii, br, drv_name, irq, mgmt_pt, net);
	if (ret) {
		kfree(xmii);
		return ret;
	}

	*com = &xmii->com;
	edgx_br_info(br, "Setup xMII-COM subsystem ('%s' <-> port %u)... done\n",
		     ifname, mgmt_pt);
	return 0;
}
