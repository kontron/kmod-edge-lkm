#ifndef __TSNIC_VPD_H
#define __TSNIC_VPD_H

#include <linux/types.h>

#define TSNIC_SNO_LEN 10

struct vpd;

struct vpd *tsnic_vpd_init(void *io_addr);
void tsnic_vpd_free(struct vpd *vpd);
int tsnic_vpd_eth_hw_addr(struct vpd *vpd, u8 *addr);
int tsnic_vpd_asset_tag(struct vpd *vpd, char *asset, size_t len);

#endif /* __TSNIC_VPD_H */
