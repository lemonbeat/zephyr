/*
 * Copyright (c) 2020 Lemonbeat GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(net_ppp_mgmt, CONFIG_NET_L2_PPP_LOG_LEVEL);

#include <net/ppp.h>

void ppp_mgmt_raise_carrier_on_event(struct net_if *iface)
{
	net_mgmt_event_notify(NET_EVENT_PPP_CARRIER_ON, iface);
}

void ppp_mgmt_raise_carrier_off_event(struct net_if *iface)
{
	net_mgmt_event_notify(NET_EVENT_PPP_CARRIER_OFF, iface);
}
