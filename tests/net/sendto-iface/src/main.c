/* main.c - Application main entry point */

/*
 * Copyright (c) 2020 Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(net_test, CONFIG_NET_IPV6_LOG_LEVEL);

#include <zephyr/types.h>
#include <ztest.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <linker/sections.h>

#include <tc_util.h>

#include <net/ethernet.h>
#include <net/dummy.h>
#include <net/buf.h>
#include <net/net_ip.h>
#include <net/net_if.h>
#include <net/socket.h>

#define NET_LOG_ENABLED 1
#include "net_private.h"
#include "ipv6.h"

#ifdef CONFIG_NET_IPV6_NBR_CACHE
#include "nbr.h"
#endif /* CONFIG_NET_IPV6_NBR_CACHE */

#if defined(CONFIG_NET_IPV6_LOG_LEVEL_DBG)
#define DBG(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define DBG(fmt, ...)
#endif

static int udp_socket;

/* Interface 1 is the default host and it has iface1_addr assigned to it */
static struct in6_addr iface1_addr = { { { 0xfc, 0, 0, 0, 0, 0, 0, 0,
					  0, 0, 0, 0, 0, 0, 0, 0x1 } } };

/* Interface 2 is the secondary host and it has iface2_addr assigned to it */
static struct in6_addr iface2_addr = { { { 0xfc, 0x01, 0, 0, 0, 0, 0, 0,
				    0, 0, 0, 0, 0, 0, 0, 0x1 } } };

/* The dest_addr1 is only reachable via iface1 */
static struct in6_addr dest_addr1 = { { { 0xfc, 0, 0, 0, 0, 0, 0, 0,
					 0, 0, 0, 0, 0, 0, 0, 0x2 } } };

/* The dest_addr2 is only reachable via iface2 */
static struct in6_addr dest_addr2 = { { { 0xfc, 0x01, 0, 0, 0, 0, 0, 0,
					 0, 0, 0, 0, 0, 0, 0, 0x2 } } };

/* Extra address is assigned to ll_addr */
static struct in6_addr ll_addr = { { { 0xfe, 0x80, 0x43, 0xb8, 0, 0, 0, 0,
				       0, 0, 0, 0xf2, 0xaa, 0x29, 0x02,
				       0x04 } } };

static struct net_if *iface1;
static struct net_if *iface2;

static struct device *last_send_dev;

K_SEM_DEFINE(wait_data, 0, UINT_MAX);

#define WAIT_TIME K_MSEC(250)

struct net_sendto_test {
	u8_t mac_addr[sizeof(struct net_eth_addr)];
	struct net_linkaddr ll_addr;
};

int net_sendto_dev_init(struct device *dev)
{
	return 0;
}

static u8_t *net_sendto_get_mac(struct device *dev)
{
	struct net_sendto_test *route = dev->driver_data;

	if (route->mac_addr[2] == 0x00) {
		/* 00-00-5E-00-53-xx Documentation RFC 7042 */
		route->mac_addr[0] = 0x00;
		route->mac_addr[1] = 0x00;
		route->mac_addr[2] = 0x5E;
		route->mac_addr[3] = 0x00;
		route->mac_addr[4] = 0x53;
		route->mac_addr[5] = sys_rand32_get();
	}

	route->ll_addr.addr = route->mac_addr;
	route->ll_addr.len = 6U;

	return route->mac_addr;
}

static void net_sendto_iface_init(struct net_if *iface)
{
	u8_t *mac = net_sendto_get_mac(net_if_get_device(iface));

	net_if_set_link_addr(iface, mac, sizeof(struct net_eth_addr),
			     NET_LINK_ETHERNET);
}

static int net_sendto_iface_send(struct device *dev, struct net_pkt *pkt)
{
	if (!pkt->frags) {
		TC_ERROR("No data to send!\n");
		return -ENODATA;
	}

	DBG("if %p pkt %p to be sent len %lu\n",
	    net_if_lookup_by_dev(dev), pkt, net_pkt_get_len(pkt));

	last_send_dev = dev;

	k_sem_give(&wait_data);

	return 0;
}

struct net_sendto_test net_sendto_data_1;
struct net_sendto_test net_sendto_data_2;

static struct dummy_api net_sendto_if_api = {
	.iface_api.init = net_sendto_iface_init,
	.send = net_sendto_iface_send,
};

#define _ETH_L2_LAYER DUMMY_L2
#define _ETH_L2_CTX_TYPE NET_L2_GET_CTX_TYPE(DUMMY_L2)

NET_DEVICE_INIT_INSTANCE(net_sendto_test_1, "net_sendto_test_1", iface1,
			 net_sendto_dev_init, device_pm_control_nop,
			 &net_sendto_data_1, NULL,
			 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
			 &net_sendto_if_api, _ETH_L2_LAYER,
			 _ETH_L2_CTX_TYPE, 127);

NET_DEVICE_INIT_INSTANCE(net_sendto_test_2, "net_sendto_test_2", iface2,
			 net_sendto_dev_init, device_pm_control_nop,
			 &net_sendto_data_2, NULL,
			 CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,
			 &net_sendto_if_api, _ETH_L2_LAYER,
			 _ETH_L2_CTX_TYPE, 127);

static void test_init(void)
{
	struct net_if_addr *ifaddr;
	struct net_if_ipv6_prefix *ifprefix;

	iface1 = net_if_get_default();
	iface2 = net_if_get_default() + 1;

	DBG("Interfaces: [%d] iface1 %p, [%d] iface2 %p\n",
	    net_if_get_by_iface(iface1), iface1,
	    net_if_get_by_iface(iface2), iface2);

	zassert_not_null(iface1,
			 "Interface is NULL");

	zassert_not_null(iface2,
			 "Interface is NULL");

	ifaddr = net_if_ipv6_addr_add(iface1, &iface1_addr,
				      NET_ADDR_MANUAL, 0);
	zassert_not_null(ifaddr,
			 "Cannot add IPv6 address");

	/* For testing purposes we need to set the adddresses preferred */
	ifaddr->addr_state = NET_ADDR_PREFERRED;

	ifaddr = net_if_ipv6_addr_add(iface1, &ll_addr,
				      NET_ADDR_MANUAL, 0);
	zassert_not_null(ifaddr,
			 "Cannot add IPv6 address");

	ifaddr->addr_state = NET_ADDR_PREFERRED;


	ifprefix = net_if_ipv6_prefix_add(iface1, &iface1_addr, 64, 0);

	zassert_not_null(ifprefix,
			 "Cannot set IPv6 prefix");

	ifaddr = net_if_ipv6_addr_add(iface2, &iface2_addr,
				      NET_ADDR_MANUAL, 0);
	zassert_not_null(ifaddr,
			 "Cannot add IPv6 address");

	/* For testing purposes we need to set the adddresses preferred */
	ifaddr->addr_state = NET_ADDR_PREFERRED;

	ifaddr = net_if_ipv6_addr_add(iface2, &ll_addr,
				      NET_ADDR_MANUAL, 0);
	zassert_not_null(ifaddr,
			 "Cannot add IPv6 address");

	ifaddr->addr_state = NET_ADDR_PREFERRED;

	ifprefix = net_if_ipv6_prefix_add(iface2, &iface2_addr, 64, 0);

	zassert_not_null(ifprefix,
			 "Cannot set IPv6 prefix");
}

static void test_socket_create(void)
{
	int res;

	struct sockaddr_in6 addr6 = {
		.sin6_family = AF_INET6,
		.sin6_port = 1024,
		.sin6_addr = IN6ADDR_ANY_INIT,
	};

	udp_socket = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);

	zassert_true(udp_socket >= 0, "Opening socket failed");

	res = bind(udp_socket, (struct sockaddr *)&addr6, sizeof(addr6));
	zassert_true(res >= 0, "Binding socket failed");
}

static void test_sendto_1(void)
{
	int send;
	static const u8_t data[] = {1, 2, 3, 4};
	struct sockaddr_in6 addr = {
		.sin6_family = AF_INET6,
		.sin6_port = 2048,
		.sin6_addr = dest_addr1
	};

	last_send_dev = NULL;

	send = sendto(udp_socket, data, sizeof(data), 0,
		      (const struct sockaddr *)&addr, sizeof(addr));

	zassert_equal(sizeof(data), send, "Error while sending data");

	k_sem_take(&wait_data, WAIT_TIME);

	zassert_equal(last_send_dev, net_if_get_device(iface1),
		      "Sent on wrong interface");

}

static void test_sendto_2(void)
{
	int send;
	static const u8_t data[] = {1, 2, 3, 4};
	struct sockaddr_in6 addr = {
		.sin6_family = AF_INET6,
		.sin6_port = 2048,
		.sin6_addr = dest_addr2
	};

	last_send_dev = NULL;

	send = sendto(udp_socket, data, sizeof(data), 0,
		      (const struct sockaddr *)&addr, sizeof(addr));

	zassert_equal(sizeof(data), send, "Error while sending data");

	k_sem_take(&wait_data, WAIT_TIME);

	zassert_equal(last_send_dev, net_if_get_device(iface2),
		      "Sent on wrong interface");
}

#ifdef CONFIG_NET_IPV6_NBR_CACHE
static bool net_test_nbr_lookup_ok(struct net_if *iface,
				   struct in6_addr *addr)
{
	struct net_nbr *nbr;

	nbr = net_ipv6_nbr_lookup(iface, addr);
	if (!nbr) {
		TC_ERROR("Neighbor %s not found in cache\n",
			 net_sprint_ipv6_addr(addr));
		return false;
	}

	return true;
}

static void test_populate_nbr_cache(void)
{
	struct net_nbr *nbr;

	nbr = net_ipv6_nbr_add(iface1,
			       &dest_addr1,
			       &net_sendto_data_1.ll_addr,
			       false,
			       NET_IPV6_NBR_STATE_REACHABLE);
	zassert_not_null(nbr, "Cannot add peer to neighbor cache");

	nbr = net_ipv6_nbr_add(iface2,
			       &dest_addr2,
			       &net_sendto_data_2.ll_addr,
			       false,
			       NET_IPV6_NBR_STATE_REACHABLE);
	zassert_not_null(nbr, "Cannot add peer to neighbor cache");

	zassert_true(net_test_nbr_lookup_ok(iface1, &dest_addr1), NULL);

	zassert_true(net_test_nbr_lookup_ok(iface2, &dest_addr2), NULL);
}
#endif /* CONFIG_NET_IPV6_NBR_CACHE */

/*test case main entry*/
void test_main(void)
{
	ztest_test_suite(test_route,
			ztest_unit_test(test_init),
			ztest_unit_test(test_socket_create),
#ifdef CONFIG_NET_IPV6_NBR_CACHE
			ztest_unit_test(test_populate_nbr_cache),
#endif /* CONFIG_NET_IPV6_NBR_CACHE */
			ztest_unit_test(test_sendto_1),
			 ztest_unit_test(test_sendto_2));
	ztest_run_test_suite(test_route);
}
