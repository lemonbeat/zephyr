/*
 * Copyright (c) 2019 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <logging/log.h>
LOG_MODULE_REGISTER(net_pkt_sock_sample, LOG_LEVEL_DBG);

#include <zephyr.h>
#include <errno.h>
#include <stdio.h>

#include <net/socket.h>
#include <net/ethernet.h>

#define STACK_SIZE 1024
#define THREAD_PRIORITY K_PRIO_COOP(8)
#define RECV_BUFFER_SIZE 1280
#define WAIT_TIME CONFIG_NET_SAMPLE_SEND_WAIT_TIME

#define FLOOD (CONFIG_NET_SAMPLE_SEND_WAIT_TIME ? 0 : 1)

static struct k_sem quit_lock;

struct packet_data {
	int send_sock;
	int recv_sock;
	char recv_buffer[RECV_BUFFER_SIZE];
};

static struct packet_data packet;

static void recv_packet(void);
static void send_packet(void);

K_THREAD_DEFINE(receiver_thread_id, STACK_SIZE, recv_packet, NULL, NULL, NULL,
		THREAD_PRIORITY, 0, -1);
K_THREAD_DEFINE(sender_thread_id, STACK_SIZE, send_packet, NULL, NULL, NULL,
		THREAD_PRIORITY, 0, -1);

/* Generated by http://www.lipsum.com/
 * 2 paragraphs, 179 words, 1160 bytes of Lorem Ipsum
 */
const char lorem_ipsum[] =
	"Lorem ipsum dolor sit amet, consectetur adipiscing elit. Quisque "
	"sodales lorem lorem, sed congue enim vehicula a. Sed finibus diam sed "
	"odio ultrices pharetra. Nullam dictum arcu ultricies turpis congue, "
	"vel venenatis turpis venenatis. Nam tempus arcu eros, ac congue libero "
	"tristique congue. Proin velit lectus, euismod sit amet quam in, "
	"maximus condimentum urna. Cras vel erat luctus, mattis orci ut, varius "
	"urna. Nam eu lobortis velit."
	"\n"
	"Nullam sit amet diam vel odio sodales cursus vehicula eu arcu. Proin "
	"fringilla, enim nec consectetur mollis, lorem orci interdum nisi, "
	"vitae suscipit nisi mauris eu mi. Proin diam enim, mollis ac rhoncus "
	"vitae, placerat et eros. Suspendisse convallis, ipsum nec rhoncus "
	"aliquam, ex augue ultrices nisl, id aliquet mi diam quis ante. "
	"Pellentesque venenatis ornare ultrices. Quisque et porttitor lectus. "
	"Ut venenatis nunc et urna imperdiet porttitor non laoreet massa. Donec "
	"eleifend eros in mi sagittis egestas. Sed et mi nunc. Nunc vulputate, "
	"mauris non ullamcorper viverra, lorem nulla vulputate diam, et congue "
	"dui velit non erat. Duis interdum leo et ipsum tempor consequat. In "
	"faucibus enim quis purus vulputate nullam."
	"\n";

static void quit(void)
{
	k_sem_give(&quit_lock);
}

static int start_socket(int *sock)
{
	struct sockaddr_ll dst = { 0 };
	int ret;

	*sock = socket(AF_PACKET,
		       IS_ENABLED(CONFIG_NET_SAMPLE_ENABLE_PACKET_DGRAM) ?
				     SOCK_DGRAM :
				     SOCK_RAW,
		       ETH_P_ALL);
	if (*sock < 0) {
		LOG_ERR("Failed to create %s socket : %d",
			IS_ENABLED(CONFIG_NET_SAMPLE_ENABLE_PACKET_DGRAM) ?
				      "DGRAM" :
				      "RAW",
			errno);
		return -errno;
	}

	dst.sll_ifindex = net_if_get_by_iface(net_if_get_default());
	dst.sll_family = AF_PACKET;

	ret = bind(*sock, (const struct sockaddr *)&dst,
		   sizeof(struct sockaddr_ll));
	if (ret < 0) {
		LOG_ERR("Failed to bind packet socket : %d", errno);
		return -errno;
	}

	return 0;
}

static int recv_packet_socket(struct packet_data *packet)
{
	int ret = 0;
	int received;

	LOG_INF("Waiting for packets ...");

	do {
		received = recv(packet->recv_sock, packet->recv_buffer,
				sizeof(packet->recv_buffer), 0);

		if (received < 0) {
			LOG_ERR("RAW : recv error %d", errno);
			ret = -errno;
			break;
		}

		LOG_DBG("Received %d bytes", received);

	} while (true);

	return ret;
}

static void recv_packet(void)
{
	int ret;

	ret = start_socket(&packet.recv_sock);
	if (ret < 0) {
		quit();
		return;
	}

	while (ret == 0) {
		ret = recv_packet_socket(&packet);
		if (ret < 0) {
			quit();
			return;
		}
	}
}

static int send_packet_socket(struct packet_data *packet)
{
	struct sockaddr_ll dst = { 0 };
	size_t send = 100U;
	int ret;

	dst.sll_ifindex = net_if_get_by_iface(net_if_get_default());

	if (IS_ENABLED(CONFIG_NET_SAMPLE_ENABLE_PACKET_DGRAM)) {
		dst.sll_halen = sizeof(struct net_eth_addr);

		/* FIXME: assume IP data atm */
		dst.sll_protocol = htons(ETH_P_IP);

		ret = net_bytes_from_str(dst.sll_addr, dst.sll_halen,
					 CONFIG_NET_SAMPLE_DESTINATION_ADDR);
		if (ret < 0) {
			LOG_ERR("Invalid MAC address '%s'",
				CONFIG_NET_SAMPLE_DESTINATION_ADDR);
		}
	}

	do {
		/* Sending dummy data */
		ret = sendto(packet->send_sock, lorem_ipsum, send, 0,
			     (const struct sockaddr *)&dst,
			     sizeof(struct sockaddr_ll));
		if (ret < 0) {
			LOG_ERR("Failed to send, errno %d", errno);
			break;
		} else {
			if (!FLOOD) {
				LOG_DBG("Sent %zd bytes", send);
			}
		}

		if (!FLOOD) {
			k_msleep(WAIT_TIME);
		}
	} while (true);

	return ret;
}

static void send_packet(void)
{
	int ret;

	ret = start_socket(&packet.send_sock);
	if (ret < 0) {
		quit();
		return;
	}

	while (ret == 0) {
		ret = send_packet_socket(&packet);
		if (ret < 0) {
			quit();
			return;
		}
	}
}

void main(void)
{
	k_sem_init(&quit_lock, 0, UINT_MAX);

	LOG_INF("Packet socket sample is running");

	k_thread_start(receiver_thread_id);
	k_thread_start(sender_thread_id);

	k_sem_take(&quit_lock, K_FOREVER);

	LOG_INF("Stopping...");

	k_thread_abort(receiver_thread_id);
	k_thread_abort(sender_thread_id);

	if (packet.recv_sock > 0) {
		(void)close(packet.recv_sock);
	}

	if (packet.send_sock > 0) {
		(void)close(packet.send_sock);
	}
}
