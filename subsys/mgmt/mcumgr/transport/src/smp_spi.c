/*
 * Copyright (c) 2026 Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file
 * @brief SPI transport for the mcumgr SMP protocol.
 *
 * SPI slave (peripheral) transport for MCUmgr. The device operates as an SPI
 * peripheral and receives raw SMP packets from a controller (master). A GPIO
 * "data-ready" line is asserted when a response is available for the controller
 * to clock out.
 *
 * Devicetree configuration (compatible: "zephyr,smp-spi"):
 *   smp_spi: smp-spi {
 *       compatible = "zephyr,smp-spi";
 *       spi-dev = <&spi1>;
 *       data-ready-gpios = <&gpio0 10 GPIO_ACTIVE_LOW>;
 *   };
 *
 * Wire protocol (length-prefixed framing):
 *   [2-byte big-endian payload length][payload (raw SMP)][2-byte zero padding]
 *
 * The controller writes a request frame and, after the data-ready GPIO is
 * asserted, clocks out the response frame using the same framing.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/net_buf.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/smp/smp.h>
#include <zephyr/mgmt/mcumgr/transport/smp.h>

#include <mgmt/mcumgr/transport/smp_internal.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(mcumgr_smp, CONFIG_MCUMGR_TRANSPORT_LOG_LEVEL);

#define SMP_SPI_NODE DT_NODELABEL(smp_spi)

BUILD_ASSERT(CONFIG_MCUMGR_TRANSPORT_SPI_MTU != 0,
	     "CONFIG_MCUMGR_TRANSPORT_SPI_MTU must be > 0");

/* Length-prefix header size (2 bytes big-endian) */
#define SMP_SPI_FRAME_HDR_SIZE 2
/* Trailing padding after payload */
#define SMP_SPI_FRAME_TAIL_SIZE 2
#define SMP_SPI_FRAME_OVERHEAD (SMP_SPI_FRAME_HDR_SIZE + SMP_SPI_FRAME_TAIL_SIZE)

#define SMP_SPI_BUF_SIZE (CONFIG_MCUMGR_TRANSPORT_SPI_MTU + SMP_SPI_FRAME_OVERHEAD)

static const struct device *spi_dev = DEVICE_DT_GET(DT_PHANDLE(SMP_SPI_NODE, spi_dev));
static const struct gpio_dt_spec data_ready_gpio =
	GPIO_DT_SPEC_GET(SMP_SPI_NODE, data_ready_gpios);

static struct smp_transport smp_spi_transport;

#ifdef CONFIG_SMP_CLIENT
static struct smp_client_transport_entry smp_spi_client_transport;
#endif

static K_SEM_DEFINE(smp_spi_tx_sem, 0, 1);
static struct net_buf *smp_spi_tx_nb;
static K_MUTEX_DEFINE(smp_spi_tx_mutex);

static K_THREAD_STACK_DEFINE(smp_spi_rx_stack,
			     CONFIG_MCUMGR_TRANSPORT_SPI_RX_THREAD_STACK_SIZE);
static struct k_thread smp_spi_rx_thread_data;

static uint8_t smp_spi_rx_buf[SMP_SPI_BUF_SIZE];
static uint8_t smp_spi_tx_buf[SMP_SPI_BUF_SIZE];

static const struct spi_config smp_spi_cfg = {
	.frequency = 0,
	.operation = SPI_OP_MODE_SLAVE | SPI_WORD_SET(8) | SPI_TRANSFER_MSB,
	.slave = 0,
};

static void smp_spi_data_ready_set(bool ready)
{
	if (ready) {
		gpio_pin_set_dt(&data_ready_gpio, 1);
	} else {
		gpio_pin_set_dt(&data_ready_gpio, 0);
	}
}

static uint16_t smp_spi_get_mtu(const struct net_buf *nb)
{
	return CONFIG_MCUMGR_TRANSPORT_SPI_MTU;
}

static int smp_spi_tx_pkt(struct net_buf *nb)
{
	k_mutex_lock(&smp_spi_tx_mutex, K_FOREVER);

	if (smp_spi_tx_nb != NULL) {
		smp_packet_free(smp_spi_tx_nb);
	}
	smp_spi_tx_nb = nb;

	smp_spi_data_ready_set(true);
	k_sem_give(&smp_spi_tx_sem);

	k_mutex_unlock(&smp_spi_tx_mutex);
	return 0;
}

static int smp_spi_build_tx_frame(struct net_buf *nb)
{
	uint16_t len;

	if (nb->len > CONFIG_MCUMGR_TRANSPORT_SPI_MTU) {
		LOG_ERR("SPI SMP TX payload %u exceeds MTU %d", nb->len,
			CONFIG_MCUMGR_TRANSPORT_SPI_MTU);
		return -ENOMEM;
	}

	len = nb->len;
	sys_put_be16(len, smp_spi_tx_buf);
	memcpy(smp_spi_tx_buf + SMP_SPI_FRAME_HDR_SIZE, nb->data, len);
	memset(smp_spi_tx_buf + SMP_SPI_FRAME_HDR_SIZE + len, 0, SMP_SPI_FRAME_TAIL_SIZE);

	return (int)(SMP_SPI_FRAME_HDR_SIZE + len + SMP_SPI_FRAME_TAIL_SIZE);
}

static void smp_spi_rx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		struct spi_buf rx_spi_buf = {
			.buf = smp_spi_rx_buf,
			.len = sizeof(smp_spi_rx_buf),
		};
		const struct spi_buf_set rx_set = {
			.buffers = &rx_spi_buf,
			.count = 1,
		};
		struct spi_buf tx_spi_buf = {
			.buf = smp_spi_tx_buf,
			.len = 0,
		};
		const struct spi_buf_set tx_set = {
			.buffers = &tx_spi_buf,
			.count = 1,
		};
		int rc;
		uint16_t payload_len;
		struct net_buf *nb;

		k_mutex_lock(&smp_spi_tx_mutex, K_FOREVER);
		if (smp_spi_tx_nb != NULL) {
			int frame_len = smp_spi_build_tx_frame(smp_spi_tx_nb);

			if (frame_len > 0) {
				tx_spi_buf.len = frame_len;
			}

			smp_packet_free(smp_spi_tx_nb);
			smp_spi_tx_nb = NULL;
			smp_spi_data_ready_set(false);
		}
		k_mutex_unlock(&smp_spi_tx_mutex);

		rc = spi_transceive(spi_dev, &smp_spi_cfg, &tx_set, &rx_set);
		if (rc != 0) {
			LOG_ERR("SPI SMP transceive failed: %d", rc);
			k_sleep(K_MSEC(10));
			continue;
		}

		if (rx_spi_buf.len < SMP_SPI_FRAME_HDR_SIZE) {
			continue;
		}

		payload_len = sys_get_be16(smp_spi_rx_buf);

		if (payload_len == 0) {
			continue;
		}

		if (payload_len > CONFIG_MCUMGR_TRANSPORT_SPI_MTU) {
			LOG_WRN("SPI SMP RX payload %u exceeds MTU %d, dropping",
				payload_len, CONFIG_MCUMGR_TRANSPORT_SPI_MTU);
			continue;
		}

		if ((SMP_SPI_FRAME_HDR_SIZE + payload_len) > rx_spi_buf.len) {
			LOG_WRN("SPI SMP RX frame truncated");
			continue;
		}

		nb = smp_packet_alloc();
		if (nb == NULL) {
			LOG_ERR("SPI SMP failed to allocate net_buf");
			continue;
		}

		if (net_buf_tailroom(nb) < payload_len) {
			LOG_ERR("SPI SMP net_buf too small for payload (%u)", payload_len);
			smp_packet_free(nb);
			continue;
		}

		net_buf_add_mem(nb, smp_spi_rx_buf + SMP_SPI_FRAME_HDR_SIZE, payload_len);

		smp_rx_req(&smp_spi_transport, nb);
	}
}

static int smp_spi_init(void)
{
	int rc;

	if (!device_is_ready(spi_dev)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&data_ready_gpio)) {
		LOG_ERR("Data-ready GPIO not ready");
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&data_ready_gpio, GPIO_OUTPUT_INACTIVE);
	if (rc != 0) {
		LOG_ERR("Failed to configure data-ready GPIO: %d", rc);
		return rc;
	}

	smp_spi_transport.functions.output = smp_spi_tx_pkt;
	smp_spi_transport.functions.get_mtu = smp_spi_get_mtu;

	rc = smp_transport_init(&smp_spi_transport);
	if (rc != 0) {
		LOG_ERR("SPI SMP transport init failed: %d", rc);
		return rc;
	}

#ifdef CONFIG_SMP_CLIENT
	smp_spi_client_transport.smpt = &smp_spi_transport;
	smp_spi_client_transport.smpt_type = SMP_SPI_TRANSPORT;
	smp_client_transport_register(&smp_spi_client_transport);
#endif

	k_thread_create(&smp_spi_rx_thread_data, smp_spi_rx_stack,
			K_THREAD_STACK_SIZEOF(smp_spi_rx_stack),
			smp_spi_rx_thread, NULL, NULL, NULL,
			CONFIG_MCUMGR_TRANSPORT_SPI_RX_THREAD_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&smp_spi_rx_thread_data, "smp_spi_rx");

	LOG_INF("SPI SMP transport initialized");

	return 0;
}

SYS_INIT(smp_spi_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
