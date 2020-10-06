/*
 * Copyright (c) 2020,  NXP
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <fsl_iopctl.h>
#include <soc.h>

static int mimxrt685_evk_pinmux_init(const struct device *dev)
{
	ARG_UNUSED(dev);

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(flexcomm0), nxp_lpc_usart, okay) && \
	CONFIG_SERIAL
	/* USART0 RX,  TX */
	const uint32_t port0_pin1_config =
		(/* Pin is configured as FC0_TXD_SCL_MISO_WS */
		 IOPCTL_PIO_FUNC1 |
		 /* Disable pull-up / pull-down function */
		 IOPCTL_PIO_PUPD_DI |
		 /* Enable pull-down function */
		 IOPCTL_PIO_PULLDOWN_EN |
		 /* Disable input buffer function */
		 IOPCTL_PIO_INBUF_DI |
		 /* Normal mode */
		 IOPCTL_PIO_SLEW_RATE_NORMAL |
		 /* Normal drive */
		 IOPCTL_PIO_FULLDRIVE_DI |
		 /* Analog mux is disabled */
		 IOPCTL_PIO_ANAMUX_DI |
		 /* Pseudo Output Drain is disabled */
		 IOPCTL_PIO_PSEDRAIN_DI |
		 /* Input function is not inverted */
		 IOPCTL_PIO_INV_DI);
	/* PORT0 PIN1 (coords: G2) is configured as FC0_TXD_SCL_MISO_WS */
	IOPCTL_PinMuxSet(IOPCTL, 0U, 1U, port0_pin1_config);

	const uint32_t port0_pin2_config =
		(/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
		 IOPCTL_PIO_FUNC1 |
		 /* Disable pull-up / pull-down function */
		 IOPCTL_PIO_PUPD_DI |
		 /* Enable pull-down function */
		 IOPCTL_PIO_PULLDOWN_EN |
		 /* Enables input buffer function */
		 IOPCTL_PIO_INBUF_EN |
		 /* Normal mode */
		 IOPCTL_PIO_SLEW_RATE_NORMAL |
		 /* Normal drive */
		 IOPCTL_PIO_FULLDRIVE_DI |
		 /* Analog mux is disabled */
		 IOPCTL_PIO_ANAMUX_DI |
		 /* Pseudo Output Drain is disabled */
		 IOPCTL_PIO_PSEDRAIN_DI |
		 /* Input function is not inverted */
		 IOPCTL_PIO_INV_DI);
	/* PORT0 PIN2 (coords: G4) is configured as FC0_RXD_SDA_MOSI_DATA */
	IOPCTL_PinMuxSet(IOPCTL, 0U, 2U, port0_pin2_config);
#endif

#if DT_PHA_HAS_CELL(DT_ALIAS(sw0), gpios, pin)
	const uint32_t port1_pin1_config =
		(/* Pin is configured as PIO1_1 */
		 IOPCTL_PIO_FUNC0 |
		 /* Disable pull-up / pull-down function */
		 IOPCTL_PIO_PUPD_DI |
		 /* Enable pull-down function */
		 IOPCTL_PIO_PULLDOWN_EN |
		 /* Enables input buffer function */
		 IOPCTL_PIO_INBUF_EN |
		 /* Normal mode */
		 IOPCTL_PIO_SLEW_RATE_NORMAL |
		 /* Normal drive */
		 IOPCTL_PIO_FULLDRIVE_DI |
		 /* Analog mux is disabled */
		 IOPCTL_PIO_ANAMUX_DI |
		 /* Pseudo Output Drain is disabled */
		 IOPCTL_PIO_PSEDRAIN_DI |
		 /* Input function is not inverted */
		 IOPCTL_PIO_INV_DI);
	/* PORT1 PIN1 (coords: G15) is configured as PIO1_1 */
	IOPCTL_PinMuxSet(IOPCTL, 1U, 1U, port1_pin1_config);
#endif

#if DT_PHA_HAS_CELL(DT_ALIAS(sw1), gpios, pin)
	const uint32_t port0_pin10_config =
		(/* Pin is configured as PIO0_10 */
		 IOPCTL_PIO_FUNC0 |
		 /* Disable pull-up / pull-down function */
		 IOPCTL_PIO_PUPD_DI |
		 /* Enable pull-down function */
		 IOPCTL_PIO_PULLDOWN_EN |
		 /* Enables input buffer function */
		 IOPCTL_PIO_INBUF_EN |
		 /* Normal mode */
		 IOPCTL_PIO_SLEW_RATE_NORMAL |
		 /* Normal drive */
		 IOPCTL_PIO_FULLDRIVE_DI |
		 /* Analog mux is disabled */
		 IOPCTL_PIO_ANAMUX_DI |
		 /* Pseudo Output Drain is disabled */
		 IOPCTL_PIO_PSEDRAIN_DI |
		 /* Input function is not inverted */
		 IOPCTL_PIO_INV_DI);
	/* PORT0 PIN10 (coords: J3) is configured as PIO0_10 */
	IOPCTL_PinMuxSet(IOPCTL, 0U, 10U, port0_pin10_config);
#endif

#ifdef DT_GPIO_LEDS_LED_1_GPIOS_CONTROLLER
	const uint32_t port0_pin14_config =
		(/* Pin is configured as PIO0_14 */
		 IOPCTL_PIO_FUNC0 |
		 /* Disable pull-up / pull-down function */
		 IOPCTL_PIO_PUPD_DI |
		 /* Enable pull-down function */
		 IOPCTL_PIO_PULLDOWN_EN |
		 /* Disable input buffer function */
		 IOPCTL_PIO_INBUF_DI |
		 /* Normal mode */
		 IOPCTL_PIO_SLEW_RATE_NORMAL |
		 /* Normal drive */
		 IOPCTL_PIO_FULLDRIVE_DI |
		 /* Analog mux is disabled */
		 IOPCTL_PIO_ANAMUX_DI |
		 /* Pseudo Output Drain is disabled */
		 IOPCTL_PIO_PSEDRAIN_DI |
		 /* Input function is not inverted */
		 IOPCTL_PIO_INV_DI);
	/* PORT0 PIN14 (coords: A3) is configured as PIO0_14 */
	IOPCTL_PinMuxSet(IOPCTL, 0U, 14U, port0_pin14_config);
#endif

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(flexcomm2), nxp_lpc_i2c, okay) && \
	CONFIG_I2C
	const uint32_t port0_pin17_config =
		(/* Pin is configured as FC2_CTS_SDA_SSEL0 */
		 IOPCTL_PIO_FUNC1 |
		 /* Enable pull-up / pull-down function */
		 IOPCTL_PIO_PUPD_EN |
		 /* Enable pull-up function */
		 IOPCTL_PIO_PULLUP_EN |
		 /* Enables input buffer function */
		 IOPCTL_PIO_INBUF_EN |
		 /* Normal mode */
		 IOPCTL_PIO_SLEW_RATE_NORMAL |
		 /* Full drive enable */
		 IOPCTL_PIO_FULLDRIVE_EN |
		 /* Analog mux is disabled */
		 IOPCTL_PIO_ANAMUX_DI |
		 /* Pseudo Output Drain is enabled */
		 IOPCTL_PIO_PSEDRAIN_EN |
		 /* Input function is not inverted */
		 IOPCTL_PIO_INV_DI);
	/* PORT0 PIN17 (coords: D7) is configured as FC2_CTS_SDA_SSEL0 */
	IOPCTL_PinMuxSet(IOPCTL, 0U, 17U, port0_pin17_config);

	const uint32_t port0_pin18_config =
		(/* Pin is configured as FC2_RTS_SCL_SSEL1 */
		 IOPCTL_PIO_FUNC1 |
		 /* Enable pull-up / pull-down function */
		 IOPCTL_PIO_PUPD_EN |
		 /* Enable pull-up function */
		 IOPCTL_PIO_PULLUP_EN |
		 /* Enables input buffer function */
		 IOPCTL_PIO_INBUF_EN |
		 /* Normal mode */
		 IOPCTL_PIO_SLEW_RATE_NORMAL |
		 /* Full drive enable */
		 IOPCTL_PIO_FULLDRIVE_EN |
		 /* Analog mux is disabled */
		 IOPCTL_PIO_ANAMUX_DI |
		 /* Pseudo Output Drain is enabled */
		 IOPCTL_PIO_PSEDRAIN_EN |
		 /* Input function is not inverted */
		 IOPCTL_PIO_INV_DI);
	/* PORT0 PIN18 (coords: B7) is configured as FC2_RTS_SCL_SSEL1 */
	IOPCTL_PinMuxSet(IOPCTL, 0U, 18U, port0_pin18_config);
#endif

#ifdef CONFIG_FXOS8700_TRIGGER
	const uint32_t port1_pin5_config =
		(/* Pin is configured as PIO1_5 */
		 IOPCTL_PIO_FUNC0 |
		 /* Disable pull-up / pull-down function */
		 IOPCTL_PIO_PUPD_DI |
		 /* Enable pull-down function */
		 IOPCTL_PIO_PULLDOWN_EN |
		 /* Enables input buffer function */
		 IOPCTL_PIO_INBUF_EN |
		 /* Normal mode */
		 IOPCTL_PIO_SLEW_RATE_NORMAL |
		 /* Normal drive */
		 IOPCTL_PIO_FULLDRIVE_DI |
		 /* Analog mux is disabled */
		 IOPCTL_PIO_ANAMUX_DI |
		 /* Pseudo Output Drain is disabled */
		 IOPCTL_PIO_PSEDRAIN_DI |
		 /* Input function is not inverted */
		 IOPCTL_PIO_INV_DI);
	/* PORT1 PIN5 (coords: J16) is configured as PIO1_5 */
	IOPCTL_PinMuxSet(IOPCTL, 1U, 5U, port1_pin5_config);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(flexcomm5), okay) && CONFIG_SPI
	const uint32_t port1_pin3_config =
		(/* Pin is configured as FC5_SCK */
		 IOPCTL_PIO_FUNC1 |
		 /* Disable pull-up / pull-down function */
		 IOPCTL_PIO_PUPD_DI |
		 /* Enable pull-down function */
		 IOPCTL_PIO_PULLDOWN_EN |
		 /* Enables input buffer function */
		 IOPCTL_PIO_INBUF_EN |
		 /* Normal mode */
		 IOPCTL_PIO_SLEW_RATE_NORMAL |
		 /* Normal drive */
		 IOPCTL_PIO_FULLDRIVE_DI |
		 /* Analog mux is disabled */
		 IOPCTL_PIO_ANAMUX_DI |
		 /* Pseudo Output Drain is disabled */
		 IOPCTL_PIO_PSEDRAIN_DI |
		 /* Input function is not inverted */
		 IOPCTL_PIO_INV_DI);
	/* PORT1 PIN3 (coords: G16) is configured as FC5_SCK */
	IOPCTL_PinMuxSet(IOPCTL, 1U, 3U, port1_pin3_config);

	const uint32_t port1_pin4_config =
		(/* Pin is configured as FC5_MISO */
		 IOPCTL_PIO_FUNC1 |
		 /* Disable pull-up / pull-down function */
		 IOPCTL_PIO_PUPD_DI |
		 /* Enable pull-down function */
		 IOPCTL_PIO_PULLDOWN_EN |
		 /* Enables input buffer function */
		 IOPCTL_PIO_INBUF_EN |
		 /* Normal mode */
		 IOPCTL_PIO_SLEW_RATE_NORMAL |
		 /* Normal drive */
		 IOPCTL_PIO_FULLDRIVE_DI |
		 /* Analog mux is disabled */
		 IOPCTL_PIO_ANAMUX_DI |
		 /* Pseudo Output Drain is disabled */
		 IOPCTL_PIO_PSEDRAIN_DI |
		 /* Input function is not inverted */
		 IOPCTL_PIO_INV_DI);
	/* PORT1 PIN4 (coords: G17) is configured as FC5_TXD_SCL_MISO_WS */
	IOPCTL_PinMuxSet(IOPCTL, 1U, 4U, port1_pin4_config);

	const uint32_t port1_pin5_config =
		(/* Pin is configured as FC5_MOSI */
		 IOPCTL_PIO_FUNC1 |
		 /* Disable pull-up / pull-down function */
		 IOPCTL_PIO_PUPD_DI |
		 /* Enable pull-down function */
		 IOPCTL_PIO_PULLDOWN_EN |
		 /* Enables input buffer function */
		 IOPCTL_PIO_INBUF_EN |
		 /* Normal mode */
		 IOPCTL_PIO_SLEW_RATE_NORMAL |
		 /* Normal drive */
		 IOPCTL_PIO_FULLDRIVE_DI |
		 /* Analog mux is disabled */
		 IOPCTL_PIO_ANAMUX_DI |
		 /* Pseudo Output Drain is disabled */
		 IOPCTL_PIO_PSEDRAIN_DI |
		 /* Input function is not inverted */
		 IOPCTL_PIO_INV_DI);
	/* PORT1 PIN5 (coords: J16) is configured as FC5_RXD_SDA_MOSI_DATA */
	IOPCTL_PinMuxSet(IOPCTL, 1U, 5U, port1_pin5_config);

	const uint32_t port1_pin6_config =
		(/* Pin is configured as FC5_SSEL0 */
		 IOPCTL_PIO_FUNC1 |
		 /* Disable pull-up / pull-down function */
		 IOPCTL_PIO_PUPD_DI |
		 /* Enable pull-down function */
		 IOPCTL_PIO_PULLDOWN_EN |
		 /* Enables input buffer function */
		 IOPCTL_PIO_INBUF_EN |
		 /* Normal mode */
		 IOPCTL_PIO_SLEW_RATE_NORMAL |
		 /* Normal drive */
		 IOPCTL_PIO_FULLDRIVE_DI |
		 /* Analog mux is disabled */
		 IOPCTL_PIO_ANAMUX_DI |
		 /* Pseudo Output Drain is disabled */
		 IOPCTL_PIO_PSEDRAIN_DI |
		 /* Input function is not inverted */
		 IOPCTL_PIO_INV_DI);
	/* PORT1 PIN6 (coords: J17) is configured as FC5_CTS_SDA_SSEL0 */
	IOPCTL_PinMuxSet(IOPCTL, 1U, 6U, port1_pin6_config);
#endif

	return 0;
}

SYS_INIT(mimxrt685_evk_pinmux_init, PRE_KERNEL_1, CONFIG_PINMUX_INIT_PRIORITY);
