/*
 * Copyright (c) 2016 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/pinmux.h>
#include <sys/sys_io.h>

#include <pinmux/stm32/pinmux_stm32.h>

/* pin assignments for STM32F429I-DISC1 board */
static const struct pin_config pinconf[] = {
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart1), okay) && CONFIG_SERIAL
	{ STM32_PIN_PA9, STM32F4_PINMUX_FUNC_PA9_USART1_TX },
	{ STM32_PIN_PA10, STM32F4_PINMUX_FUNC_PA10_USART1_RX },
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(usart2), okay) && CONFIG_SERIAL
	{ STM32_PIN_PA2, STM32F4_PINMUX_FUNC_PA2_USART2_TX },
	{ STM32_PIN_PA3, STM32F4_PINMUX_FUNC_PA3_USART2_RX },
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(spi5), okay) && CONFIG_SPI
#ifdef CONFIG_SPI_STM32_USE_HW_SS
	{ STM32_PIN_PF6, STM32F4_PINMUX_FUNC_PF6_SPI5_MASTER_NSS },
#endif /* CONFIG_SPI_STM32_USE_HW_SS */
	{ STM32_PIN_PF7, STM32F4_PINMUX_FUNC_PF7_SPI5_MASTER_SCK },
	{ STM32_PIN_PF8, STM32F4_PINMUX_FUNC_PF8_SPI5_MASTER_MISO },
	{ STM32_PIN_PF9, STM32F4_PINMUX_FUNC_PF9_SPI5_MASTER_MOSI },
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay) && CONFIG_I2C
	{ STM32_PIN_PB8, STM32F4_PINMUX_FUNC_PB8_I2C1_SCL },
	{ STM32_PIN_PB9, STM32F4_PINMUX_FUNC_PB9_I2C1_SDA },
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2), okay) && CONFIG_I2C
	{ STM32_PIN_PB10, STM32F4_PINMUX_FUNC_PB10_I2C2_SCL },
	{ STM32_PIN_PB11, STM32F4_PINMUX_FUNC_PB11_I2C2_SDA },
#endif
#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c3), okay) && CONFIG_I2C
	{ STM32_PIN_PA8, STM32F4_PINMUX_FUNC_PA8_I2C3_SCL },
	{ STM32_PIN_PC9, STM32F4_PINMUX_FUNC_PC9_I2C3_SDA },
#endif
};

static int pinmux_stm32_init(const struct device *port)
{
	ARG_UNUSED(port);

	stm32_setup_pins(pinconf, ARRAY_SIZE(pinconf));

	return 0;
}

SYS_INIT(pinmux_stm32_init, PRE_KERNEL_1,
	 CONFIG_PINMUX_STM32_DEVICE_INITIALIZATION_PRIORITY);
