/*
 * Copyright (c) 2018, Diego Sueiro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include "device_imx.h"

static int warp7_m4_pinmux_init(const struct device *dev)
{
	ARG_UNUSED(dev);
#if defined(CONFIG_FXOS8700) || defined(CONFIG_FXAS21002)
	IOMUXC_SW_MUX_CTL_PAD_ENET1_RGMII_RD0 =
		IOMUXC_SW_MUX_CTL_PAD_ENET1_RGMII_RD0_MUX_MODE(5);
	IOMUXC_SW_PAD_CTL_PAD_ENET1_RGMII_RD0 =
		IOMUXC_SW_PAD_CTL_PAD_ENET1_RGMII_RD0_PE_MASK |
		IOMUXC_SW_PAD_CTL_PAD_ENET1_RGMII_RD0_PS(1) |
		IOMUXC_SW_PAD_CTL_PAD_ENET1_RGMII_RD0_HYS_MASK |
		IOMUXC_SW_PAD_CTL_PAD_ENET1_RGMII_RD0_DSE(0);
#endif

#if DT_PHA_HAS_CELL(DT_ALIAS(sw0), gpios, pin)
	IOMUXC_SW_MUX_CTL_PAD_ENET1_RGMII_RD1 =
		IOMUXC_SW_MUX_CTL_PAD_ENET1_RGMII_RD1_MUX_MODE(5);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart2), okay) && CONFIG_SERIAL
	IOMUXC_SW_MUX_CTL_PAD_UART2_RX_DATA =
		IOMUXC_SW_MUX_CTL_PAD_UART2_RX_DATA_MUX_MODE(0);
	IOMUXC_SW_MUX_CTL_PAD_UART2_TX_DATA =
		IOMUXC_SW_MUX_CTL_PAD_UART2_TX_DATA_MUX_MODE(0);
	IOMUXC_SW_PAD_CTL_PAD_UART2_RX_DATA =
		IOMUXC_SW_PAD_CTL_PAD_UART2_RX_DATA_PE_MASK |
		IOMUXC_SW_PAD_CTL_PAD_UART2_RX_DATA_PS(3) |
		IOMUXC_SW_PAD_CTL_PAD_UART2_RX_DATA_HYS_MASK |
		IOMUXC_SW_PAD_CTL_PAD_UART2_RX_DATA_DSE(0);

	IOMUXC_SW_PAD_CTL_PAD_UART2_TX_DATA =
		IOMUXC_SW_PAD_CTL_PAD_UART2_TX_DATA_PE_MASK |
		IOMUXC_SW_PAD_CTL_PAD_UART2_TX_DATA_PS(3) |
		IOMUXC_SW_PAD_CTL_PAD_UART2_RX_DATA_HYS_MASK |
		IOMUXC_SW_PAD_CTL_PAD_UART2_TX_DATA_DSE(0);

	/* Select TX_PAD for RX data (DCE mode...) */
	IOMUXC_UART2_RX_DATA_SELECT_INPUT =
		IOMUXC_UART2_RX_DATA_SELECT_INPUT_DAISY(2);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(uart6), okay) && CONFIG_SERIAL
	IOMUXC_SW_MUX_CTL_PAD_ECSPI1_SCLK =
		IOMUXC_SW_MUX_CTL_PAD_ECSPI1_SCLK_MUX_MODE(1);
	IOMUXC_SW_MUX_CTL_PAD_ECSPI1_MOSI =
		IOMUXC_SW_MUX_CTL_PAD_ECSPI1_MOSI_MUX_MODE(1);
	IOMUXC_SW_PAD_CTL_PAD_ECSPI1_SCLK =
		IOMUXC_SW_PAD_CTL_PAD_ECSPI1_SCLK_PE_MASK |
		IOMUXC_SW_PAD_CTL_PAD_ECSPI1_SCLK_PS(3) |
		IOMUXC_SW_PAD_CTL_PAD_ECSPI1_SCLK_HYS_MASK |
		IOMUXC_SW_PAD_CTL_PAD_ECSPI1_SCLK_DSE(0);

	IOMUXC_SW_PAD_CTL_PAD_ECSPI1_MOSI =
		IOMUXC_SW_PAD_CTL_PAD_ECSPI1_MOSI_PE_MASK |
		IOMUXC_SW_PAD_CTL_PAD_ECSPI1_MOSI_PS(3) |
		IOMUXC_SW_PAD_CTL_PAD_ECSPI1_MOSI_HYS_MASK |
		IOMUXC_SW_PAD_CTL_PAD_ECSPI1_MOSI_DSE(0);

	/* Select ECSPI1_SCLK_ALT1 for RX data */
	IOMUXC_UART6_RX_DATA_SELECT_INPUT =
		IOMUXC_UART6_RX_DATA_SELECT_INPUT_DAISY(2);
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c1), okay) && CONFIG_I2C
	IOMUXC_SW_MUX_CTL_PAD_I2C1_SCL =
		IOMUXC_SW_MUX_CTL_PAD_I2C1_SCL_MUX_MODE(0) |
		IOMUXC_SW_MUX_CTL_PAD_I2C1_SCL_SION_MASK;
	IOMUXC_SW_MUX_CTL_PAD_I2C1_SDA =
		IOMUXC_SW_MUX_CTL_PAD_I2C1_SDA_MUX_MODE(0) |
		IOMUXC_SW_MUX_CTL_PAD_I2C1_SDA_SION_MASK;

	IOMUXC_I2C1_SCL_SELECT_INPUT = IOMUXC_I2C1_SCL_SELECT_INPUT_DAISY(1);
	IOMUXC_I2C1_SDA_SELECT_INPUT = IOMUXC_I2C1_SDA_SELECT_INPUT_DAISY(1);

	IOMUXC_SW_PAD_CTL_PAD_I2C1_SCL =
		IOMUXC_SW_PAD_CTL_PAD_I2C1_SCL_PE_MASK |
		IOMUXC_SW_PAD_CTL_PAD_I2C1_SCL_PS(3) |
		IOMUXC_SW_PAD_CTL_PAD_I2C1_SCL_DSE(0) |
		IOMUXC_SW_PAD_CTL_PAD_I2C1_SCL_HYS_MASK;

	IOMUXC_SW_PAD_CTL_PAD_I2C1_SDA =
		IOMUXC_SW_PAD_CTL_PAD_I2C1_SDA_PE_MASK |
		IOMUXC_SW_PAD_CTL_PAD_I2C1_SDA_PS(3) |
		IOMUXC_SW_PAD_CTL_PAD_I2C1_SDA_DSE(0) |
		IOMUXC_SW_PAD_CTL_PAD_I2C1_SDA_HYS_MASK;
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c2), okay) && CONFIG_I2C
	IOMUXC_SW_MUX_CTL_PAD_I2C2_SCL =
		IOMUXC_SW_MUX_CTL_PAD_I2C2_SCL_MUX_MODE(0) |
		IOMUXC_SW_MUX_CTL_PAD_I2C2_SCL_SION_MASK;
	IOMUXC_SW_MUX_CTL_PAD_I2C2_SDA =
		IOMUXC_SW_MUX_CTL_PAD_I2C2_SDA_MUX_MODE(0) |
		IOMUXC_SW_MUX_CTL_PAD_I2C2_SDA_SION_MASK;

	IOMUXC_I2C2_SCL_SELECT_INPUT = IOMUXC_I2C2_SCL_SELECT_INPUT_DAISY(1);
	IOMUXC_I2C2_SDA_SELECT_INPUT = IOMUXC_I2C2_SDA_SELECT_INPUT_DAISY(1);

	IOMUXC_SW_PAD_CTL_PAD_I2C2_SCL =
		IOMUXC_SW_PAD_CTL_PAD_I2C2_SCL_PE_MASK |
		IOMUXC_SW_PAD_CTL_PAD_I2C2_SCL_PS(3) |
		IOMUXC_SW_PAD_CTL_PAD_I2C2_SCL_DSE(0) |
		IOMUXC_SW_PAD_CTL_PAD_I2C2_SCL_HYS_MASK;

	IOMUXC_SW_PAD_CTL_PAD_I2C2_SDA =
		IOMUXC_SW_PAD_CTL_PAD_I2C2_SDA_PE_MASK |
		IOMUXC_SW_PAD_CTL_PAD_I2C2_SDA_PS(3) |
		IOMUXC_SW_PAD_CTL_PAD_I2C2_SDA_DSE(0) |
		IOMUXC_SW_PAD_CTL_PAD_I2C2_SDA_HYS_MASK;
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c3), okay) && CONFIG_I2C
	IOMUXC_SW_MUX_CTL_PAD_I2C3_SCL =
		IOMUXC_SW_MUX_CTL_PAD_I2C3_SCL_MUX_MODE(0) |
		IOMUXC_SW_MUX_CTL_PAD_I2C3_SCL_SION_MASK;
	IOMUXC_SW_MUX_CTL_PAD_I2C3_SDA =
		IOMUXC_SW_MUX_CTL_PAD_I2C3_SDA_MUX_MODE(0) |
		IOMUXC_SW_MUX_CTL_PAD_I2C3_SDA_SION_MASK;

	IOMUXC_I2C3_SCL_SELECT_INPUT = IOMUXC_I2C3_SCL_SELECT_INPUT_DAISY(2);
	IOMUXC_I2C3_SDA_SELECT_INPUT = IOMUXC_I2C3_SDA_SELECT_INPUT_DAISY(2);

	IOMUXC_SW_PAD_CTL_PAD_I2C3_SCL =
		IOMUXC_SW_PAD_CTL_PAD_I2C3_SCL_PE_MASK |
		IOMUXC_SW_PAD_CTL_PAD_I2C3_SCL_PS(3) |
		IOMUXC_SW_PAD_CTL_PAD_I2C3_SCL_DSE(0) |
		IOMUXC_SW_PAD_CTL_PAD_I2C3_SCL_HYS_MASK;

	IOMUXC_SW_PAD_CTL_PAD_I2C3_SDA =
		IOMUXC_SW_PAD_CTL_PAD_I2C3_SDA_PE_MASK |
		IOMUXC_SW_PAD_CTL_PAD_I2C3_SDA_PS(3) |
		IOMUXC_SW_PAD_CTL_PAD_I2C3_SDA_DSE(0) |
		IOMUXC_SW_PAD_CTL_PAD_I2C3_SDA_HYS_MASK;
#endif

#if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c4), okay) && CONFIG_I2C
	IOMUXC_SW_MUX_CTL_PAD_I2C4_SCL =
		IOMUXC_SW_MUX_CTL_PAD_I2C4_SCL_MUX_MODE(0) |
		IOMUXC_SW_MUX_CTL_PAD_I2C4_SCL_SION_MASK;
	IOMUXC_SW_MUX_CTL_PAD_I2C4_SDA =
		IOMUXC_SW_MUX_CTL_PAD_I2C4_SDA_MUX_MODE(0) |
		IOMUXC_SW_MUX_CTL_PAD_I2C4_SDA_SION_MASK;

	IOMUXC_I2C4_SCL_SELECT_INPUT = IOMUXC_I2C4_SCL_SELECT_INPUT_DAISY(2);
	IOMUXC_I2C4_SDA_SELECT_INPUT = IOMUXC_I2C4_SDA_SELECT_INPUT_DAISY(2);

	IOMUXC_SW_PAD_CTL_PAD_I2C4_SCL =
		IOMUXC_SW_PAD_CTL_PAD_I2C4_SCL_PE_MASK |
		IOMUXC_SW_PAD_CTL_PAD_I2C4_SCL_PS(3) |
		IOMUXC_SW_PAD_CTL_PAD_I2C4_SCL_DSE(0) |
		IOMUXC_SW_PAD_CTL_PAD_I2C4_SCL_HYS_MASK;

	IOMUXC_SW_PAD_CTL_PAD_I2C4_SDA =
		IOMUXC_SW_PAD_CTL_PAD_I2C4_SDA_PE_MASK |
		IOMUXC_SW_PAD_CTL_PAD_I2C4_SDA_PS(3) |
		IOMUXC_SW_PAD_CTL_PAD_I2C4_SDA_DSE(0) |
		IOMUXC_SW_PAD_CTL_PAD_I2C4_SDA_HYS_MASK;
#endif

	return 0;
}

SYS_INIT(warp7_m4_pinmux_init, PRE_KERNEL_1, 0);
