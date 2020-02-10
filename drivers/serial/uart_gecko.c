/*
 * Copyright (c) 2017, Christian Taedcke
 * Copyright (c) 2019 Lemonbeat GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <drivers/uart.h>
#include <em_usart.h>
#include <em_gpio.h>
#include <em_cmu.h>
#include <soc.h>

#define USART_PREFIX cmuClock_USART
#define UART_PREFIX cmuClock_UART
#define CLOCK_ID_PRFX2(prefix, suffix) prefix##suffix
#define CLOCK_ID_PRFX(prefix, suffix) CLOCK_ID_PRFX2(prefix, suffix)
#define CLOCK_USART(id) CLOCK_ID_PRFX(USART_PREFIX, id)
#define CLOCK_UART(id) CLOCK_ID_PRFX(UART_PREFIX, id)

/* Helper define to determine if SOC supports hardware flow control */
#if ((_SILICON_LABS_32B_SERIES > 0) || \
	(defined(_USART_ROUTEPEN_RTSPEN_MASK) && \
	defined(_USART_ROUTEPEN_CTSPEN_MASK)))
#define HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC
#endif

/* Sanity check for hardware flow control */
#if ((DT_INST_0_SILABS_GECKO_USART_HW_FLOW_CONTROL == 1) || \
	(DT_INST_1_SILABS_GECKO_USART_HW_FLOW_CONTROL == 1)  || \
	(DT_INST_2_SILABS_GECKO_USART_HW_FLOW_CONTROL == 1)  || \
	(DT_INST_3_SILABS_GECKO_USART_HW_FLOW_CONTROL == 1)  || \
	(DT_INST_0_SILABS_GECKO_UART_HW_FLOW_CONTROL == 1)   || \
	(DT_INST_1_SILABS_GECKO_UART_HW_FLOW_CONTROL == 1))  && \
	(!(defined(HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC)))
#error "Hardware flow control is activated for at least one UART/USART, \
but not supported by this SOC"
#endif

/**
 * @brief Maps init value for hw_flowcontrol
 *
 * @param fc_enabled Value for DT_INST_i_SILABS_GECKO_USART_HW_FLOW_CONTROL [0 or 1]
 * @return Mapped value according to enum USART_HwFlowControl_TypeDef
 */
#ifdef HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC
#define MAP_HW_FLOWCONTROL_INIT_VAL(fc_enabled) \
	(fc_enabled == 1 ? usartHwFlowControlCtsAndRts : usartHwFlowControlNone)
#endif

/**
 * @brief Config struct for UART
 */
struct uart_gecko_config {
	USART_TypeDef *base;
	CMU_Clock_TypeDef clock;
	u32_t baud_rate;
#ifdef HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC
	USART_HwFlowControl_TypeDef hw_flowcontrol;
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(struct device *dev);
#endif
	struct soc_gpio_pin pin_rx;
	struct soc_gpio_pin pin_tx;
#ifdef HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC
	struct soc_gpio_pin pin_rts;
	struct soc_gpio_pin pin_cts;
#endif
#ifdef CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION
	u8_t loc_rx;
	u8_t loc_tx;
#ifdef HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC
	u8_t loc_rts;
	u8_t loc_cts;
#endif
#else
	u8_t loc;
#endif
};

struct uart_gecko_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
};

/** @defgroup uart_gecko_group1 Polling-based UART
 *  This is a part of the UART driver using polling.
 *  @{
 */

/**
 * @brief get received data for polling-based operation
 *
 * @param dev The device to read from
 * @param c Pointer to received data
 */
static int uart_gecko_poll_in(struct device *dev, unsigned char *c)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	u32_t flags = USART_StatusGet(config->base);

	if (flags & USART_STATUS_RXDATAV) {
		*c = USART_Rx(config->base);
		return 0;
	}

	return -1;
}

/**
 * @brief write data for polling-based operation
 *
 * @param dev The device to write to
 * @param c Data to be transmitted
 */
static void uart_gecko_poll_out(struct device *dev, unsigned char c)
{
	const struct uart_gecko_config *config = dev->config->config_info;

	USART_Tx(config->base, c);
}

/**
 * @brief Check if a device has errors
 *
 * @param dev The device to be checked
 * @return Status of type uart_rx_stop_reason (enum)
 */
static int uart_gecko_err_check(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	u32_t flags = USART_IntGet(config->base);
	int err = 0;

	if (flags & USART_IF_RXOF) {
		err |= UART_ERROR_OVERRUN;
	}

	if (flags & USART_IF_PERR) {
		err |= UART_ERROR_PARITY;
	}

	if (flags & USART_IF_FERR) {
		err |= UART_ERROR_FRAMING;
	}

	USART_IntClear(config->base, USART_IF_RXOF |
		       USART_IF_PERR |
		       USART_IF_FERR);

	return err;
}
/** @} */ /* end of uart_gecko_group1 */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
/** @defgroup uart_gecko_group2 Interrupt-driven UART
 *  This is a part of the UART driver using Interrupts.
 *  @{
 */
static int uart_gecko_fifo_fill(struct device *dev, const u8_t *tx_data,
			       int len)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	u8_t num_tx = 0U;

	while ((len - num_tx > 0) &&
	       (config->base->STATUS & USART_STATUS_TXBL)) {

		config->base->TXDATA = (u32_t)tx_data[num_tx++];
	}

	return num_tx;
}

static int uart_gecko_fifo_read(struct device *dev, u8_t *rx_data,
			       const int len)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	u8_t num_rx = 0U;

	while ((len - num_rx > 0) &&
	       (config->base->STATUS & USART_STATUS_RXDATAV)) {

		rx_data[num_rx++] = (u8_t)config->base->RXDATA;
	}

	return num_rx;
}

static void uart_gecko_irq_tx_enable(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	u32_t mask = USART_IEN_TXBL | USART_IEN_TXC;

	USART_IntEnable(config->base, mask);
}

static void uart_gecko_irq_tx_disable(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	u32_t mask = USART_IEN_TXBL | USART_IEN_TXC;

	USART_IntDisable(config->base, mask);
}

static int uart_gecko_irq_tx_complete(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	u32_t flags = USART_IntGet(config->base);

	USART_IntClear(config->base, USART_IF_TXC);

	return (flags & USART_IF_TXC) != 0U;
}

static int uart_gecko_irq_tx_ready(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	u32_t flags = USART_IntGet(config->base);

	return (flags & USART_IF_TXBL) != 0U;
}

static void uart_gecko_irq_rx_enable(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	u32_t mask = USART_IEN_RXDATAV;

	USART_IntEnable(config->base, mask);
}

static void uart_gecko_irq_rx_disable(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	u32_t mask = USART_IEN_RXDATAV;

	USART_IntDisable(config->base, mask);
}

static int uart_gecko_irq_rx_full(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	u32_t flags = USART_IntGet(config->base);

	return (flags & USART_IF_RXDATAV) != 0U;
}

static int uart_gecko_irq_rx_ready(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	u32_t mask = USART_IEN_RXDATAV;

	return (config->base->IEN & mask)
		&& uart_gecko_irq_rx_full(dev);
}

static void uart_gecko_irq_err_enable(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;

	USART_IntEnable(config->base, USART_IF_RXOF |
			 USART_IF_PERR |
			 USART_IF_FERR);
}

static void uart_gecko_irq_err_disable(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;

	USART_IntDisable(config->base, USART_IF_RXOF |
			 USART_IF_PERR |
			 USART_IF_FERR);
}

static int uart_gecko_irq_is_pending(struct device *dev)
{
	return uart_gecko_irq_tx_ready(dev) || uart_gecko_irq_rx_ready(dev);
}

static int uart_gecko_irq_update(struct device *dev)
{
	return 1;
}

static void uart_gecko_irq_callback_set(struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_gecko_data *data = dev->driver_data;

	data->callback = cb;
	data->cb_data = cb_data;
}

static void uart_gecko_isr(void *arg)
{
	struct device *dev = arg;
	struct uart_gecko_data *data = dev->driver_data;

	if (data->callback) {
		data->callback(data->cb_data);
	}
}
/** @} */ /* end of uart_gecko_group2 */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

/**
 * @brief Subroutine initializer of UART pins
 *
 * @param dev UART device to configure
 */
static void uart_gecko_init_pins(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;

	/* configure pins (port, pin, mode, out) */
	soc_gpio_configure(&config->pin_rx);
	soc_gpio_configure(&config->pin_tx);

#ifdef CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION
	/*
	 * For SOCs with configurable pin locations (set in SOC Kconfig)
	 */
	config->base->ROUTEPEN = USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN;
	config->base->ROUTELOC0 =
		(config->loc_tx << _USART_ROUTELOC0_TXLOC_SHIFT) |
		(config->loc_rx << _USART_ROUTELOC0_RXLOC_SHIFT);
	config->base->ROUTELOC1 = _USART_ROUTELOC1_RESETVALUE;
#else
	/*
	 * For olders SOCs with only one pin location
 	 */
	config->base->ROUTE = USART_ROUTE_RXPEN | USART_ROUTE_TXPEN
		| (config->loc << 8);
#endif /* CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION */

#ifdef HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC
 	/* configure HW flow control (RTS, CTS) */
	if(config->hw_flowcontrol != usartHwFlowControlNone){
		/* activate flow control in general */
		soc_gpio_configure(&config->pin_rts);
		soc_gpio_configure(&config->pin_cts);

#ifdef CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION
		/* set enable flag in USARTn_ROUTEPEN (I/O Routing Pin Enable Register) */
		config->base->ROUTEPEN = USART_ROUTEPEN_RXPEN | USART_ROUTEPEN_TXPEN \
			| USART_ROUTEPEN_RTSPEN | USART_ROUTEPEN_CTSPEN;
		/*
		 * set location of RTS/CTS in USARTn_ROUTELOC1
		 * (I/O Routing Location Register)
		 */
		config->base->ROUTELOC1 =
			(config->loc_rts << _USART_ROUTELOC1_RTSLOC_SHIFT) |
			(config->loc_cts << _USART_ROUTELOC1_CTSLOC_SHIFT);
#endif /* CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION */
		/*
		 * enable flow control by settings CTSEN in USARTn_CTRLX
		 * (Control Register Extended)
		 */
		config->base->CTRLX |= USART_CTRLX_CTSEN;
	}
#endif /* HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC */
}

/**
 * @brief Main initializer for UART
 *
 * @param dev UART device to be initialized
 * @return int 0
 */
static int uart_gecko_init(struct device *dev)
{
	const struct uart_gecko_config *config = dev->config->config_info;
	USART_InitAsync_TypeDef usartInit = USART_INITASYNC_DEFAULT;

	/* The peripheral and gpio clock are already enabled from soc and gpio
	 * driver
	 */

	/* Enable USART clock */
	CMU_ClockEnable(config->clock, true);

	/* Init USART */
	usartInit.baudrate = config->baud_rate;
#ifdef HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC
	usartInit.hwFlowControl = config->hw_flowcontrol;
#endif
	USART_InitAsync(config->base, &usartInit);

	/* Initialize USART pins */
	uart_gecko_init_pins(dev);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif

	return 0;
}

/**
 * @brief Main configuration of driver API for polling- or IR-based operation
 */
static const struct uart_driver_api uart_gecko_driver_api = {
	.poll_in = uart_gecko_poll_in,
	.poll_out = uart_gecko_poll_out,
	.err_check = uart_gecko_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_gecko_fifo_fill,
	.fifo_read = uart_gecko_fifo_read,
	.irq_tx_enable = uart_gecko_irq_tx_enable,
	.irq_tx_disable = uart_gecko_irq_tx_disable,
	.irq_tx_complete = uart_gecko_irq_tx_complete,
	.irq_tx_ready = uart_gecko_irq_tx_ready,
	.irq_rx_enable = uart_gecko_irq_rx_enable,
	.irq_rx_disable = uart_gecko_irq_rx_disable,
	.irq_rx_ready = uart_gecko_irq_rx_ready,
	.irq_err_enable = uart_gecko_irq_err_enable,
	.irq_err_disable = uart_gecko_irq_err_disable,
	.irq_is_pending = uart_gecko_irq_is_pending,
	.irq_update = uart_gecko_irq_update,
	.irq_callback_set = uart_gecko_irq_callback_set,
#endif
};

/**
 * @brief Helper to define a single UART/USART pin
 *
 * Defines for UART/USART pins are generated from the devicetree
 * Notation as struct `soc_gpio_pin`, i.e. {port, pin, mode, out}
 *
 * @param uart_type UART or USART
 * @param nr number of UART/USART (0,...,3 for USART0,...,USART3)
 * @param pin name of pin to define, i.e. RX, TX, RTS, CTS
 * @param pin_mode termination/direction of the pin (gpioModeInput, gpioModePushPull, ...)
 */
#define UART_PIN_INIT(uart_type, nr, pin, pin_mode) \
	{DT_INST_##nr##_SILABS_GECKO_##uart_type##_LOCATION_##pin##_1, \
	 DT_INST_##nr##_SILABS_GECKO_##uart_type##_LOCATION_##pin##_2, pin_mode, 1}

/**
 * @brief Set GPIO assignment for UART pins (RX, TX, RTS, CTS)
 *
 * @param uart_type UART or USART
 * @param nr number of UART/USART (0,...,3 for USART0,...,USART3)
 */
#define CONFIG_UART_PINS_WITH_FLOW_CONTROL(uart_type, nr) \
	.pin_rx  = UART_PIN_INIT(uart_type, nr, RX, gpioModeInput), \
	.pin_tx  = UART_PIN_INIT(uart_type, nr, TX, gpioModePushPull), \
	.pin_rts = UART_PIN_INIT(uart_type, nr, RTS, gpioModePushPull), \
	.pin_cts = UART_PIN_INIT(uart_type, nr, CTS, gpioModeInput),

/**
 * @brief Set GPIO assignment for UART pins (RX, TX)
 *
 * @param uart_type UART or USART
 * @param nr number of UART/USART (0,...,3 for USART0,...,USART3)
 */
#define CONFIG_UART_PINS(uart_type, nr) \
	.pin_rx  = UART_PIN_INIT(uart_type, nr, RX, gpioModeInput), \
	.pin_tx  = UART_PIN_INIT(uart_type, nr, TX, gpioModePushPull),


#ifdef CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION
/**
 * @brief Set alternate function location for UART pins (RX, TX, RTS, CTS)
 *
 * @param uart_type UART or USART
 * @param nr number of UART/USART (0,...,3 for USART0,...,USART3)
 */
#define CONFIG_UART_LOCATIONS_WITH_FLOW_CONTROL(uart_type, nr) \
		.loc_rx  = DT_INST_##nr##_SILABS_GECKO_##uart_type##_LOCATION_RX_0,  \
		.loc_tx  = DT_INST_##nr##_SILABS_GECKO_##uart_type##_LOCATION_TX_0,  \
		.loc_rts = DT_INST_##nr##_SILABS_GECKO_##uart_type##_LOCATION_RTS_0, \
		.loc_cts = DT_INST_##nr##_SILABS_GECKO_##uart_type##_LOCATION_CTS_0,

/**
 * @brief Set alternate function location for UART pins (RX, TX)
 *
 * @param uart_type UART or USART
 * @param nr number of UART/USART (0,...,3 for USART0,...,USART3)
 */
#define CONFIG_UART_LOCATIONS(uart_type, nr) \
		.loc_rx  = DT_INST_##nr##_SILABS_GECKO_##uart_type##_LOCATION_RX_0,  \
		.loc_tx  = DT_INST_##nr##_SILABS_GECKO_##uart_type##_LOCATION_TX_0,
#else
	/* no individual pin location */
#define CONFIG_UART_LOCATIONS(uart_type, nr) \
	.loc = DT_INST_##nr##_SILABS_GECKO_##uart_type##_LOCATION_RX_0,
#endif /* CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION */

#ifdef DT_INST_0_SILABS_GECKO_UART
/**
 * Defines for UART0 pins
 */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_gecko_config_func_0(struct device *dev);
#endif

#if (!defined CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION) && \
	(DT_INST_0_SILABS_GECKO_UART_LOCATION_RX_0 \
	!= DT_INST_0_SILABS_GECKO_UART_LOCATION_TX_0)
#error UART DTS location-* properties must have identical value
#endif

static const struct uart_gecko_config uart_gecko_0_config = {
	.base = (USART_TypeDef *)DT_INST_0_SILABS_GECKO_UART_BASE_ADDRESS,
	.clock = CLOCK_UART(DT_INST_0_SILABS_GECKO_UART_PERIPHERAL_ID),
	.baud_rate = DT_INST_0_SILABS_GECKO_UART_CURRENT_SPEED,
#ifdef HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC
	.hw_flowcontrol = \
		MAP_HW_FLOWCONTROL_INIT_VAL(DT_INST_0_SILABS_GECKO_UART_HW_FLOW_CONTROL),
#endif

#if DT_INST_0_SILABS_GECKO_UART_HW_FLOW_CONTROL == 1
	/* set pin_rx, pin_tx, pin_rts, pin_cts */
	CONFIG_UART_PINS_WITH_FLOW_CONTROL(UART, 0)
	/* set alternate function location of pins */
	CONFIG_UART_LOCATIONS_WITH_FLOW_CONTROL(UART, 0)
#else
	CONFIG_UART_PINS(UART, 0)
	CONFIG_UART_LOCATIONS(UART, 0)
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = uart_gecko_config_func_0,
#endif
};

static struct uart_gecko_data uart_gecko_0_data;

DEVICE_AND_API_INIT(uart_0, DT_INST_0_SILABS_GECKO_UART_LABEL, &uart_gecko_init,
		    &uart_gecko_0_data, &uart_gecko_0_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &uart_gecko_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_gecko_config_func_0(struct device *dev)
{
	IRQ_CONNECT(DT_INST_0_SILABS_GECKO_UART_IRQ_RX,
		    DT_INST_0_SILABS_GECKO_UART_IRQ_RX_PRIORITY, uart_gecko_isr,
		    DEVICE_GET(uart_0), 0);
	IRQ_CONNECT(DT_INST_0_SILABS_GECKO_UART_IRQ_TX,
		    DT_INST_0_SILABS_GECKO_UART_IRQ_TX_PRIORITY, uart_gecko_isr,
		    DEVICE_GET(uart_0), 0);

	irq_enable(DT_INST_0_SILABS_GECKO_UART_IRQ_RX);
	irq_enable(DT_INST_0_SILABS_GECKO_UART_IRQ_TX);
}
#endif

#endif /* DT_INST_0_SILABS_GECKO_UART */

#ifdef DT_INST_1_SILABS_GECKO_UART
/**
 * Defines for UART1 pins
 */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_gecko_config_func_1(struct device *dev);
#endif

#if (!defined CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION) && \
    (DT_INST_1_SILABS_GECKO_UART_LOCATION_RX_0 \
	!= DT_INST_1_SILABS_GECKO_UART_LOCATION_TX_0)
#error UART DTS location-* properties must have identical value
#endif

static const struct uart_gecko_config uart_gecko_1_config = {
	.base = (USART_TypeDef *)DT_INST_1_SILABS_GECKO_UART_BASE_ADDRESS,
	.clock = CLOCK_UART(DT_INST_1_SILABS_GECKO_UART_PERIPHERAL_ID),
	.baud_rate = DT_INST_1_SILABS_GECKO_UART_CURRENT_SPEED,
#ifdef HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC
	.hw_flowcontrol = \
		MAP_HW_FLOWCONTROL_INIT_VAL(DT_INST_1_SILABS_GECKO_UART_HW_FLOW_CONTROL),
#endif
#if DT_INST_1_SILABS_GECKO_UART_HW_FLOW_CONTROL == 1
	/* set pin_rx, pin_tx, pin_rts, pin_cts */
	CONFIG_UART_PINS_WITH_FLOW_CONTROL(UART, 1)
	/* set alternate function location of pins */
	CONFIG_UART_LOCATIONS_WITH_FLOW_CONTROL(UART, 1)
#else
	CONFIG_UART_PINS(UART, 1)
	CONFIG_UART_LOCATIONS(UART, 1)
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = uart_gecko_config_func_1,
#endif
};

static struct uart_gecko_data uart_gecko_1_data;

DEVICE_AND_API_INIT(uart_1, DT_INST_1_SILABS_GECKO_UART_LABEL, &uart_gecko_init,
		    &uart_gecko_1_data, &uart_gecko_1_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &uart_gecko_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_gecko_config_func_1(struct device *dev)
{
	IRQ_CONNECT(DT_INST_1_SILABS_GECKO_UART_IRQ_RX,
		    DT_INST_1_SILABS_GECKO_UART_IRQ_RX_PRIORITY, uart_gecko_isr,
		    DEVICE_GET(uart_1), 0);
	IRQ_CONNECT(DT_INST_1_SILABS_GECKO_UART_IRQ_TX,
		    DT_INST_1_SILABS_GECKO_UART_IRQ_TX_PRIORITY, uart_gecko_isr,
		    DEVICE_GET(uart_1), 0);

	irq_enable(DT_INST_1_SILABS_GECKO_UART_IRQ_RX);
	irq_enable(DT_INST_1_SILABS_GECKO_UART_IRQ_TX);
}
#endif

#endif /* DT_INST_1_SILABS_GECKO_UART */

#ifdef DT_INST_0_SILABS_GECKO_USART
/**
 * Defines for USART0 pins
 */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_gecko_config_func_0(struct device *dev);
#endif

#if (!defined CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION) && \
    (DT_INST_0_SILABS_GECKO_USART_LOCATION_RX_0 \
	!= DT_INST_0_SILABS_GECKO_USART_LOCATION_TX_0)
#error USART DTS location-* properties must have identical value
#endif

static const struct uart_gecko_config usart_gecko_0_config = {
	.base = (USART_TypeDef *)DT_INST_0_SILABS_GECKO_USART_BASE_ADDRESS,
	.clock = CLOCK_USART(DT_INST_0_SILABS_GECKO_USART_PERIPHERAL_ID),
	.baud_rate = DT_INST_0_SILABS_GECKO_USART_CURRENT_SPEED,
#ifdef HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC
	.hw_flowcontrol = \
		MAP_HW_FLOWCONTROL_INIT_VAL(DT_INST_0_SILABS_GECKO_USART_HW_FLOW_CONTROL),
#endif

#if DT_INST_0_SILABS_GECKO_USART_HW_FLOW_CONTROL == 1
	/* set pin_rx, pin_tx, pin_rts, pin_cts */
	CONFIG_UART_PINS_WITH_FLOW_CONTROL(USART, 0)
	/* set alternate function location of pins */
	CONFIG_UART_LOCATIONS_WITH_FLOW_CONTROL(USART, 0)
#else
	CONFIG_UART_PINS(USART, 0)
	CONFIG_UART_LOCATIONS(USART, 0)
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = usart_gecko_config_func_0,
#endif
};

static struct uart_gecko_data usart_gecko_0_data;

DEVICE_AND_API_INIT(usart_0, DT_INST_0_SILABS_GECKO_USART_LABEL,
		    &uart_gecko_init, &usart_gecko_0_data,
		    &usart_gecko_0_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &uart_gecko_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_gecko_config_func_0(struct device *dev)
{
	IRQ_CONNECT(DT_INST_0_SILABS_GECKO_USART_IRQ_RX,
		    DT_INST_0_SILABS_GECKO_USART_IRQ_RX_PRIORITY,
		    uart_gecko_isr, DEVICE_GET(usart_0), 0);
	IRQ_CONNECT(DT_INST_0_SILABS_GECKO_USART_IRQ_TX,
		    DT_INST_0_SILABS_GECKO_USART_IRQ_TX_PRIORITY,
		    uart_gecko_isr, DEVICE_GET(usart_0), 0);

	irq_enable(DT_INST_0_SILABS_GECKO_USART_IRQ_RX);
	irq_enable(DT_INST_0_SILABS_GECKO_USART_IRQ_TX);
}
#endif

#endif /* DT_INST_0_SILABS_GECKO_USART */

#ifdef DT_INST_1_SILABS_GECKO_USART
/**
 * Defines for USART1 pins
 */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_gecko_config_func_1(struct device *dev);
#endif

#if (!defined CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION) && \
	(DT_INST_1_SILABS_GECKO_USART_LOCATION_RX_0 \
	!= DT_INST_1_SILABS_GECKO_USART_LOCATION_TX_0)
#error USART DTS location-* properties must have identical value
#endif

static const struct uart_gecko_config usart_gecko_1_config = {
	.base = (USART_TypeDef *)DT_INST_1_SILABS_GECKO_USART_BASE_ADDRESS,
	.clock = CLOCK_USART(DT_INST_1_SILABS_GECKO_USART_PERIPHERAL_ID),
	.baud_rate = DT_INST_1_SILABS_GECKO_USART_CURRENT_SPEED,
#ifdef HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC
	.hw_flowcontrol = \
		MAP_HW_FLOWCONTROL_INIT_VAL(DT_INST_1_SILABS_GECKO_USART_HW_FLOW_CONTROL),
#endif

#if DT_INST_1_SILABS_GECKO_USART_HW_FLOW_CONTROL == 1
	/* set pin_rx, pin_tx, pin_rts, pin_cts */
	CONFIG_UART_PINS_WITH_FLOW_CONTROL(USART, 1)
	/* set alternate function location of pins */
	CONFIG_UART_LOCATIONS_WITH_FLOW_CONTROL(USART, 1)
#else
	CONFIG_UART_PINS(USART, 1)
	CONFIG_UART_LOCATIONS(USART, 1)
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = usart_gecko_config_func_1,
#endif
};

static struct uart_gecko_data usart_gecko_1_data;

DEVICE_AND_API_INIT(usart_1, DT_INST_1_SILABS_GECKO_USART_LABEL,
		    &uart_gecko_init, &usart_gecko_1_data,
		    &usart_gecko_1_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &uart_gecko_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_gecko_config_func_1(struct device *dev)
{
	IRQ_CONNECT(DT_INST_1_SILABS_GECKO_USART_IRQ_RX,
		    DT_INST_1_SILABS_GECKO_USART_IRQ_RX_PRIORITY,
		    uart_gecko_isr, DEVICE_GET(usart_1), 0);
	IRQ_CONNECT(DT_INST_1_SILABS_GECKO_USART_IRQ_TX,
		    DT_INST_1_SILABS_GECKO_USART_IRQ_TX_PRIORITY,
		    uart_gecko_isr, DEVICE_GET(usart_1), 0);

	irq_enable(DT_INST_1_SILABS_GECKO_USART_IRQ_RX);
	irq_enable(DT_INST_1_SILABS_GECKO_USART_IRQ_TX);
}
#endif

#endif /* DT_INST_1_SILABS_GECKO_USART */

#ifdef DT_INST_2_SILABS_GECKO_USART
/**
 * Defines for USART2 pins
 */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_gecko_config_func_2(struct device *dev);
#endif

#if (!defined CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION) && \
	(DT_INST_2_SILABS_GECKO_USART_LOCATION_RX_0 \
	!= DT_INST_2_SILABS_GECKO_USART_LOCATION_TX_0)
#error USART DTS location-* properties must have identical value
#endif

static const struct uart_gecko_config usart_gecko_2_config = {
	.base = (USART_TypeDef *)DT_INST_2_SILABS_GECKO_USART_BASE_ADDRESS,
	.clock = CLOCK_USART(DT_INST_2_SILABS_GECKO_USART_PERIPHERAL_ID),
	.baud_rate = DT_INST_2_SILABS_GECKO_USART_CURRENT_SPEED,
#ifdef HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC
	.hw_flowcontrol = \
		MAP_HW_FLOWCONTROL_INIT_VAL(DT_INST_2_SILABS_GECKO_USART_HW_FLOW_CONTROL),
#endif

#if DT_INST_2_SILABS_GECKO_USART_HW_FLOW_CONTROL == 1
	/* set pin_rx, pin_tx, pin_rts, pin_cts */
	CONFIG_UART_PINS_WITH_FLOW_CONTROL(USART, 2)
	/* set alternate function location of pins */
	CONFIG_UART_LOCATIONS_WITH_FLOW_CONTROL(USART, 2)
#else
	CONFIG_UART_PINS(USART, 2)
	CONFIG_UART_LOCATIONS(USART, 2)
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = usart_gecko_config_func_2,
#endif
};

static struct uart_gecko_data usart_gecko_2_data;

DEVICE_AND_API_INIT(usart_2, DT_INST_2_SILABS_GECKO_USART_LABEL,
		    &uart_gecko_init, &usart_gecko_2_data,
		    &usart_gecko_2_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &uart_gecko_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_gecko_config_func_2(struct device *dev)
{
	IRQ_CONNECT(DT_INST_2_SILABS_GECKO_USART_IRQ_RX,
		    DT_INST_2_SILABS_GECKO_USART_IRQ_RX_PRIORITY,
		    uart_gecko_isr, DEVICE_GET(usart_2), 0);
	IRQ_CONNECT(DT_INST_2_SILABS_GECKO_USART_IRQ_TX,
		    DT_INST_2_SILABS_GECKO_USART_IRQ_TX_PRIORITY,
		    uart_gecko_isr, DEVICE_GET(usart_2), 0);

	irq_enable(DT_INST_2_SILABS_GECKO_USART_IRQ_RX);
	irq_enable(DT_INST_2_SILABS_GECKO_USART_IRQ_TX);
}
#endif

#endif /* DT_INST_2_SILABS_GECKO_USART */

#ifdef DT_INST_3_SILABS_GECKO_USART
/**
 * Defines for UART3 pins
 */

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_gecko_config_func_3(struct device *dev);
#endif

#if (!defined CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION) && \
	(DT_INST_3_SILABS_GECKO_USART_LOCATION_RX_0 \
	!= DT_INST_3_SILABS_GECKO_USART_LOCATION_TX_0)
#error USART DTS location-* properties must have identical value
#endif

static const struct uart_gecko_config usart_gecko_3_config = {
	.base = (USART_TypeDef *)DT_INST_3_SILABS_GECKO_USART_BASE_ADDRESS,
	.clock = CLOCK_USART(DT_INST_3_SILABS_GECKO_USART_PERIPHERAL_ID),
	.baud_rate = DT_INST_3_SILABS_GECKO_USART_CURRENT_SPEED,
#ifdef HW_FLOWCONTROL_IS_SUPPORTED_BY_SOC
	.hw_flowcontrol = \
		MAP_HW_FLOWCONTROL_INIT_VAL(DT_INST_3_SILABS_GECKO_USART_HW_FLOW_CONTROL),
#endif

#if DT_INST_3_SILABS_GECKO_USART_HW_FLOW_CONTROL == 1
	/* set pin_rx, pin_tx, pin_rts, pin_cts */
	CONFIG_UART_PINS_WITH_FLOW_CONTROL(USART, 3)
	/* set alternate function location of pins */
	CONFIG_UART_LOCATIONS_WITH_FLOW_CONTROL(USART, 3)
#else
	CONFIG_UART_PINS(USART, 3)
	CONFIG_UART_LOCATIONS(USART, 3)
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = usart_gecko_config_func_3,
#endif
};

static struct uart_gecko_data usart_gecko_3_data;

DEVICE_AND_API_INIT(usart_3, DT_INST_3_SILABS_GECKO_USART_LABEL,
		    &uart_gecko_init, &usart_gecko_3_data,
		    &usart_gecko_3_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &uart_gecko_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_gecko_config_func_3(struct device *dev)
{
	IRQ_CONNECT(DT_INST_3_SILABS_GECKO_USART_IRQ_RX,
		    DT_INST_3_SILABS_GECKO_USART_IRQ_RX_PRIORITY,
		    uart_gecko_isr, DEVICE_GET(usart_3), 0);
	IRQ_CONNECT(DT_INST_3_SILABS_GECKO_USART_IRQ_TX,
		    DT_INST_3_SILABS_GECKO_USART_IRQ_TX_PRIORITY,
		    uart_gecko_isr, DEVICE_GET(usart_3), 0);

	irq_enable(DT_INST_3_SILABS_GECKO_USART_IRQ_RX);
	irq_enable(DT_INST_3_SILABS_GECKO_USART_IRQ_TX);
}
#endif

#endif /* DT_INST_3_SILABS_GECKO_USART */

#ifdef DT_INST_4_SILABS_GECKO_USART

#define PIN_USART4_RXD {DT_INST_4_SILABS_GECKO_USART_LOCATION_RX_1, \
		DT_INST_4_SILABS_GECKO_USART_LOCATION_RX_2, gpioModeInput, 1}
#define PIN_USART4_TXD {DT_INST_4_SILABS_GECKO_USART_LOCATION_TX_1, \
		DT_INST_4_SILABS_GECKO_USART_LOCATION_TX_2, gpioModePushPull, 1}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_gecko_config_func_4(struct device *dev);
#endif

static const struct uart_gecko_config usart_gecko_4_config = {
	.base = (USART_TypeDef *)DT_INST_4_SILABS_GECKO_USART_BASE_ADDRESS,
	.clock = CLOCK_USART(DT_INST_4_SILABS_GECKO_USART_PERIPHERAL_ID),
	.baud_rate = DT_INST_4_SILABS_GECKO_USART_CURRENT_SPEED,
	.pin_rx = PIN_USART4_RXD,
	.pin_tx = PIN_USART4_TXD,
#ifdef CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION
	.loc_rx = DT_INST_4_SILABS_GECKO_USART_LOCATION_RX_0,
	.loc_tx = DT_INST_4_SILABS_GECKO_USART_LOCATION_TX_0,
#else
#if DT_INST_4_SILABS_GECKO_USART_LOCATION_RX_0 \
	!= DT_INST_4_SILABS_GECKO_USART_LOCATION_TX_0
#error USART_4 DTS location-* properties must have identical value
#endif
	.loc = DT_INST_4_SILABS_GECKO_USART_LOCATION_RX_0,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = usart_gecko_config_func_4,
#endif
};

static struct uart_gecko_data usart_gecko_4_data;

DEVICE_AND_API_INIT(usart_4, DT_INST_4_SILABS_GECKO_USART_LABEL,
		    &uart_gecko_init, &usart_gecko_4_data,
		    &usart_gecko_4_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &uart_gecko_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_gecko_config_func_4(struct device *dev)
{
	IRQ_CONNECT(DT_INST_4_SILABS_GECKO_USART_IRQ_RX,
		    DT_INST_4_SILABS_GECKO_USART_IRQ_RX_PRIORITY,
		    uart_gecko_isr, DEVICE_GET(usart_4), 0);
	IRQ_CONNECT(DT_INST_4_SILABS_GECKO_USART_IRQ_TX,
		    DT_INST_4_SILABS_GECKO_USART_IRQ_TX_PRIORITY,
		    uart_gecko_isr, DEVICE_GET(usart_4), 0);

	irq_enable(DT_INST_4_SILABS_GECKO_USART_IRQ_RX);
	irq_enable(DT_INST_4_SILABS_GECKO_USART_IRQ_TX);
}
#endif

#endif /* DT_INST_4_SILABS_GECKO_USART */

#ifdef DT_INST_5_SILABS_GECKO_USART

#define PIN_USART5_RXD {DT_INST_5_SILABS_GECKO_USART_LOCATION_RX_1, \
		DT_INST_5_SILABS_GECKO_USART_LOCATION_RX_2, gpioModeInput, 1}
#define PIN_USART5_TXD {DT_INST_5_SILABS_GECKO_USART_LOCATION_TX_1, \
		DT_INST_5_SILABS_GECKO_USART_LOCATION_TX_2, gpioModePushPull, 1}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_gecko_config_func_5(struct device *dev);
#endif

static const struct uart_gecko_config usart_gecko_5_config = {
	.base = (USART_TypeDef *)DT_INST_5_SILABS_GECKO_USART_BASE_ADDRESS,
	.clock = CLOCK_USART(DT_INST_5_SILABS_GECKO_USART_PERIPHERAL_ID),
	.baud_rate = DT_INST_5_SILABS_GECKO_USART_CURRENT_SPEED,
	.pin_rx = PIN_USART5_RXD,
	.pin_tx = PIN_USART5_TXD,
#ifdef CONFIG_SOC_GECKO_HAS_INDIVIDUAL_PIN_LOCATION
	.loc_rx = DT_INST_5_SILABS_GECKO_USART_LOCATION_RX_0,
	.loc_tx = DT_INST_5_SILABS_GECKO_USART_LOCATION_TX_0,
#else
#if DT_INST_5_SILABS_GECKO_USART_LOCATION_RX_0 \
	!= DT_INST_5_SILABS_GECKO_USART_LOCATION_TX_0
#error USART_5 DTS location-* properties must have identical value
#endif
	.loc = DT_INST_5_SILABS_GECKO_USART_LOCATION_RX_0,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = usart_gecko_config_func_5,
#endif
};

static struct uart_gecko_data usart_gecko_5_data;

DEVICE_AND_API_INIT(usart_5, DT_INST_5_SILABS_GECKO_USART_LABEL,
		    &uart_gecko_init, &usart_gecko_5_data,
		    &usart_gecko_5_config, PRE_KERNEL_1,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &uart_gecko_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void usart_gecko_config_func_5(struct device *dev)
{
	IRQ_CONNECT(DT_INST_5_SILABS_GECKO_USART_IRQ_RX,
		    DT_INST_5_SILABS_GECKO_USART_IRQ_RX_PRIORITY,
		    uart_gecko_isr, DEVICE_GET(usart_5), 0);
	IRQ_CONNECT(DT_INST_5_SILABS_GECKO_USART_IRQ_TX,
		    DT_INST_5_SILABS_GECKO_USART_IRQ_TX_PRIORITY,
		    uart_gecko_isr, DEVICE_GET(usart_5), 0);

	irq_enable(DT_INST_5_SILABS_GECKO_USART_IRQ_RX);
	irq_enable(DT_INST_5_SILABS_GECKO_USART_IRQ_TX);
}
#endif

#endif /* DT_INST_5_SILABS_GECKO_USART */
