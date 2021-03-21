/*
 * Copyright (c) 2021 Sun Amar.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_gecko_pwm

#include <errno.h>
#include <drivers/pwm.h>
#include <dt-bindings/pwm/pwm.h>
#include "em_cmu.h"
#include "em_timer.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(pwm_gecko, CONFIG_PWM_LOG_LEVEL);

/** PWM configuration. */
struct pwm_gecko_config {
	TIMER_TypeDef *timer;
	CMU_Clock_TypeDef clock;
	TIMER_Prescale_TypeDef prescale_enum;
	uint8_t channel;
	uint8_t location;
	uint8_t port;
	uint8_t pin;
};

struct pwm_gecko_data {
	int prescaler;
};

#define DEV_CFG(dev) \
	((const struct pwm_gecko_config * const)(dev)->config)
#define DEV_DATA(dev) \
	((struct pwm_gecko_data *const)(dev)->data)

static int pwm_gecko_pin_set(const struct device *dev, uint32_t pwm,
			     uint32_t period_cycles, uint32_t pulse_cycles,
			     pwm_flags_t flags)
{
	const struct pwm_gecko_config *cfg = DEV_CFG(dev);

	if (BUS_RegMaskedRead(&cfg->timer->CC[pwm].CTRL,
		_TIMER_CC_CTRL_MODE_MASK) != timerCCModePWM) {

#ifdef _TIMER_ROUTE_MASK
		BUS_RegMaskedWrite(&cfg->timer->ROUTE,
			_TIMER_ROUTE_LOCATION_MASK,
			cfg->location << _TIMER_ROUTE_LOCATION_SHIFT);
		BUS_RegMaskedSet(&pwm->timer->ROUTE, 1 << pwm);
#elif defined(_TIMER_ROUTELOC0_MASK)
		BUS_RegMaskedWrite(&cfg->timer->ROUTELOC0,
			_TIMER_ROUTELOC0_CC0LOC_MASK << (pwm * _TIMER_ROUTELOC0_CC1LOC_SHIFT),
			cfg->location << (pwm * _TIMER_ROUTELOC0_CC1LOC_SHIFT));
		BUS_RegMaskedSet(&cfg->timer->ROUTEPEN, 1 << pwm);
#else
		LOG_DBG("This specific gecko mcu not supported yet");
		return -ENOTSUP;
#endif

		TIMER_InitCC_TypeDef compare_config = TIMER_INITCC_DEFAULT;

		compare_config.mode = timerCCModePWM;
		TIMER_InitCC(cfg->timer, pwm, &compare_config);
	}

	cfg->timer->CC[pwm].CTRL |= (flags & PWM_POLARITY_INVERTED) ?
		TIMER_CC_CTRL_OUTINV : 0;

	TIMER_TopSet(cfg->timer, period_cycles);

	TIMER_CompareBufSet(cfg->timer, pwm, pulse_cycles);

	return 0;
}

static int pwm_gecko_get_cycles_per_sec(const struct device *dev,
					uint32_t pwm,
					uint64_t *cycles)
{
	const struct pwm_gecko_config *cfg = DEV_CFG(dev);
	struct pwm_gecko_data *const data = DEV_DATA(dev);

	*cycles = CMU_ClockFreqGet(cfg->clock) / data->prescaler;

	return 0;
}

static const struct pwm_driver_api pwm_gecko_driver_api = {
	.pin_set = pwm_gecko_pin_set,
	.get_cycles_per_sec = pwm_gecko_get_cycles_per_sec
};

static int pwm_gecko_init(const struct device *dev)
{
	const struct pwm_gecko_config *cfg = DEV_CFG(dev);

	CMU_ClockEnable(cfg->clock, true);

	CMU_ClockEnable(cmuClock_GPIO, true);
	GPIO_PinModeSet(cfg->port, cfg->pin, gpioModePushPull, 0);

	TIMER_Init_TypeDef timer = TIMER_INIT_DEFAULT;

	timer.prescale = cfg->prescale_enum;
	TIMER_Init(cfg->timer, &timer);

	return 0;
}

#define CLOCK_TIMER(id) _CONCAT(cmuClock_TIMER, id)
#define PRESCALING_FACTOR(factor) \
	((_CONCAT(timerPrescale, factor)))

#define PWM_GECKO_INIT(index)							\
	static struct pwm_gecko_data pwm_gecko_data_##index = {			\
		.prescaler = DT_INST_PROP(index, prescaler)			\
	};									\
										\
	static const struct pwm_gecko_config pwm_gecko_config_##index = {	\
		.timer = (TIMER_TypeDef *)DT_REG_ADDR(                          \
			DT_PARENT(DT_DRV_INST(index))),				\
		.clock = CLOCK_TIMER(index),					\
		.prescale_enum = (TIMER_Prescale_TypeDef)			\
			PRESCALING_FACTOR(DT_INST_PROP(index, prescaler)),	\
		.location = DT_INST_PROP_BY_IDX(index, pin_location, 0),	\
		.port = (GPIO_Port_TypeDef)					\
			DT_INST_PROP_BY_IDX(index, pin_location, 1),		\
		.pin = DT_INST_PROP_BY_IDX(index, pin_location, 2),		\
	};									\
										\
	DEVICE_DT_INST_DEFINE(index, &pwm_gecko_init, device_pm_control_nop,	\
				&pwm_gecko_data_##index,			\
				&pwm_gecko_config_##index, POST_KERNEL,		\
				CONFIG_KERNEL_INIT_PRIORITY_DEVICE,		\
			    &pwm_gecko_driver_api);

DT_INST_FOREACH_STATUS_OKAY(PWM_GECKO_INIT)
