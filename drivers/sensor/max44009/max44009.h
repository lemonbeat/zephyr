/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_MAX44009_MAX44009_H_
#define ZEPHYR_DRIVERS_SENSOR_MAX44009_MAX44009_H_

#include <sys/util.h>

#define MAX44009_I2C_ADDRESS DT_INST_REG_ADDR(0)

#define MAX44009_SAMPLING_CONTROL_BIT BIT(7)
#define MAX44009_CONTINUOUS_SAMPLING BIT(7)
#define MAX44009_SAMPLE_EXPONENT_SHIFT 12
#define MAX44009_MANTISSA_HIGH_NIBBLE_MASK 0xf00
#define MAX44009_MANTISSA_LOW_NIBBLE_MASK 0xf

#define MAX44009_REG_CONFIG 0x02
#define MAX44009_REG_LUX_HIGH_BYTE 0x03
#define MAX44009_REG_LUX_LOW_BYTE 0x04

struct max44009_data {
	const struct device *i2c;
	uint16_t sample;
};

#endif /* _SENSOR_MAX44009_ */
