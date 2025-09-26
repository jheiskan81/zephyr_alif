/*
 * Copyright (c) 2015 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_GPIO_GPIO_DW_H_
#define ZEPHYR_DRIVERS_GPIO_GPIO_DW_H_

#include <zephyr/types.h>
#include <zephyr/drivers/gpio.h>
#include "gpio_dw_registers.h"

#ifdef CONFIG_PINCTRL
#include <zephyr/drivers/pinctrl.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*gpio_config_irq_t)(const struct device *port);

#define VBAT_INIT_DONE 0xeabceabc
struct gpio_dw_vbat {
	uint32_t init_done;
};

struct gpio_dw_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	uint32_t ngpios;
	uint32_t irq_num; /* set to 0 if GPIO port cannot interrupt */
	gpio_config_irq_t config_func;
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pcfg;
#endif
	bool vbat_support;
};

struct gpio_dw_runtime {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	uint32_t base_addr;
	sys_slist_t callbacks;
	struct gpio_dw_vbat *vbat_resume;
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_DRIVERS_GPIO_GPIO_DW_H_ */
