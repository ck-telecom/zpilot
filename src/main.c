/*
 * Copyright (c) 2021 zpilot.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>

#ifdef CONFIG_EVENTS

#define EVT_500HZ       BIT(0)
#define EVT_200HZ       BIT(1)
#define EVT_100HZ       BIT(2)
#define EVT_50HZ        BIT(3)
#define EVT_10HZ        BIT(4)
#define EVT_1HZ         BIT(5)

static K_EVENT_DEFINE(worker_event);
#endif


void main(void)
{
	uint32_t events = 0;

	printk("Hello World! %s\n", CONFIG_BOARD);
#ifdef CONFIG_EVENTS
	while (1) {
		if (events = k_event_wait(&worker_event,
				   EVT_500HZ | EVT_200HZ |
				   EVT_100HZ | EVT_50HZ | EVT_10HZ,
				   false, K_FOREVER)) {
			if (events & EVT_500HZ) {

			}

			if (events & EVT_200HZ) {

			}

			if (events & EVT_100HZ) {

			}

			if (events & EVT_50HZ) {

			}

			if (events & EVT_10HZ) {

			}
		}
	}
#endif
}
