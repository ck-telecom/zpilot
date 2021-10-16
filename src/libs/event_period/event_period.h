#ifndef _EVENT_PERIOD_H
#define _EVENT_PERIOD_H

#include <stdint.h>
#include <zephyr.h>
#include <sys/slist.h>

#ifdef EVENT_LISTENER
#ifndef GEVENT_MAX_SOURCE_LISTENERS
#define GEVENT_MAX_SOURCE_LISTENERS 32
#endif

union event {

};

typedef union event event_t;

typedef void (*event_cb)(void *parm, event_t *e);

struct event_listener {
	struct k_sem 	sem;
	uint16_t 		flags;
	event_cb 		cb;
	void 			*parm;
	event_t 		event;
};

struct source_listener {

};
#endif
typedef void (*event_period_cb)(void *param);

struct event_period_node {
	event_period_cb cb;
	void *param;
	sys_snode_t node;
};

enum event_period {
	EVENT_PERIOD_5MS,      /**< 200HZ */
	EVENT_PERIOD_10MS,     /**< 100HZ */
	EVENT_PERIOD_20MS,     /**< 50HZ */
	EVENT_PERIOD_100MS,    /**< 10HZ */
	EVENT_PERIOD_1000MS,   /**< 1HZ */
	EVENT_PERIOD_MAX
};

event_period_register_cb(enum event_period, struct event_period_node *node);

#endif /* _EVENT_PERIOD_H */