#include "event_period.h"
#ifdef EVENT_LISTENER
static struct k_mutex mutex;

static struct event_listener handlers[GEVENT_MAX_SOURCE_LISTENERS];

static void event_init()
{
	k_mutex_init(&mutex);
}

static void event_listener_init(struct event_listener *ev)
{
	k_sem_init(&ev->sem);
	ev->cb = NULL;
	ev->flags = 0;
}

boolean event_register()
{
	k_mutex_lock(&mutex);
	pslfree = 0;
	for (psl = handlers; psl < handlers + GEVENT_MAX_SOURCE_LISTENERS; psl++) {
		if (pl == psl->pListener && gsh == psl->pSource) {
			psl->listenerflags = flags;
			k_mutex_unlock(&mutex);
			return true;
		}
		if (!psl->pListener && !pslfree)
			pslfree = psl;
	}
	if (pslfree) {
		pslfree->pListener = pl;
		pslfree->pSource = ghs;
		pslfree->listenflags = flags;
		pslfree->srcflags = 0;
	}
	k_mutex_unlock( &mutex);

	return pslfree != NULL;
}

GSourceListener *getSourceListener(event_t ev, GSourceListener *itr)
{
	if (!ev)
		return NULL;

	k_mutex_lock(&mutex);
	k_mutex_unlock(&mutex);
}

void geventSendEvent(struct event_listener *psl) {
	k_mutex_lock(&mutex);
	if (psl->pListener->cb) {

		// Mark it back as free and as sent. This is early to be marking as free but it protects
		//	if the callback alters the listener in any way
		psl->pListener->flags = 0;
		k_mutex_unlock(&mutex);

		// Do the callback
		psl->pListener->cb(psl->pListener->param, &psl->pListener->event);

	} else {
		// Wake up the listener
		psl->pListener->flags = GLISTENER_WITHLISTENER;
		gfxSemSignal(&psl->pListener->waitqueue);
		k_mutex_unlock(&mutex);
	}
}
#endif

static sys_slist_t event_period_list[EVENT_PERIOD_MAX] = { 0 };//SYS_SLIST_STATIC_INIT(&event_period_list);

static
event_period_register_cb(enum event_period, struct event_period_node *e)
{
	sys_slist_t *head = &event_period_list[event_period];
//locker
	sys_slist_append(head, &e->node);
}
