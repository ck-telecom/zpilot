#include <drivers/uart.h>
#include <logging/log.h>

#include "mavlink.h"

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS 1

#define MAVLINK_COM_DEV "UART_3"

#define TELEMETRY_STACK_SIZE	1024
#define TELEMETRY_PRIORITY		5

LOG_MODULE_REGISTER(telemetry, LOG_LEVEL_DBG);

K_MSGQ_DEFINE(mavlink_uart_msgq, sizeof(uint8_t), 32, 4);

static const struct device *uart_dev = NULL;

static void uart_rx_isr(const struct device *dev, void *user_data)
{
//	const struct device *dev = user_data;

	if (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
		uint8_t c;
		uart_fifo_read(dev, &c, 1);

		while (k_msgq_put(&mavlink_uart_msgq, &c, K_NO_WAIT) != 0) {
			k_msgq_purge(&mavlink_uart_msgq);
		}
	}
}

int telemetry_init(const struct device *dev)
{
	uart_dev = device_get_binding(MAVLINK_COM_DEV);
	if (!uart_dev) {
		LOG_ERR("can not open uart device: %s", MAVLINK_COM_DEV);
		return -ENODEV;
	}
	/* setup serial */
	uart_irq_rx_disable(uart_dev);
	//uart_irq_tx_disable(uart_dev);
	// flush();
	uart_irq_callback_user_data_set(uart_dev, uart_rx_isr, NULL);
	uart_irq_rx_enable(uart_dev);

	return 0;
}
SYS_INIT(telemetry_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

int mavlink_get_char(uint8_t *c)
{
	return k_msgq_get(&mavlink_uart_msgq, c, K_FOREVER);
}

static void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
	if (uart_dev)
		uart_poll_out(uart_dev, (unsigned char)ch);
}

void telemetry_rx_thread()
{
	int retval = 0;
	uint8_t chan = 0;
	uint8_t c = 0;
	mavlink_message_t msg = { 0 };
	mavlink_status_t status = { 0 };

	while (1) {
		retval = mavlink_get_char(&c);
		if (retval) {
			LOG_ERR("failed to get char");
			continue;
		}
		if (mavlink_parse_char(chan, c, &msg, &status)) {
			// LOG_DBG();
			switch (msg.msgid) {
			case MAVLINK_MSG_ID_HEARTBEAT:
				break;

			case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
			//mavlink_send_single_param
				break;

			default:
				break;
			}
		}
	}
}
K_THREAD_DEFINE(telemetry_rx_tid, TELEMETRY_STACK_SIZE,
				telemetry_rx_thread, NULL, NULL, NULL, TELEMETRY_PRIORITY, 0, 0);