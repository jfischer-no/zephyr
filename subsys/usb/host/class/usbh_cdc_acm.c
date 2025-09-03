/*
 * Copyright Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/usb/usbh.h>
#include <zephyr/usb/usb_ch9.h>
#include <zephyr/usb/class/usb_cdc.h>

#include <zephyr/drivers/usb/uhc.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(usbh_cdc_acm, CONFIG_USBH_CDC_ACM_LOG_LEVEL);

#define CDC_ACM_DEFAULT_LINECODING	{sys_cpu_to_le32(115200), 0, 0, 8}

static struct k_work_q cdc_acm_work_q;
static K_KERNEL_STACK_DEFINE(cdc_acm_stack,
			     CONFIG_USBH_CDC_ACM_STACK_SIZE);

struct usbh_cdc_acm_desc {
	struct usb_association_descriptor iad;
	struct usb_if_descriptor if0;
	struct cdc_header_descriptor if0_header;
	struct cdc_cm_descriptor if0_cm;
	struct cdc_acm_descriptor if0_acm;
	struct cdc_union_descriptor if0_union;
	struct usb_ep_descriptor if0_int_ep;

	struct usb_if_descriptor if1;
	struct usb_ep_descriptor if1_in_ep;
	struct usb_ep_descriptor if1_out_ep;
};

struct cdc_acm_uart_config {
	/* Pointer to the class interface descriptors */
	struct usbh_cdc_acm_desc *const desc;
};

struct cdc_acm_uart_data {
	const struct device *dev;
	/* Line Coding Structure */
	struct cdc_acm_line_coding line_coding;
	/* SetControlLineState bitmap */
	uint16_t line_state;
	/* Serial state bitmap */
	uint16_t serial_state;
	/* UART actual configuration */
	struct uart_config uart_cfg;
	/* UART actual RTS state */
	bool line_state_rts;
	/* UART actual DTR state */
	bool line_state_dtr;
	/* When flow_ctrl is set, poll out is blocked when the buffer is full,
	 * roughly emulating flow control.
	 */
	bool flow_ctrl;

	struct k_sem notif_sem;

	/* UART API IRQ callback */
	uart_callback_t async_cb;
	/* UART API user callback data */
	void *async_cb_data;
};

static int cdc_acm_callback_set(const struct device *dev,
				const uart_callback_t callback, void *const user_data)
{
	struct cdc_acm_uart_data *const data = dev->data;

	data->async_cb = callback;
	data->async_cb_data = user_data;

	return 0;
}

static int cdc_acm_tx(const struct device *dev, const uint8_t *const buf,
		      const size_t len, const int32_t timeout)
{
	return 0;
}

static int cdc_acm_tx_abort(const struct device *dev)
{
	return 0;
}

static int cdc_acm_rx_enable(const struct device *dev, uint8_t *const buf,
			     const size_t len, const int32_t timeout)
{
	return 0;
}

static int cdc_acm_rx_buf_rsp(const struct device *dev, uint8_t *const buf,
			      const size_t len)
{
	return 0;
}

static int cdc_acm_rx_disable(const struct device *dev)
{
	return 0;
}

static int cdc_acm_poll_in(const struct device *dev, unsigned char *const c)
{
	return 0;
}

static void cdc_acm_poll_out(const struct device *dev, const unsigned char c)
{
}

#ifdef CONFIG_UART_LINE_CTRL
static int cdc_acm_line_ctrl_set(const struct device *dev,
				 const uint32_t ctrl, const uint32_t val)
{
	struct cdc_acm_uart_data *const data = dev->data;
	uint32_t flag = 0;

	switch (ctrl) {
	case USB_CDC_LINE_CTRL_BAUD_RATE:
		/* Ignore since it can not be used for notification anyway */
		return 0;
	case USB_CDC_LINE_CTRL_DCD:
		flag = USB_CDC_SERIAL_STATE_RXCARRIER;
		break;
	case USB_CDC_LINE_CTRL_DSR:
		flag = USB_CDC_SERIAL_STATE_TXCARRIER;
		break;
	case USB_CDC_LINE_CTRL_BREAK:
		flag = USB_CDC_SERIAL_STATE_BREAK;
		break;
	case USB_CDC_LINE_CTRL_RING_SIGNAL:
		flag = USB_CDC_SERIAL_STATE_RINGSIGNAL;
		break;
	case USB_CDC_LINE_CTRL_FRAMING:
		flag = USB_CDC_SERIAL_STATE_FRAMING;
		break;
	case USB_CDC_LINE_CTRL_PARITY:
		flag = USB_CDC_SERIAL_STATE_PARITY;
		break;
	case USB_CDC_LINE_CTRL_OVER_RUN:
		flag = USB_CDC_SERIAL_STATE_OVERRUN;
		break;
	default:
		return -EINVAL;
	}

	if (val) {
		data->serial_state |= flag;
	} else {
		data->serial_state &= ~flag;
	}

	return 0;
}

static int cdc_acm_line_ctrl_get(const struct device *dev,
				 const uint32_t ctrl, uint32_t *const val)
{
	struct cdc_acm_uart_data *const data = dev->data;

	switch (ctrl) {
	case UART_LINE_CTRL_BAUD_RATE:
		*val = data->uart_cfg.baudrate;
		return 0;
	case UART_LINE_CTRL_RTS:
		*val = data->line_state_rts;
		return 0;
	case UART_LINE_CTRL_DTR:
		*val = data->line_state_dtr;
		return 0;
	}

	return -ENOTSUP;
}
#endif

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
static int cdc_acm_configure(const struct device *dev,
			     const struct uart_config *const cfg)
{
	struct cdc_acm_uart_data *const data = dev->data;

	switch (cfg->flow_ctrl) {
	case UART_CFG_FLOW_CTRL_NONE:
		data->flow_ctrl = false;
		break;
	case UART_CFG_FLOW_CTRL_RTS_CTS:
		data->flow_ctrl = true;
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int cdc_acm_config_get(const struct device *dev,
			      struct uart_config *const cfg)
{
	struct cdc_acm_uart_data *const data = dev->data;

	memcpy(cfg, &data->uart_cfg, sizeof(struct uart_config));

	return 0;
}
#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

static DEVICE_API(uart, cdc_acm_uart_api) = {
	.poll_in = cdc_acm_poll_in,
	.poll_out = cdc_acm_poll_out,
#ifdef CONFIG_UART_LINE_CTRL
	.line_ctrl_set = cdc_acm_line_ctrl_set,
	.line_ctrl_get = cdc_acm_line_ctrl_get,
#endif
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = cdc_acm_configure,
	.config_get = cdc_acm_config_get,
#endif
	.callback_set = cdc_acm_callback_set,
	.tx = cdc_acm_tx,
	.tx_abort = cdc_acm_tx_abort,
	.rx_enable = cdc_acm_rx_enable,
	.rx_buf_rsp = cdc_acm_rx_buf_rsp,
	.rx_disable = cdc_acm_rx_disable,
};

static int cdc_acm_request(struct usbh_class_data *const c_data,
			   struct uhc_transfer *const xfer, int err)
{
	return 0;
}

static void cdc_acm_suspended(struct usbh_class_data *const c_data,
			      struct usb_device *const udev)
{
	return;
}

static void cdc_acm_resumed(struct usbh_class_data *const c_data,
			    struct usb_device *const udev)
{
	return;
}

static int cdc_acm_probe(struct usbh_class_data *const c_data,
			 struct usb_device *const udev,
			 const uint8_t iface)
{
	LOG_ERR("Probe %s bInterfaceNumber %u", c_data->name, iface);

	return 0;
}

static int cdc_acm_removed(struct usbh_class_data *const c_data,
			   struct usb_device *const udev,
			   const uint8_t iface)
{
	LOG_ERR("Removed %s bInterfaceNumber %u", c_data->name, iface);

	return 0;
}

static int cdc_acm_init(struct usbh_class_data *const c_data)
{
	LOG_DBG("%s: Init CDC ACM class driver", c_data->name);

	return 0;
}

struct usbh_class_api usbh_cdc_acm_api = {
	.request = cdc_acm_request,
	.suspended = cdc_acm_suspended,
	.resumed = cdc_acm_resumed,
	.probe = cdc_acm_probe,
	.removed = cdc_acm_removed,
	.init = cdc_acm_init,
};

static int usbh_cdc_acm_init_wq(void)
{
	k_work_queue_init(&cdc_acm_work_q);
	k_work_queue_start(&cdc_acm_work_q, cdc_acm_stack,
			   K_KERNEL_STACK_SIZEOF(cdc_acm_stack),
			   CONFIG_SYSTEM_WORKQUEUE_PRIORITY, NULL);
	k_thread_name_set(&cdc_acm_work_q.thread, "cdc_acm_work_q");

	return 0;
}

static int usbh_cdc_acm_preinit(const struct device *dev)
{
	return 0;
}

const static struct usbh_code_triple cdc_acm_code = {
	.dclass = USB_BCC_CDC_CONTROL,
	.sub = ACM_SUBCLASS,
	.proto = 0,
};

#define DT_DRV_COMPAT zephyr_host_cdc_acm_uart

#define USBH_CDC_ACM_DT_DEVICE_DEFINE(n)					\
	USBH_DEFINE_CLASS(cdc_acm_##n,						\
			  &usbh_cdc_acm_api,					\
			  (void *)DEVICE_DT_GET(DT_DRV_INST(n)),		\
			  &cdc_acm_code);					\
										\
	static const struct cdc_acm_uart_config uart_config_##n = {		\
	};									\
										\
	static struct cdc_acm_uart_data uart_data_##n = {			\
		.dev = DEVICE_DT_GET(DT_DRV_INST(n)),				\
		.line_coding = CDC_ACM_DEFAULT_LINECODING,			\
		.flow_ctrl = DT_INST_PROP(n, hw_flow_control),			\
		.notif_sem = Z_SEM_INITIALIZER(uart_data_##n.notif_sem, 0, 1),	\
	};									\
										\
	DEVICE_DT_INST_DEFINE(n, usbh_cdc_acm_preinit, NULL,			\
		&uart_data_##n, &uart_config_##n,				\
		PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,			\
		&cdc_acm_uart_api);

DT_INST_FOREACH_STATUS_OKAY(USBH_CDC_ACM_DT_DEVICE_DEFINE);

SYS_INIT(usbh_cdc_acm_init_wq, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
