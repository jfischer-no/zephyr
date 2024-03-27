/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <nrfs_backend_ipc_service.h>
#include <nrfs_usb.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

static void usbhs_periph_handler(nrfs_usb_evt_t const *p_evt, void *context)
{
	switch (p_evt->type) {
	case NRFS_USB_EVT_VBUS_STATUS_CHANGE:
		printf("USBHS new status, pll_ok = %d vreg_ok = %d vbus_detected = %d",
			p_evt->usbhspll_ok, p_evt->vregusb_ok, p_evt->vbus_detected);

		break;
	case NRFS_USB_EVT_REJECT:
		printf("Request rejected");
		break;
	default:
		printf("Unexpected event: 0x%x", p_evt->type);
		break;
	}
}

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

int main(void)
{
	nrfs_err_t status;
	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	ret = nrfs_backend_wait_for_connection(K_MSEC(1000));
	if (ret) {
		printf("NRFS backend connection timeout");
		return ret;
	}

	status = nrfs_usb_init(usbhs_periph_handler);
	if (status != NRFS_SUCCESS) {
		printf("Failed to init nrfs USB service: %d", status);
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) {
			return 0;
		}

		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}
	return 0;
}
