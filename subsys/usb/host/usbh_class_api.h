/*
 * Copyright Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_USBH_CLASS_API_H
#define ZEPHYR_INCLUDE_USBH_CLASS_API_H

#include <zephyr/usb/usbh.h>

static inline int usbh_class_request(struct usbh_class_data *const c_data,
				     struct uhc_transfer *const xfer, int err)
{
	const struct usbh_class_api *api = c_data->api;

	if (api->request != NULL) {
		return api->request(c_data, xfer, err);
	}

	return -ENOTSUP;
}

static inline void usbh_class_suspended(struct usbh_class_data *const c_data,
					struct usb_device *const udev)
{
	const struct usbh_class_api *api = c_data->api;

	if (api->suspended != NULL) {
		api->suspended(c_data, udev);
	}
}


static inline void usbh_class_resumed(struct usbh_class_data *const c_data,
				      struct usb_device *const udev)
{
	const struct usbh_class_api *api = c_data->api;

	if (api->resumed != NULL) {
		api->resumed(c_data, udev);
	}
}

static inline int usbh_class_probe(struct usbh_class_data *const c_data,
				   struct usb_device *const udev,
				   const uint8_t iface)
{
	const struct usbh_class_api *api = c_data->api;

	if (api->probe != NULL) {
		return api->probe(c_data, udev, iface);
	}

	return -ENOTSUP;
}

static inline int usbh_class_removed(struct usbh_class_data *const c_data,
				     struct usb_device *const udev,
				     const uint8_t iface)
{
	const struct usbh_class_api *api = c_data->api;

	if (api->removed != NULL) {
		return api->removed(c_data, udev, iface);
	}

	return -ENOTSUP;
}

static inline int usbh_class_init(struct usbh_class_data *const c_data)
{
	const struct usbh_class_api *api = c_data->api;

	if (api->init != NULL) {
		return api->init(c_data);
	}

	return -ENOTSUP;
}

#endif /* ZEPHYR_INCLUDE_USBH_CLASS_API_H */
