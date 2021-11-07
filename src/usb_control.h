#pragma once

#include <stdint.h>

#include <libopencm3/usb/usbd.h>

void usb_device_init(void);
void send_test(void);
void usb_send_packet(const void *buf, int len);
void usb_block_until_sent(void);

extern void usb_ready_callback(usbd_device *usbd_dev);

extern uint8_t usb_ready;