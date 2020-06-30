#pragma once

#include <stdint.h>

void usb_device_init(void);
void send_test(void);
void usb_send_packet(const void *buf, int len);

extern uint8_t usb_ready;