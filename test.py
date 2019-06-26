from __future__ import print_function

import colorsys
import sys
import time
import usb1

import random

VENDOR_ID = 0x26BA
PRODUCT_ID = 0x8002

def controlWrite(handle, data):
    handle.controlWrite(
        usb1.TYPE_VENDOR | usb1.RECIPIENT_DEVICE,
        usb1.REQUEST_SET_CONFIGURATION,
        0,
        0,
        data,
        timeout=2000
    )

    return True

def send_colour(handle, a, b, c, d):
    data = bytearray(a + b + c + d)
    handle.interruptWrite(
        1,
        data
    )

def float_to_byte(colour):
    """Convert a float array to a byte array (0.0 to 1.0 -> 0 to 255)"""
    return [
        int(colour[0] * 255),
        int(colour[1] * 255),
        int(colour[2] * 255)
    ]

with usb1.USBContext() as context:
    handle = context.openByVendorIDAndProductID(
        VENDOR_ID,
        PRODUCT_ID,
        skip_on_error=True,
    )

    if handle is None:
    	print("Failed to find a device")
    	sys.exit(1)
    else:
        print("Got device...")

    with handle.claimInterface(0):
        print("Doot")

        hue = 0
        while True:
            # time.sleep(1/160.0)
            send_colour(handle,
                float_to_byte(colorsys.hsv_to_rgb(hue % 1.0, 1, 1)),
                float_to_byte(colorsys.hsv_to_rgb((hue + 0.03) % 1.0, 1, 1)),
                float_to_byte(colorsys.hsv_to_rgb((hue + 0.06) % 1.0, 1, 1)),
                float_to_byte(colorsys.hsv_to_rgb((hue + 0.09) % 1.0, 1, 1)),
            )

            hue += 0.001