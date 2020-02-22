from __future__ import print_function

import colorsys
import sys
import time
import usb1

import random

VENDOR_ID = 0x26BA
PRODUCT_ID = 0x8002

class UsbMultiHandle():
    """ Open handles to all devices matching the vendor and product ID """
    def __init__(self, context, vendorId, productId, skipOnError=False):
        self.handles = []

        self.context = context

        self.vendorId = vendorId
        self.productId = productId
        self.skipOnError = skipOnError

    def __enter__(self):
        for device in self.context.getDeviceIterator(skip_on_error=self.skipOnError):
            if device.getVendorID() == self.vendorId and device.getProductID() == self.productId:
                handle = device.open()

                if handle:
                    print(device)
                    handle.claimInterface(0)
                    self.handles.append(handle)
                else:
                    print("Failed to open device:", device)


        return self.handles

    def __exit__(self, type, value, traceback):
        for device in self.handles:
            device.releaseInterface(0)
            device.close()

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
    with UsbMultiHandle(context, VENDOR_ID, PRODUCT_ID, skipOnError=True) as handles:
        if not handles:
            print("No devices found!")
            sys.exit(1)

        hue = 0
        offset = 0.01
        updateCount = 0

        hues = [hue/16.0 for hue in range(0, 16)]

        while True:
            time.sleep(1/60.0)

            colours = []

            # Update values
            for i in range(len(hues)):
                colours.append(float_to_byte(colorsys.hsv_to_rgb(hues[i] % 1.0, 1, 0.5)))
                hues[i] += offset

            # Send values
            index = 0

            for handle in handles:
                send_colour(handle,
                    colours[index],
                    colours[index + 1],
                    colours[index + 2],
                    colours[index + 3],
                )

                index += 4