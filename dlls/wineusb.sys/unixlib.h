/*
 * wineusb Unix library interface
 *
 * Copyright 2022 Zebediah Figura for CodeWeavers
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA
 */

#ifndef __WINE_WINEUSB_UNIXLIB_H
#define __WINE_WINEUSB_UNIXLIB_H

#include "windef.h"
#include "winternl.h"
#include "ddk/wdm.h"
#include "wine/unixlib.h"

enum usb_event_type
{
    USB_EVENT_ADD_DEVICE,
    USB_EVENT_REMOVE_DEVICE,
    USB_EVENT_TRANSFER_COMPLETE,
};

struct usb_event
{
    enum usb_event_type type;

    union
    {
        struct usb_add_device_event
        {
            struct unix_device *device;
            /* For an interface pseudo-device, the device it belongs to.
             * Windows parents interface devnodes on the composite device, and
             * libusb needs that to attach each interface's path to it. */
            struct unix_device *parent;
            UINT16 vendor, product, revision, usbver;
            UINT8 class, subclass, protocol, busnum, portnum;
            bool interface;
            INT16 interface_index;
        } added_device;
        struct unix_device *removed_device;
        IRP *completed_irp;
    } u;
};

struct usb_main_loop_params
{
    struct usb_event *event;
};

struct usb_submit_urb_params
{
    struct unix_device *device;
    IRP *irp;
    void *transfer_buffer;
};

struct usb_cancel_transfer_params
{
    void *transfer;
};

struct usb_destroy_device_params
{
    struct unix_device *device;
};

struct usb_claim_interface_params
{
    struct unix_device *device;
    UINT8 interface_number;
    BOOL claim;                 /* FALSE to release */
};

struct usb_get_descriptor_params
{
    struct unix_device *device;
    UINT8 type;
    UINT8 index;
    UINT16 langid;
    void *buffer;
    UINT32 size;
    UINT32 *ret_size;
};

enum unix_funcs
{
    unix_usb_main_loop,
    unix_usb_init,
    unix_usb_exit,
    unix_usb_submit_urb,
    unix_usb_cancel_transfer,
    unix_usb_destroy_device,
    unix_usb_get_descriptor,
    unix_usb_claim_interface,
};

#endif
