/*
 * Copyright (C) 2022 Mohamad Al-Jaf
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

#ifndef _WINUSB_H_
#define _WINUSB_H_

#ifdef __cplusplus
extern "C" {
#endif

typedef PVOID WINUSB_INTERFACE_HANDLE, *PWINUSB_INTERFACE_HANDLE;

typedef enum _USBD_PIPE_TYPE
{
    UsbdPipeTypeControl,
    UsbdPipeTypeIsochronous,
    UsbdPipeTypeBulk,
    UsbdPipeTypeInterrupt,
} USBD_PIPE_TYPE;

typedef struct _WINUSB_PIPE_INFORMATION
{
    USBD_PIPE_TYPE PipeType;
    UCHAR  PipeId;
    USHORT MaximumPacketSize;
    UCHAR  Interval;
} WINUSB_PIPE_INFORMATION, *PWINUSB_PIPE_INFORMATION;

#include <pshpack1.h>
typedef struct _USB_INTERFACE_DESCRIPTOR
{
    UCHAR bLength;
    UCHAR bDescriptorType;
    UCHAR bInterfaceNumber;
    UCHAR bAlternateSetting;
    UCHAR bNumEndpoints;
    UCHAR bInterfaceClass;
    UCHAR bInterfaceSubClass;
    UCHAR bInterfaceProtocol;
    UCHAR iInterface;
} USB_INTERFACE_DESCRIPTOR, *PUSB_INTERFACE_DESCRIPTOR;
#include <poppack.h>

#include <pshpack1.h>
typedef struct _WINUSB_SETUP_PACKET
{
    UCHAR  RequestType;
    UCHAR  Request;
    USHORT Value;
    USHORT Index;
    USHORT Length;
} WINUSB_SETUP_PACKET, *PWINUSB_SETUP_PACKET;
#include <poppack.h>

BOOL WINAPI WinUsb_Free(WINUSB_INTERFACE_HANDLE);

#ifdef __cplusplus
}
#endif

#endif /* _WINUSB_H_ */
