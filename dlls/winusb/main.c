/*
 * WinUSB implementation
 *
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

#include <stdarg.h>
#include <stdlib.h>

#include "windef.h"
#include "winbase.h"
#include "winioctl.h"
#include "winusb.h"

#include "wine/debug.h"

WINE_DEFAULT_DEBUG_CHANNEL(winusb);

/* Private interface to wineusb.sys. Keep in sync with
 * dlls/wineusb.sys/wineusb.c. */
#ifndef FILE_DEVICE_USB
#define FILE_DEVICE_USB 0x00000022
#endif

#define IOCTL_WINEUSB_CONTROL_TRANSFER  CTL_CODE(FILE_DEVICE_USB, 0x800, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_WINEUSB_PIPE_TRANSFER     CTL_CODE(FILE_DEVICE_USB, 0x801, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_WINEUSB_ABORT_PIPE        CTL_CODE(FILE_DEVICE_USB, 0x802, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_WINEUSB_RESET_PIPE        CTL_CODE(FILE_DEVICE_USB, 0x803, METHOD_BUFFERED, FILE_ANY_ACCESS)

struct wineusb_control_request
{
    UCHAR setup[8];
};

struct wineusb_pipe_request
{
    UCHAR endpoint;
};

BOOL WINAPI WinUsb_GetDescriptor(WINUSB_INTERFACE_HANDLE handle, UCHAR type, UCHAR index,
        USHORT language, UCHAR *buffer, ULONG length, ULONG *transferred);

struct winusb_handle
{
    HANDLE device;
    UCHAR interface_index;
    UCHAR *config;      /* cached configuration descriptor */
    ULONG config_size;
    BOOL  config_failed;
    BOOL  in_query;
    UCHAR alt_setting;

    /* Pipe policies are accepted and echoed back; wineusb.sys does not
     * implement them, but callers expect the calls to succeed. */
    ULONG pipe_timeout;
    UCHAR short_packet_terminate;
    UCHAR auto_clear_stall;
    UCHAR raw_io;
    UCHAR ignore_short_packets;
    UCHAR allow_partial_reads;
};

/* Walk the cached configuration descriptor and return the nth interface
 * descriptor with the given alternate setting, or NULL. */
/* Fetch and cache the configuration descriptor on first use. Doing this in
 * WinUsb_Initialize() deadlocks libusb, which calls Initialize from
 * claim_interface() while holding its context lock. */
/* Fetching the configuration descriptor requires a control transfer, which
 * cannot be done synchronously here: libusb calls QueryPipe and
 * QueryInterfaceSettings from claim_interface() while holding its context
 * lock, and waiting for the transfer deadlocks. Fetch it lazily elsewhere. */
static void ensure_config(struct winusb_handle *usb)
{
    if (usb->in_query)
        return;

    UCHAR header[9];
    ULONG got = 0, total;

    if (usb->config || usb->config_failed)
        return;

    if (WinUsb_GetDescriptor((WINUSB_INTERFACE_HANDLE)usb, 0x02, 0, 0,
            header, sizeof(header), &got) && got >= 9)
    {
        total = header[2] | (header[3] << 8);
        if (total >= 9 && (usb->config = malloc(total)))
        {
            if (WinUsb_GetDescriptor((WINUSB_INTERFACE_HANDLE)usb, 0x02, 0, 0,
                    usb->config, total, &got) && got >= 9)
                usb->config_size = got;
            else
            {
                free(usb->config);
                usb->config = NULL;
            }
        }
    }
    if (!usb->config)
        usb->config_failed = TRUE;
}

static const UCHAR *find_interface(const struct winusb_handle *usb,
        UCHAR index, UCHAR alt)
{
    const UCHAR *p = usb->config, *end;
    UCHAR seen = 0xff;

    if (!p || usb->config_size < 9)
        return NULL;
    end = p + usb->config_size;

    for (p += p[0]; p + 2 <= end && p[0]; p += p[0])
    {
        if (p[1] != 0x04 || p + 9 > end)     /* INTERFACE descriptor */
            continue;
        if (p[3] != alt)
            continue;
        seen++;
        if (seen == index)
            return p;
    }
    return NULL;
}

/* Shared helper: send a buffered IOCTL where the input header is followed by
 * the caller's transfer buffer coming back in the same buffer. */
static BOOL do_transfer(HANDLE device, DWORD code, const void *header, DWORD header_size,
        void *buffer, ULONG length, ULONG *transferred, OVERLAPPED *overlapped)
{
    OVERLAPPED sync_overlapped;
    DWORD returned = 0;
    BOOL ret;

    /* Caller-supplied OVERLAPPED: hand it straight to the I/O manager and let
     * the caller collect the result with WinUsb_GetOverlappedResult(). */
    if (overlapped)
    {
        ret = DeviceIoControl(device, code, (void *)header, header_size,
                buffer, length, &returned, overlapped);
        if (ret && transferred)
            *transferred = returned;
        return ret;
    }

    /* Synchronous: use a private event so this works whether or not the
     * handle was opened with FILE_FLAG_OVERLAPPED. */
    memset(&sync_overlapped, 0, sizeof(sync_overlapped));
    if (!(sync_overlapped.hEvent = CreateEventW(NULL, TRUE, FALSE, NULL)))
        return FALSE;

    ret = DeviceIoControl(device, code, (void *)header, header_size,
            buffer, length, &returned, &sync_overlapped);
    if (!ret && GetLastError() == ERROR_IO_PENDING)
        ret = GetOverlappedResult(device, &sync_overlapped, &returned, TRUE);

    if (ret && transferred)
        *transferred = returned;

    CloseHandle(sync_overlapped.hEvent);
    return ret;
}

/* Transfers whose payload travels *to* the device.
 *
 * These IOCTLs are METHOD_BUFFERED, so only the input buffer is copied into
 * the driver's SystemBuffer -- the output buffer is space for a reply. Passing
 * outgoing data as the output buffer therefore sent whatever happened to be in
 * that memory, not the caller's data. Append the payload to the header
 * instead. The copy into SystemBuffer happens while DeviceIoControl() runs, so
 * the temporary is safe to free on return even for an overlapped request. */
static BOOL do_out_transfer(HANDLE device, DWORD code, const void *header, DWORD header_size,
        const void *buffer, ULONG length, ULONG *transferred, OVERLAPPED *overlapped)
{
    BOOL ret;
    char *in;

    if (!(in = malloc(header_size + length)))
    {
        SetLastError(ERROR_NOT_ENOUGH_MEMORY);
        return FALSE;
    }

    memcpy(in, header, header_size);
    if (length)
        memcpy(in + header_size, buffer, length);

    ret = do_transfer(device, code, in, header_size + length, NULL, 0, transferred, overlapped);

    free(in);
    return ret;
}

/***********************************************************************
 *           WinUsb_Initialize (winusb.@)
 */
BOOL WINAPI WinUsb_Initialize(HANDLE file, WINUSB_INTERFACE_HANDLE *handle)
{
    struct winusb_handle *usb;

    TRACE("file %p, handle %p.\n", file, handle);

    if (!handle || file == INVALID_HANDLE_VALUE)
    {
        SetLastError(ERROR_INVALID_HANDLE);
        return FALSE;
    }

    if (!(usb = calloc(1, sizeof(*usb))))
    {
        SetLastError(ERROR_NOT_ENOUGH_MEMORY);
        return FALSE;
    }

    usb->device = file;
    usb->interface_index = 0;

    *handle = (WINUSB_INTERFACE_HANDLE)usb;
    TRACE("-> produced handle %p.\n", usb);
    return TRUE;
}

/***********************************************************************
 *           WinUsb_QueryInterfaceSettings (winusb.@)
 */
BOOL WINAPI WinUsb_QueryInterfaceSettings(WINUSB_INTERFACE_HANDLE handle,
        UCHAR alt_setting, USB_INTERFACE_DESCRIPTOR *descriptor)
{
    struct winusb_handle *usb = handle;
    const UCHAR *iface;

    TRACE("handle %p, alt_setting %u, descriptor %p.\n", handle, alt_setting, descriptor);

    if (!usb || !descriptor)
    {
        SetLastError(ERROR_INVALID_PARAMETER);
        return FALSE;
    }

    ensure_config(usb);


    if (!(iface = find_interface(usb, usb->interface_index, alt_setting)))
    {
        SetLastError(ERROR_NO_MORE_ITEMS);
        return FALSE;
    }

    memcpy(descriptor, iface, sizeof(*descriptor));
    return TRUE;
}

/***********************************************************************
 *           WinUsb_QueryPipe (winusb.@)
 */
BOOL WINAPI WinUsb_QueryPipe(WINUSB_INTERFACE_HANDLE handle, UCHAR alt_setting,
        UCHAR index, WINUSB_PIPE_INFORMATION *info)
{
    struct winusb_handle *usb = handle;
    const UCHAR *p, *end, *iface;
    UCHAR seen = 0;

    TRACE("handle %p, alt_setting %u, index %u, info %p.\n",
            handle, alt_setting, index, info);

    if (!usb || !info)
    {
        SetLastError(ERROR_INVALID_PARAMETER);
        return FALSE;
    }

    ensure_config(usb);


    if (!(iface = find_interface(usb, usb->interface_index, alt_setting)))
    {
        SetLastError(ERROR_NO_MORE_ITEMS);
        return FALSE;
    }

    end = usb->config + usb->config_size;

    for (p = iface + iface[0]; p + 2 <= end && p[0]; p += p[0])
    {
        if (p[1] == 0x04)       /* next interface: stop */
            break;
        if (p[1] != 0x05 || p + 7 > end)    /* ENDPOINT descriptor */
            continue;

        if (seen++ != index)
            continue;

        info->PipeId = p[2];
        info->MaximumPacketSize = p[4] | (p[5] << 8);
        info->Interval = p[6];
        switch (p[3] & 3)
        {
            case 0:  info->PipeType = UsbdPipeTypeControl; break;
            case 1:  info->PipeType = UsbdPipeTypeIsochronous; break;
            case 2:  info->PipeType = UsbdPipeTypeBulk; break;
            default: info->PipeType = UsbdPipeTypeInterrupt; break;
        }
        TRACE("pipe %#x, type %u, packet size %u.\n",
                info->PipeId, info->PipeType, info->MaximumPacketSize);
        return TRUE;
    }

    SetLastError(ERROR_NO_MORE_ITEMS);
    return FALSE;
}

/***********************************************************************
 *           WinUsb_Free (winusb.@)
 */
BOOL WINAPI WinUsb_Free(WINUSB_INTERFACE_HANDLE handle)
{
    struct winusb_handle *usb = handle;

    TRACE("handle %p.\n", handle);

    if (usb)
        free(usb->config);
    free(usb);
    return TRUE;
}

/***********************************************************************
 *           WinUsb_ControlTransfer (winusb.@)
 */
BOOL WINAPI WinUsb_ControlTransfer(WINUSB_INTERFACE_HANDLE handle, WINUSB_SETUP_PACKET setup,
        UCHAR *buffer, ULONG length, ULONG *transferred, OVERLAPPED *overlapped)
{
    struct winusb_handle *usb = handle;
    struct wineusb_control_request req;

    TRACE("handle %p, type %#x, request %#x, value %#x, index %#x, length %lu.\n",
            handle, setup.RequestType, setup.Request, setup.Value, setup.Index, length);

    if (!usb)
    {
        SetLastError(ERROR_INVALID_HANDLE);
        return FALSE;
    }

    req.setup[0] = setup.RequestType;
    req.setup[1] = setup.Request;
    req.setup[2] = setup.Value & 0xff;
    req.setup[3] = setup.Value >> 8;
    req.setup[4] = setup.Index & 0xff;
    req.setup[5] = setup.Index >> 8;
    req.setup[6] = setup.Length & 0xff;
    req.setup[7] = setup.Length >> 8;

    /* An OUT control transfer carries its data to the device. */
    if (!(setup.RequestType & 0x80))
        return do_out_transfer(usb->device, IOCTL_WINEUSB_CONTROL_TRANSFER,
                &req, sizeof(req), buffer, length, transferred, overlapped);

    return do_transfer(usb->device, IOCTL_WINEUSB_CONTROL_TRANSFER,
            &req, sizeof(req), buffer, length, transferred, overlapped);
}

/***********************************************************************
 *           WinUsb_GetDescriptor (winusb.@)
 */
BOOL WINAPI WinUsb_GetDescriptor(WINUSB_INTERFACE_HANDLE handle, UCHAR type, UCHAR index,
        USHORT language, UCHAR *buffer, ULONG length, ULONG *transferred)
{
    struct winusb_handle *usb = handle;
    struct wineusb_control_request req;

    TRACE("handle %p, type %#x, index %#x, language %#x, length %lu.\n",
            handle, type, index, language, length);

    if (!usb)
    {
        SetLastError(ERROR_INVALID_HANDLE);
        return FALSE;
    }

    req.setup[0] = 0x80;    /* device-to-host, standard, device */
    req.setup[1] = 0x06;    /* GET_DESCRIPTOR */
    req.setup[2] = index;
    req.setup[3] = type;
    req.setup[4] = language & 0xff;
    req.setup[5] = language >> 8;
    req.setup[6] = length & 0xff;
    req.setup[7] = length >> 8;

    return do_transfer(usb->device, IOCTL_WINEUSB_CONTROL_TRANSFER,
            &req, sizeof(req), buffer, length, transferred, NULL);
}

/***********************************************************************
 *           WinUsb_ReadPipe (winusb.@)
 */
BOOL WINAPI WinUsb_ReadPipe(WINUSB_INTERFACE_HANDLE handle, UCHAR pipe_id, UCHAR *buffer,
        ULONG length, ULONG *transferred, OVERLAPPED *overlapped)
{
    struct winusb_handle *usb = handle;
    struct wineusb_pipe_request req;

    TRACE("handle %p, pipe %#x, length %lu.\n", handle, pipe_id, length);

    if (!usb)
    {
        SetLastError(ERROR_INVALID_HANDLE);
        return FALSE;
    }

    req.endpoint = pipe_id | 0x80;

    return do_transfer(usb->device, IOCTL_WINEUSB_PIPE_TRANSFER,
            &req, sizeof(req), buffer, length, transferred, overlapped);
}

/***********************************************************************
 *           WinUsb_WritePipe (winusb.@)
 */
BOOL WINAPI WinUsb_WritePipe(WINUSB_INTERFACE_HANDLE handle, UCHAR pipe_id, UCHAR *buffer,
        ULONG length, ULONG *transferred, OVERLAPPED *overlapped)
{
    struct winusb_handle *usb = handle;
    struct wineusb_pipe_request req;

    TRACE("handle %p, pipe %#x, length %lu.\n", handle, pipe_id, length);

    if (!usb)
    {
        SetLastError(ERROR_INVALID_HANDLE);
        return FALSE;
    }

    req.endpoint = pipe_id & 0x7f;

    return do_out_transfer(usb->device, IOCTL_WINEUSB_PIPE_TRANSFER,
            &req, sizeof(req), buffer, length, transferred, overlapped);
}

/***********************************************************************
 *           WinUsb_GetOverlappedResult (winusb.@)
 */
BOOL WINAPI WinUsb_GetOverlappedResult(WINUSB_INTERFACE_HANDLE handle,
        OVERLAPPED *overlapped, ULONG *transferred, BOOL wait)
{
    struct winusb_handle *usb = handle;
    DWORD returned = 0;
    BOOL ret;

    TRACE("handle %p, overlapped %p, wait %d.\n", handle, overlapped, wait);

    if (!usb || !overlapped)
    {
        SetLastError(ERROR_INVALID_PARAMETER);
        return FALSE;
    }

    ret = GetOverlappedResult(usb->device, overlapped, &returned, wait);
    if (ret && transferred)
        *transferred = returned;
    return ret;
}

/* Policy identifiers from winusbio.h */
#define WINUSB_SHORT_PACKET_TERMINATE 0x01
#define WINUSB_AUTO_CLEAR_STALL       0x02
#define WINUSB_PIPE_TRANSFER_TIMEOUT  0x03
#define WINUSB_IGNORE_SHORT_PACKETS   0x04
#define WINUSB_ALLOW_PARTIAL_READS    0x05
#define WINUSB_AUTO_FLUSH             0x06
#define WINUSB_RAW_IO                 0x07

/***********************************************************************
 *           WinUsb_SetPipePolicy (winusb.@)
 */
BOOL WINAPI WinUsb_SetPipePolicy(WINUSB_INTERFACE_HANDLE handle, UCHAR pipe_id,
        ULONG policy_type, ULONG value_length, void *value)
{
    struct winusb_handle *usb = handle;

    TRACE("handle %p, pipe %#x, policy %#lx, length %lu.\n",
            handle, pipe_id, policy_type, value_length);

    if (!usb || !value)
    {
        SetLastError(ERROR_INVALID_PARAMETER);
        return FALSE;
    }

    switch (policy_type)
    {
    case WINUSB_PIPE_TRANSFER_TIMEOUT:
        if (value_length < sizeof(ULONG)) break;
        usb->pipe_timeout = *(ULONG *)value;
        return TRUE;
    case WINUSB_SHORT_PACKET_TERMINATE:
        usb->short_packet_terminate = *(UCHAR *)value;
        return TRUE;
    case WINUSB_AUTO_CLEAR_STALL:
        usb->auto_clear_stall = *(UCHAR *)value;
        return TRUE;
    case WINUSB_RAW_IO:
        usb->raw_io = *(UCHAR *)value;
        return TRUE;
    case WINUSB_IGNORE_SHORT_PACKETS:
        usb->ignore_short_packets = *(UCHAR *)value;
        return TRUE;
    case WINUSB_ALLOW_PARTIAL_READS:
        usb->allow_partial_reads = *(UCHAR *)value;
        return TRUE;
    case WINUSB_AUTO_FLUSH:
        return TRUE;
    default:
        FIXME("Unhandled policy %#lx.\n", policy_type);
        return TRUE;
    }

    SetLastError(ERROR_INVALID_PARAMETER);
    return FALSE;
}

/***********************************************************************
 *           WinUsb_GetPipePolicy (winusb.@)
 */
BOOL WINAPI WinUsb_GetPipePolicy(WINUSB_INTERFACE_HANDLE handle, UCHAR pipe_id,
        ULONG policy_type, ULONG *value_length, void *value)
{
    struct winusb_handle *usb = handle;

    TRACE("handle %p, pipe %#x, policy %#lx.\n", handle, pipe_id, policy_type);

    if (!usb || !value || !value_length)
    {
        SetLastError(ERROR_INVALID_PARAMETER);
        return FALSE;
    }

    switch (policy_type)
    {
    case WINUSB_PIPE_TRANSFER_TIMEOUT:
        if (*value_length < sizeof(ULONG)) break;
        *(ULONG *)value = usb->pipe_timeout;
        *value_length = sizeof(ULONG);
        return TRUE;
    case WINUSB_SHORT_PACKET_TERMINATE:
        *(UCHAR *)value = usb->short_packet_terminate; *value_length = 1; return TRUE;
    case WINUSB_AUTO_CLEAR_STALL:
        *(UCHAR *)value = usb->auto_clear_stall; *value_length = 1; return TRUE;
    case WINUSB_RAW_IO:
        *(UCHAR *)value = usb->raw_io; *value_length = 1; return TRUE;
    case WINUSB_IGNORE_SHORT_PACKETS:
        *(UCHAR *)value = usb->ignore_short_packets; *value_length = 1; return TRUE;
    case WINUSB_ALLOW_PARTIAL_READS:
        *(UCHAR *)value = usb->allow_partial_reads; *value_length = 1; return TRUE;
    default:
        FIXME("Unhandled policy %#lx.\n", policy_type);
        break;
    }

    SetLastError(ERROR_INVALID_PARAMETER);
    return FALSE;
}

/***********************************************************************
 *           WinUsb_ResetPipe (winusb.@)
 */
BOOL WINAPI WinUsb_ResetPipe(WINUSB_INTERFACE_HANDLE handle, UCHAR pipe_id)
{
    struct winusb_handle *usb = handle;
    struct wineusb_pipe_request req;

    TRACE("handle %p, pipe %#x.\n", handle, pipe_id);

    if (!usb)
    {
        SetLastError(ERROR_INVALID_HANDLE);
        return FALSE;
    }

    /* Clears a stall and discards anything still queued on the pipe. Callers
     * use this to resynchronise a request/reply protocol, so leaving it a stub
     * meant a stale reply was read in place of the next one. */
    req.endpoint = pipe_id;

    return do_transfer(usb->device, IOCTL_WINEUSB_RESET_PIPE,
            &req, sizeof(req), NULL, 0, NULL, NULL);
}

/***********************************************************************
 *           WinUsb_AbortPipe (winusb.@)
 */
BOOL WINAPI WinUsb_AbortPipe(WINUSB_INTERFACE_HANDLE handle, UCHAR pipe_id)
{
    struct winusb_handle *usb = handle;
    struct wineusb_pipe_request req;

    TRACE("handle %p, pipe %#x.\n", handle, pipe_id);

    if (!usb)
    {
        SetLastError(ERROR_INVALID_HANDLE);
        return FALSE;
    }

    /* Cancels the transfers queued on this pipe. They complete separately,
     * with ERROR_OPERATION_ABORTED, which is how a caller waiting on an
     * overlapped read learns that its timeout expired. */
    req.endpoint = pipe_id;

    return do_transfer(usb->device, IOCTL_WINEUSB_ABORT_PIPE,
            &req, sizeof(req), NULL, 0, NULL, NULL);
}

/***********************************************************************
 *           WinUsb_FlushPipe (winusb.@)
 */
BOOL WINAPI WinUsb_FlushPipe(WINUSB_INTERFACE_HANDLE handle, UCHAR pipe_id)
{
    struct winusb_handle *usb = handle;
    struct wineusb_pipe_request req;

    TRACE("handle %p, pipe %#x.\n", handle, pipe_id);

    if (!usb)
    {
        SetLastError(ERROR_INVALID_HANDLE);
        return FALSE;
    }

    /* Discards data cached on the pipe. We have nothing host-side to drop, so
     * cancelling whatever is queued on the endpoint is the closest thing. */
    req.endpoint = pipe_id;

    return do_transfer(usb->device, IOCTL_WINEUSB_ABORT_PIPE,
            &req, sizeof(req), NULL, 0, NULL, NULL);
}

/***********************************************************************
 *           WinUsb_GetCurrentAlternateSetting (winusb.@)
 */
BOOL WINAPI WinUsb_GetCurrentAlternateSetting(WINUSB_INTERFACE_HANDLE handle, UCHAR *setting)
{
    struct winusb_handle *usb = handle;

    TRACE("handle %p, setting %p.\n", handle, setting);

    if (!usb || !setting) { SetLastError(ERROR_INVALID_PARAMETER); return FALSE; }
    *setting = usb->alt_setting;
    return TRUE;
}

/***********************************************************************
 *           WinUsb_SetCurrentAlternateSetting (winusb.@)
 */
BOOL WINAPI WinUsb_SetCurrentAlternateSetting(WINUSB_INTERFACE_HANDLE handle, UCHAR setting)
{
    struct winusb_handle *usb = handle;
    struct wineusb_control_request req;

    TRACE("handle %p, setting %u.\n", handle, setting);

    if (!usb) { SetLastError(ERROR_INVALID_HANDLE); return FALSE; }

    /* SET_INTERFACE */
    req.setup[0] = 0x01;
    req.setup[1] = 0x0b;
    req.setup[2] = setting;
    req.setup[3] = 0;
    req.setup[4] = usb->interface_index;
    req.setup[5] = 0;
    req.setup[6] = 0;
    req.setup[7] = 0;

    if (!do_transfer(usb->device, IOCTL_WINEUSB_CONTROL_TRANSFER,
            &req, sizeof(req), NULL, 0, NULL, NULL))
        return FALSE;

    usb->alt_setting = setting;
    return TRUE;
}

/***********************************************************************
 *           WinUsb_QueryDeviceInformation (winusb.@)
 */
BOOL WINAPI WinUsb_QueryDeviceInformation(WINUSB_INTERFACE_HANDLE handle,
        ULONG info_type, ULONG *buffer_length, void *buffer)
{
    struct winusb_handle *usb = handle;

    TRACE("handle %p, type %#lx.\n", handle, info_type);

    if (!usb || !buffer || !buffer_length)
    {
        SetLastError(ERROR_INVALID_PARAMETER);
        return FALSE;
    }

    if (info_type == 1)     /* DEVICE_SPEED */
    {
        if (*buffer_length < 1) { SetLastError(ERROR_INSUFFICIENT_BUFFER); return FALSE; }
        *(UCHAR *)buffer = 3;   /* HighSpeed */
        *buffer_length = 1;
        return TRUE;
    }

    FIXME("Unhandled information type %#lx.\n", info_type);
    SetLastError(ERROR_INVALID_PARAMETER);
    return FALSE;
}

/***********************************************************************
 *           WinUsb_GetAssociatedInterface (winusb.@)
 */
BOOL WINAPI WinUsb_GetAssociatedInterface(WINUSB_INTERFACE_HANDLE handle, UCHAR index,
        WINUSB_INTERFACE_HANDLE *associated)
{
    struct winusb_handle *usb = handle, *new_handle;

    TRACE("handle %p, index %u, associated %p.\n", handle, index, associated);

    if (!usb || !associated)
    {
        SetLastError(ERROR_INVALID_HANDLE);
        return FALSE;
    }

    /* Callers enumerate associated interfaces until this fails, so it must
     * stop once the requested interface does not exist. That needs the
     * configuration descriptor, so fetch it if this is the first use --
     * libusb calls this during claim_interface() and treats a failure here as
     * the interface being unusable. */
    ensure_config(usb);

    {
        const UCHAR *p = usb->config, *end;
        unsigned int count = 0;

        if (!p)
        {
            TRACE("-> no config descriptor available, failing.\n");
            SetLastError(ERROR_NO_MORE_ITEMS);
            return FALSE;
        }
        end = p + usb->config_size;
        for (p += p[0]; p + 2 <= end && p[0]; p += p[0])
            if (p[1] == 0x04 && p + 9 <= end && p[3] == 0)
                count++;

        if (usb->interface_index + index + 1 >= count)
        {
            TRACE("-> no interface at index %u (have %u, base %u), failing.\n",
                    index, count, usb->interface_index);
            SetLastError(ERROR_NO_MORE_ITEMS);
            return FALSE;
        }
    }

    if (!(new_handle = calloc(1, sizeof(*new_handle))))
    {
        SetLastError(ERROR_NOT_ENOUGH_MEMORY);
        return FALSE;
    }

    new_handle->device = usb->device;
    new_handle->interface_index = usb->interface_index + index + 1;

    *associated = (WINUSB_INTERFACE_HANDLE)new_handle;
    TRACE("-> produced handle %p for interface %u.\n",
            new_handle, new_handle->interface_index);
    return TRUE;
}
