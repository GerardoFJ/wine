/*
 * USB root device enumerator using libusb
 *
 * Copyright 2020 Zebediah Figura
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

#include <assert.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#define INITGUID

#include "ntstatus.h"
#include "windef.h"
#include "winioctl.h"
#include "winternl.h"
#include "ddk/wdm.h"
#include "ddk/usb.h"
#include "ddk/usbioctl.h"
#include "wine/asm.h"
#include "wine/debug.h"
#include "wine/list.h"

#include "unixlib.h"

WINE_DEFAULT_DEBUG_CHANNEL(wineusb);

static const GUID GUID_DEVINTERFACE_USB_HOST_CONTROLLER =
        {0x3abf6f2d, 0x71c4, 0x462a, {0x8a, 0x92, 0x1e, 0x68, 0x61, 0xe6, 0xaf, 0x27}};

static const GUID GUID_DEVINTERFACE_USB_HUB =
        {0xf18a0e88, 0xc30c, 0x11d0, {0x88, 0x15, 0x00, 0xa0, 0xc9, 0x06, 0xbe, 0xd8}};

static const GUID GUID_DEVINTERFACE_USB_DEVICE =
        {0xa5dcbf10, 0x6530, 0x11d2, {0x90, 0x1f, 0x00, 0xc0, 0x4f, 0xb9, 0x51, 0xed}};

static const GUID GUID_DEVINTERFACE_LIBUSBK =
        {0xf9f3ff14, 0xae21, 0x48a0, {0x8a, 0x25, 0x80, 0x11, 0xa7, 0xa9, 0x31, 0xd9}};

static const GUID GUID_DEVINTERFACE_WINUSB =
        {0xdee824ef, 0x729b, 0x4a0e, {0x9c, 0x14, 0xb7, 0x11, 0x7d, 0x33, 0xa8, 0x17}};

/* Private interface between winusb.dll and wineusb.sys. Keep in sync with
 * dlls/winusb/main.c. */
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

/* USB hub IOCTLs. Wine's ddk/usbioctl.h does not define these. */
#define IOCTL_USB_GET_NODE_INFORMATION \
    CTL_CODE(FILE_DEVICE_USB, 258, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_USB_GET_NODE_CONNECTION_INFORMATION \
    CTL_CODE(FILE_DEVICE_USB, 259, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_USB_GET_NODE_CONNECTION_INFORMATION_EX \
    CTL_CODE(FILE_DEVICE_USB, 274, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_USB_GET_NODE_CONNECTION_INFORMATION_EX_V2 \
    CTL_CODE(FILE_DEVICE_USB, 279, METHOD_BUFFERED, FILE_ANY_ACCESS)
#define IOCTL_USB_GET_DESCRIPTOR_FROM_NODE_CONNECTION \
    CTL_CODE(FILE_DEVICE_USB, 260, METHOD_BUFFERED, FILE_ANY_ACCESS)

#include <pshpack1.h>
typedef struct
{
    UCHAR  bLength, bDescriptorType, bEndpointAddress, bmAttributes;
    USHORT wMaxPacketSize;
    UCHAR  bInterval;
} hub_endpoint_desc;

typedef struct
{
    hub_endpoint_desc EndpointDescriptor;
    ULONG ScheduleOffset;
} hub_pipe_info;

typedef struct
{
    UCHAR  bLength, bDescriptorType;
    USHORT bcdUSB;
    UCHAR  bDeviceClass, bDeviceSubClass, bDeviceProtocol, bMaxPacketSize0;
    USHORT idVendor, idProduct, bcdDevice;
    UCHAR  iManufacturer, iProduct, iSerialNumber, bNumConfigurations;
} hub_device_desc;

typedef struct
{
    ULONG           ConnectionIndex;
    hub_device_desc DeviceDescriptor;
    UCHAR           CurrentConfigurationValue;
    UCHAR           Speed;
    BOOLEAN         DeviceIsHub;
    USHORT          DeviceAddress;
    ULONG           NumberOfOpenPipes;
    ULONG           ConnectionStatus;
    hub_pipe_info   PipeList[1];
} hub_node_connection_info_ex;

typedef struct
{
    ULONG ConnectionIndex;
    struct
    {
        UCHAR  bmRequest, bRequest;
        USHORT wValue, wIndex, wLength;
    } SetupPacket;
    UCHAR Data[1];
} hub_descriptor_request;
#include <poppack.h>

C_ASSERT(offsetof(hub_node_connection_info_ex, PipeList) == 35);
C_ASSERT(offsetof(hub_descriptor_request, Data) == 12);

#ifdef __ASM_USE_FASTCALL_WRAPPER

extern void * WINAPI wrap_fastcall_func1( void *func, const void *a );
__ASM_STDCALL_FUNC( wrap_fastcall_func1, 8,
                   "popl %ecx\n\t"
                   "popl %eax\n\t"
                   "xchgl (%esp),%ecx\n\t"
                   "jmp *%eax" );

#define call_fastcall_func1(func,a) wrap_fastcall_func1(func,a)

#else

#define call_fastcall_func1(func,a) func(a)

#endif

#define DECLARE_CRITICAL_SECTION(cs) \
    static CRITICAL_SECTION cs; \
    static CRITICAL_SECTION_DEBUG cs##_debug = \
    { 0, 0, &cs, { &cs##_debug.ProcessLocksList, &cs##_debug.ProcessLocksList }, \
      0, 0, { (DWORD_PTR)(__FILE__ ": " # cs) }}; \
    static CRITICAL_SECTION cs = { &cs##_debug, -1, 0, 0, 0, 0 };

DECLARE_CRITICAL_SECTION(wineusb_cs);

static struct list device_list = LIST_INIT(device_list);

struct usb_device
{
    struct list entry;
    BOOL removed;

    DEVICE_OBJECT *device_obj;

    bool interface;
    int16_t interface_index;
    struct usb_device *parent;      /* composite device, for interfaces */

    uint8_t class, subclass, protocol, busnum, portnum;

    uint16_t vendor, product, revision, usbver;

    struct unix_device *unix_device;

    UNICODE_STRING iface;               /* GUID_DEVINTERFACE_WINUSB */
    UNICODE_STRING iface_libusbk;
    UNICODE_STRING iface_usbdev;        /* GUID_DEVINTERFACE_USB_DEVICE */
    BOOL iface_enabled;
    BOOL is_hub;

    LIST_ENTRY irp_list;
};

static DEVICE_OBJECT *hub_pdo;

static DRIVER_OBJECT *driver_obj;
static DEVICE_OBJECT *bus_fdo, *bus_pdo;

static void destroy_unix_device(struct unix_device *unix_device)
{
    struct usb_destroy_device_params params =
    {
        .device = unix_device,
    };

    WINE_UNIX_CALL(unix_usb_destroy_device, &params);
}

static void add_unix_device(const struct usb_add_device_event *event)
{
    static unsigned int name_index;
    struct usb_device *device;
    DEVICE_OBJECT *device_obj;
    UNICODE_STRING string;
    NTSTATUS status;
    WCHAR name[26];

    TRACE("Adding new device %p, vendor %04x, product %04x.\n", event->device,
            event->vendor, event->product);

    swprintf(name, ARRAY_SIZE(name), L"\\Device\\USBPDO-%u", name_index++);
    RtlInitUnicodeString(&string, name);
    if ((status = IoCreateDevice(driver_obj, sizeof(*device), &string,
            FILE_DEVICE_USB, 0, FALSE, &device_obj)))
    {
        ERR("Failed to create device, status %#lx.\n", status);
        return;
    }

    device = device_obj->DeviceExtension;
    device->device_obj = device_obj;
    device->unix_device = event->device;
    InitializeListHead(&device->irp_list);
    device->removed = FALSE;

    device->interface = event->interface;
    device->interface_index = event->interface_index;
    device->parent = NULL;

    RtlInitUnicodeString(&device->iface, NULL);
    RtlInitUnicodeString(&device->iface_libusbk, NULL);
    RtlInitUnicodeString(&device->iface_usbdev, NULL);
    device->iface_enabled = FALSE;

    device->class = event->class;
    device->subclass = event->subclass;
    device->protocol = event->protocol;
    device->busnum = event->busnum;
    device->portnum = event->portnum;

    device->vendor = event->vendor;
    device->product = event->product;
    device->revision = event->revision;

    device->usbver = event->usbver;

    EnterCriticalSection(&wineusb_cs);
    if (event->parent)
    {
        struct usb_device *parent;

        LIST_FOR_EACH_ENTRY(parent, &device_list, struct usb_device, entry)
        {
            if (parent->unix_device == event->parent)
            {
                device->parent = parent;
                break;
            }
        }
    }
    list_add_tail(&device_list, &device->entry);
    LeaveCriticalSection(&wineusb_cs);

    /* An interface belongs to its composite device, everything else to the
     * hub. Invalidating the right parent is what gives the devnode the
     * correct DEVPKEY_Device_Parent. */
    if (device->parent)
        IoInvalidateDeviceRelations(device->parent->device_obj, BusRelations);
    else
        IoInvalidateDeviceRelations(hub_pdo ? hub_pdo : bus_pdo, BusRelations);
}

static void remove_unix_device(struct unix_device *unix_device)
{
    struct usb_device *device, *parent = NULL;

    TRACE("Removing device %p.\n", unix_device);

    EnterCriticalSection(&wineusb_cs);
    LIST_FOR_EACH_ENTRY(device, &device_list, struct usb_device, entry)
    {
        if (device->unix_device == unix_device)
        {
            if (!device->removed)
            {
                device->removed = TRUE;
                list_remove(&device->entry);
            }
            parent = device->parent;
            break;
        }
    }
    LeaveCriticalSection(&wineusb_cs);

    /* Only the hub is refreshed here. The parent of a departing interface is
     * on its way out too -- its interfaces are removed first, then itself --
     * and invalidating it would leave a relations query in flight against a
     * device object that IRP_MN_REMOVE_DEVICE is about to delete. */
    IoInvalidateDeviceRelations(hub_pdo ? hub_pdo : bus_pdo, BusRelations);
}

static HANDLE event_thread;

static void complete_irp(IRP *irp)
{
    NTSTATUS status = STATUS_SUCCESS;
    URB *urb;

    EnterCriticalSection(&wineusb_cs);
    IoSetCancelRoutine(irp, NULL);
    RemoveEntryList(&irp->Tail.Overlay.ListEntry);
    LeaveCriticalSection(&wineusb_cs);

    /* User-mode requests (see driver_ioctl) stash their URB here so the
     * transferred length can be reported back through IoStatus. */
    if ((urb = irp->Tail.Overlay.DriverContext[1]))
    {
        if (urb->UrbHeader.Function == URB_FUNCTION_GET_DESCRIPTOR_FROM_DEVICE)
            irp->IoStatus.Information = urb->UrbControlDescriptorRequest.TransferBufferLength;
        else if (urb->UrbHeader.Function == URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER)
            irp->IoStatus.Information = urb->UrbBulkOrInterruptTransfer.TransferBufferLength;
        else
            irp->IoStatus.Information = urb->UrbControlVendorClassRequest.TransferBufferLength;
        /* A user-mode caller sees only the IRP status, so a failed or aborted
         * transfer has to be reported through it. libusb in particular needs
         * ERROR_OPERATION_ABORTED to tell a timed-out read from a completed
         * one. Internal callers read urb->UrbHeader.Status instead, so this
         * only applies to requests that came through driver_ioctl(). */
        switch (urb->UrbHeader.Status)
        {
            case USBD_STATUS_SUCCESS:
                break;
            case USBD_STATUS_CANCELED:
                status = STATUS_CANCELLED;
                break;
            case USBD_STATUS_TIMEOUT:
                status = STATUS_IO_TIMEOUT;
                break;
            case USBD_STATUS_DEVICE_GONE:
                status = STATUS_DEVICE_NOT_CONNECTED;
                break;
            case USBD_STATUS_ENDPOINT_HALTED:
                status = STATUS_DEVICE_DATA_ERROR;
                break;
            default:
                status = STATUS_UNSUCCESSFUL;
                break;
        }
        if (status != STATUS_SUCCESS)
            irp->IoStatus.Information = 0;

        TRACE("completing user request, status %#lx, %Iu bytes.\n",
                status, irp->IoStatus.Information);
        irp->Tail.Overlay.DriverContext[1] = NULL;
        ExFreePool(urb);
    }

    irp->IoStatus.Status = status;
    IoCompleteRequest(irp, IO_NO_INCREMENT);
}

/* Cancel queued transfers on one pipe, or on every pipe if endpoint is 0.
 * ABORT_PIPE may complete before the outstanding requests do, so there is no
 * need to wait for them here; each completes through the usual event path. */
static void abort_pipe(struct usb_device *device, UCHAR endpoint)
{
    LIST_ENTRY *entry, *mark;

    TRACE("device %p, endpoint %#x.\n", device, endpoint);

    EnterCriticalSection(&wineusb_cs);
    mark = &device->irp_list;
    for (entry = mark->Flink; entry != mark; entry = entry->Flink)
    {
        IRP *queued_irp = CONTAINING_RECORD(entry, IRP, Tail.Overlay.ListEntry);
        URB *queued_urb = IoGetCurrentIrpStackLocation(queued_irp)->Parameters.Others.Argument1;
        struct usb_cancel_transfer_params params;

        if (endpoint && queued_urb
                && queued_urb->UrbHeader.Function == URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER
                && (UCHAR)(ULONG_PTR)queued_urb->UrbBulkOrInterruptTransfer.PipeHandle != endpoint)
            continue;

        params.transfer = queued_irp->Tail.Overlay.DriverContext[0];
        WINE_UNIX_CALL(unix_usb_cancel_transfer, &params);
    }
    LeaveCriticalSection(&wineusb_cs);
}

/* Installed on every pending transfer IRP so that CancelIo()/CancelIoEx()
 * reach the device. libusb relies on this for its transfer timeouts: it
 * cancels the overlapped read and expects ERROR_OPERATION_ABORTED back. */
static void WINAPI cancel_transfer_irp(DEVICE_OBJECT *device_obj, IRP *irp)
{
    struct usb_device *device = device_obj->DeviceExtension;
    LIST_ENTRY *entry, *mark;

    TRACE("device_obj %p, irp %p.\n", device_obj, irp);

    IoReleaseCancelSpinLock(irp->CancelIrql);

    /* Only cancel while the IRP is still queued. complete_irp() dequeues under
     * the same lock, and libusb frees the transfer once its callback has run. */
    EnterCriticalSection(&wineusb_cs);
    mark = &device->irp_list;
    for (entry = mark->Flink; entry != mark; entry = entry->Flink)
    {
        if (CONTAINING_RECORD(entry, IRP, Tail.Overlay.ListEntry) == irp)
        {
            struct usb_cancel_transfer_params params =
            {
                .transfer = irp->Tail.Overlay.DriverContext[0],
            };

            WINE_UNIX_CALL(unix_usb_cancel_transfer, &params);
            break;
        }
    }
    LeaveCriticalSection(&wineusb_cs);
}

static DWORD CALLBACK event_thread_proc(void *arg)
{
    struct usb_event event;
    struct usb_main_loop_params params =
    {
        .event = &event,
    };

    TRACE("Starting event thread.\n");

    if (WINE_UNIX_CALL(unix_usb_init, NULL) != STATUS_SUCCESS)
        return 0;

    while (WINE_UNIX_CALL(unix_usb_main_loop, &params) == STATUS_PENDING)
    {
        switch (event.type)
        {
            case USB_EVENT_ADD_DEVICE:
                add_unix_device(&event.u.added_device);
                break;

            case USB_EVENT_REMOVE_DEVICE:
                remove_unix_device(event.u.removed_device);
                break;

            case USB_EVENT_TRANSFER_COMPLETE:
                complete_irp(event.u.completed_irp);
                break;
        }
    }

    TRACE("Shutting down event thread.\n");
    return 0;
}

static NTSTATUS fdo_pnp(IRP *irp)
{
    IO_STACK_LOCATION *stack = IoGetCurrentIrpStackLocation(irp);
    NTSTATUS ret;

    TRACE("irp %p, minor function %#x.\n", irp, stack->MinorFunction);

    switch (stack->MinorFunction)
    {
        case IRP_MN_QUERY_DEVICE_RELATIONS:
        {
            struct usb_device *device;
            DEVICE_RELATIONS *devices;
            unsigned int i = 0;

            if (stack->Parameters.QueryDeviceRelations.Type != BusRelations)
            {
                FIXME("Unhandled device relations type %#x.\n", stack->Parameters.QueryDeviceRelations.Type);
                break;
            }

            /* The bus exposes only the synthetic root hub; the USB devices
             * are reported as children of that hub instead. */
            if (!(devices = ExAllocatePool(PagedPool,
                    offsetof(DEVICE_RELATIONS, Objects[1]))))
            {
                irp->IoStatus.Status = STATUS_NO_MEMORY;
                break;
            }

            if (hub_pdo)
            {
                devices->Objects[i++] = hub_pdo;
                call_fastcall_func1(ObfReferenceObject, hub_pdo);
            }
            (void)device;

            devices->Count = i;
            irp->IoStatus.Information = (ULONG_PTR)devices;
            irp->IoStatus.Status = STATUS_SUCCESS;
            break;
        }

        case IRP_MN_START_DEVICE:
            /* Announce the synthetic root hub. The event thread is started
             * once the hub itself has started, so that devices are
             * enumerated as children of the hub rather than of the bus. */
            IoInvalidateDeviceRelations(bus_pdo, BusRelations);
            irp->IoStatus.Status = STATUS_SUCCESS;
            break;

        case IRP_MN_SURPRISE_REMOVAL:
            irp->IoStatus.Status = STATUS_SUCCESS;
            break;

        case IRP_MN_REMOVE_DEVICE:
        {
            struct usb_device *device, *cursor;

            WINE_UNIX_CALL(unix_usb_exit, NULL);
            WaitForSingleObject(event_thread, INFINITE);
            CloseHandle(event_thread);

            EnterCriticalSection(&wineusb_cs);
            /* Normally we unlink all devices either:
             *
             * - as a result of hot-unplug, which unlinks the device, and causes
             *   a subsequent IRP_MN_REMOVE_DEVICE which will free it;
             *
             * - if the parent is deleted (at shutdown time), in which case
             *   ntoskrnl will send us IRP_MN_SURPRISE_REMOVAL and
             *   IRP_MN_REMOVE_DEVICE unprompted.
             *
             * But we can get devices hotplugged between when shutdown starts
             * and now, in which case they'll be stuck in this list and never
             * freed.
             *
             * FIXME: This is still broken, though. If a device is hotplugged
             * and then removed, it'll be unlinked and never freed. */
            LIST_FOR_EACH_ENTRY_SAFE(device, cursor, &device_list, struct usb_device, entry)
            {
                assert(!device->removed);
                destroy_unix_device(device->unix_device);
                list_remove(&device->entry);
                IoDeleteDevice(device->device_obj);
            }
            LeaveCriticalSection(&wineusb_cs);

            irp->IoStatus.Status = STATUS_SUCCESS;
            IoSkipCurrentIrpStackLocation(irp);
            ret = IoCallDriver(bus_pdo, irp);
            IoDetachDevice(bus_pdo);
            IoDeleteDevice(bus_fdo);
            return ret;
        }

        case IRP_MN_QUERY_ID:
            break;

        default:
            FIXME("Unhandled minor function %#x.\n", stack->MinorFunction);
    }

    IoSkipCurrentIrpStackLocation(irp);
    return IoCallDriver(bus_pdo, irp);
}

struct string_buffer
{
    WCHAR *string;
    size_t len;
};

static void WINAPIV append_id(struct string_buffer *buffer, const WCHAR *format, ...)
{
    va_list args;
    WCHAR *string;
    int len;

    va_start(args, format);

    len = _vsnwprintf(NULL, 0, format, args) + 1;
    if (!(string = ExAllocatePool(PagedPool, (buffer->len + len) * sizeof(WCHAR))))
    {
        if (buffer->string)
            ExFreePool(buffer->string);
        buffer->string = NULL;
        return;
    }
    if (buffer->string)
    {
        memcpy(string, buffer->string, buffer->len * sizeof(WCHAR));
        ExFreePool(buffer->string);
    }
    _vsnwprintf(string + buffer->len, len, format, args);
    buffer->string = string;
    buffer->len += len;

    va_end(args);
}

static void get_device_id(const struct usb_device *device, struct string_buffer *buffer)
{
    if (device->interface)
        append_id(buffer, L"USB\\VID_%04X&PID_%04X&MI_%02X",
                device->vendor, device->product, device->interface_index);
    else
        append_id(buffer, L"USB\\VID_%04X&PID_%04X", device->vendor, device->product);
}

static void get_instance_id(const struct usb_device *device, struct string_buffer *buffer)
{
    append_id(buffer, L"%u&%u&%u&%u", device->usbver, device->revision, device->busnum, device->portnum);
}

static void get_hardware_ids(const struct usb_device *device, struct string_buffer *buffer)
{
    if (device->interface)
        append_id(buffer, L"USB\\VID_%04X&PID_%04X&REV_%04X&MI_%02X",
                device->vendor, device->product, device->revision, device->interface_index);
    else
        append_id(buffer, L"USB\\VID_%04X&PID_%04X&REV_%04X",
                device->vendor, device->product, device->revision);

    get_device_id(device, buffer);
    append_id(buffer, L"");
}

static void get_compatible_ids(const struct usb_device *device, struct string_buffer *buffer)
{
    if (device->interface_index != -1)
    {
        append_id(buffer, L"USB\\Class_%02x&SubClass_%02x&Prot_%02x",
                device->class, device->subclass, device->protocol);
        append_id(buffer, L"USB\\Class_%02x&SubClass_%02x", device->class, device->subclass);
        append_id(buffer, L"USB\\Class_%02x", device->class);
    }
    else
    {
        append_id(buffer, L"USB\\DevClass_%02x&SubClass_%02x&Prot_%02x",
                device->class, device->subclass, device->protocol);
        append_id(buffer, L"USB\\DevClass_%02x&SubClass_%02x", device->class, device->subclass);
        append_id(buffer, L"USB\\DevClass_%02x", device->class);
    }
    append_id(buffer, L"");
}

static NTSTATUS query_id(struct usb_device *device, IRP *irp, BUS_QUERY_ID_TYPE type)
{
    struct string_buffer buffer = {0};

    TRACE("type %#x.\n", type);

    if (device->is_hub)
    {
        static const WCHAR device_id[] = L"USB\\ROOT_HUB20";
        static const WCHAR instance_id[] = L"0&ROOTHUB&0";
        const WCHAR *str;
        WCHAR *out;

        switch (type)
        {
            case BusQueryDeviceID:      str = device_id; break;
            case BusQueryInstanceID:    str = instance_id; break;
            case BusQueryHardwareIDs:   str = device_id; break;
            case BusQueryCompatibleIDs: str = device_id; break;
            default:
                return irp->IoStatus.Status;
        }

        if (!(out = ExAllocatePool(PagedPool, (wcslen(str) + 2) * sizeof(WCHAR))))
            return STATUS_NO_MEMORY;
        wcscpy(out, str);
        out[wcslen(str) + 1] = 0;   /* double-NUL for the ID lists */
        irp->IoStatus.Information = (ULONG_PTR)out;
        return STATUS_SUCCESS;
    }

    switch (type)
    {
        case BusQueryDeviceID:
            get_device_id(device, &buffer);
            break;

        case BusQueryInstanceID:
            get_instance_id(device, &buffer);
            break;

        case BusQueryHardwareIDs:
            get_hardware_ids(device, &buffer);
            break;

        case BusQueryCompatibleIDs:
            get_compatible_ids(device, &buffer);
            break;

        default:
            FIXME("Unhandled ID query type %#x.\n", type);
            return irp->IoStatus.Status;
    }

    if (!buffer.string)
        return STATUS_NO_MEMORY;

    irp->IoStatus.Information = (ULONG_PTR)buffer.string;
    return STATUS_SUCCESS;
}

static void remove_pending_irps(struct usb_device *device)
{
    LIST_ENTRY *entry;
    IRP *irp;

    while ((entry = RemoveHeadList(&device->irp_list)) != &device->irp_list)
    {
        irp = CONTAINING_RECORD(entry, IRP, Tail.Overlay.ListEntry);
        irp->IoStatus.Status = STATUS_DELETE_PENDING;
        irp->IoStatus.Information = 0;
        IoCompleteRequest(irp, IO_NO_INCREMENT);
    }
}

static NTSTATUS pdo_pnp(DEVICE_OBJECT *device_obj, IRP *irp)
{
    IO_STACK_LOCATION *stack = IoGetCurrentIrpStackLocation(irp);
    struct usb_device *device = device_obj->DeviceExtension;
    NTSTATUS ret = irp->IoStatus.Status;

    TRACE("device_obj %p, irp %p, minor function %#x.\n", device_obj, irp, stack->MinorFunction);

    if (stack->MinorFunction == IRP_MN_QUERY_DEVICE_RELATIONS
            && stack->Parameters.QueryDeviceRelations.Type == BusRelations)
    {
        BOOL is_hub = (device_obj == hub_pdo);
        struct usb_device *child, *self = NULL;
        DEVICE_RELATIONS *devices;
        unsigned int i = 0;

        EnterCriticalSection(&wineusb_cs);

        /* ntoskrnl can deliver a relations query after IRP_MN_REMOVE_DEVICE
         * has already deleted the device object, so the extension must not be
         * read until the object is known to still be linked. The hub is never
         * in device_list and is never removed, so it is matched by pointer. */
        if (!is_hub)
        {
            LIST_FOR_EACH_ENTRY(child, &device_list, struct usb_device, entry)
            {
                if (child->device_obj == device_obj)
                {
                    self = child;
                    break;
                }
            }

            /* Gone, or an interface, which has no children of its own. */
            if (!self || self->interface)
            {
                NTSTATUS status = irp->IoStatus.Status;

                LeaveCriticalSection(&wineusb_cs);
                IoCompleteRequest(irp, IO_NO_INCREMENT);
                return status;
            }
        }

        if (!(devices = ExAllocatePool(PagedPool,
                offsetof(DEVICE_RELATIONS, Objects[list_count(&device_list)]))))
        {
            LeaveCriticalSection(&wineusb_cs);
            irp->IoStatus.Status = STATUS_NO_MEMORY;
            IoCompleteRequest(irp, IO_NO_INCREMENT);
            return STATUS_NO_MEMORY;
        }

        LIST_FOR_EACH_ENTRY(child, &device_list, struct usb_device, entry)
        {
            /* The hub reports the devices plugged into it; each composite
             * device reports its own interfaces, the way usbccgp does on
             * Windows. libusb walks that parentage to attach an interface's
             * device path to the composite device, and without it
             * claim_interface() has no path and fails. */
            if (is_hub)
            {
                if (child->interface)
                    continue;
            }
            else if (child->parent != self)
                continue;

            devices->Objects[i++] = child->device_obj;
            call_fastcall_func1(ObfReferenceObject, child->device_obj);
        }

        LeaveCriticalSection(&wineusb_cs);

        devices->Count = i;
        irp->IoStatus.Information = (ULONG_PTR)devices;
        irp->IoStatus.Status = STATUS_SUCCESS;
        IoCompleteRequest(irp, IO_NO_INCREMENT);
        TRACE("%s reported %u children.\n", is_hub ? "Root hub" : "Device", i);
        return STATUS_SUCCESS;
    }

    switch (stack->MinorFunction)
    {
        case IRP_MN_QUERY_ID:
            ret = query_id(device, irp, stack->Parameters.QueryId.IdType);
            break;

        case IRP_MN_QUERY_CAPABILITIES:
        {
            DEVICE_CAPABILITIES *caps = stack->Parameters.DeviceCapabilities.Capabilities;

            caps->RawDeviceOK = 1;

            ret = STATUS_SUCCESS;
            break;
        }

        case IRP_MN_START_DEVICE:
            if (device->is_hub && !device->iface_enabled)
            {
                NTSTATUS hs = IoRegisterDeviceInterface(device_obj,
                        &GUID_DEVINTERFACE_USB_HUB, NULL, &device->iface);
                if (!hs)
                {
                    hs = IoSetDeviceInterfaceState(&device->iface, TRUE);
                    device->iface_enabled = TRUE;
                }
                TRACE("root hub interface -> %#lx.\n", hs);
                /* Devices may already have been discovered before the hub
                 * was started; re-enumerate so they are parented on it. */
                if (!event_thread)
                    event_thread = CreateThread(NULL, 0, event_thread_proc, NULL, 0, NULL);
                ret = STATUS_SUCCESS;
                break;
            }
            if (!device->iface_enabled)
            {
                NTSTATUS status = IoRegisterDeviceInterface(device_obj,
                        &GUID_DEVINTERFACE_WINUSB, NULL, &device->iface);
                if (status)
                {
                    WARN("Failed to register device interface, status %#lx.\n", status);
                    RtlInitUnicodeString(&device->iface, NULL);
                }
                else
                {
                    NTSTATUS status_k = IoRegisterDeviceInterface(device_obj,
                            &GUID_DEVINTERFACE_LIBUSBK, NULL, &device->iface_libusbk);
                    if (!IoRegisterDeviceInterface(device_obj,
                            &GUID_DEVINTERFACE_USB_DEVICE, NULL, &device->iface_usbdev))
                        IoSetDeviceInterfaceState(&device->iface_usbdev, TRUE);
                    if (!status_k)
                        status_k = IoSetDeviceInterfaceState(&device->iface_libusbk, TRUE);
                    TRACE("libusbK interface for %04x:%04x -> %#lx.\n",
                            device->vendor, device->product, status_k);
                    status = IoSetDeviceInterfaceState(&device->iface, TRUE);
                    TRACE("Registered interface for %04x:%04x, enable status %#lx.\n",
                            device->vendor, device->product, status);
                    device->iface_enabled = TRUE;
                }
            }
            ret = STATUS_SUCCESS;
            break;

        case IRP_MN_SURPRISE_REMOVAL:
            if (device->iface_enabled)
            {
                if (device->iface.Buffer)
                    IoSetDeviceInterfaceState(&device->iface, FALSE);
                if (device->iface_libusbk.Buffer)
                    IoSetDeviceInterfaceState(&device->iface_libusbk, FALSE);
                if (device->iface_usbdev.Buffer)
                    IoSetDeviceInterfaceState(&device->iface_usbdev, FALSE);
                device->iface_enabled = FALSE;
            }
            EnterCriticalSection(&wineusb_cs);
            remove_pending_irps(device);
            if (!device->removed)
            {
                device->removed = TRUE;
                list_remove(&device->entry);
            }
            LeaveCriticalSection(&wineusb_cs);
            ret = STATUS_SUCCESS;
            break;

        case IRP_MN_REMOVE_DEVICE:
            assert(device->removed);
            remove_pending_irps(device);

            destroy_unix_device(device->unix_device);

            IoDeleteDevice(device->device_obj);
            ret = STATUS_SUCCESS;
            break;

        case IRP_MN_QUERY_DEVICE_TEXT:
            WARN("Unhandled IRP_MN_QUERY_DEVICE_TEXT text type %u.\n", stack->Parameters.QueryDeviceText.DeviceTextType);
            break;

        default:
            FIXME("Unhandled minor function %#x.\n", stack->MinorFunction);
    }

    irp->IoStatus.Status = ret;
    IoCompleteRequest(irp, IO_NO_INCREMENT);
    return ret;
}

static NTSTATUS WINAPI driver_pnp(DEVICE_OBJECT *device, IRP *irp)
{
    if (device == bus_fdo)
        return fdo_pnp(irp);
    return pdo_pnp(device, irp);
}

static NTSTATUS usb_submit_urb(struct usb_device *device, IRP *irp)
{
    URB *urb = IoGetCurrentIrpStackLocation(irp)->Parameters.Others.Argument1;
    NTSTATUS status;

    TRACE("type %#x.\n", urb->UrbHeader.Function);

    switch (urb->UrbHeader.Function)
    {
        case URB_FUNCTION_ABORT_PIPE:
            abort_pipe(device, (UCHAR)(ULONG_PTR)urb->UrbPipeRequest.PipeHandle);
            return STATUS_SUCCESS;

        case URB_FUNCTION_CONTROL_TRANSFER:
        case URB_FUNCTION_SYNC_RESET_PIPE_AND_CLEAR_STALL:
        case URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER:
        case URB_FUNCTION_GET_DESCRIPTOR_FROM_DEVICE:
        case URB_FUNCTION_SELECT_CONFIGURATION:
        case URB_FUNCTION_VENDOR_DEVICE:
        case URB_FUNCTION_VENDOR_INTERFACE:
        case URB_FUNCTION_VENDOR_ENDPOINT:
        {
            struct usb_submit_urb_params params =
            {
                .device = device->unix_device,
                .irp = irp,
            };

            switch (urb->UrbHeader.Function)
            {
                case URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER:
                {
                    struct _URB_BULK_OR_INTERRUPT_TRANSFER *req = &urb->UrbBulkOrInterruptTransfer;
                    if (req->TransferBufferMDL)
                        params.transfer_buffer = MmGetSystemAddressForMdlSafe(req->TransferBufferMDL, NormalPagePriority);
                    else
                        params.transfer_buffer = req->TransferBuffer;
                    break;
                }

                case URB_FUNCTION_GET_DESCRIPTOR_FROM_DEVICE:
                {
                    struct _URB_CONTROL_DESCRIPTOR_REQUEST *req = &urb->UrbControlDescriptorRequest;
                    if (req->TransferBufferMDL)
                        params.transfer_buffer = MmGetSystemAddressForMdlSafe(req->TransferBufferMDL, NormalPagePriority);
                    else
                        params.transfer_buffer = req->TransferBuffer;
                    break;
                }

                case URB_FUNCTION_CONTROL_TRANSFER:
                {
                    struct _URB_CONTROL_TRANSFER *req = &urb->UrbControlTransfer;
                    if (req->TransferBufferMDL)
                        params.transfer_buffer = MmGetSystemAddressForMdlSafe(req->TransferBufferMDL, NormalPagePriority);
                    else
                        params.transfer_buffer = req->TransferBuffer;
                    break;
                }

                case URB_FUNCTION_VENDOR_DEVICE:
                case URB_FUNCTION_VENDOR_INTERFACE:
                case URB_FUNCTION_VENDOR_ENDPOINT:
                {
                    struct _URB_CONTROL_VENDOR_OR_CLASS_REQUEST *req = &urb->UrbControlVendorClassRequest;
                    if (req->TransferBufferMDL)
                        params.transfer_buffer = MmGetSystemAddressForMdlSafe(req->TransferBufferMDL, NormalPagePriority);
                    else
                        params.transfer_buffer = req->TransferBuffer;
                    break;
                }
            }

            /* Hold the wineusb lock while submitting and queuing, and
             * similarly hold it in complete_irp(). That way, if libusb reports
             * completion between submitting and queuing, we won't try to
             * dequeue the IRP until it's actually been queued. */
            EnterCriticalSection(&wineusb_cs);
            status = WINE_UNIX_CALL(unix_usb_submit_urb, &params);
            if (status == STATUS_PENDING)
            {
                IoMarkIrpPending(irp);
                InsertTailList(&device->irp_list, &irp->Tail.Overlay.ListEntry);

                IoSetCancelRoutine(irp, cancel_transfer_irp);
                if (irp->Cancel && IoSetCancelRoutine(irp, NULL))
                {
                    /* Cancelled before the routine was installed, so it will
                     * not run; cancel the transfer here instead. */
                    struct usb_cancel_transfer_params cancel_params =
                    {
                        .transfer = irp->Tail.Overlay.DriverContext[0],
                    };

                    WINE_UNIX_CALL(unix_usb_cancel_transfer, &cancel_params);
                }
            }
            LeaveCriticalSection(&wineusb_cs);

            return status;
        }

        default:
            FIXME("Unhandled function %#x.\n", urb->UrbHeader.Function);
    }

    return STATUS_NOT_IMPLEMENTED;
}

static NTSTATUS WINAPI driver_internal_ioctl(DEVICE_OBJECT *device_obj, IRP *irp)
{
    IO_STACK_LOCATION *stack = IoGetCurrentIrpStackLocation(irp);
    ULONG code = stack->Parameters.DeviceIoControl.IoControlCode;
    struct usb_device *device = device_obj->DeviceExtension;
    NTSTATUS status = STATUS_NOT_IMPLEMENTED;
    BOOL removed;

    TRACE("device_obj %p, irp %p, code %#lx.\n", device_obj, irp, code);

    EnterCriticalSection(&wineusb_cs);
    removed = device->removed;
    LeaveCriticalSection(&wineusb_cs);

    if (removed)
    {
        irp->IoStatus.Status = STATUS_DELETE_PENDING;
        IoCompleteRequest(irp, IO_NO_INCREMENT);
        return STATUS_DELETE_PENDING;
    }

    switch (code)
    {
        case IOCTL_INTERNAL_USB_SUBMIT_URB:
            status = usb_submit_urb(device, irp);
            break;

        default:
            FIXME("Unhandled ioctl %#lx (device %#lx, access %#lx, function %#lx, method %#lx).\n",
                    code, code >> 16, (code >> 14) & 3, (code >> 2) & 0xfff, code & 3);
    }

    if (status != STATUS_PENDING)
    {
        irp->IoStatus.Status = status;
        IoCompleteRequest(irp, IO_NO_INCREMENT);
    }
    return status;
}

static NTSTATUS usb_submit_urb(struct usb_device *device, IRP *irp);

static NTSTATUS WINAPI driver_create_close(DEVICE_OBJECT *device_obj, IRP *irp)
{
    IO_STACK_LOCATION *stack = IoGetCurrentIrpStackLocation(irp);
    struct usb_device *device = device_obj->DeviceExtension;

    TRACE("device_obj %p, irp %p.\n", device_obj, irp);

    /* Take the interface for as long as the caller holds the device open, so
     * that a Linux kernel driver bound to it (cdc_acm and friends) is detached
     * and transfers can actually be submitted. Hubs are answered directly and
     * own no interface. */
    if (!device->is_hub && device->unix_device)
    {
        struct usb_claim_interface_params params =
        {
            .device = device->unix_device,
            .interface_number = device->interface ? device->interface_index : 0,
            .claim = (stack->MajorFunction == IRP_MJ_CREATE),
        };

        WINE_UNIX_CALL(unix_usb_claim_interface, &params);
    }

    irp->IoStatus.Status = STATUS_SUCCESS;
    irp->IoStatus.Information = 0;
    IoCompleteRequest(irp, IO_NO_INCREMENT);
    return STATUS_SUCCESS;
}

/* Answer hub port queries so that libusb can enumerate our children. */
static NTSTATUS hub_ioctl(IRP *irp, ULONG code, ULONG out_size)
{
    hub_node_connection_info_ex *info = irp->AssociatedIrp.SystemBuffer;
    struct usb_device *device;
    unsigned int index, i = 1;
    NTSTATUS status = STATUS_INVALID_PARAMETER;

    switch (code)
    {
    case IOCTL_USB_GET_NODE_CONNECTION_INFORMATION:
    case IOCTL_USB_GET_NODE_CONNECTION_INFORMATION_EX:
    case IOCTL_USB_GET_NODE_CONNECTION_INFORMATION_EX_V2:
        /* Callers may pass a buffer without the trailing pipe array. */
        if (!info || out_size < offsetof(hub_node_connection_info_ex, PipeList))
        {
            TRACE("hub ioctl buffer too small: %lu (want %Iu).\n",
                    out_size, offsetof(hub_node_connection_info_ex, PipeList));
            break;
        }

        index = info->ConnectionIndex;
        memset((BYTE *)info + sizeof(info->ConnectionIndex), 0,
                out_size - sizeof(info->ConnectionIndex));
        info->ConnectionIndex = index;

        EnterCriticalSection(&wineusb_cs);
        LIST_FOR_EACH_ENTRY(device, &device_list, struct usb_device, entry)
        {
            (void)i;
            if (device->portnum != index)
                continue;

            info->DeviceDescriptor.bLength = 18;
            info->DeviceDescriptor.bDescriptorType = 1;
            info->DeviceDescriptor.bcdUSB = device->usbver;
            info->DeviceDescriptor.bDeviceClass = device->class;
            info->DeviceDescriptor.bDeviceSubClass = device->subclass;
            info->DeviceDescriptor.bDeviceProtocol = device->protocol;
            info->DeviceDescriptor.bMaxPacketSize0 = 64;
            info->DeviceDescriptor.idVendor = device->vendor;
            info->DeviceDescriptor.idProduct = device->product;
            info->DeviceDescriptor.bcdDevice = device->revision;
            info->DeviceDescriptor.bNumConfigurations = 1;

            info->CurrentConfigurationValue = 1;
            info->Speed = (device->usbver >= 0x0200) ? 2 : 1;   /* high : full */
            info->DeviceIsHub = FALSE;
            info->DeviceAddress = index;
            info->NumberOfOpenPipes = 0;
            info->ConnectionStatus = 1;     /* DeviceConnected */

            status = STATUS_SUCCESS;
            break;
        }
        LeaveCriticalSection(&wineusb_cs);

        if (status == STATUS_SUCCESS)
        {
            irp->IoStatus.Information = out_size;
            TRACE("hub port %u -> %04x:%04x.\n", index,
                    info->DeviceDescriptor.idVendor, info->DeviceDescriptor.idProduct);
        }
        else
        {
            TRACE("hub port %u is empty.\n", index);
            info->ConnectionStatus = 0;     /* NoDeviceConnected */
            irp->IoStatus.Information = out_size;
            status = STATUS_SUCCESS;
        }
        break;

    case IOCTL_USB_GET_DESCRIPTOR_FROM_NODE_CONNECTION:
    {
        static const ULONG header_size = offsetof(hub_descriptor_request, Data);
        hub_descriptor_request *req = irp->AssociatedIrp.SystemBuffer;
        struct unix_device *unix_device = NULL;
        UINT32 ret_size = 0;
        ULONG want;

        if (!req || out_size < header_size)
        {
            TRACE("descriptor request buffer too small: %lu (want %lu).\n",
                    out_size, header_size);
            break;
        }

        index = req->ConnectionIndex;
        want = out_size - header_size;
        if (want > req->SetupPacket.wLength)
            want = req->SetupPacket.wLength;

        EnterCriticalSection(&wineusb_cs);
        LIST_FOR_EACH_ENTRY(device, &device_list, struct usb_device, entry)
        {
            if (device->is_hub || device->interface || device->portnum != index)
                continue;
            unix_device = device->unix_device;
            break;
        }
        LeaveCriticalSection(&wineusb_cs);

        if (!unix_device)
        {
            TRACE("hub port %u is empty.\n", index);
            break;
        }

        {
            struct usb_get_descriptor_params params =
            {
                .device = unix_device,
                .type = req->SetupPacket.wValue >> 8,
                .index = req->SetupPacket.wValue & 0xff,
                .langid = req->SetupPacket.wIndex,
                .buffer = req->Data,
                .size = want,
                .ret_size = &ret_size,
            };

            status = WINE_UNIX_CALL(unix_usb_get_descriptor, &params);
        }

        if (status == STATUS_SUCCESS)
        {
            irp->IoStatus.Information = header_size + ret_size;
            TRACE("hub port %u descriptor %#x index %#x -> %u bytes.\n", index,
                    req->SetupPacket.wValue >> 8, req->SetupPacket.wValue & 0xff, ret_size);
        }
        break;
    }

    default:
        FIXME("Unhandled hub ioctl %#lx.\n", code);
        status = STATUS_NOT_IMPLEMENTED;
    }

    return status;
}

/* Build a URB from a user-mode request and hand it to usb_submit_urb(). */
static NTSTATUS WINAPI driver_ioctl(DEVICE_OBJECT *device_obj, IRP *irp)
{
    IO_STACK_LOCATION *stack = IoGetCurrentIrpStackLocation(irp);
    ULONG code = stack->Parameters.DeviceIoControl.IoControlCode;
    ULONG in_size = stack->Parameters.DeviceIoControl.InputBufferLength;
    ULONG out_size = stack->Parameters.DeviceIoControl.OutputBufferLength;
    struct usb_device *device = device_obj->DeviceExtension;
    void *buffer = irp->AssociatedIrp.SystemBuffer;
    NTSTATUS status = STATUS_NOT_IMPLEMENTED;
    URB *purb;

    TRACE("device_obj %p, irp %p, code %#lx.\n", device_obj, irp, code);

    if (device->is_hub)
    {
        status = hub_ioctl(irp, code, out_size);
        irp->IoStatus.Status = status;
        IoCompleteRequest(irp, IO_NO_INCREMENT);
        return status;
    }

    if (!(purb = ExAllocatePool(NonPagedPool, sizeof(*purb))))
    {
        irp->IoStatus.Status = STATUS_NO_MEMORY;
        IoCompleteRequest(irp, IO_NO_INCREMENT);
        return STATUS_NO_MEMORY;
    }
    memset(purb, 0, sizeof(*purb));
#define urb (*purb)

    switch (code)
    {
        case IOCTL_WINEUSB_CONTROL_TRANSFER:
        {
            struct wineusb_control_request *req = buffer;
            struct _URB_CONTROL_VENDOR_OR_CLASS_REQUEST *r = &urb.UrbControlVendorClassRequest;
            UCHAR setup[8];
            UCHAR type;

            if (in_size < sizeof(*req))
            {
                status = STATUS_INVALID_PARAMETER;
                break;
            }
            memcpy(setup, req->setup, sizeof(setup));
            type = (setup[0] >> 5) & 3;
            TRACE("control ioctl on device_obj %p, in_size %lu, setup %02x %02x %02x %02x %02x %02x %02x %02x\n",
                    device_obj, in_size, setup[0], setup[1], setup[2], setup[3],
                    setup[4], setup[5], setup[6], setup[7]);

            if (type == 0 && setup[1] == 0x06)
            {
                struct _URB_CONTROL_DESCRIPTOR_REQUEST *d = &urb.UrbControlDescriptorRequest;

                d->Hdr.Length = sizeof(*d);
                d->Hdr.Function = URB_FUNCTION_GET_DESCRIPTOR_FROM_DEVICE;
                d->TransferBufferLength = out_size;
                d->TransferBuffer = out_size ? buffer : NULL;
                d->Index = setup[2];
                d->DescriptorType = setup[3];
                d->LanguageId = setup[4] | (setup[5] << 8);
            }
            else
            {
                r->Hdr.Length = sizeof(*r);
                if (type == 1)
                    r->Hdr.Function = URB_FUNCTION_CLASS_DEVICE;
                else
                    r->Hdr.Function = URB_FUNCTION_VENDOR_DEVICE;
                r->TransferFlags = USBD_SHORT_TRANSFER_OK;
                if (setup[0] & 0x80)
                {
                    r->TransferFlags |= USBD_TRANSFER_DIRECTION_IN;
                    r->TransferBufferLength = out_size;
                    r->TransferBuffer = out_size ? buffer : NULL;
                }
                else
                {
                    r->TransferBufferLength = in_size - sizeof(*req);
                    r->TransferBuffer = r->TransferBufferLength ? (char *)buffer + sizeof(*req) : NULL;
                }
                r->RequestTypeReservedBits = setup[0];
                r->Request = setup[1];
                r->Value = setup[2] | (setup[3] << 8);
                r->Index = setup[4] | (setup[5] << 8);
            }

            stack->Parameters.Others.Argument1 = purb;
            irp->Tail.Overlay.DriverContext[1] = purb;
            status = usb_submit_urb(device, irp);
            if (status != STATUS_PENDING)
            {
                if (urb.UrbHeader.Function == URB_FUNCTION_GET_DESCRIPTOR_FROM_DEVICE)
                    irp->IoStatus.Information = urb.UrbControlDescriptorRequest.TransferBufferLength;
                else
                    irp->IoStatus.Information = r->TransferBufferLength;
                TRACE("control transfer done, status %#lx, %Iu bytes.\n",
                        status, irp->IoStatus.Information);
            }
            break;
        }

        case IOCTL_WINEUSB_PIPE_TRANSFER:
        {
            struct wineusb_pipe_request *req = buffer;
            struct _URB_BULK_OR_INTERRUPT_TRANSFER *r = &urb.UrbBulkOrInterruptTransfer;

            if (in_size < sizeof(*req))
            {
                status = STATUS_INVALID_PARAMETER;
                break;
            }

            r->Hdr.Length = sizeof(*r);
            r->Hdr.Function = URB_FUNCTION_BULK_OR_INTERRUPT_TRANSFER;
            r->TransferFlags = USBD_SHORT_TRANSFER_OK;
            if (req->endpoint & 0x80)
            {
                /* Device to host: the reply lands in the output buffer. */
                r->TransferFlags |= USBD_TRANSFER_DIRECTION_IN;
                r->TransferBufferLength = out_size;
                r->TransferBuffer = out_size ? buffer : NULL;
            }
            else
            {
                /* Host to device: METHOD_BUFFERED only delivers the input
                 * buffer, so the payload follows the header there. */
                r->TransferBufferLength = in_size - sizeof(*req);
                r->TransferBuffer = r->TransferBufferLength ? (char *)buffer + sizeof(*req) : NULL;
            }

            /* Same packing as make_pipe_handle() in unixlib.c: the low byte
             * is the endpoint address, the next byte the pipe type. */
            r->PipeHandle = (HANDLE)(ULONG_PTR)(req->endpoint |
                    (UsbdPipeTypeBulk << 8));

            stack->Parameters.Others.Argument1 = purb;
            irp->Tail.Overlay.DriverContext[1] = purb;
            status = usb_submit_urb(device, irp);
            if (status != STATUS_PENDING)
                irp->IoStatus.Information = r->TransferBufferLength;
            break;
        }

        case IOCTL_WINEUSB_RESET_PIPE:
        {
            struct wineusb_pipe_request *req = buffer;
            struct _URB_PIPE_REQUEST *r = &urb.UrbPipeRequest;

            if (in_size < sizeof(*req))
            {
                status = STATUS_INVALID_PARAMETER;
                break;
            }

            /* Clears a stall and drops whatever the endpoint still holds, so
             * that a caller resynchronising a protocol does not read a reply
             * left over from an earlier, timed-out request. */
            abort_pipe(device, req->endpoint);

            r->Hdr.Length = sizeof(*r);
            r->Hdr.Function = URB_FUNCTION_SYNC_RESET_PIPE_AND_CLEAR_STALL;
            r->PipeHandle = (HANDLE)(ULONG_PTR)(req->endpoint |
                    (UsbdPipeTypeBulk << 8));

            stack->Parameters.Others.Argument1 = purb;
            irp->Tail.Overlay.DriverContext[1] = purb;
            status = usb_submit_urb(device, irp);
            break;
        }

        case IOCTL_WINEUSB_ABORT_PIPE:
        {
            struct wineusb_pipe_request *req = buffer;

            if (in_size < sizeof(*req))
            {
                status = STATUS_INVALID_PARAMETER;
                break;
            }

            abort_pipe(device, req->endpoint);
            status = STATUS_SUCCESS;
            break;
        }

        default:
            FIXME("Unhandled ioctl %#lx.\n", code);
    }

#undef urb
    if (status != STATUS_PENDING)
    {
        irp->Tail.Overlay.DriverContext[1] = NULL;
        ExFreePool(purb);
        irp->IoStatus.Status = status;
        IoCompleteRequest(irp, IO_NO_INCREMENT);
    }
    return status;
}

/* Expose the bus root as both a USB host controller and a hub. libusb walks
 * the devnode ancestry looking for a listed controller/hub and discards any
 * device whose ancestors it has not seen. */
static void register_bus_interfaces(DEVICE_OBJECT *pdo)
{
    static UNICODE_STRING hcd_iface, hub_iface;
    NTSTATUS status;

    status = IoRegisterDeviceInterface(pdo, &GUID_DEVINTERFACE_USB_HOST_CONTROLLER,
            NULL, &hcd_iface);
    if (!status)
    {
        status = IoSetDeviceInterfaceState(&hcd_iface, TRUE);
        TRACE("host controller interface -> %#lx.\n", status);
    }
    else
        WARN("Failed to register host controller interface, status %#lx.\n", status);


}

/* Synthetic root hub: libusb requires a hub between the host controller and
 * the USB devices, and discards devices whose ancestry it cannot resolve. */
static void create_hub_pdo(DRIVER_OBJECT *driver)
{
    UNICODE_STRING string;
    struct usb_device *hub;
    NTSTATUS status;

    RtlInitUnicodeString(&string, L"\\Device\\USBROOTHUB");
    if ((status = IoCreateDevice(driver, sizeof(*hub), &string,
            FILE_DEVICE_USB, 0, FALSE, &hub_pdo)))
    {
        ERR("Failed to create root hub, status %#lx.\n", status);
        hub_pdo = NULL;
        return;
    }

    hub = hub_pdo->DeviceExtension;
    memset(hub, 0, sizeof(*hub));
    hub->device_obj = hub_pdo;
    hub->is_hub = TRUE;
    hub->vendor = 0x1d6b;       /* Linux Foundation */
    hub->product = 0x0002;      /* 2.0 root hub      */
    hub->revision = 0x0100;
    hub->usbver = 0x0200;
    hub->class = 0x09;          /* hub */
    InitializeListHead(&hub->irp_list);

    hub_pdo->Flags &= ~DO_DEVICE_INITIALIZING;
    TRACE("Created synthetic root hub %p.\n", hub_pdo);
}

static NTSTATUS WINAPI driver_add_device(DRIVER_OBJECT *driver, DEVICE_OBJECT *pdo)
{
    NTSTATUS ret;

    TRACE("driver %p, pdo %p.\n", driver, pdo);

    create_hub_pdo(driver);

    register_bus_interfaces(pdo);

    if ((ret = IoCreateDevice(driver, 0, NULL, FILE_DEVICE_BUS_EXTENDER, 0, FALSE, &bus_fdo)))
    {
        ERR("Failed to create FDO, status %#lx.\n", ret);
        return ret;
    }

    IoAttachDeviceToDeviceStack(bus_fdo, pdo);
    bus_pdo = pdo;
    bus_fdo->Flags &= ~DO_DEVICE_INITIALIZING;

    return STATUS_SUCCESS;
}

static void WINAPI driver_unload(DRIVER_OBJECT *driver)
{
}

NTSTATUS WINAPI DriverEntry(DRIVER_OBJECT *driver, UNICODE_STRING *path)
{
    NTSTATUS status;

    TRACE("driver %p, path %s.\n", driver, debugstr_w(path->Buffer));

    if ((status = __wine_init_unix_call()))
    {
        ERR("Failed to initialize Unix library, status %#lx.\n", status);
        return status;
    }

    driver_obj = driver;

    driver->DriverExtension->AddDevice = driver_add_device;
    driver->DriverUnload = driver_unload;
    driver->MajorFunction[IRP_MJ_PNP] = driver_pnp;
    driver->MajorFunction[IRP_MJ_INTERNAL_DEVICE_CONTROL] = driver_internal_ioctl;
    driver->MajorFunction[IRP_MJ_CREATE] = driver_create_close;
    driver->MajorFunction[IRP_MJ_CLOSE] = driver_create_close;
    driver->MajorFunction[IRP_MJ_DEVICE_CONTROL] = driver_ioctl;

    return STATUS_SUCCESS;
}
