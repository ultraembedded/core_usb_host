#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "usb_hw.h"
#include "usb_core.h"
#include "usb_defs.h"

//-----------------------------------------------------------------
// Logging levels:
//-----------------------------------------------------------------
#define USBLOG_NONE     0
#define USBLOG_ERR      1
#define USBLOG_ENUM     2
#define USBLOG_INFO     3
#define USBLOG_DATA     4

// Current log level
#define USBLOG_LEVEL    USBLOG_ERR

#define USBLOG(l,a)     do { if (l <= USBLOG_LEVEL) printf a; } while (0)

//-----------------------------------------------------------------
// Defines:
//-----------------------------------------------------------------

// Arbitrary buffer size!
#define MAX_CONF_SIZE               256

#define USB_CONTROL_RETRIES         5
#define USB_CONTROL_NAK_RETRY_MS    2
#define USB_CONTROL_NAK_RETRIES     (50/USB_CONTROL_NAK_RETRY_MS)

//-----------------------------------------------------------------
// Locals:
//-----------------------------------------------------------------
static uint8_t _desc_buffer[MAX_CONF_SIZE];

//-----------------------------------------------------------------
// usb_setup_packet: Create & send SETUP packet
//-----------------------------------------------------------------
static int usb_setup_packet(int device_address, uint8_t request_type, uint8_t request, uint16_t value, uint16_t index, uint16_t length)
{
    uint8_t usb_request[8];
    int idx = 0;

    /* bmRequestType:
        D7 Data Phase Transfer Direction
        0 = Host to Device
        1 = Device to Host
        D6..5 Type
        0 = Standard
        1 = Class
        2 = Vendor
        3 = Reserved
        D4..0 Recipient
        0 = Device
        1 = Interface
        2 = Endpoint
        3 = Other
     */
    usb_request[idx++] = request_type;
    usb_request[idx++] = request;
    usb_request[idx++] = (value >> 0) & 0xFF;
    usb_request[idx++] = (value >> 8) & 0xFF;
    usb_request[idx++] = (index >> 0) & 0xFF;
    usb_request[idx++] = (index >> 8) & 0xFF;
    usb_request[idx++] = (length >> 0) & 0xFF;
    usb_request[idx++] = (length >> 8) & 0xFF;

    // Send SETUP token + DATA0 (always DATA0)
    return usbhw_transfer_out(PID_SETUP, device_address, 0, 1, PID_DATA0, usb_request, idx) == USB_RES_OK;
}
//-----------------------------------------------------------------
// usb_send_zlp: Send zero length packet (TOKEN + DATA)
//-----------------------------------------------------------------
static int usb_send_zlp(int device_address, uint8_t token)
{
    // DATAx + CRC (always 0 for zero length packet)

    // Send OUT token + DATAx
    return usbhw_transfer_out(PID_OUT, device_address, 0, 1, token, 0, 0) == USB_RES_OK;
}
//-----------------------------------------------------------------
// usb_control_read: Perform a read on a control endpoint (IN)
//-----------------------------------------------------------------
static int usb_control_read(struct usb_device *dev, uint8_t *p, int length)
{
    struct usb_endpoint *endp = &dev->interfaces[0].endpoint[0];
    int res;
    int count = 0;
    uint8_t response;
    uint8_t expected = endp->data_toggle ? PID_DATA1 : PID_DATA0;

    // Send IN request
    while (count++ < USB_CONTROL_NAK_RETRIES)
    {
        res = usbhw_transfer_in(PID_IN, dev->address, 0, &response, p, length);

        // Check for incorrect DATAx response
        if ((response == PID_DATA0 && endp->data_toggle) || (response == PID_DATA1 && !endp->data_toggle))
        {
            USBLOG(USBLOG_ERR, ("usb_control_read: Duplicate DATAx response recieved %x\n", response));
            // Retry ...
        }
        // If device NAK'd, try again
        else if (res != USB_RES_NAK)
            break;

        // Wait a couple of ms on retries        
        usbhw_timer_sleep(USB_CONTROL_NAK_RETRY_MS);
    }

    return res;
}
//-----------------------------------------------------------------
// usb_set_device_address: Set device address
//-----------------------------------------------------------------
static int usb_set_device_address(struct usb_device *dev, int device_address)
{
    return usb_send_control_write(dev,         /* current address */
                                (REQDIR_HOSTTODEVICE | REQREC_DEVICE | REQTYPE_STANDARD),
                                REQ_SET_ADDRESS,
                                (uint16_t) device_address, /* value */
                                0,              /* index */
                                0,              /* length*/
                                0               /* data pointer */
                                );
}
//-----------------------------------------------------------------
// usb_set_configuration: Select device configuration
//-----------------------------------------------------------------
static int usb_set_configuration(struct usb_device *dev, uint8_t configuration)
{
    int i;
    int j;

    // Reset endpoint toggle to DATA0
    for (j=0;j<MAX_INTERFACES;j++)
        for (i=0;i<MAX_ENDPOINTS;i++)
            dev->interfaces[j].endpoint[i].data_toggle = 0;

    return usb_send_control_write(dev,
                                (REQDIR_HOSTTODEVICE | REQREC_DEVICE | REQTYPE_STANDARD),
                                REQ_SET_CONFIGURATION,
                                configuration,  /* value */
                                0,              /* index */
                                0,              /* length*/
                                0               /* data pointer */
                                );
}
//-----------------------------------------------------------------
// usb_get_configuration: Retrieve device configuration
//-----------------------------------------------------------------
static int usb_get_configuration(struct usb_device *dev, uint8_t *configuration)
{
    int res = usb_send_control_read(dev,
                                (REQDIR_DEVICETOHOST | REQREC_DEVICE | REQTYPE_STANDARD), /* bmRequestType */
                                REQ_GET_CONFIGURATION, /* bRequest */
                                0, /* wValue */
                                0, /* wIndex */
                                1, /* wLength*/
                                configuration,
                                1);
    return res;
}
//-----------------------------------------------------------------
// usb_get_descriptor:
//-----------------------------------------------------------------
static int usb_get_descriptor(struct usb_device *dev, uint16_t value, uint16_t index, uint8_t *buffer, uint16_t length)
{
    struct usb_endpoint *endp = &dev->interfaces[0].endpoint[0];

    // Perform initial descriptor fetch
    int res = usb_send_control_read(dev,
                                (REQDIR_DEVICETOHOST | REQREC_DEVICE | REQTYPE_STANDARD), /* bmRequestType */
                                REQ_GET_DESCRIPTOR, /* bRequest */
                                (value<<8), /* wValue */
                                index, /* wIndex */
                                length, /* wLength*/
                                buffer,
                                endp->max_packet_size);
    int received = endp->max_packet_size;

    // Read more data after initial setup?
    while (res && received < length)
    {
        res = usb_control_read(dev, &buffer[received], endp->max_packet_size);
        if (res < 0)
            break;

        received         += res;
        endp->data_toggle = !endp->data_toggle;
    }

    // On success, send ZLP for STATUS stage
    if (res >= 0)
        usb_send_zlp(dev->address, PID_DATA1);
    else
        received = -1;

    return received;
}
//-----------------------------------------------------------------
// usb_send_control_read:
//-----------------------------------------------------------------
int usb_send_control_read(struct usb_device *dev, uint8_t request_type, uint8_t request, uint16_t value, uint16_t index, uint16_t length, uint8_t *rx_buf, uint16_t rx_length)
{
    struct usb_endpoint *endp = &dev->interfaces[0].endpoint[0];
    int res;
    int retries = 0;

    USBLOG(USBLOG_INFO, ("USB: Control Read %x %x %x %x %x\n", request_type, request, value, index, length));

    do
    {
        // SETUP + DATA0 + ACK (device)
        endp->data_toggle = 0;
        res = usb_setup_packet(dev->address, request_type, request, value, index, length);
        if (res)
        {
            // IN + DATA1(device) + ACK
            endp->data_toggle = 1;
            res = usb_control_read(dev, rx_buf, rx_length);
            if (res < 0)
            {
                USBLOG(USBLOG_ERR, ("USB: Control Read IN failed\n"));
                res = 0;
            }
            else
                res = 1;
        }
        else
        {
            USBLOG(USBLOG_ERR, ("USB: Control Read SETUP failed\n"));
        }
    }
    while (!res && retries++ < USB_CONTROL_RETRIES);

    USBLOG(USBLOG_INFO, ("USB: Control Read E%d\n", res));

    return res;
}
//-----------------------------------------------------------------
// usb_send_control_write:
//-----------------------------------------------------------------
int usb_send_control_write(struct usb_device *dev, uint8_t request_type, uint8_t request, uint16_t value, uint16_t index, uint16_t length, uint8_t *data)
{
    struct usb_endpoint *endp = &dev->interfaces[0].endpoint[0];
    int res;
    int retries = 0;

    USBLOG(USBLOG_INFO, ("USB: Control Write %x %x %x %x %x\n", request_type, request, value, index, length));

    do
    {
        // SETUP + DATA0 + ACK (device)
        endp->data_toggle = 0;
        res = usb_setup_packet(dev->address, request_type, request, value, index, length);

        // [Optional] Data packet
        if (res && length && data)
        {
            int remain = length;
            int xfer_size;

            // Next is DATA1
            endp->data_toggle = !endp->data_toggle;

            // Transfer in 8 byte chunks
            while (remain && res)
            {
                if (remain < 8)
                    xfer_size = remain;
                else
                    xfer_size = 8;

                res = (usbhw_transfer_out(PID_OUT, dev->address, 0, 1, endp->data_toggle ? PID_DATA1 : PID_DATA0, data, xfer_size) == USB_RES_OK);
                if (!res)
                {
                    USBLOG(USBLOG_ERR, ("USB: Control Write OUT failed\n"));
                    break;
                }
                else
                    endp->data_toggle = !endp->data_toggle;

                remain -= xfer_size;
                data += xfer_size;
            }
        }

        if (res)
        {
            // Response should be DATAx (ZLP)
            uint8_t response;

            // IN(HOST) + DATA1(DEVICE) + ACK(HOST)
            endp->data_toggle = 1;
            res = usb_control_read(dev, &response, 1);
            if (res < 0)
            {
                USBLOG(USBLOG_ERR, ("USB: Control Write IN failed\n"));
                res = 0;
            }
            else
                res = 1;
        }
        else
        {
            USBLOG(USBLOG_ERR, ("USB: Control Write SETUP failed\n"));
        }
    }
    while (!res && retries++ < USB_CONTROL_RETRIES);

    USBLOG(USBLOG_INFO, ("USB: Control Write E%d\n", res));

    return res;
}
//-----------------------------------------------------------------
// usb_clear_stall:
//-----------------------------------------------------------------
int usb_clear_stall(struct usb_endpoint *endp)
{
    int res;

    USBLOG(USBLOG_INFO, ("USB: Clear stall EP%d\n", endp->endpoint));
    res = usb_send_control_write(endp->dev,
                                  (REQDIR_HOSTTODEVICE | REQREC_ENDPOINT | REQTYPE_STANDARD), /* bmRequestType */
                                  REQ_CLEAR_FEATURE, /* bRequest */
                                  0x00 /*ENDPOINT_HALT*/, /* wValue */
                                  endp->endpoint, /* wIndex (Endpoint number) */
                                  0, /* wLength*/
                                  0  /* data pointer */
                                  );
    USBLOG(USBLOG_INFO, ("USB: Clear stall res%d\n", res));

    return res;
}
//-----------------------------------------------------------------
// usb_bulk_write:
//-----------------------------------------------------------------
int usb_bulk_write(struct usb_endpoint *endp, uint8_t *p, int length, int timeout_ms)
{
    int res;
    int count = 0;
    uint8_t request;
    t_time t_start, t_now;

    USBLOG(USBLOG_INFO, ("USB: Bulk Write EP%d Len %d S\n", endp->endpoint, length));

    // Transfer is either DATA0 or DATA1
    request = endp->data_toggle ? PID_DATA1 : PID_DATA0;

    t_start = usbhw_timer_now();

    // Send DATAx bulk transfer (with retries)
    do
    {
        // Send OUT + DATAx request
        res = usbhw_transfer_out(PID_OUT, endp->dev->address, endp->endpoint, 1, request, p, length);

        // Retry on NAK
        if (res != USB_RES_NAK)
            break;

        // Only NAKs should result in retries
        t_now = usbhw_timer_now();
    }
    while (usbhw_timer_diff(t_now, t_start) < timeout_ms);

    // Toggle DATAx on success
    if (res == USB_RES_OK)
        endp->data_toggle = !endp->data_toggle;
    else
    {
        USBLOG(USBLOG_ERR, ("USB: Bulk Write Err %d\n", res));
    }

    USBLOG(USBLOG_INFO, ("USB: Bulk Write E%d\n", res));

    return res == USB_RES_OK;
}
//-----------------------------------------------------------------
// usb_bulk_read:
//-----------------------------------------------------------------
int usb_bulk_read(struct usb_endpoint *endp, uint8_t *p, int length, int timeout_ms)
{
    int res;
    int count = 0;
    uint8_t response;
    t_time t_start, t_now;

    USBLOG(USBLOG_INFO, ("USB: Bulk Read EP%d Len %d S\n", endp->endpoint, length));

    t_start = usbhw_timer_now();

    // Send DATAx bulk transfer (with retries)
    do
    {
        // Send IN then wait for DATAx then respond with ACK
        res = usbhw_transfer_in(PID_IN, endp->dev->address, endp->endpoint, &response, p, length);

        if (res >= USB_RES_OK)
        {
            // Check for incorrect DATAx response
            if ((response == PID_DATA0 && endp->data_toggle) || (response == PID_DATA1 && !endp->data_toggle))
            {
                USBLOG(USBLOG_ERR, ("USB: Duplicate DATAx response recieved %x\n", response));
                // Retry ...
            }
            else if (response == PID_DATA0 || response == PID_DATA1)
            {
                USBLOG(USBLOG_INFO, ("USB: Bulk Read E%d\n", res));

                // Toggle expected data on valid read
                endp->data_toggle = !endp->data_toggle;
                return res;
            }
            else
            {
                USBLOG(USBLOG_ERR, ("USB: Bulk Read Resp Err %x\n", response));
                res = USB_RES_STALL;
            }
        }
        // STALL means no!
        else if (res == USB_RES_STALL)
        {
            USBLOG(USBLOG_ERR, ("USB: Bulk Read STALL\n"));
            break;
        }
        // Timeout is a failure
        else if (res == USB_RES_TIMEOUT)
        {
            USBLOG(USBLOG_ERR, ("USB: Bulk Read timeout\n"));
            break;
        }
        else if (timeout_ms == 0)
            break;

        // Only NAKs should result in retries
        t_now = usbhw_timer_now();
    }
    while (usbhw_timer_diff(t_now, t_start) < timeout_ms);

    USBLOG(USBLOG_INFO, ("USB: Bulk Read E%d\n", res));

    return res;
}
//-----------------------------------------------------------------
// usb_iso_write:
//-----------------------------------------------------------------
int usb_iso_write(struct usb_endpoint *endp, uint8_t *p, int length)
{
    uint8_t request;

    USBLOG(USBLOG_INFO, ("USB: ISO Write EP%d Len %d S\n", endp->endpoint, length));

    // Transfer is either DATA0 or DATA1
    request = endp->data_toggle ? PID_DATA1 : PID_DATA0;

    // Send DATAx ISO transfer (no response expected)
    usbhw_transfer_out(PID_OUT, endp->dev->address, endp->endpoint, 0, request, p, length);

    // Toggle DATAx on success
    endp->data_toggle = !endp->data_toggle;

    return USB_RES_OK;
}
//-----------------------------------------------------------------
// usb_init:
//-----------------------------------------------------------------
void usb_init(void)
{

}
//-----------------------------------------------------------------
// usb_reset_device: Reset device state
//-----------------------------------------------------------------
void usb_reset_device(struct usb_device *dev)
{
    int i;
    int j;

    dev->address   = 0;
    dev->state = USB_DEVSTATE_DEFAULT;
    
    for (j=0;j<MAX_INTERFACES;j++)
    {
        dev->interfaces[j].if_endpoints= 0;
        dev->interfaces[j].if_class    = 0;
        dev->interfaces[j].if_subclass = 0;
        dev->interfaces[j].if_protocol = 0;

        for (i=0;i<MAX_ENDPOINTS;i++)
        {            
            // Default to 8 byte max transfer size
            dev->interfaces[j].endpoint[i].max_packet_size = 8;

            dev->interfaces[j].endpoint[i].endpoint_type = 0;
            dev->interfaces[j].endpoint[i].dev = dev;
            dev->interfaces[j].endpoint[i].endpoint = 0;
            
            // Reset endpoint toggle to DATA0
            dev->interfaces[j].endpoint[i].data_toggle = 0;
        }
    }
}
//-----------------------------------------------------------------
// usb_configure_device: Try and poll, address & configure device
//-----------------------------------------------------------------
int usb_configure_device(struct usb_device *dev, int device_address)
{
    int res;
    struct UsbDeviceDescriptor *dev_desc;
    struct UsbConfigurationDescriptor *conf_desc;

    //-----------------------
    // State = DEFAULT
    //-----------------------

    // Reset device state
    usb_reset_device(dev);

    dev->interfaces[0].endpoint[0].max_packet_size = sizeof(struct UsbDeviceDescriptor);

    // Get full device descriptor
    dev_desc = (struct UsbDeviceDescriptor *)&_desc_buffer[0];
    res = usb_get_descriptor(dev, DESC_DEVICE, 0, _desc_buffer, sizeof(struct UsbDeviceDescriptor));
    if (res < 0)
    {
        USBLOG(USBLOG_ERR, ("Fetch DESC_DEVICE failed\n"));
        return 0;
    }    

    // Set device address away from default
    res = usb_set_device_address(dev, device_address);
    if (!res)
    {
        USBLOG(USBLOG_ERR, ("SET_ADDRESS failed\n"));
        return 0;
    }

    dev->address = device_address;
    dev->state   = USB_DEVSTATE_ADDRESSED;

    //-----------------------
    // State = ADDRESSED
    //-----------------------

    // Get device descriptor (first 8 bytes)
    dev_desc = (struct UsbDeviceDescriptor *)&_desc_buffer[0];
    res = usb_get_descriptor(dev, DESC_DEVICE, 0, _desc_buffer, 8);
    if (res < 0)
    {
        USBLOG(USBLOG_ERR, ("Fetch DESC_DEVICE failed\n"));
        return 0;
    }

    // Endpoint 0 default setup
    dev->interfaces[0].endpoint[0].max_packet_size = dev_desc->bMaxPacketSize0;
    dev->interfaces[0].endpoint[0].endpoint_type   = ENDPOINT_TYPE_CONTROL;
    dev->interfaces[0].endpoint[0].endpoint        = 0;
    dev->interfaces[0].endpoint[0].direction       = 0;

    // Get full device descriptor
    dev_desc = (struct UsbDeviceDescriptor *)&_desc_buffer[0];
    res = usb_get_descriptor(dev, DESC_DEVICE, 0, _desc_buffer, sizeof(struct UsbDeviceDescriptor));
    if (res < 0)
    {
        USBLOG(USBLOG_ERR, ("Fetch DESC_DEVICE failed\n"));
        return 0;
    }

    dev->vid = USB_BYTE_SWAP16(dev_desc->idVendor);
    dev->pid = USB_BYTE_SWAP16(dev_desc->idProduct);

    USBLOG(USBLOG_ENUM, ("Device VID=%04x PID=%04x:\n", USB_BYTE_SWAP16(dev_desc->idVendor), USB_BYTE_SWAP16(dev_desc->idProduct)));
    USBLOG(USBLOG_ENUM, ("  bcdUSB=%x\n", USB_BYTE_SWAP16(dev_desc->bcdUSB)));
    USBLOG(USBLOG_ENUM, ("  bDeviceClass=%x\n", dev_desc->bDeviceClass));
    USBLOG(USBLOG_ENUM, ("  bDeviceSubClass=%x\n", dev_desc->bDeviceSubClass));
    USBLOG(USBLOG_ENUM, ("  bDeviceProtocol=%x\n", dev_desc->bDeviceProtocol));
    USBLOG(USBLOG_ENUM, ("  bMaxPacketSize0=%d\n", dev_desc->bMaxPacketSize0));
    USBLOG(USBLOG_ENUM, ("  bcdDevice=%x\n", USB_BYTE_SWAP16(dev_desc->bcdDevice)));

    // Get configuration descriptor (configuration 0)
    conf_desc = (struct UsbConfigurationDescriptor *)&_desc_buffer[0];
    res = usb_get_descriptor(dev, DESC_CONFIGURATION, 0, _desc_buffer, sizeof(struct UsbConfigurationDescriptor));
    if (res < 0)
    {
        USBLOG(USBLOG_ERR, ("Fetch DESC_CONF failed\n"));
        return 0;
    }

    // Select configuration
    res = usb_set_configuration(dev, conf_desc->bConfigurationValue);
    if (!res)
    {
        USBLOG(USBLOG_ERR, ("CONFIGURE failed\n"));
        return 0;
    }

    //-----------------------
    // State = CONFIGURED
    //-----------------------
    dev->state = USB_DEVSTATE_CONFIGURED;

    return 1;
}
//-----------------------------------------------------------------
// usb_enumerate: Try and enumerate the attached device
//-----------------------------------------------------------------
int usb_enumerate(struct usb_device *dev, 
                  int (*enum_interface)(struct usb_device *dev, struct usb_interface *intp),
                  int (*enum_class_specific)(struct usb_device *dev, struct usb_interface *intp, void *desc))
{
    int res;
    struct UsbConfigurationDescriptor *conf_desc;
    int num_interfaces;
    int num_endpoints;
    int offset;
    uint16_t conf_size;
    int current_int = 0;
    uint8_t *data;

    // Need callback function to help with this!
    if (!enum_interface)
        return 0;

    // Device state must be configured before enumeration
    if (dev->state != USB_DEVSTATE_CONFIGURED)
        return 0;

    // Get configuration descriptor (configuration 0)
    conf_desc = (struct UsbConfigurationDescriptor *)&_desc_buffer[0];
    res = usb_get_descriptor(dev, DESC_CONFIGURATION, 0, _desc_buffer, sizeof(struct UsbConfigurationDescriptor));
    if (res < 0)
    {
        USBLOG(USBLOG_ERR, ("Fetch DESC_CONFIG failed\n"));
        return 0;
    }

    // Decode required fields
    conf_size = USB_BYTE_SWAP16(conf_desc->wTotalLength);
    if (conf_size > sizeof(_desc_buffer))
    {
        USBLOG(USBLOG_ERR, ("Configuration desc too long %d!\n", conf_size));
        conf_size = sizeof(_desc_buffer);
    }

    // Fetch number of interfaces
    num_interfaces = conf_desc->bNumInterfaces;
    if (num_interfaces > MAX_INTERFACES)
    {
        USBLOG(USBLOG_ERR, ("Too many interfaces %d!\n", num_interfaces));
        return 0;
    }

    // Get complete configuration descriptor including interfaces & endpoints
    res = usb_get_descriptor(dev, DESC_CONFIGURATION, 0, _desc_buffer, conf_size);
    if (res < 0)
    {
        USBLOG(USBLOG_ERR, ("Fetch full DESC_CONFIG failed\n"));
        return 0;
    }

    current_int = -1;

    USBLOG(USBLOG_ENUM, ("Configuration: %d bytes\n", conf_size));

    // Process descriptor contents
    for (offset = conf_desc->bLength;(offset < conf_size) && res; )
    {
        struct UsbDescriptorHeader *desc_hdr = (struct UsbDescriptorHeader *)&_desc_buffer[offset];

        data = (uint8_t*)(&_desc_buffer[offset] + sizeof(struct UsbDescriptorHeader));
        offset += desc_hdr->size;

        switch (desc_hdr->type)
        {
            case DESC_INTERFACE:
            {
                struct UsbInterfaceDescriptor *if_desc = (struct UsbInterfaceDescriptor *)desc_hdr;

                // Notify system of complete interface with endpoints
                if (current_int != -1)
                {
                    res = enum_interface(dev, &dev->interfaces[current_int]);
                    if (!res)
                    {
                        return 0;
                    }
                }

                USBLOG(USBLOG_ENUM, ("Interface %d:\n", current_int + 1));
                USBLOG(USBLOG_ENUM, (" bInterfaceNumber=%x\n", if_desc->bInterfaceNumber));
                USBLOG(USBLOG_ENUM, (" bAlternateSetting=%x\n", if_desc->bAlternateSetting));
                USBLOG(USBLOG_ENUM, (" bNumEndpoints=%x\n", if_desc->bNumEndpoints));
                USBLOG(USBLOG_ENUM, (" bInterfaceClass=%x\n", if_desc->bInterfaceClass));
                USBLOG(USBLOG_ENUM, (" bInterfaceSubClass=%x\n", if_desc->bInterfaceSubClass));
                USBLOG(USBLOG_ENUM, (" bInterfaceProtocol=%x\n", if_desc->bInterfaceProtocol));
                USBLOG(USBLOG_ENUM, (" iInterface=%x\n", if_desc->iInterface));

                if (++current_int >= MAX_INTERFACES)
                {
                    USBLOG(USBLOG_ERR, ("Too many interfaces %d!\n", current_int));
                    return 0;
                }

                num_endpoints = if_desc->bNumEndpoints;
                if ((num_endpoints+1) > MAX_ENDPOINTS)
                {
                    USBLOG(USBLOG_ERR, ("Too many endpoints in interface %d!\n", (num_endpoints+1)));
                    return 0;
                }

                // Extract interface details
                dev->interfaces[current_int].if_class     = if_desc->bInterfaceClass;
                dev->interfaces[current_int].if_subclass  = if_desc->bInterfaceSubClass;
                dev->interfaces[current_int].if_protocol  = if_desc->bInterfaceProtocol;
                dev->interfaces[current_int].if_endpoints = num_endpoints + 1;
            }
            break;

            case DESC_ENDPOINT:
            {
                struct UsbEndpointDescriptor *endp_desc = (struct UsbEndpointDescriptor *)desc_hdr;
                uint8_t ep_addr = endp_desc->bEndpointAddress & ENDPOINT_ADDR_MASK;

                USBLOG(USBLOG_ENUM, ("Endpoint %x:\n", ep_addr));
                USBLOG(USBLOG_ENUM, (" bEndpointAddress=%x\n", endp_desc->bEndpointAddress));
                USBLOG(USBLOG_ENUM, (" bmAttributes=%x\n", endp_desc->bmAttributes));
                USBLOG(USBLOG_ENUM, (" wMaxPacketSize=%d\n",  USB_BYTE_SWAP16(endp_desc->wMaxPacketSize)));
                USBLOG(USBLOG_ENUM, (" bInterval=%d\n", endp_desc->bInterval));

                if (current_int == -1 || ep_addr >= MAX_ENDPOINTS)
                {
                    USBLOG(USBLOG_ERR, ("Endpoint Error!\n"));
                    return 0;
                }

                // TODO: Endpoints can have duplicate address for IN & OUT?!??

                // Store details about endpoint
                dev->interfaces[current_int].endpoint[ep_addr].max_packet_size = USB_BYTE_SWAP16(endp_desc->wMaxPacketSize);
                dev->interfaces[current_int].endpoint[ep_addr].endpoint_type   = endp_desc->bmAttributes & ENDPOINT_TYPE_MASK;
                dev->interfaces[current_int].endpoint[ep_addr].endpoint        = ep_addr;
                dev->interfaces[current_int].endpoint[ep_addr].direction       = endp_desc->bEndpointAddress & ENDPOINT_DIR_MASK;  
            }
            break;

            case DESC_STRING:
            case DESC_DEV_QUALIFIER:
            case DESC_OTHER_SPEED_CONF:
            case DESC_IF_POWER:
            {

            }
            break;

            // Class specific?
            default:
            {
                if (current_int == -1)
                {
                    USBLOG(USBLOG_ERR, ("Class specific error!\n"));
                    return 0;
                }

                if (desc_hdr->size == 0)
                {
                    USBLOG(USBLOG_ERR, ("Invalid descriptor size, check configuration length!\n"));
                    return 0;
                }
                
                USBLOG(USBLOG_ENUM, ("Class Descriptor (Interface %d)\n", current_int));
                USBLOG(USBLOG_ENUM, (" Type=%x\n", desc_hdr->type));
                USBLOG(USBLOG_ENUM, (" Length=%d\n", desc_hdr->size));

                if (enum_class_specific)
                    res = enum_class_specific(dev, &dev->interfaces[current_int], desc_hdr);
                else
                {
                    USBLOG(USBLOG_ERR, ("Unknown descriptor 0x%x Len %d!\n", desc_hdr->type, desc_hdr->size));
                    return 0;
                }
            }
            break;
        }
    }

    // Notify system of last interface
    if (current_int != -1 && res)
    {
        res = enum_interface(dev, &dev->interfaces[current_int]);
        if (!res)
        {
            return 0;
        }
    }

    // Device now enumerated
    if (res)
        dev->state = USB_DEVSTATE_ENUMERATED;

    return res;
}
//-----------------------------------------------------------------
// usb_find_endpoint: Find endpoint type / dir
//-----------------------------------------------------------------
struct usb_endpoint * usb_find_endpoint(struct usb_interface *intp, uint8_t type, uint8_t dir)
{
    int i;
 
    for (i=0;i<intp->if_endpoints;i++)
        if ((intp->endpoint[i].endpoint_type == type) && (intp->endpoint[i].direction == dir))
            return &intp->endpoint[i];

    return 0;
}