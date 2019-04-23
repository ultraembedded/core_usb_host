#ifndef __USB_DEFS_H__
#define __USB_DEFS_H__

#include <stdint.h>

//-----------------------------------------------------------------
// Defines:
//-----------------------------------------------------------------

// Response values
#define USB_RES_OK          0
#define USB_RES_NAK         -1
#define USB_RES_STALL       -2
#define USB_RES_TIMEOUT     -3

// USB PID generation macro
#define PID_GENERATE(pid3, pid2, pid1, pid0) ((pid0 << 0) | (pid1 << 1) | (pid2 << 2) | (pid3 << 3) | ((!pid0) << 4) | ((!pid1) << 5) | ((!pid2) << 6)  | ((!pid3) << 7))

// USB PID values
#define PID_OUT        PID_GENERATE(0,0,0,1) // 0xE1
#define PID_IN         PID_GENERATE(1,0,0,1) // 0x69
#define PID_SOF        PID_GENERATE(0,1,0,1) // 0xA5
#define PID_SETUP      PID_GENERATE(1,1,0,1) // 0x2D

#define PID_DATA0      PID_GENERATE(0,0,1,1) // 0xC3
#define PID_DATA1      PID_GENERATE(1,0,1,1) // 0x4B

#define PID_ACK        PID_GENERATE(0,0,1,0) // 0xD2
#define PID_NAK        PID_GENERATE(1,0,1,0) // 0x5A
#define PID_STALL      PID_GENERATE(1,1,1,0) // 0x1E

// Standard requests (via SETUP packets)
#define REQ_GET_STATUS        0x00
#define REQ_CLEAR_FEATURE     0x01
#define REQ_SET_FEATURE       0x03
#define REQ_SET_ADDRESS       0x05
#define REQ_GET_DESCRIPTOR    0x06
#define REQ_SET_DESCRIPTOR    0x07
#define REQ_GET_CONFIGURATION 0x08
#define REQ_SET_CONFIGURATION 0x09
#define REQ_GET_INTERFACE     0x0A
#define REQ_SET_INTERFACE     0x0B
#define REQ_SYNC_FRAME        0x0C

// Descriptor types
#define DESC_DEVICE           0x01
#define DESC_CONFIGURATION    0x02
#define DESC_STRING           0x03
#define DESC_INTERFACE        0x04
#define DESC_ENDPOINT         0x05
#define DESC_DEV_QUALIFIER    0x06
#define DESC_OTHER_SPEED_CONF 0x07
#define DESC_IF_POWER         0x08

// Device class
#define DEV_CLASS_RESERVED      0x00
#define DEV_CLASS_AUDIO         0x01
#define DEV_CLASS_COMMS         0x02
#define DEV_CLASS_HID           0x03
#define DEV_CLASS_MONITOR       0x04
#define DEV_CLASS_PHY_IF        0x05
#define DEV_CLASS_POWER         0x06
#define DEV_CLASS_PRINTER       0x07
#define DEV_CLASS_STORAGE       0x08
#define DEV_CLASS_HUB           0x09
#define DEV_CLASS_TMC           0xFE
#define DEV_CLASS_VENDOR_CUSTOM 0xFF

// Device Requests (bmRequestType)
#define REQDIR_HOSTTODEVICE        (0 << 7)
#define REQDIR_DEVICETOHOST        (1 << 7)
#define REQTYPE_STANDARD           (0 << 5)
#define REQTYPE_CLASS              (1 << 5)
#define REQTYPE_VENDOR             (2 << 5)
#define REQREC_DEVICE              (0 << 0)
#define REQREC_INTERFACE           (1 << 0)
#define REQREC_ENDPOINT            (2 << 0)
#define REQREC_OTHER               (3 << 0)

// Endpoints
#define ENDPOINT_DIR_MASK          (1 << 7)
#define ENDPOINT_DIR_IN            (1 << 7)
#define ENDPOINT_DIR_OUT           (0 << 7)
#define ENDPOINT_ADDR_MASK         (0x7F)
#define ENDPOINT_TYPE_MASK         (0x3)
#define ENDPOINT_TYPE_CONTROL      (0)
#define ENDPOINT_TYPE_ISO          (1)
#define ENDPOINT_TYPE_BULK         (2)
#define ENDPOINT_TYPE_INTERRUPT    (3)

//-----------------------------------------------------------------
// Structures:
//-----------------------------------------------------------------
struct UsbDeviceDescriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t bcdUSB;
    uint8_t bDeviceClass;
    uint8_t bDeviceSubClass;
    uint8_t bDeviceProtocol;
    uint8_t bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t iManufacturer;
    uint8_t iProduct;
    uint8_t iSerialNumber;
    uint8_t bNumConfigurations;
} __attribute__ ((packed));

struct UsbConfigurationDescriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint16_t wTotalLength;
    uint8_t bNumInterfaces;
    uint8_t bConfigurationValue;
    uint8_t iConfiguration;
    uint8_t bmAttributes;
    uint8_t bMaxPower;
} __attribute__ ((packed));

struct UsbInterfaceDescriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} __attribute__ ((packed));

struct UsbEndpointDescriptor
{
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bEndpointAddress;
    uint8_t bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t bInterval;
} __attribute__ ((packed));

struct UsbDescriptorHeader
{
    uint8_t size;
    uint8_t type;
} __attribute__ ((packed));

#endif // __USB_DEFS_H__
