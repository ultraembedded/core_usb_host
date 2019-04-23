#ifndef __USB_CORE_H__
#define __USB_CORE_H__

#include "usb_defs.h"
#include "usb_hw.h"

//-----------------------------------------------------------------
// Defines:
//-----------------------------------------------------------------
#define MAX_ENDPOINTS               6
#define MAX_INTERFACES              6

// For Big Endian CPUs
#ifdef USB_BIG_ENDIAN
    #define USB_BYTE_SWAP16(n)          ((((uint16_t)((n) & 0xff)) << 8) | (((n) & 0xff00) >> 8))
#else
    #define USB_BYTE_SWAP16(n)          n
#endif

// Compare PID/VID to usb_device structure values
#define USB_COMPARE_VID_PID(dev, v, p)  (((dev)->vid == (v)) && ((dev)->pid == (p)))

//-----------------------------------------------------------------
// Structures:
//-----------------------------------------------------------------
typedef enum
{
    USB_DEVSTATE_DEFAULT,
    USB_DEVSTATE_ADDRESSED,
    USB_DEVSTATE_CONFIGURED,
    USB_DEVSTATE_ENUMERATED
}  usb_device_state;

struct usb_device;

struct usb_endpoint
{
    struct usb_device *  dev;
    uint8_t              endpoint;
    uint8_t              direction;
    uint8_t              endpoint_type;
    uint16_t             max_packet_size;
    uint8_t              data_toggle;
};

struct usb_interface
{
    uint8_t             if_class;
    uint8_t             if_subclass;
    uint8_t             if_protocol;
    uint8_t             if_endpoints;
    struct usb_endpoint endpoint[MAX_ENDPOINTS];
};

struct usb_device
{
    uint8_t              address;
    usb_device_state     state;
    uint16_t             vid;
    uint16_t             pid;
    struct usb_interface interfaces[MAX_INTERFACES];
};

//-----------------------------------------------------------------
// Prototypes:
//-----------------------------------------------------------------
#ifdef __cplusplus
extern "C" {
#endif

void usb_init(void);
void usb_reset_device(struct usb_device *dev);
int  usb_configure_device(struct usb_device *dev, int device_address);
int  usb_enumerate(struct usb_device *dev, 
                   int (*enum_interface)(struct usb_device *dev, struct usb_interface *intp),
                   int (*enum_class_specific)(struct usb_device *dev, struct usb_interface *intp, void *desc));
struct usb_endpoint * usb_find_endpoint(struct usb_interface *intp, uint8_t type, uint8_t dir);
int  usb_clear_stall(struct usb_endpoint *endp);
int  usb_send_control_read(struct usb_device *dev, uint8_t request_type, uint8_t request, uint16_t value, uint16_t index, uint16_t length, uint8_t *rx_buf, uint16_t rx_length);
int  usb_send_control_write(struct usb_device *dev, uint8_t request_type, uint8_t request, uint16_t value, uint16_t index, uint16_t length, uint8_t *data);
int  usb_bulk_write(struct usb_endpoint *endp, uint8_t *p, int length, int timeout_ms);
int  usb_bulk_read(struct usb_endpoint *endp, uint8_t *p, int length, int timeout_ms);
int  usb_iso_write(struct usb_endpoint *endp, uint8_t *p, int length);

#ifdef __cplusplus
}
#endif

#endif // __USB_CORE_H__
