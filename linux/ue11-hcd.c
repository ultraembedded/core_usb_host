// SPDX-License-Identifier: GPL-2.0
/*
 * HCD (Host Controller Driver) for USB.
 *
 * Heavily based on SL811HS HCD....
 *
 * Copyright (C) 2004 Psion Teklogix (for NetBook PRO)
 * Copyright (C) 2004-2005 David Brownell
 * Copyright (C) 1999 Roman Weissgaerber
 */
#define DEBUG

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/usb.h>
#include <linux/usb/hcd.h>
#include <linux/platform_device.h>
#include <linux/prefetch.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/of.h>

#include <asm/io.h>
#include <asm/irq.h>
#include <asm/byteorder.h>
#include <asm/unaligned.h>

//-----------------------------------------------------------------
// Logging levels:
//-----------------------------------------------------------------
#define USBLOG_NONE     0
#define USBLOG_ERR      1
#define USBLOG_REQ      2
#define USBLOG_INFO     3
#define USBLOG_DATA     4

// Current USB_LOG level
#define USBLOG_LEVEL    USBLOG_ERR

#define USB_LOG(l,a)    do { if (l <= USBLOG_LEVEL) printk a; } while (0)

//-----------------------------------------------------------------
// Defines:
//-----------------------------------------------------------------
#define USB_CTRL          0x0
    #define USB_CTRL_TX_FLUSH                    8
    #define USB_CTRL_TX_FLUSH_SHIFT              8
    #define USB_CTRL_TX_FLUSH_MASK               0x1

    #define USB_CTRL_PHY_DMPULLDOWN              7
    #define USB_CTRL_PHY_DMPULLDOWN_SHIFT        7
    #define USB_CTRL_PHY_DMPULLDOWN_MASK         0x1

    #define USB_CTRL_PHY_DPPULLDOWN              6
    #define USB_CTRL_PHY_DPPULLDOWN_SHIFT        6
    #define USB_CTRL_PHY_DPPULLDOWN_MASK         0x1

    #define USB_CTRL_PHY_TERMSELECT              5
    #define USB_CTRL_PHY_TERMSELECT_SHIFT        5
    #define USB_CTRL_PHY_TERMSELECT_MASK         0x1

    #define USB_CTRL_PHY_XCVRSELECT_SHIFT        3
    #define USB_CTRL_PHY_XCVRSELECT_MASK         0x3

    #define USB_CTRL_PHY_OPMODE_SHIFT            1
    #define USB_CTRL_PHY_OPMODE_MASK             0x3

    #define USB_CTRL_ENABLE_SOF                  0
    #define USB_CTRL_ENABLE_SOF_SHIFT            0
    #define USB_CTRL_ENABLE_SOF_MASK             0x1

#define USB_STATUS        0x4
    #define USB_STATUS_SOF_TIME_SHIFT            16
    #define USB_STATUS_SOF_TIME_MASK             0xffff

    #define USB_STATUS_RX_ERROR                  2
    #define USB_STATUS_RX_ERROR_SHIFT            2
    #define USB_STATUS_RX_ERROR_MASK             0x1

    #define USB_STATUS_LINESTATE_BITS_SHIFT      0
    #define USB_STATUS_LINESTATE_BITS_MASK       0x3

#define USB_IRQ_ACK       0x8
    #define USB_IRQ_ACK_DEVICE_DETECT            3
    #define USB_IRQ_ACK_DEVICE_DETECT_SHIFT      3
    #define USB_IRQ_ACK_DEVICE_DETECT_MASK       0x1

    #define USB_IRQ_ACK_ERR                      2
    #define USB_IRQ_ACK_ERR_SHIFT                2
    #define USB_IRQ_ACK_ERR_MASK                 0x1

    #define USB_IRQ_ACK_DONE                     1
    #define USB_IRQ_ACK_DONE_SHIFT               1
    #define USB_IRQ_ACK_DONE_MASK                0x1

    #define USB_IRQ_ACK_SOF                      0
    #define USB_IRQ_ACK_SOF_SHIFT                0
    #define USB_IRQ_ACK_SOF_MASK                 0x1

#define USB_IRQ_STS       0xc
    #define USB_IRQ_STS_DEVICE_DETECT            3
    #define USB_IRQ_STS_DEVICE_DETECT_SHIFT      3
    #define USB_IRQ_STS_DEVICE_DETECT_MASK       0x1

    #define USB_IRQ_STS_ERR                      2
    #define USB_IRQ_STS_ERR_SHIFT                2
    #define USB_IRQ_STS_ERR_MASK                 0x1

    #define USB_IRQ_STS_DONE                     1
    #define USB_IRQ_STS_DONE_SHIFT               1
    #define USB_IRQ_STS_DONE_MASK                0x1

    #define USB_IRQ_STS_SOF                      0
    #define USB_IRQ_STS_SOF_SHIFT                0
    #define USB_IRQ_STS_SOF_MASK                 0x1

#define USB_IRQ_MASK      0x10
    #define USB_IRQ_MASK_DEVICE_DETECT           3
    #define USB_IRQ_MASK_DEVICE_DETECT_SHIFT     3
    #define USB_IRQ_MASK_DEVICE_DETECT_MASK      0x1

    #define USB_IRQ_MASK_ERR                     2
    #define USB_IRQ_MASK_ERR_SHIFT               2
    #define USB_IRQ_MASK_ERR_MASK                0x1

    #define USB_IRQ_MASK_DONE                    1
    #define USB_IRQ_MASK_DONE_SHIFT              1
    #define USB_IRQ_MASK_DONE_MASK               0x1

    #define USB_IRQ_MASK_SOF                     0
    #define USB_IRQ_MASK_SOF_SHIFT               0
    #define USB_IRQ_MASK_SOF_MASK                0x1

#define USB_XFER_DATA     0x14
    #define USB_XFER_DATA_TX_LEN_SHIFT           0
    #define USB_XFER_DATA_TX_LEN_MASK            0xffff

#define USB_XFER_TOKEN    0x18
    #define USB_XFER_TOKEN_START                 31
    #define USB_XFER_TOKEN_START_SHIFT           31
    #define USB_XFER_TOKEN_START_MASK            0x1

    #define USB_XFER_TOKEN_IN                    30
    #define USB_XFER_TOKEN_IN_SHIFT              30
    #define USB_XFER_TOKEN_IN_MASK               0x1

    #define USB_XFER_TOKEN_ACK                   29
    #define USB_XFER_TOKEN_ACK_SHIFT             29
    #define USB_XFER_TOKEN_ACK_MASK              0x1

    #define USB_XFER_TOKEN_PID_DATAX             28
    #define USB_XFER_TOKEN_PID_DATAX_SHIFT       28
    #define USB_XFER_TOKEN_PID_DATAX_MASK        0x1

    #define USB_XFER_TOKEN_PID_BITS_SHIFT        16
    #define USB_XFER_TOKEN_PID_BITS_MASK         0xff

    #define USB_XFER_TOKEN_DEV_ADDR_SHIFT        9
    #define USB_XFER_TOKEN_DEV_ADDR_MASK         0x7f

    #define USB_XFER_TOKEN_EP_ADDR_SHIFT         5
    #define USB_XFER_TOKEN_EP_ADDR_MASK          0xf

#define USB_RX_STAT       0x1c
    #define USB_RX_STAT_START_PEND               31
    #define USB_RX_STAT_START_PEND_SHIFT         31
    #define USB_RX_STAT_START_PEND_MASK          0x1

    #define USB_RX_STAT_CRC_ERR                  30
    #define USB_RX_STAT_CRC_ERR_SHIFT            30
    #define USB_RX_STAT_CRC_ERR_MASK             0x1

    #define USB_RX_STAT_RESP_TIMEOUT             29
    #define USB_RX_STAT_RESP_TIMEOUT_SHIFT       29
    #define USB_RX_STAT_RESP_TIMEOUT_MASK        0x1

    #define USB_RX_STAT_IDLE                     28
    #define USB_RX_STAT_IDLE_SHIFT               28
    #define USB_RX_STAT_IDLE_MASK                0x1

    #define USB_RX_STAT_RESP_BITS_SHIFT          16
    #define USB_RX_STAT_RESP_BITS_MASK           0xff

    #define USB_RX_STAT_COUNT_BITS_SHIFT         0
    #define USB_RX_STAT_COUNT_BITS_MASK          0xffff

#define USB_WR_DATA       0x20
    #define USB_WR_DATA_DATA_SHIFT               0
    #define USB_WR_DATA_DATA_MASK                0xff

#define USB_RD_DATA       0x20
    #define USB_RD_DATA_DATA_SHIFT               0
    #define USB_RD_DATA_DATA_MASK                0xff

//-----------------------------------------------------------------

#define LOG2_PERIODIC_SIZE  5   /* arbitrary; this matches OHCI */
#define PERIODIC_SIZE       (1 << LOG2_PERIODIC_SIZE)

struct ue11 {
    spinlock_t          lock;
    void __iomem       *reg_base;

    unsigned long       stat_insrmv;
    unsigned long       stat_wake;
    unsigned long       stat_sof;
    unsigned long       stat_a;
    unsigned long       stat_lost;
    unsigned long       stat_overrun;

    /* sw model */
    struct timer_list   timer;
    struct ue11h_ep *   next_periodic;
    struct ue11h_ep *   next_async;

    struct ue11h_ep *   active_transfer;
    unsigned long       active_start;

    u32                 port1;
    u32                 irq_enable;
    u16                 frame;

    /* async schedule: control, bulk */
    struct list_head    async;

    /* periodic schedule: interrupt, iso */
    u16                 load[PERIODIC_SIZE];
    struct ue11h_ep *   periodic[PERIODIC_SIZE];
    unsigned            periodic_count;
};

static inline struct ue11 *hcd_to_ue11(struct usb_hcd *hcd)
{
    return (struct ue11 *) (hcd->hcd_priv);
}

static inline struct usb_hcd *ue11_to_hcd(struct ue11 *ue11)
{
    return container_of((void *) ue11, struct usb_hcd, hcd_priv);
}

struct ue11h_ep {
    struct usb_host_endpoint *hep;
    struct usb_device   *udev;

    u8          maxpacket;
    u8          epnum;
    u8          nextpid;

    u16         error_count;
    u16         nak_count;
    u16         length;     /* of current packet */

    /* periodic schedule */
    u16         period;
    u16         branch;
    u16         load;
    struct ue11h_ep *next;

    /* async schedule */
    struct list_head    schedule;
};
//-----------------------------------------------------------------


MODULE_DESCRIPTION("UE11 USB Host Controller Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:ue11-hcd");

#define DRIVER_VERSION  "20 Apr 2019"

static const char hcd_name[] = "ue11-hcd";


#if USBLOG_LEVEL >= USBLOG_REQ
//-----------------------------------------------------------------
// dbg_get_ctrl_req_str:
//-----------------------------------------------------------------
static const char *dbg_get_ctrl_req_str(uint8_t request)
{
    switch (request)
    {
        case USB_REQ_GET_STATUS:
            return "GET_STATUS";
        case USB_REQ_CLEAR_FEATURE:
            return "CLEAR_FEATURE";
        case USB_REQ_SET_FEATURE:
            return "SET_FEATURE";
        case USB_REQ_SET_ADDRESS:
            return "SET_ADDRESS";
        case USB_REQ_GET_DESCRIPTOR:
            return "GET_DESCRIPTOR";
        case USB_REQ_SET_DESCRIPTOR:
            return "SET_DESCRIPTOR";
        case USB_REQ_GET_CONFIGURATION:
            return "GET_CONFIGURATION";
        case USB_REQ_SET_CONFIGURATION:
            return "SET_CONFIGURATION";
        case USB_REQ_GET_INTERFACE:
            return "GET_INTERFACE";
        case USB_REQ_SET_INTERFACE:
            return "SET_INTERFACE";
        default:
            return "UNKNOWN";
    }
}
//-----------------------------------------------------------------
// dbg_get_ctrl_req_type_str:
//-----------------------------------------------------------------
static const char *dbg_get_ctrl_req_type_str(uint8_t requestType)
{
    if (requestType & USB_DIR_IN)
    {
        switch (requestType & USB_RECIP_MASK)
        {
            case USB_RECIP_DEVICE:
                return "IN_DEVICE";
            case USB_RECIP_INTERFACE:
                return "IN_INTERFACE";
            case USB_RECIP_ENDPOINT:
                return "IN_ENDPOINT";
            default:
                return "UNKNOWN";
        }
    }
    else
    {
        switch (requestType & USB_RECIP_MASK)
        {
            case USB_RECIP_DEVICE:
                return "OUT_DEVICE";
            case USB_RECIP_INTERFACE:
                return "OUT_INTERFACE";
            case USB_RECIP_ENDPOINT:
                return "OUT_ENDPOINT";
            default:
                return "UNKNOWN";
        }
    }
}
//-----------------------------------------------------------------
// dbg_decode_packet:
//-----------------------------------------------------------------
static void dbg_decode_setup_packet(uint8_t *p)
{
    struct usb_ctrlrequest *ctrl = (struct usb_ctrlrequest*)p;

    printk("Debug: SETUP PACKET\n");
    printk("       bRequestType 0x%x (%s)\n", ctrl->bRequestType, dbg_get_ctrl_req_type_str(ctrl->bRequestType));
    printk("       bRequest 0x%x (%s)\n", ctrl->bRequest, dbg_get_ctrl_req_str(ctrl->bRequest));
    printk("       wValue 0x%x, wIndex 0x%x, wLength %d\n", ctrl->wValue, ctrl->wIndex, ctrl->wLength);
}
#endif
//-----------------------------------------------------------------
// usbhw_hub_reset: Put bus into SE0 state (reset)
//-----------------------------------------------------------------
static void usbhw_hub_reset(struct ue11 *ue11)
{
    uint32_t val;

    USB_LOG(USBLOG_INFO, ("HW: Enter USB bus reset\n"));

    // Power-up / SE0
    val = 0;
    val |= (0 << USB_CTRL_PHY_XCVRSELECT_SHIFT);
    val |= (0 << USB_CTRL_PHY_TERMSELECT_SHIFT);
    val |= (2 << USB_CTRL_PHY_OPMODE_SHIFT);
    val |= (1 << USB_CTRL_PHY_DPPULLDOWN_SHIFT);
    val |= (1 << USB_CTRL_PHY_DMPULLDOWN_SHIFT);

    writel(val, ue11->reg_base + USB_CTRL);
}
//-----------------------------------------------------------------
// usbhw_hub_enable: Enable root hub (drive data lines to HiZ)
//                   and optionally start SOF periods
//-----------------------------------------------------------------
static void usbhw_hub_enable(struct ue11 *ue11, int full_speed, int enable_sof)
{
    uint32_t val;

    USB_LOG(USBLOG_INFO, ("HW: Enable root hub\n"));

    // Host Full Speed
    val = 0;
    val |= (1 << USB_CTRL_PHY_XCVRSELECT_SHIFT);
    val |= (1 << USB_CTRL_PHY_TERMSELECT_SHIFT);
    val |= (0 << USB_CTRL_PHY_OPMODE_SHIFT);
    val |= (1 << USB_CTRL_PHY_DPPULLDOWN_SHIFT);
    val |= (1 << USB_CTRL_PHY_DMPULLDOWN_SHIFT);
    val |= (1 << USB_CTRL_TX_FLUSH_SHIFT);

    // Enable SOF
    if (enable_sof)
        val |= (1 << USB_CTRL_ENABLE_SOF_SHIFT);

    writel(val, ue11->reg_base + USB_CTRL);
}
//-----------------------------------------------------------------
// port_power: Control USB port power enable
//-----------------------------------------------------------------
static void port_power(struct ue11 *ue11, int is_on)
{
    struct usb_hcd  *hcd = ue11_to_hcd(ue11);

    /* hub is inactive unless the port is powered */
    if (is_on) {
        if (ue11->port1 & USB_PORT_STAT_POWER)
            return;

        ue11->port1      = USB_PORT_STAT_POWER;
        ue11->irq_enable = 0;
    } else {
        ue11->port1      = 0;
        ue11->irq_enable = 0;
        hcd->state = HC_STATE_HALT;
    }

    mdelay(20);

    if (is_on)
        usbhw_hub_enable(ue11, 1, 1);
    else
        usbhw_hub_reset(ue11);

    writel(ue11->irq_enable, ue11->reg_base + USB_IRQ_MASK);
}
//-----------------------------------------------------------------
// setup_packet: 
// This is a PIO-only HCD.  Queueing appends URBs to the endpoint's queue,
// and may start I/O.  Endpoint queues are scanned during completion irq
// handlers (one per packet: ACK, NAK, faults, etc) and urb cancellation.
// SETUP starts a new control request.  Devices are not allowed to
// STALL or NAK these; they must cancel any pending control requests.
//-----------------------------------------------------------------
static void setup_packet(struct ue11 *ue11, struct ue11h_ep *ep, struct urb *urb)
{
    int l;
    uint32_t ctrl = 0;
    uint32_t token = 0; 
    uint32_t device_addr = usb_pipedevice(urb->pipe);
    uint32_t endpoint    = usb_pipeendpoint(urb->pipe);

    int len = sizeof(struct usb_ctrlrequest);

    USB_LOG(USBLOG_DATA, ("USB: Send SETUP_PACKET\n"));

    USB_LOG(USBLOG_DATA, ("TOKEN: SETUP"));
    USB_LOG(USBLOG_DATA, ("  DEV %d EP %d\n", device_addr, endpoint));

    // Load DATA0 transfer into address 0+
    USB_LOG(USBLOG_DATA, (" Tx: %02x\n", USB_PID_DATA0));
    for (l=0;l<len;l++)
    {
        USB_LOG(USBLOG_DATA, (" %02x", urb->setup_packet[l]));
        writel(urb->setup_packet[l], ue11->reg_base + USB_WR_DATA);
    }
    USB_LOG(USBLOG_DATA, ("\n"));

#if USBLOG_LEVEL >= USBLOG_REQ
    dbg_decode_setup_packet(urb->setup_packet);
#endif    

    // Transfer data length
    writel(len, ue11->reg_base + USB_XFER_DATA);

    // Configure transfer for DATAx portion
    ctrl = (1 << USB_XFER_TOKEN_START_SHIFT);
    ctrl|= (0 << USB_XFER_TOKEN_IN_SHIFT);    // Host -> Device
    ctrl|= (1 << USB_XFER_TOKEN_ACK_SHIFT);   // Expect ACK

    // Always DATA0
    ctrl|=  (0 << USB_XFER_TOKEN_PID_DATAX_SHIFT);

    // Setup token details (don't start transfer yet)
    token = (((unsigned int)USB_PID_SETUP)<<USB_XFER_TOKEN_PID_BITS_SHIFT) | (device_addr << USB_XFER_TOKEN_DEV_ADDR_SHIFT) | (endpoint << USB_XFER_TOKEN_EP_ADDR_SHIFT);
    writel(token | ctrl, ue11->reg_base + USB_XFER_TOKEN);

    ep->length = 0;
}
//-----------------------------------------------------------------
// status_packet: STATUS finishes control requests, often after 
// IN or OUT data packets
//-----------------------------------------------------------------
static void status_packet(struct ue11 *ue11, struct ue11h_ep *ep, struct urb *urb)
{
    int do_out = urb->transfer_buffer_length && usb_pipein(urb->pipe);

    if (do_out)
    {
        uint32_t ctrl = 0;
        uint32_t token = 0; 
        uint32_t device_addr = usb_pipedevice(urb->pipe);
        uint32_t endpoint    = usb_pipeendpoint(urb->pipe);

        USB_LOG(USBLOG_DATA, ("USB: Send STATUS (OUT)\n"));

        USB_LOG(USBLOG_DATA, ("TOKEN: OUT (STATUS)"));
        USB_LOG(USBLOG_DATA, ("  DEV %d EP %d\n", device_addr, endpoint));

        // Transfer data length (zero length packet - just PID)
        writel(0, ue11->reg_base + USB_XFER_DATA);

        // Configure transfer for DATAx portion
        ctrl = (1 << USB_XFER_TOKEN_START_SHIFT);
        ctrl|= (0 << USB_XFER_TOKEN_IN_SHIFT);    // Host -> Device
        ctrl|= (1 << USB_XFER_TOKEN_ACK_SHIFT);   // Expect ACK

        // Always DATA1
        ctrl|=  (1 << USB_XFER_TOKEN_PID_DATAX_SHIFT);

        // Setup token details (don't start transfer yet)
        token = (((uint32_t)USB_PID_OUT)<<USB_XFER_TOKEN_PID_BITS_SHIFT) | (device_addr << USB_XFER_TOKEN_DEV_ADDR_SHIFT) | (endpoint << USB_XFER_TOKEN_EP_ADDR_SHIFT);
        writel(token | ctrl, ue11->reg_base + USB_XFER_TOKEN);

        ep->length = 0;
    }
    else
    {
        uint32_t ctrl = 0;
        uint32_t token = 0; 
        uint32_t device_addr = usb_pipedevice(urb->pipe);
        uint32_t endpoint    = usb_pipeendpoint(urb->pipe);

        USB_LOG(USBLOG_DATA, ("USB: Send STATUS (IN)\n"));   
            
        USB_LOG(USBLOG_DATA, ("TOKEN: IN (STATUS)"));
        USB_LOG(USBLOG_DATA, ("  DEV %d EP %d\n", device_addr, endpoint));    

        // No data to send
        writel(0, ue11->reg_base + USB_XFER_DATA);

        // Configure transfer for DATAx portion
        ctrl = (1 << USB_XFER_TOKEN_START_SHIFT);
        ctrl|= (1 << USB_XFER_TOKEN_IN_SHIFT);    // Device -> Host
        ctrl|= (1 << USB_XFER_TOKEN_ACK_SHIFT);   // Respond with ACK

        // Always DATA1
        ctrl|=  (1 << USB_XFER_TOKEN_PID_DATAX_SHIFT);        

        token = (((uint32_t)USB_PID_IN)<<USB_XFER_TOKEN_PID_BITS_SHIFT) | (device_addr << USB_XFER_TOKEN_DEV_ADDR_SHIFT) | (endpoint << USB_XFER_TOKEN_EP_ADDR_SHIFT);
        writel(token | ctrl, ue11->reg_base + USB_XFER_TOKEN);

        ep->length = 0;
    }
}
//-----------------------------------------------------------------
// in_packet: IN packets can be used with any type of endpoint.
//-----------------------------------------------------------------
static void in_packet(struct ue11 *ue11, struct ue11h_ep *ep, struct urb *urb)
{
    uint32_t ctrl = 0;
    uint32_t token = 0; 
    uint32_t device_addr = usb_pipedevice(urb->pipe);
    uint32_t endpoint    = usb_pipeendpoint(urb->pipe);

    USB_LOG(USBLOG_REQ, ("USB: IN Request EP %x (%d/%d)\n", endpoint, urb->actual_length, urb->transfer_buffer_length));
        
    USB_LOG(USBLOG_DATA, ("TOKEN: IN"));
    USB_LOG(USBLOG_DATA, ("  DEV %d EP %d\n", device_addr, endpoint));    

    // No data to send
    writel(0, ue11->reg_base + USB_XFER_DATA);

    // Configure transfer for DATAx portion
    ctrl = (1 << USB_XFER_TOKEN_START_SHIFT);
    ctrl|= (1 << USB_XFER_TOKEN_IN_SHIFT);    // Device -> Host
    ctrl|= (1 << USB_XFER_TOKEN_ACK_SHIFT);   // Respond with ACK

    // DataX?
    //ctrl|=  usb_gettoggle(urb->dev, usb_pipeendpoint(urb->pipe), usb_pipeout(urb->pipe)) ? (1 << USB_XFER_TOKEN_PID_DATAX_SHIFT) : (0 << USB_XFER_TOKEN_PID_DATAX_SHIFT);

    token = (((uint32_t)USB_PID_IN)<<USB_XFER_TOKEN_PID_BITS_SHIFT) | (device_addr << USB_XFER_TOKEN_DEV_ADDR_SHIFT) | (endpoint << USB_XFER_TOKEN_EP_ADDR_SHIFT);
    writel(token | ctrl, ue11->reg_base + USB_XFER_TOKEN);

    // TODO: This isn't known yet!
    ep->length = min_t(u32, ep->maxpacket,
            urb->transfer_buffer_length - urb->actual_length);
}
//-----------------------------------------------------------------
// out_packet: OUT packets can be used with any type of endpoint.
//-----------------------------------------------------------------
static void out_packet(struct ue11 *ue11, struct ue11h_ep *ep, struct urb *urb)
{
    int l;
    uint32_t ctrl = 0;
    uint32_t token = 0; 
    uint32_t request = 0; 
    uint32_t device_addr = usb_pipedevice(urb->pipe);
    uint32_t endpoint    = usb_pipeendpoint(urb->pipe);
    uint8_t    *buf;
    int     len;

    USB_LOG(USBLOG_DATA, ("USB: Send OUT_PACKET\n"));

    buf = (uint8_t*)urb->transfer_buffer + urb->actual_length;
    prefetch(buf);

    // Limit transmit length to max packet size
    len = min_t(u32, ep->maxpacket, urb->transfer_buffer_length - urb->actual_length);

    USB_LOG(USBLOG_DATA, ("TOKEN: OUT"));
    USB_LOG(USBLOG_DATA, ("  DEV %d EP %d\n", device_addr, endpoint));

    request = usb_gettoggle(urb->dev, usb_pipeendpoint(urb->pipe), usb_pipeout(urb->pipe)) ? 
                        USB_PID_DATA1 : USB_PID_DATA0;

    USB_LOG(USBLOG_REQ, ("USB: OUT EP %x, LEN %d, PID=%x\n", endpoint, len, request));

    // Load DATAx transfer into address 0+
    USB_LOG(USBLOG_DATA, (" Tx:\n %02x", request));
    for (l=0;l<len;l++)
    {
        USB_LOG(USBLOG_DATA, (" %02x", buf[l]));
        writel(buf[l], ue11->reg_base + USB_WR_DATA);
    }
    USB_LOG(USBLOG_DATA, ("\n"));

    // Transfer data length
    writel(len, ue11->reg_base + USB_XFER_DATA);

    // Configure transfer for DATAx portion
    ctrl = (1 << USB_XFER_TOKEN_START_SHIFT);
    ctrl|= (0 << USB_XFER_TOKEN_IN_SHIFT);    // Host -> Device
    ctrl|= (1 << USB_XFER_TOKEN_ACK_SHIFT);   // Expect ACK

    // Select DATAx
    ctrl|=  ((request == USB_PID_DATA0) ? (0 << USB_XFER_TOKEN_PID_DATAX_SHIFT) : (1 << USB_XFER_TOKEN_PID_DATAX_SHIFT));

    // Setup token details (don't start transfer yet)
    token = (((unsigned int)USB_PID_OUT)<<USB_XFER_TOKEN_PID_BITS_SHIFT) | (device_addr << USB_XFER_TOKEN_DEV_ADDR_SHIFT) | (endpoint << USB_XFER_TOKEN_EP_ADDR_SHIFT);
    writel(token | ctrl, ue11->reg_base + USB_XFER_TOKEN);

    ep->length = len;
}
//-----------------------------------------------------------------
// enable_sof_interrupt: 
//-----------------------------------------------------------------
static inline void enable_sof_interrupt(struct ue11 *ue11)
{
    if (ue11->irq_enable & (1 << USB_IRQ_MASK_SOF_SHIFT))
        return;
    USB_LOG(USBLOG_INFO, ("USB: Enable SOF\n"));
    ue11->irq_enable |= (1 << USB_IRQ_MASK_SOF_SHIFT);
}
//-----------------------------------------------------------------
// disable_sof_interrupt: 
//-----------------------------------------------------------------
static inline void disable_sof_interrupt(struct ue11 *ue11)
{
    if (!(ue11->irq_enable & (1 << USB_IRQ_MASK_SOF_SHIFT)))
        return;
    USB_LOG(USBLOG_INFO, ("USB: Disable SOF\n"));
    ue11->irq_enable &= ~(1 << USB_IRQ_MASK_SOF_SHIFT);
}
//-----------------------------------------------------------------
// start_transfer: Pick the next endpoint for a transaction, and issue it.
// frames start with periodic transfers (after whatever is pending
// from the previous frame), and the rest of the time is async
// transfers, scheduled round-robin.
//-----------------------------------------------------------------
static void start_transfer(struct ue11 *ue11)
{
    struct ue11h_ep    *ep;
    struct urb      *urb;

    // Make sure hub port is active
    if (ue11->port1 & USB_PORT_STAT_SUSPEND)
        return ;

    // Only do something if no transfer in-progress
    if (ue11->active_transfer != NULL)
        return ;

    /* use endpoint at schedule head */
    if (ue11->next_periodic)
    {
        ep = ue11->next_periodic;
        ue11->next_periodic = ep->next;
    }
    else
    {
        if (ue11->next_async)
            ep = ue11->next_async;
        else if (!list_empty(&ue11->async))
            ep = container_of(ue11->async.next,
                    struct ue11h_ep, schedule);
        else
        {
            /* could set up the first fullspeed periodic
             * transfer for the next frame ...
             */
            return ;
        }

        if (ep->schedule.next == &ue11->async)
            ue11->next_async = NULL;
        else
            ue11->next_async = container_of(ep->schedule.next,
                    struct ue11h_ep, schedule);
    }

    if (unlikely(list_empty(&ep->hep->urb_list)))
    {
        dev_dbg(ue11_to_hcd(ue11)->self.controller,
            "empty %p queue?\n", ep);
        return ;
    }

    urb = container_of(ep->hep->urb_list.next, struct urb, urb_list);

    switch (ep->nextpid)
    {
    case USB_PID_IN:
        in_packet(ue11, ep, urb);
        break;
    case USB_PID_OUT:
        out_packet(ue11, ep, urb);
        break;
    case USB_PID_SETUP:
        setup_packet(ue11, ep, urb);
        break;
    case USB_PID_ACK:       /* for control status */
        status_packet(ue11, ep, urb);
        break;
    default:
        dev_dbg(ue11_to_hcd(ue11)->self.controller,
            "bad ep%p pid %02x\n", ep, ep->nextpid);
        ep = NULL;
    }

    #define MIN_JIFFIES ((msecs_to_jiffies(2) > 1) ? msecs_to_jiffies(2) : 2)

    // Record new active transfer details
    ue11->active_transfer  = ep;
    ue11->active_start     = (jiffies + MIN_JIFFIES);
}
//-----------------------------------------------------------------
// finish_request
//-----------------------------------------------------------------
static void finish_request(
    struct ue11        *ue11,
    struct ue11h_ep    *ep,
    struct urb      *urb,
    int         status
) __releases(ue11->lock) __acquires(ue11->lock)
{
    unsigned        i;

    USB_LOG(USBLOG_INFO, ("USB: URB finish %p\n", urb));

    if (usb_pipecontrol(urb->pipe))
        ep->nextpid = USB_PID_SETUP;

    usb_hcd_unlink_urb_from_ep(ue11_to_hcd(ue11), urb);
    spin_unlock(&ue11->lock);
    usb_hcd_giveback_urb(ue11_to_hcd(ue11), urb, status);
    spin_lock(&ue11->lock);

    /* leave active endpoints in the schedule */
    if (!list_empty(&ep->hep->urb_list))
        return;

    /* async deschedule? */
    if (!list_empty(&ep->schedule))
    {
        list_del_init(&ep->schedule);
        if (ep == ue11->next_async)
            ue11->next_async = NULL;
        return;
    }

    /* periodic deschedule */
    dev_dbg(ue11_to_hcd(ue11)->self.controller,
        "deschedule qh%d/%p branch %d\n", ep->period, ep, ep->branch);
    for (i = ep->branch; i < PERIODIC_SIZE; i += ep->period)
    {
        struct ue11h_ep    *temp;
        struct ue11h_ep    **prev = &ue11->periodic[i];

        while (*prev && ((temp = *prev) != ep))
            prev = &temp->next;
        if (*prev)
            *prev = ep->next;
        ue11->load[i] -= ep->load;
    }
    ep->branch = PERIODIC_SIZE;
    ue11->periodic_count--;
    ue11_to_hcd(ue11)->self.bandwidth_allocated
        -= ep->load / ep->period;
    if (ep == ue11->next_periodic)
        ue11->next_periodic = ep->next;

    /* we might turn SOFs back on again for the async schedule */
    if (ue11->periodic_count == 0)
        disable_sof_interrupt(ue11);
}
//-----------------------------------------------------------------
// process_transfer_result: Called on transfer complete / error
//-----------------------------------------------------------------
static void process_transfer_result(struct ue11 *ue11, struct ue11h_ep *ep)
{
    uint32_t    status;
    struct urb  *urb;
    int         urbstat = -EINPROGRESS;
    uint8_t     response = 0;
    int l;

    if (unlikely(!ep))
        return;

    status = readl(ue11->reg_base + USB_RX_STAT);
    response = ((status >> USB_RX_STAT_RESP_BITS_SHIFT) & USB_RX_STAT_RESP_BITS_MASK);

    USB_LOG(USBLOG_DATA, ("  STAT: %08x\n", status));
    USB_LOG(USBLOG_DATA, ("  RESP: %08x\n", response));

    // Request still pending
    BUG_ON(status & (1 << USB_RX_STAT_START_PEND_SHIFT));

    // CRC error
    if (status & (1 << USB_RX_STAT_CRC_ERR_SHIFT))
    {
        // Response PID field will be zero!
        USB_LOG(USBLOG_ERR, ("USB: CRC error detected (last pid=%x)\n", ep->nextpid));
        BUG_ON(1);
    }

    // Timeout error
    if (status & (1 << USB_RX_STAT_RESP_TIMEOUT_SHIFT))
    {
        // Response PID field will be zero!
        USB_LOG(USBLOG_ERR, ("USB: Timeout error detected (last pid=%x)\n", ep->nextpid));
        BUG_ON(1);
    }     

    urb = container_of(ep->hep->urb_list.next, struct urb, urb_list);

    // IN request sent and response received
    if (((ep->nextpid == USB_PID_IN) || (ep->nextpid == USB_PID_ACK)) && 
        (response == USB_PID_DATA0 || response == USB_PID_DATA1))
    {
        // TODO: Check DATAx is correct

        // Convert to ACK if all is well...
        response = USB_PID_ACK;
    }


    /* we can safely ignore NAKs */
    if (response == USB_PID_NAK)
    {
        USB_LOG(USBLOG_DATA, ("USB: NAK %d\n", ep->nak_count));
        if (!ep->period)
            ep->nak_count++;
        ep->error_count = 0;
    }
    /* ACK advances transfer, toggle, and maybe queue */
    else if (response == USB_PID_ACK)
    {
        struct usb_device   *udev = urb->dev;
        int         len;
        unsigned char       *buf;

        /* urb->iso_frame_desc is currently ignored here... */

        ep->nak_count = ep->error_count = 0;
        switch (ep->nextpid)
        {
        case USB_PID_OUT:
            USB_LOG(USBLOG_DATA, ("USB: PID_OUT ACK\n"));
            urb->actual_length += ep->length;
            usb_dotoggle(udev, ep->epnum, 1);
            if (urb->actual_length
                    == urb->transfer_buffer_length) {
                if (usb_pipecontrol(urb->pipe))
                    ep->nextpid = USB_PID_ACK;

                /* some bulk protocols terminate OUT transfers
                 * by a short packet, using ZLPs not padding.
                 */
                else if (ep->length < ep->maxpacket
                        || !(urb->transfer_flags
                            & URB_ZERO_PACKET))
                {
                    USB_LOG(USBLOG_REQ, ("USB: OUT EP %x Complete\n", ep->epnum));
                    urbstat = 0;
                }
            }
            break;
        case USB_PID_IN:
            USB_LOG(USBLOG_DATA, ("USB: PID_IN ACK\n"));
            buf = urb->transfer_buffer + urb->actual_length;
            prefetchw(buf);

            len = ((status >> USB_RX_STAT_COUNT_BITS_SHIFT) & USB_RX_STAT_COUNT_BITS_MASK);
            USB_LOG(USBLOG_DATA, ("USB: Received length %d, requested %d\n", urb->actual_length + len, ep->length));

            if ((urb->actual_length + len) > urb->transfer_buffer_length)
            {
                urbstat = -EOVERFLOW;
                BUG_ON(1);
            }
            else
                urb->actual_length += len;

            for (l=0;l<len;l++)
            {
                buf[l] = readl(ue11->reg_base + USB_RD_DATA);
                USB_LOG(USBLOG_DATA, (" %02x", buf[l]));
            }
            USB_LOG(USBLOG_DATA, ("\n"));

            usb_dotoggle(udev, ep->epnum, 0);
            if (urbstat == -EINPROGRESS &&
                    (len < ep->maxpacket ||
                        urb->actual_length ==
                        urb->transfer_buffer_length)) {
                if (usb_pipecontrol(urb->pipe))
                    ep->nextpid = USB_PID_ACK;
                else
                {
                    USB_LOG(USBLOG_REQ, ("USB: IN EP %x Complete\n", ep->epnum));
                    urbstat = 0;
                }
            }
            break;
        case USB_PID_SETUP:
            USB_LOG(USBLOG_DATA, ("USB: PID_SETUP ACK\n"));
            if (urb->transfer_buffer_length == urb->actual_length)
                ep->nextpid = USB_PID_ACK;
            else if (usb_pipeout(urb->pipe)) {
                usb_settoggle(udev, 0, 1, 1);
                ep->nextpid = USB_PID_OUT;
            } else {
                usb_settoggle(udev, 0, 0, 1);
                ep->nextpid = USB_PID_IN;
            }
            break;
        case USB_PID_ACK:
            USB_LOG(USBLOG_REQ, ("USB: SETUP PACKET Complete\n"));
            urbstat = 0;
            break;
        }
    }
    /* STALL stops all transfers */
    else if (response == USB_PID_STALL)
    {
        USB_LOG(USBLOG_ERR, ("USB: STALL (sts=%x)!\n", status));
        ep->nak_count = ep->error_count = 0;
        urbstat = -EPIPE;
    }
    /* error? retry, until "3 strikes" */
    else if (++ep->error_count >= 3)
    {
        USB_LOG(USBLOG_ERR, ("USB: Timeout %d (sts=%x)!\n", ep->error_count, status));
        if (status & (1 << USB_RX_STAT_RESP_TIMEOUT_SHIFT))
            urbstat = -ETIME;
        //else if (status & SL11H_STATMASK_OVF)
        //  urbstat = -EOVERFLOW;
        else
            urbstat = -EPROTO;
        ep->error_count = 0;
    }
    else
    {
        USB_LOG(USBLOG_ERR, ("USB: Timeout %d (sts=%x)!\n", ep->error_count, status));    
    }

    if (urbstat != -EINPROGRESS || urb->unlinked)
        finish_request(ue11, ep, urb, urbstat);
}
//-----------------------------------------------------------------
// ue11h_irq: IRQ handler
//-----------------------------------------------------------------
static irqreturn_t ue11h_irq(struct usb_hcd *hcd)
{
    struct ue11    *ue11 = hcd_to_ue11(hcd);
    uint32_t        irqstat;
    irqreturn_t ret = IRQ_NONE;
    unsigned    retries = 5;

    spin_lock(&ue11->lock);

retry:
    irqstat = readl(ue11->reg_base + USB_IRQ_STS);
    // Ack interrupt
    if (irqstat)
    {
        writel(irqstat, ue11->reg_base + USB_IRQ_ACK);
        irqstat &= ue11->irq_enable;
    }

    // IRQ: Packet transfer complete or error detected
    if (irqstat & ((1 << USB_IRQ_STS_DONE_SHIFT) | (1 << USB_IRQ_STS_ERR_SHIFT)))
    {
        process_transfer_result(ue11, ue11->active_transfer);
        ue11->active_transfer = NULL;
        ue11->stat_a++;
    }

    // IRQ: Start of frame interrupt
    if (irqstat & (1 << USB_IRQ_STS_SOF_SHIFT))
    {
        unsigned index;

        index = ue11->frame++ % (PERIODIC_SIZE - 1);
        ue11->stat_sof++;

        /* be graceful about almost-inevitable periodic schedule
         * overruns:  continue the previous frame's transfers iff
         * this one has nothing scheduled.
         */
        if (ue11->next_periodic)
            ue11->stat_overrun++;

        if (ue11->periodic[index])
            ue11->next_periodic = ue11->periodic[index];
    }

    if (irqstat)
    {
        if (ue11->port1 & USB_PORT_STAT_ENABLE)
            start_transfer(ue11);
        ret = IRQ_HANDLED;
        if (retries--)
            goto retry;
    }

    if (ue11->periodic_count == 0 && list_empty(&ue11->async))
        disable_sof_interrupt(ue11);
    writel(ue11->irq_enable, ue11->reg_base + USB_IRQ_MASK);

    spin_unlock(&ue11->lock);

    return ret;
}
//-----------------------------------------------------------------
// balance_load:
// usb 1.1 says max 90% of a frame is available for periodic transfers.
// this driver doesn't promise that much since it's got to handle an
// IRQ per packet; irq handling latencies also use up that time.
//
// NOTE:  the periodic schedule is a sparse tree, with the load for
// each branch minimized.  see fig 3.5 in the OHCI spec for example.
//-----------------------------------------------------------------
static int balance(struct ue11 *ue11, u16 period, u16 load)
{
    #define MAX_PERIODIC_LOAD   500 /* out of 1000 usec */
    int i, branch = -ENOSPC;

    /* search for the least loaded schedule branch of that period
     * which has enough bandwidth left unreserved.
     */
    for (i = 0; i < period ; i++) {
        if (branch < 0 || ue11->load[branch] > ue11->load[i]) {
            int j;

            for (j = i; j < PERIODIC_SIZE; j += period) {
                if ((ue11->load[j] + load)
                        > MAX_PERIODIC_LOAD)
                    break;
            }
            if (j < PERIODIC_SIZE)
                continue;
            branch = i;
        }
    }
    return branch;
}
//-----------------------------------------------------------------
// ue11h_urb_enqueue
//-----------------------------------------------------------------
static int ue11h_urb_enqueue(struct usb_hcd *hcd, struct urb *urb, gfp_t mem_flags)
{
    struct ue11        *ue11   = hcd_to_ue11(hcd);
    struct usb_device   *udev  = urb->dev;
    unsigned int        pipe   = urb->pipe;
    int                 is_out = !usb_pipein(pipe);
    int                 type   = usb_pipetype(pipe);
    int                 epnum  = usb_pipeendpoint(pipe);
    struct ue11h_ep    *ep     = NULL;
    unsigned long       flags;
    int                 i;
    int                 retval;
    struct usb_host_endpoint    *hep = urb->ep;

    USB_LOG(USBLOG_INFO, ("USB: URB queue %p\n", urb));

    // NOTE: ISO transfer not supported
    if (type == PIPE_ISOCHRONOUS)
    {
        USB_LOG(USBLOG_ERR, ("USB: Isochronous transfers not supported\n"));
        return -ENOSPC;
    }

    // NOTE: Low speed devices are not supported!
    if (udev->speed == USB_SPEED_LOW)
    {
        USB_LOG(USBLOG_ERR, ("USB: Low speed devices not supported\n"));
        return -ENOSPC;
    }    

    /* avoid all allocations within spinlocks */
    if (!hep->hcpriv) {
        ep = kzalloc(sizeof *ep, mem_flags);
        if (ep == NULL)
            return -ENOMEM;
    }

    spin_lock_irqsave(&ue11->lock, flags);

    /* don't submit to a dead or disabled port */
    if (!(ue11->port1 & USB_PORT_STAT_ENABLE)
            || !HC_IS_RUNNING(hcd->state))
    {
        retval = -ENODEV;
        kfree(ep);
        goto fail_not_linked;
    }
    // Link URB to host controller
    retval = usb_hcd_link_urb_to_ep(hcd, urb);
    if (retval)
    {
        kfree(ep);
        goto fail_not_linked;
    }

    if (hep->hcpriv)
    {
        kfree(ep);
        ep = hep->hcpriv;
    }
    else if (!ep) 
    {
        retval = -ENOMEM;
        goto fail;
    }
    else
    {
        INIT_LIST_HEAD(&ep->schedule);
        ep->udev = udev;
        ep->epnum = epnum;
        ep->maxpacket = usb_maxpacket(udev, urb->pipe, is_out);
        usb_settoggle(udev, epnum, is_out, 0);

        if (type == PIPE_CONTROL)
            ep->nextpid = USB_PID_SETUP;
        else if (is_out)
            ep->nextpid = USB_PID_OUT;
        else
            ep->nextpid = USB_PID_IN;

        if (udev->speed == USB_SPEED_LOW)
        {
            // TODO: Low speed mode...
        }
        switch (type)
        {
        case PIPE_ISOCHRONOUS:
        case PIPE_INTERRUPT:
            if (urb->interval > PERIODIC_SIZE)
                urb->interval = PERIODIC_SIZE;
            ep->period = urb->interval;
            ep->branch = PERIODIC_SIZE;
            if (type == PIPE_ISOCHRONOUS)
                ;// TODO: ISO
            ep->load = usb_calc_bus_time(udev->speed, !is_out,
                (type == PIPE_ISOCHRONOUS),
                usb_maxpacket(udev, pipe, is_out))
                    / 1000;
            break;
        }

        ep->hep = hep;
        hep->hcpriv = ep;
    }

    /* maybe put endpoint into schedule */
    switch (type)
    {
    case PIPE_CONTROL:
    case PIPE_BULK:
        if (list_empty(&ep->schedule))
            list_add_tail(&ep->schedule, &ue11->async);
        break;
    case PIPE_ISOCHRONOUS:
    case PIPE_INTERRUPT:
        urb->interval = ep->period;
        if (ep->branch < PERIODIC_SIZE)
        {
            /* NOTE:  the phase is correct here, but the value
             * needs offsetting by the transfer queue depth.
             * All current drivers ignore start_frame, so this
             * is unlikely to ever matter...
             */
            urb->start_frame = (ue11->frame & (PERIODIC_SIZE - 1))
                        + ep->branch;
            break;
        }

        retval = balance(ue11, ep->period, ep->load);
        if (retval < 0)
            goto fail;
        ep->branch = retval;
        retval = 0;
        urb->start_frame = (ue11->frame & (PERIODIC_SIZE - 1))
                    + ep->branch;

        /* sort each schedule branch by period (slow before fast)
         * to share the faster parts of the tree without needing
         * dummy/placeholder nodes
         */
        dev_dbg(hcd->self.controller, "schedule qh%d/%p branch %d\n",
            ep->period, ep, ep->branch);
        for (i = ep->branch; i < PERIODIC_SIZE; i += ep->period)
        {
            struct ue11h_ep    **prev = &ue11->periodic[i];
            struct ue11h_ep    *here = *prev;

            while (here && ep != here) {
                if (ep->period > here->period)
                    break;
                prev = &here->next;
                here = *prev;
            }
            if (ep != here) {
                ep->next = here;
                *prev = ep;
            }
            ue11->load[i] += ep->load;
        }
        ue11->periodic_count++;
        hcd->self.bandwidth_allocated += ep->load / ep->period;
        enable_sof_interrupt(ue11);
    }

    urb->hcpriv = hep;
    // Start transfer if one is not already in-progress
    start_transfer(ue11);
    // Enable interrupts
    writel(ue11->irq_enable, ue11->reg_base + USB_IRQ_MASK);
fail:
    if (retval)
        usb_hcd_unlink_urb_from_ep(hcd, urb);
fail_not_linked:
    spin_unlock_irqrestore(&ue11->lock, flags);
    return retval;
}
//-----------------------------------------------------------------
// ue11h_urb_dequeue
//-----------------------------------------------------------------
static int ue11h_urb_dequeue(struct usb_hcd *hcd, struct urb *urb, int status)
{
    struct ue11        *ue11 = hcd_to_ue11(hcd);
    struct usb_host_endpoint *hep;
    unsigned long       flags;
    struct ue11h_ep    *ep;
    int         retval;

    spin_lock_irqsave(&ue11->lock, flags);
    retval = usb_hcd_check_unlink_urb(hcd, urb, status);
    if (retval)
        goto fail;

    hep = urb->hcpriv;
    ep = hep->hcpriv;
    if (ep) {
        /* finish right away if this urb can't be active ...
         * note that some drivers wrongly expect delays
         */
        if (ep->hep->urb_list.next != &urb->urb_list) {
            /* not front of queue?  never active */
        }
        /* for active transfers, we expect an IRQ */
        else if (ue11->active_transfer == ep)
        {
            if (time_before_eq(ue11->active_start, jiffies))
            {
                /* happens a lot with lowspeed?? */
                USB_LOG(USBLOG_ERR, ("USB: Giving up on transfer....\n"));
                ue11->active_transfer = NULL;
            }
            else
                urb = NULL;
        }
        else
        {
            /* front of queue for inactive endpoint */
        }

        if (urb)
            finish_request(ue11, ep, urb, 0);
        else
            dev_dbg(ue11_to_hcd(ue11)->self.controller,
                "dequeue, urb %p active %s; wait4irq\n", urb,
                (ue11->active_transfer == ep) ? "A" : "B");
    } else
        retval = -EINVAL;
 fail:
    spin_unlock_irqrestore(&ue11->lock, flags);
    return retval;
}
//-----------------------------------------------------------------
// ue11h_endpoint_disable
//-----------------------------------------------------------------
static void ue11h_endpoint_disable(struct usb_hcd *hcd, struct usb_host_endpoint *hep)
{
    struct ue11h_ep    *ep = hep->hcpriv;

    if (!ep)
        return;

    /* assume we'd just wait for the irq */
    if (!list_empty(&hep->urb_list))
        msleep(3);
    if (!list_empty(&hep->urb_list))
        dev_warn(hcd->self.controller, "ep %p not empty?\n", ep);

    kfree(ep);
    hep->hcpriv = NULL;
}
//-----------------------------------------------------------------
// ue11h_get_frame
//-----------------------------------------------------------------
static int ue11h_get_frame(struct usb_hcd *hcd)
{
    struct ue11 *ue11 = hcd_to_ue11(hcd);

    /* wrong except while periodic transfers are scheduled;
     * never matches the on-the-wire frame;
     * subject to overruns.
     */
    return ue11->frame;
}
//-----------------------------------------------------------------
// ue11h_hub_status_data: Virtual root hub port status check
//-----------------------------------------------------------------
static int ue11h_hub_status_data(struct usb_hcd *hcd, char *buf)
{
    struct ue11 *ue11 = hcd_to_ue11(hcd);

    // No status changes
    if (!(ue11->port1 & (0xffff << 16)))        
        return 0;

    /* tell hub_wq port 1 changed */
    pr_info("ue11h_hub_status_data: Port 1 changed state\n");
    *buf = (1 << 1);
    return 1;
}
//-----------------------------------------------------------------
// ue11h_hub_descriptor
//-----------------------------------------------------------------
static void
ue11h_hub_descriptor (
    struct ue11            *ue11,
    struct usb_hub_descriptor   *desc
) {
    u16     temp = 0;

    desc->bDescriptorType = USB_DT_HUB;
    desc->bHubContrCurrent = 0;

    desc->bNbrPorts = 1;
    desc->bDescLength = 9;

    /* per-port power switching (gang of one!), or none */
    desc->bPwrOn2PwrGood = 0;

    /* no per port power switching or overcurrent errors detection/handling */
    temp = HUB_CHAR_NO_LPSM | HUB_CHAR_NO_OCPM;

    desc->wHubCharacteristics = cpu_to_le16(temp);

    /* ports removable, and legacy PortPwrCtrlMask */
    desc->u.hs.DeviceRemovable[0] = 0 << 1;
    desc->u.hs.DeviceRemovable[1] = ~0;
}
//-----------------------------------------------------------------
// ue11h_timer: Device detect timer callback
//-----------------------------------------------------------------
static void ue11h_timer(struct timer_list *t)
{
    unsigned long    flags;
    struct ue11    *ue11 = from_timer(ue11, t, timer);

    spin_lock_irqsave(&ue11->lock, flags);

    // De-assert USB reset
    usbhw_hub_enable(ue11, 1, 1);

    // Small delay to allow data lines to settle
    udelay(3);

    // Force device detection
    ue11->port1 |= (USB_PORT_STAT_POWER | USB_PORT_STAT_ENABLE);
    ue11->port1 &= ~USB_PORT_STAT_RESET;

    // Enable USB interrupts
    ue11->irq_enable |= ((1 << USB_IRQ_MASK_ERR_SHIFT) | (1 << USB_IRQ_MASK_DONE_SHIFT));

    /* reenable irqs */
    writel(ue11->irq_enable, ue11->reg_base + USB_IRQ_MASK);
    spin_unlock_irqrestore(&ue11->lock, flags);
}
//-----------------------------------------------------------------
// ue11h_hub_control
//-----------------------------------------------------------------
static int
ue11h_hub_control(
    struct usb_hcd  *hcd,
    u16     typeReq,
    u16     wValue,
    u16     wIndex,
    char        *buf,
    u16     wLength
) {
    USB_LOG(USBLOG_INFO,("USB: ue11h_hub_control typeReq %x wValue %x wIndex %x\n", typeReq, wValue, wIndex));
    struct ue11    *ue11 = hcd_to_ue11(hcd);
    int     retval = 0;
    unsigned long   flags;

    spin_lock_irqsave(&ue11->lock, flags);

    switch (typeReq) {
    case ClearHubFeature:
    case SetHubFeature:
        USB_LOG(USBLOG_INFO,("USB: Set/Clear hub feature 0x%x\n", wValue));
        switch (wValue) {
        case C_HUB_OVER_CURRENT:
        case C_HUB_LOCAL_POWER:
            break;
        default:
            goto error;
        }
        break;
    case ClearPortFeature:
        USB_LOG(USBLOG_INFO,("USB: Clear Port feature 0x%x\n", wValue));
        if (wIndex != 1 || wLength != 0)
            goto error;

        switch (wValue) {
        case USB_PORT_FEAT_ENABLE:
            pr_info("ue11h_hub_control: USB_PORT_FEAT_ENABLE (DISABLE)\n");
            ue11->port1 &= USB_PORT_STAT_POWER;
            ue11->irq_enable = 0;
            writel(ue11->irq_enable, ue11->reg_base + USB_IRQ_MASK);
            break;
        case USB_PORT_FEAT_SUSPEND:
            if (!(ue11->port1 & USB_PORT_STAT_SUSPEND))
                break;
            pr_info("ue11h_hub_control: USB_PORT_FEAT_SUSPEND (RESUME)\n");
            break;
        case USB_PORT_FEAT_POWER:
            USB_LOG(USBLOG_INFO,("USB: Clear - USB_PORT_FEAT_POWER\n"));
            break;
        case USB_PORT_FEAT_C_ENABLE:
        case USB_PORT_FEAT_C_SUSPEND:
        case USB_PORT_FEAT_C_CONNECTION:
        case USB_PORT_FEAT_C_OVER_CURRENT:
        case USB_PORT_FEAT_C_RESET:
            USB_LOG(USBLOG_INFO,("USB: Clear - Other\n"));
            break;
        default:
            goto error;
        }
        ue11->port1 &= ~(1 << wValue);
        USB_LOG(USBLOG_INFO,(" - (Port=%x)\n", ue11->port1));
        break;
    case GetHubDescriptor:
        USB_LOG(USBLOG_INFO,("USB: Get hub descriptor\n"));
        ue11h_hub_descriptor(ue11, (struct usb_hub_descriptor *) buf);
        break;
    case GetHubStatus:
        USB_LOG(USBLOG_INFO,("USB: Get hub status 0x%x\n", wValue));
        put_unaligned_le32(0, buf);
        break;
    case GetPortStatus:
        USB_LOG(USBLOG_INFO,("USB: Get port status 0x%x (Port=%x)\n", wValue, ue11->port1));
        if (wIndex != 1)
            goto error;
        put_unaligned_le32(ue11->port1, buf);
        break;
    case SetPortFeature:
        if (wIndex != 1 || wLength != 0)
            goto error;
        switch (wValue) {
        case USB_PORT_FEAT_SUSPEND:
            if (ue11->port1 & USB_PORT_STAT_RESET)
                goto error;
            if (!(ue11->port1 & USB_PORT_STAT_ENABLE))
                goto error;
            pr_info("ue11h_hub_control: USB_PORT_FEAT_SUSPEND (SUSPEND)\n");
            break;
        case USB_PORT_FEAT_POWER:
            pr_info("ue11h_hub_control: USB_PORT_FEAT_POWER (power on)\n");
            ue11->port1 |= USB_PORT_STAT_CONNECTION;
            ue11->port1 |= USB_PORT_STAT_C_CONNECTION << 16;
            break;
        case USB_PORT_FEAT_RESET:
            if (ue11->port1 & USB_PORT_STAT_SUSPEND)
                goto error;
            if (!(ue11->port1 & USB_PORT_STAT_POWER))
                break;

            pr_info("ue11h_hub_control: USB_PORT_FEAT_RESET\n");

            /* 50 msec of reset/SE0 signaling, irqs blocked */
            ue11->irq_enable = 0;
            writel(ue11->irq_enable, ue11->reg_base + USB_IRQ_MASK);
            usbhw_hub_reset(ue11);
            ue11->port1 |= USB_PORT_STAT_RESET;
            mod_timer(&ue11->timer, jiffies
                    + msecs_to_jiffies(50));
            break;
        default:
            goto error;
        }
        ue11->port1 |= 1 << wValue;
        USB_LOG(USBLOG_INFO,(" - (Port=%x)\n", ue11->port1));
        break;

    default:
error:
        /* "protocol stall" on error */
        retval = -EPIPE;
    }

    spin_unlock_irqrestore(&ue11->lock, flags);
    return retval;
}

#ifdef  CONFIG_PM
//-----------------------------------------------------------------
// ue11h_bus_suspend
//-----------------------------------------------------------------
static int ue11h_bus_suspend(struct usb_hcd *hcd)
{
    // SOFs off
    dev_dbg(hcd->self.controller, "%s\n", __func__);
    return 0;
}
//-----------------------------------------------------------------
// ue11h_bus_resume
//-----------------------------------------------------------------
static int ue11h_bus_resume(struct usb_hcd *hcd)
{
    // SOFs on
    dev_dbg(hcd->self.controller, "%s\n", __func__);
    return 0;
}

#else

#define ue11h_bus_suspend  NULL
#define ue11h_bus_resume   NULL

#endif
//-----------------------------------------------------------------
// ue11h_stop
//-----------------------------------------------------------------
static void ue11h_stop(struct usb_hcd *hcd)
{
    struct ue11    *ue11 = hcd_to_ue11(hcd);
    unsigned long   flags;

    del_timer_sync(&hcd->rh_timer);

    spin_lock_irqsave(&ue11->lock, flags);
    port_power(ue11, 0);
    spin_unlock_irqrestore(&ue11->lock, flags);
}
//-----------------------------------------------------------------
// ue11h_start
//-----------------------------------------------------------------
static int ue11h_start(struct usb_hcd *hcd)
{
    struct ue11        *ue11 = hcd_to_ue11(hcd);

    /* chip has been reset, VBUS power is off */
    hcd->state = HC_STATE_RUNNING;
    /* enable power and interrupts */
    port_power(ue11, 1);

    return 0;
}
//-----------------------------------------------------------------
// ue11h_hc_driver structure
//-----------------------------------------------------------------
static const struct hc_driver ue11h_hc_driver = {
    .description =      hcd_name,
    .hcd_priv_size =    sizeof(struct ue11),

    /*
     * generic hardware linkage
     */
    .irq =          ue11h_irq,
    .flags =        HCD_USB11 ,//| HCD_MEMORY,

    /* Basic lifecycle operations */
    .start =        ue11h_start,
    .stop =         ue11h_stop,

    /*
     * managing i/o requests and associated device resources
     */
    .urb_enqueue =      ue11h_urb_enqueue,
    .urb_dequeue =      ue11h_urb_dequeue,
    .endpoint_disable = ue11h_endpoint_disable,

    /*
     * periodic schedule support
     */
    .get_frame_number = ue11h_get_frame,

    /*
     * root hub support
     */
    .hub_status_data =  ue11h_hub_status_data,
    .hub_control =      ue11h_hub_control,
    .bus_suspend =      ue11h_bus_suspend,
    .bus_resume =       ue11h_bus_resume,
};
//-----------------------------------------------------------------
// ue11h_remove:
//-----------------------------------------------------------------
static int ue11h_remove(struct platform_device *dev)
{
    struct usb_hcd      *hcd = platform_get_drvdata(dev);
    struct ue11        *ue11 = hcd_to_ue11(hcd);

    usb_remove_hcd(hcd);
    usb_put_hcd(hcd);
    return 0;
}
//-----------------------------------------------------------------
// ue11h_probe:
//-----------------------------------------------------------------
static int ue11h_probe(struct platform_device *dev)
{
    struct usb_hcd      *hcd;
    struct ue11        *ue11;
    struct resource *iores;

    int         irq;
    int         retval;
    void __iomem *dev_base;

    if (usb_disabled())
        return -ENODEV;

    iores = platform_get_resource(dev, IORESOURCE_MEM, 0);
    if (!iores)
        return -ENODEV; 

    // Get IRQ for device
    irq = platform_get_irq(dev, 0);
    if (irq < 0)
        return -ENODEV;

    // Get device memory
    dev_base = devm_ioremap_resource(&dev->dev, iores);
    if (IS_ERR(dev_base))
        return PTR_ERR(dev_base);

    /* allocate and initialize hcd */
    hcd = usb_create_hcd(&ue11h_hc_driver, &dev->dev, dev_name(&dev->dev));
    if (!hcd)
    {
        retval = -ENOMEM;
        goto err5;
    }

    hcd->rsrc_start = dev_base;
    ue11 = hcd_to_ue11(hcd);

    spin_lock_init(&ue11->lock);
    INIT_LIST_HEAD(&ue11->async);
    timer_setup(&ue11->timer, ue11h_timer, 0);
    ue11->reg_base = dev_base;

    spin_lock_irq(&ue11->lock);
    port_power(ue11, 0);
    spin_unlock_irq(&ue11->lock);
    msleep(200);

    /* The chip's IRQ is level triggered, active high.  A requirement
     * for platform device setup is to cope with things like signal
     * inverters (e.g. CF is active low) or working only with edge
     * triggers (e.g. most ARM CPUs).  Initial driver stress testing
     * was on a system with single edge triggering, so most sorts of
     * triggering arrangement should work.
     *
     * Use resource IRQ flags if set by platform device setup.
     */
    retval = usb_add_hcd(hcd, irq, IRQF_SHARED);
    if (retval != 0)
    {
        goto err6;
    }

    device_wakeup_enable(hcd->self.controller);

    return retval;

 err6:
    usb_put_hcd(hcd);
 err5:    
    dev_dbg(&dev->dev, "init error, %d\n", retval);
    return retval;
}

#ifdef  CONFIG_PM

/* for this device there's no useful distinction between the controller
 * and its root hub.
 */
//-----------------------------------------------------------------
// ue11h_suspend:
//-----------------------------------------------------------------
static int ue11h_suspend(struct platform_device *dev, pm_message_t state)
{
    struct usb_hcd  *hcd = platform_get_drvdata(dev);
    struct ue11    *ue11 = hcd_to_ue11(hcd);
    int     retval = 0;

    switch (state.event) {
    case PM_EVENT_FREEZE:
        retval = ue11h_bus_suspend(hcd);
        break;
    case PM_EVENT_SUSPEND:
    case PM_EVENT_HIBERNATE:
    case PM_EVENT_PRETHAW:      /* explicitly discard hw state */
        port_power(ue11, 0);
        break;
    }
    return retval;
}
//-----------------------------------------------------------------
// ue11h_resume:
//-----------------------------------------------------------------
static int ue11h_resume(struct platform_device *dev)
{
    struct usb_hcd  *hcd = platform_get_drvdata(dev);
    struct ue11    *ue11 = hcd_to_ue11(hcd);

    /* with no "check to see if VBUS is still powered" board hook,
     * let's assume it'd only be powered to enable remote wakeup.
     */
    if (!ue11->port1 || !device_can_wakeup(&hcd->self.root_hub->dev)) {
        ue11->port1 = 0;
        port_power(ue11, 1);
        usb_root_hub_lost_power(hcd->self.root_hub);
        return 0;
    }

    return ue11h_bus_resume(hcd);
}

#else

#define ue11h_suspend  NULL
#define ue11h_resume   NULL

#endif

static const struct of_device_id my_match_table[] = {
     { .compatible = "ue11-hcd" },
     {},
};
MODULE_DEVICE_TABLE(of, my_match_table);


/* this driver is exported so ue11_cs can depend on it */
struct platform_driver ue11h_driver = {
    .probe =    ue11h_probe,
    .remove =   ue11h_remove,

    .suspend =  ue11h_suspend,
    .resume =   ue11h_resume,
    .driver = {
        .name = (char *) hcd_name,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(my_match_table),
    },
};
EXPORT_SYMBOL(ue11h_driver);

module_platform_driver(ue11h_driver);
