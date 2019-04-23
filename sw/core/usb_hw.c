#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "usb_defs.h"
#include "usb_hw.h"

#ifndef USB_TESTBENCH
#include "timer.h"
#endif

//-----------------------------------------------------------------
// Logging levels:
//-----------------------------------------------------------------
#define USBLOG_NONE     0
#define USBLOG_ERR      1
#define USBLOG_INFO     2
#define USBLOG_DATA     3

// Current log level
#define USBLOG_LEVEL    USBLOG_INFO

#define LOG(l,a)        do { if (l <= USBLOG_LEVEL) printf a; } while (0)

//-----------------------------------------------------------------
// Defines:
//-----------------------------------------------------------------
#define USB_CTRL          0x0
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

    #define USB_CTRL_TX_FLUSH                    1
    #define USB_CTRL_TX_FLUSH_SHIFT              1
    #define USB_CTRL_TX_FLUSH_MASK               0x1

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
// Locals:
//-----------------------------------------------------------------
static uint32_t _usb_base;
static int      _usb_fs_device;

#ifdef USB_TESTBENCH
extern void usbhw_reg_write(uint32_t addr, uint32_t data);
extern uint32_t usbhw_reg_read(uint32_t addr);
#else
//-----------------------------------------------------------------
// usbhw_reg_write:
//-----------------------------------------------------------------
static void usbhw_reg_write(uint32_t addr, uint32_t data)
{
    *((volatile uint32_t*)(_usb_base + addr)) = data;
}
//-----------------------------------------------------------------
// usbhw_reg_read:
//-----------------------------------------------------------------
static uint32_t usbhw_reg_read(uint32_t addr)
{
    return *((volatile uint32_t*)(_usb_base + addr));
}
#endif
//-----------------------------------------------------------------
// usbhw_init:
//-----------------------------------------------------------------
void usbhw_init(uint32_t base)
{
    _usb_base      = base;
    _usb_fs_device = 0;
}
//-----------------------------------------------------------------
// usbhw_transfer_out: Send token then some DATA to the device
//-----------------------------------------------------------------
int usbhw_transfer_out(uint8_t pid, int device_addr, int endpoint, int handshake, uint8_t request, uint8_t *tx, int tx_length)
{
    int l;
    uint32_t ctrl = 0;
    uint32_t token = 0; 
    uint32_t resp;
    uint32_t status;

    LOG(USBLOG_DATA, ("TOKEN: %s", (pid == PID_SETUP) ? "SETUP" : (pid == PID_DATA0) ? "DATA0": (pid == PID_DATA1) ? "DATA1" : (pid == PID_IN) ? "IN" : "OUT"));
    LOG(USBLOG_DATA, ("  DEV %d EP %d\n", device_addr, endpoint));

    // Load DATAx transfer into address 0+
    LOG(USBLOG_DATA, (" Tx:\n %02x", request));
    for (l=0;l<tx_length;l++)
    {
        LOG(USBLOG_DATA, (" %02x", tx[l]));
        usbhw_reg_write(USB_WR_DATA, tx[l]);
    }
    LOG(USBLOG_DATA, ("\n"));

    // Transfer data length
    usbhw_reg_write(USB_XFER_DATA, tx_length);

    // Configure transfer for DATAx portion
    ctrl = (1 << USB_XFER_TOKEN_START_SHIFT);

    // Wait for response or timeout
    ctrl|= (handshake ? (1 << USB_XFER_TOKEN_ACK_SHIFT) : 0);

    ctrl|= ((request == PID_DATA1) ? (1 << USB_XFER_TOKEN_PID_DATAX_SHIFT) : (0 << USB_XFER_TOKEN_PID_DATAX_SHIFT));

    // Setup token details (don't start transfer yet)
    token = (((uint32_t)pid)<<USB_XFER_TOKEN_PID_BITS_SHIFT) | (device_addr << USB_XFER_TOKEN_DEV_ADDR_SHIFT) | (endpoint << USB_XFER_TOKEN_EP_ADDR_SHIFT);
    usbhw_reg_write(USB_XFER_TOKEN, token | ctrl);

    // Wait for Tx to start
    while ((status = usbhw_reg_read(USB_RX_STAT)) & (1 << USB_RX_STAT_START_PEND_SHIFT))
        ;

    // No handshaking? We are done
    if (!handshake)
        return USB_RES_OK;

    // Wait for idle
    while (!((status = usbhw_reg_read(USB_RX_STAT)) & (1 << USB_RX_STAT_IDLE_SHIFT)))
        ;

    if (status & (1 << USB_RX_STAT_RESP_TIMEOUT_SHIFT))
    {
       LOG(USBLOG_DATA, ("  TIMEOUT\n"));
       LOG(USBLOG_ERR, ("USB: OUT timeout\n"));
       return USB_RES_TIMEOUT;
    }

    // Check for NAK / STALL
    resp = ((status >> USB_RX_STAT_RESP_BITS_SHIFT) & USB_RX_STAT_RESP_BITS_MASK);
    if (resp == PID_ACK)
    {
       LOG(USBLOG_DATA, ("  ACK\n"));
       return USB_RES_OK;
    }
    else if (resp == PID_NAK)
    {
       LOG(USBLOG_DATA, ("  NAK\n"));
       return USB_RES_NAK;
    }
    else if (resp == PID_STALL)
    {
       LOG(USBLOG_DATA, ("  STALL\n"));
       LOG(USBLOG_ERR, ("USB: OUT STALL\n"));
       return USB_RES_STALL;
    }

    LOG(USBLOG_ERR, ("USB: Unknown OUT response (%02x)\n", resp));

    // Unknown
    return USB_RES_STALL;
}
//-----------------------------------------------------------------
// usbhw_transfer_in: Perform IN request and expect DATA from device
//-----------------------------------------------------------------
int usbhw_transfer_in(uint8_t pid, int device_addr, int endpoint, uint8_t *response, uint8_t *rx, int rx_length)
{
    int l;
    int rx_count;
    uint32_t token = 0;
    uint8_t data;
    uint32_t status;

    LOG(USBLOG_DATA, ("TOKEN: %s", (pid == PID_SETUP) ? "SETUP" : (pid == PID_DATA0) ? "DATA0": (pid == PID_DATA1) ? "DATA1" : (pid == PID_IN) ? "IN" : "OUT"));
    LOG(USBLOG_DATA, ("  DEV %d EP %d\n", device_addr, endpoint));    

    // No data to send
    usbhw_reg_write(USB_XFER_DATA, 0);

    // Configure transfer
    token = (((uint32_t)pid)<<USB_XFER_TOKEN_PID_BITS_SHIFT) | (device_addr << USB_XFER_TOKEN_DEV_ADDR_SHIFT) | (endpoint << USB_XFER_TOKEN_EP_ADDR_SHIFT);
    token|= (1 << USB_XFER_TOKEN_START_SHIFT);
    token|= (1 << USB_XFER_TOKEN_IN_SHIFT);
    token|= (1 << USB_XFER_TOKEN_ACK_SHIFT);
    usbhw_reg_write(USB_XFER_TOKEN, token);

    while ((status = usbhw_reg_read(USB_RX_STAT)) & (1 << USB_RX_STAT_START_PEND))
        ;

    // Wait for rx idle
    while (!((status = usbhw_reg_read(USB_RX_STAT)) & (1 << USB_RX_STAT_IDLE_SHIFT)))
        ;

    if (status & (1 << USB_RX_STAT_CRC_ERR_SHIFT))
    {
       LOG(USBLOG_DATA, ("  CRC ERROR\n"));
       LOG(USBLOG_ERR, ("USB: CRC ERROR\n"));
       return USB_RES_TIMEOUT;
    }

    if (status & (1 << USB_RX_STAT_RESP_TIMEOUT_SHIFT))
    {
       LOG(USBLOG_DATA, ("  TIMEOUT\n"));
       LOG(USBLOG_ERR, ("USB: IN timeout\n"));
       return USB_RES_TIMEOUT;
    }

    // Check for NAK / STALL
    *response = ((status >> USB_RX_STAT_RESP_BITS_SHIFT) & USB_RX_STAT_RESP_BITS_MASK);

    if (*response == PID_NAK)
    {
       LOG(USBLOG_DATA, ("  NAK\n"));
       return USB_RES_NAK;
    }
    else if (*response == PID_STALL)
    {
       LOG(USBLOG_DATA, ("  STALL\n"));
       LOG(USBLOG_ERR, ("USB: IN STALL\n"));
       return USB_RES_STALL;
    }

    // Check CRC is ok
    if (status & (1 << USB_RX_STAT_CRC_ERR_SHIFT))
    {
        LOG(USBLOG_DATA, ("  CRC ERR\n"));
        LOG(USBLOG_ERR, ("USB: CRC Error\n"));

        return USB_RES_STALL;
    }

    // How much data was actually received?
    rx_count = ((status >> USB_RX_STAT_COUNT_BITS_SHIFT) & USB_RX_STAT_COUNT_BITS_MASK);

    LOG(USBLOG_DATA, (" Rx %d (PID=%x):\n", rx_count, *response));

    // Assert that user buffer is big enough for the response.
    // NOTE: It's not critical to do this, but we can't easily check CRCs without
    // reading the whole response into a buffer.
    // Hitting this condition may point towards issues with higher level protocol
    // implementation...
    assert(rx_length >= rx_count);
    
    for (l=0;l<rx_count;l++)
    {
        data = usbhw_reg_read(USB_RD_DATA);
        LOG(USBLOG_DATA, (" %02x", data));
        rx[l] = data;
    }
    LOG(USBLOG_DATA, ("\n"));

    return rx_count;
}
//-----------------------------------------------------------------
// usbhw_hub_reset: Put bus into SE0 state (reset)
//-----------------------------------------------------------------
void usbhw_hub_reset(void)
{
    uint32_t val;

    LOG(USBLOG_INFO, ("HW: Enter USB bus reset\n"));

    // Power-up / SE0
    val = 0;
    val |= (1 << USB_CTRL_PHY_XCVRSELECT_SHIFT);
    val |= (0 << USB_CTRL_PHY_TERMSELECT_SHIFT);
    val |= (0 << USB_CTRL_PHY_OPMODE_SHIFT);
    val |= (1 << USB_CTRL_PHY_DPPULLDOWN_SHIFT);
    val |= (1 << USB_CTRL_PHY_DMPULLDOWN_SHIFT);
    usbhw_reg_write(USB_CTRL, val);
}
//-----------------------------------------------------------------
// usbhw_hub_device_detected: Detect device inserted
//-----------------------------------------------------------------
int usbhw_hub_device_detected(void)
{
    // Get line state
    uint32_t status = usbhw_reg_read(USB_STATUS);
    status >>= USB_STATUS_LINESTATE_BITS_SHIFT;
    status &= USB_STATUS_LINESTATE_BITS_MASK;

    // FS: D+ pulled high
    // LS: D- pulled high
    _usb_fs_device = (status & 1);

    return (status != 0);
}
//-----------------------------------------------------------------
// usbhw_hub_full_speed_device: Speed detection
//-----------------------------------------------------------------
int usbhw_hub_full_speed_device(void)
{
    return _usb_fs_device;
}
//-----------------------------------------------------------------
// usbhw_hub_enable: Enable root hub (drive data lines to HiZ)
//                   and optionally start SOF periods
//-----------------------------------------------------------------
void usbhw_hub_enable(int full_speed, int enable_sof)
{
    uint32_t val;

    LOG(USBLOG_INFO, ("HW: Enable root hub\n"));

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

    usbhw_reg_write(USB_CTRL, val);
}
//-----------------------------------------------------------------
// usbhw_reset: Perform USB reset
//-----------------------------------------------------------------
int usbhw_reset(void)
{    
#ifdef USB_TESTBENCH
    // Assert SE0 / reset
    usbhw_hub_reset();

    // Enable root hub
    usbhw_hub_enable(1, 0);
#else
    // Assert SE0 / reset
    usbhw_hub_reset();

    // Wait for some time
    usbhw_timer_sleep(11);

    // Stop asserting SE0, set data lines to Hi-Z
    usbhw_hub_enable(1, 0);
    usbhw_timer_sleep(3);

    LOG(USBLOG_INFO, ("HW: Waiting for device insertion\n"));

    // Wait for device detect
    while (!usbhw_hub_device_detected())
        ;

    LOG(USBLOG_INFO, ("HW: Device detected\n"));

    // Enable SOF
    usbhw_hub_enable(usbhw_hub_full_speed_device(), 1);
    usbhw_timer_sleep(3);
#endif
}
//-----------------------------------------------------------------
// usbhw_timer_sleep: Perform USB Reset
//-----------------------------------------------------------------
#ifndef USB_TESTBENCH
void usbhw_timer_sleep(int ms)
{
    timer_sleep(ms);
}
t_time usbhw_timer_now(void)
{
    return timer_now();
}
#else
extern void usbhw_timer_sleep(int ms);
extern t_time usbhw_timer_now(void);
#endif
