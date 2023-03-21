/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "usb_host_config.h"
#include "usb_host.h"
#include "fsl_device_registers.h"
#include "usb_host_msd.h"
#include "host_msd_fatfs.h"
#include "fsl_common.h"
#include "board.h"

#include "fsl_power.h"
#include "usb_phy.h"
#include "usb_host_devices.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
/* PIN MUX */
#define IOCON_PIO_DIGITAL_EN 0x0100u  /*!<@brief Enables digital function */
#define IOCON_PIO_FUNC1 0x01u         /*!<@brief Selects pin function 1 */
#define IOCON_PIO_FUNC4 0x04u         /*!<@brief Selects pin function 4 */
#define IOCON_PIO_FUNC7 0x07u         /*!<@brief Selects pin function 7 */
#define IOCON_PIO_INV_DI 0x00u        /*!<@brief Input function is not inverted */
#define IOCON_PIO_MODE_INACT 0x00u    /*!<@brief No addition pin function */
#define IOCON_PIO_MODE_PULLUP 0x20u   /*!<@brief Selects pull-up function */
#define IOCON_PIO_OPENDRAIN_DI 0x00u  /*!<@brief Open drain is disabled */
#define IOCON_PIO_SLEW_STANDARD 0x00u /*!<@brief Standard mode, output slew rate control is enabled */


/* USB */
#define CONTROLLER_ID kUSB_ControllerIp3516Hs0
#define USB_HOST_INTERRUPT_PRIORITY (6U)

/*******************************************************************************
 * Variables
 ******************************************************************************/
usb_host_handle g_HostHandle;
usb_host_device_instance_t* g_HostDeviceInstance;
/*******************************************************************************
 * Declarations
 ******************************************************************************/

static void usb1_pin_init(void);
static void usb1_host_clock_init(void);
static void usb1_host_isr_enable(void);
static usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
                                  usb_host_configuration_handle configurationHandle,
                                  uint32_t eventCode);


/*******************************************************************************
 * Functions
 ******************************************************************************/
void USB1_IRQHandler(void)
{
    USB_HostIp3516HsIsrFunction(g_HostHandle);
}

static void usb1_pin_init(void)
{
	CLOCK_EnableClock(kCLOCK_Iocon);
	//     const uint32_t port1_pin29_config = (/* Pin is configured as USB1_PORTPWRN */
    //                                      IOCON_PIO_FUNC4 |
    //                                      /* Selects pull-up function */
    //                                      IOCON_PIO_MODE_PULLUP |
    //                                      /* Standard mode, output slew rate control is enabled */
    //                                      IOCON_PIO_SLEW_STANDARD |
    //                                      /* Input function is not inverted */
    //                                      IOCON_PIO_INV_DI |
    //                                      /* Enables digital function */
    //                                      IOCON_PIO_DIGITAL_EN |
    //                                      /* Open drain is disabled */
    //                                      IOCON_PIO_OPENDRAIN_DI);
    // /* PORT1 PIN29 (coords: 80) is configured as USB1_PORTPWRN */
    // IOCON_PinMuxSet(IOCON, 1U, 29U, port1_pin29_config);

    // const uint32_t port1_pin30_config = (/* Pin is configured as USB1_OVERCURRENTN */
    //                                      IOCON_PIO_FUNC4 |
    //                                      /* Selects pull-up function */
    //                                      IOCON_PIO_MODE_PULLUP |
    //                                      /* Standard mode, output slew rate control is enabled */
    //                                      IOCON_PIO_SLEW_STANDARD |
    //                                      /* Input function is not inverted */
    //                                      IOCON_PIO_INV_DI |
    //                                      /* Enables digital function */
    //                                      IOCON_PIO_DIGITAL_EN |
    //                                      /* Open drain is disabled */
    //                                      IOCON_PIO_OPENDRAIN_DI);
    // /* PORT1 PIN30 (coords: 65) is configured as USB1_OVERCURRENTN */
    // IOCON_PinMuxSet(IOCON, 1U, 30U, port1_pin30_config);

    //     const uint32_t port0_pin29_config = (/* Pin is configured as FC0_RXD_SDA_MOSI_DATA */
    //                                      IOCON_PIO_FUNC1 |
    //                                      /* No addition pin function */
    //                                      IOCON_PIO_MODE_INACT |
    //                                      /* Standard mode, output slew rate control is enabled */
    //                                      IOCON_PIO_SLEW_STANDARD |
    //                                      /* Input function is not inverted */
    //                                      IOCON_PIO_INV_DI |
    //                                      /* Enables digital function */
    //                                      IOCON_PIO_DIGITAL_EN |
    //                                      /* Open drain is disabled */
    //                                      IOCON_PIO_OPENDRAIN_DI);
    // /* PORT0 PIN29 (coords: 92) is configured as FC0_RXD_SDA_MOSI_DATA */
    // IOCON_PinMuxSet(IOCON, 0U, 29U, port0_pin29_config);

    // const uint32_t port0_pin30_config = (/* Pin is configured as FC0_TXD_SCL_MISO_WS */
    //                                      IOCON_PIO_FUNC1 |
    //                                      /* No addition pin function */
    //                                      IOCON_PIO_MODE_INACT |
    //                                      /* Standard mode, output slew rate control is enabled */
    //                                      IOCON_PIO_SLEW_STANDARD |
    //                                      /* Input function is not inverted */
    //                                      IOCON_PIO_INV_DI |
    //                                      /* Enables digital function */
    //                                      IOCON_PIO_DIGITAL_EN |
    //                                      /* Open drain is disabled */
    //                                      IOCON_PIO_OPENDRAIN_DI);
    // /* PORT0 PIN30 (coords: 94) is configured as FC0_TXD_SCL_MISO_WS */
    // IOCON_PinMuxSet(IOCON, 0U, 30U, port0_pin30_config);
//dylan

	const uint32_t port0_pin22_config = (/* Pin is configured as USB0_VBUS */
										 IOCON_PIO_FUNC7 |
										 /* No addition pin function */
										 IOCON_PIO_MODE_INACT |
										 /* Standard mode, output slew rate control is enabled */
										 IOCON_PIO_SLEW_STANDARD |
										 /* Input function is not inverted */
										 IOCON_PIO_INV_DI |
										 /* Enables digital function */
										 IOCON_PIO_DIGITAL_EN |
										 /* Open drain is disabled */
										 IOCON_PIO_OPENDRAIN_DI);
	/* PORT0 PIN22 (coords: 78) is configured as USB0_VBUS */
	IOCON_PinMuxSet(IOCON, 0U, 22U, port0_pin22_config);
}

static void usb1_host_clock_init(void)
{
    CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_UsbPhySrcExt, BOARD_XTAL0_CLK_HZ);
    CLOCK_EnableUsbhs0HostClock(kCLOCK_UsbSrcUnused, 0U);
    USB_EhciPhyInit(CONTROLLER_ID, BOARD_XTAL0_CLK_HZ, NULL);

    for (int i = 0; i < (FSL_FEATURE_USBHSH_USB_RAM >> 2); i++)
    {
        ((uint32_t *)FSL_FEATURE_USBHSH_USB_RAM_BASE_ADDRESS)[i] = 0U;
    }
}

static void usb1_host_isr_enable(void)
{
    uint8_t irqNumber;

    IRQn_Type usbHsIrqs[] = {(IRQn_Type)USB1_IRQn};
    irqNumber             = usbHsIrqs[CONTROLLER_ID - kUSB_ControllerIp3516Hs0];

/* Install isr, set priority, and enable IRQ. */
    NVIC_SetPriority((IRQn_Type)irqNumber, USB_HOST_INTERRUPT_PRIORITY);
    EnableIRQ((IRQn_Type)irqNumber);
}


static usb_status_t USB_HostEvent(usb_device_handle deviceHandle,
                                  usb_host_configuration_handle configurationHandle,
                                  uint32_t eventCode)
{
    usb_status_t status = kStatus_USB_Success;
    switch (eventCode & 0x0000FFFFU)
    {
        case kUSB_HostEventAttach:

            status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
            break;

        case kUSB_HostEventNotSupported:

            usb_echo("Unsupported Device\r\n");
            break;

        case kUSB_HostEventEnumerationDone:
		
            status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
            break;

        case kUSB_HostEventDetach:

            status = USB_HostMsdEvent(deviceHandle, configurationHandle, eventCode);
            break;

        case kUSB_HostEventEnumerationFail:
            usb_echo("enumeration failed\r\n");
            break;

        default:
            break;
    }
    return status;
}


int usb1_host_app_init(void)
{
    usb_status_t status = kStatus_USB_Success;

	usb1_pin_init();
    // BOARD_InitDebugConsole(); // dylan
	
	NVIC_ClearPendingIRQ(USB1_IRQn);
    NVIC_ClearPendingIRQ(USB1_NEEDCLK_IRQn);
	
	POWER_DisablePD(kPDRUNCFG_PD_USB1_PHY);
	
	RESET_PeripheralReset(kUSB1H_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSB1D_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSB1_RST_SHIFT_RSTn);
    RESET_PeripheralReset(kUSB1RAM_RST_SHIFT_RSTn);

    usb1_host_clock_init(); 
    status = USB_HostInit(CONTROLLER_ID, &g_HostHandle, USB_HostEvent);
    if (status != kStatus_USB_Success)
    {
        usb_echo("host init error(%d)\r\n",status);
        return status;
    }
    usb1_host_isr_enable();
    usb_echo("host init done\r\n");
    return kStatus_USB_Success;
}

void usb1_host_task(void *param)
{
    USB_HostIp3516HsTaskFunction(param);
}

void hs_usb_loop(void)
{
    usb1_host_task(g_HostHandle);
    USB_HostMsdTask(&g_MsdFatfsInstance);
}