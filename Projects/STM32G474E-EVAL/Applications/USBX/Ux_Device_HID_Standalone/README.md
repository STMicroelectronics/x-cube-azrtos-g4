# <b>Ux_Device_HID_Standalone application description</b>

This application provides an example of Azure RTOS USBX stack usage on STM32G474E-EVAL board,
it shows how to develop USB Device Human Interface "HID" mouse based bare metal application.

The application is designed to emulate an USB HID mouse device, the code provides all required device descriptors framework
and associated Class descriptor report to build a compliant USB HID mouse device.

The application's main calls the MX_USBX_Device_Init() function in order to Initialize USBX and USBX_Device_Process in the while loop.

As stated earlier, the present application runs in standalone mode without ThreadX, for this reason, the standalone variant of USBX is enabled by adding the following flag in ux_user.h:

 - #define UX_STANDALONE

To customize the HID application by sending the mouse position step by step every click of user button, the application calls the GetPointerData() API to update the mouse position (x, y) and send the report buffer through the ux_device_class_hid_event_set() API.

#### <b>Expected success behavior</b>

When plugged to PC host, the STM32G474E-EVAL must be properly enumerated as an USB HID mouse device.
During the enumeration phase, device provides host with the requested descriptors (Device, configuration, string).
Those descriptors are used by host driver to identify the device capabilities.
Once the STM32G474E-EVAL USB device successfully completed the enumeration phase, the device sends a HID report after a user button press.
Each report sent should move the mouse cursor by one step on host side.

#### <b>Error behaviors</b>

Host PC shows that USB device does not operate as designed (Mouse enumeration failed, the mouse pointer doesn't move).

#### <b>Assumptions if any</b>

User is familiar with USB 2.0 "Universal Serial BUS" Specification and HID class Specification.

#### <b>Known limitations</b>

The remote wakeup feature is not yet implemented (used to bring the USB suspended bus back to the active condition).

### <b>Keywords</b>

Standalone, USBXDevice, USB, Full Speed, HID, Mouse,

### <b>Hardware and Software environment</b>

  - This example runs on STM32G4xx devices.
  - This example has been tested with STMicroelectronics STM32G474E-EVAL boards Revision MB1397 Revision B-05 and can be easily tailored to any other supported device and development board.

### <b>How to use it ?</b>

In order to make the program work, you must do the following :

 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the application