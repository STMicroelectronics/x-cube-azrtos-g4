/**************************************************************************/
/*                                                                        */
/*       Copyright (c) Microsoft Corporation. All rights reserved.        */
/*                                                                        */
/*       This software is licensed under the Microsoft Software License   */
/*       Terms for Microsoft Azure RTOS. Full text of the license can be  */
/*       found in the LICENSE file at https://aka.ms/AzureRTOS_EULA       */
/*       and in the root directory of this software.                      */
/*                                                                        */
/**************************************************************************/

/**************************************************************************/
/**************************************************************************/
/**                                                                       */
/** USBX Component                                                        */
/**                                                                       */
/**   User Specific                                                       */
/**                                                                       */
/**************************************************************************/
/**************************************************************************/

/**************************************************************************/
/*                                                                        */
/*  PORT SPECIFIC C INFORMATION                            RELEASE        */
/*                                                                        */
/*    ux_user.h                                           PORTABLE C      */
/*                                                           6.0          */
/*                                                                        */
/*  AUTHOR                                                                */
/*                                                                        */
/*    Chaoqiong Xiao, Microsoft Corporation                               */
/*                                                                        */
/*  DESCRIPTION                                                           */
/*                                                                        */
/*    This file contains user defines for configuring USBX in specific    */
/*    ways. This file will have an effect only if the application and     */
/*    USBX library are built with UX_INCLUDE_USER_DEFINE_FILE defined.    */
/*    Note that all the defines in this file may also be made on the      */
/*    command line when building USBX library and application objects.    */
/*                                                                        */
/*  RELEASE HISTORY                                                       */
/*                                                                        */
/*    DATE              NAME                      DESCRIPTION             */
/*                                                                        */
/*  05-19-2020     Chaoqiong Xiao           Initial Version 6.0           */
/*                                                                        */
/**************************************************************************/

#ifndef UX_USER_H
#define UX_USER_H

/* Define various build options for the USBX port.  The application should either make changes
   here by commenting or un-commenting the conditional compilation defined OR supply the defines
   through the compiler's equivalent of the -D option. Override various options with default values
   already assigned in ux_api.h or ux_port.h. Please also refer to ux_port.h for descriptions on
   each of these options. */

/* This value is the size of the stack in bytes for the USBX threads. It can be typically 1024 bytes
   or 2048 bytes depending on the processor used and the host controller. */

/* #define UX_THREAD_STACK_SIZE                                (2 * 1024)
*/

/* Defined, this value represents how many ticks per seconds for a specific hardware platform.
   The default is 1000 indicating 1 tick per millisecond.  */

/* #define UX_PERIODIC_RATE (TX_TIMER_TICKS_PER_SECOND)*/

/* Defined, this value is the maximum number of classes in the device stack that can be loaded by
   USBX.  */
#define UX_MAX_SLAVE_CLASS_DRIVER                         1

/* Defined, this value is the maximum number of interfaces in the device framework.  */

/* #define UX_MAX_SLAVE_INTERFACES    16 */

/* Defined, this value represents the maximum number of devices that can be attached to the USB.
   Normally, the theoretical maximum number on a single USB is 127 devices. This value can be
   scaled down to conserve memory. Note that this value represents the total number of devices
   regardless of the number of USB buses in the system.  */

/* #define UX_MAX_DEVICES  127 */

/* Defined, this value represents the current number of SCSI logical units represented in the device
   storage class driver.  */

/* #define UX_MAX_SLAVE_LUN    1 */

/* Defined, this value represents the maximum number of bytes received on a control endpoint in
   the device stack. The default is 256 bytes but can be reduced in memory constraint environments.  */

/* #define UX_SLAVE_REQUEST_CONTROL_MAX_LENGTH        256 */

/* Defined, this value represents the maximum number of bytes that can be received or transmitted
   on any endpoint. This value cannot be less than the maximum packet size of any endpoint. The default
   is 4096 bytes but can be reduced in memory constraint environments. For cd-rom support in the storage
   class, this value cannot be less than 2048.  */

#define UX_SLAVE_REQUEST_DATA_MAX_LENGTH                                   512

/* Defined, this value includes code to handle storage Multi-Media Commands (MMC). E.g., DVD-ROM. */

/* #define UX_SLAVE_CLASS_STORAGE_INCLUDE_MMC */

/* Defined, this value forces the memory allocation scheme to enforce alignment
   of memory with the UX_SAFE_ALIGN field.
*/

/* #define UX_ENFORCE_SAFE_ALIGNMENT */

/* Defined, this value represents the number of packets in the CDC_ECM device class.
   The default is 16.
*/

/* #define UX_DEVICE_CLASS_CDC_ECM_NX_PKPOOL_ENTRIES           4 */

/* Defined, this value represents the number of milliseconds to wait for packet
   allocation until invoking the application's error callback and retrying.
*/
/* #define UX_DEVICE_CLASS_CDC_ECM_PACKET_POOL_WAIT                                           1000 */

/* Defined, this value represents the the maximum length of HID reports on the
   device.
 */

/* #define UX_DEVICE_CLASS_HID_EVENT_BUFFER_LENGTH                                           64 */

/* Defined, this value represents the the maximum number of HID events/reports
   that can be queued at once.
 */

/* #define UX_DEVICE_CLASS_HID_MAX_EVENTS_QUEUE                                           8 */

/* Defined, this macro will disable DFU_UPLOAD support.  */

/* #define UX_DEVICE_CLASS_DFU_UPLOAD_DISABLE  */

/* Defined, this macro will enable DFU_GETSTATUS and DFU_GETSTATE in dfuERROR.  */

/* #define UX_DEVICE_CLASS_DFU_ERROR_GET_ENABLE  */

/* Defined, this macro will change status mode.
   0 - simple mode,
       status is queried from application in dfuDNLOAD-SYNC and dfuMANIFEST-SYNC state,
       no bwPollTimeout.
   1 - status is queried from application once requested,
       b0-3 : media status
       b4-7 : bStatus
       b8-31: bwPollTimeout
       bwPollTimeout supported.
*/

#define UX_DEVICE_CLASS_DFU_STATUS_MODE                  1

/* Defined, this value represents the default DFU status bwPollTimeout.
   The value is 3 bytes long (max 0xFFFFFFu).
   By default the bwPollTimeout is 1 (means 1ms).
 */

#define UX_DEVICE_CLASS_DFU_STATUS_POLLTIMEOUT             0

/* Defined, this macro will enable custom request process callback.  */

/* #define UX_DEVICE_CLASS_DFU_CUSTOM_REQUEST_ENABLE  */

/* Defined, this value will only enable the device side of usbx.  */

#define UX_DEVICE_SIDE_ONLY

/* Defined, this value represents the size of the log pool.
*/
/* #define UX_DEBUG_LOG_SIZE                                   (1024 * 16) */

/* This is the ThreadX priority value for the USBX enumeration threads that monitors the bus topology */

/* #define UX_THREAD_PRIORITY_ENUM             20 */

/* This is the ThreadX priority value for the standard USBX threads */

/* #define UX_THREAD_PRIORITY_CLASS             20 */

/* This value actually defines the time slice that will be used for threads. For example, if defined to 0, the ThreadX target port does not use time slices. */

/* #define UX_NO_TIME_SLICE             0 */

/* it define USBX device max number of endpoints (1~n). */

/* #define UX_MAX_DEVICE_ENDPOINTS             6 */

/* it define USBX device max number of interfacess (1~n). */

/* #define UX_MAX_DEVICE_INTERFACES             6 */

/* Determine if tracing is enabled.  */

/*#define UX_TRACE_INSERT_MACROS*/

/* Defined, this macro enables device bi-directional endpoint support.  */

#define UX_DEVICE_BIDIRECTIONAL_ENDPOINT_SUPPORT

#endif

