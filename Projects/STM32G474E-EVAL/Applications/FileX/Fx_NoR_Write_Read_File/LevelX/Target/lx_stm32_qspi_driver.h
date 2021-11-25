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

#ifndef LX_STM32_QSPI_DRIVER_H
#define LX_STM32_QSPI_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "lx_api.h"
#include "stm32g4xx_hal.h"
#include "mt25ql512abb.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* The following semaphores are being to notify about RX/TX completion. They need to be released in the transfer callbacks */
extern TX_SEMAPHORE qspi_tx_semaphore;
extern TX_SEMAPHORE qspi_rx_semaphore;

/* Exported constants --------------------------------------------------------*/

/* the QuadSPI instance, default value set to 0 */
#define LX_STM32_QSPI_INSTANCE                           0
#define LX_STM32_QSPI_DEFAULT_TIMEOUT                    10 * TX_TIMER_TICKS_PER_SECOND
#define LX_STM32_DEFAULT_SECTOR_SIZE                     LX_STM32_QSPI_SECTOR_SIZE

/* Use the QSPI DMA API */
#define LX_STM32_QSPI_DMA_API                            1

/* when set to 1 LevelX is initializing the QuadSPI memory,
 * otherwise it is the up to the application to perform it.
 */
#define LX_STM32_QSPI_INIT                               1

/* allow the driver to fully erase the QuadSPI chip. This should be used carefully.
 * the call is blocking and takes a while. by default it is set to 0.
 */
#define LX_STM32_QSPI_ERASE                              1

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

#define LX_STM32_QSPI_CURRENT_TIME                      tx_time_get

/* Macro called after initializing the QSPI driver
 * e.g. create semaphores used for TX/RX transfer notification */
 /* USER CODE BEGIN LX_STM32_QSPI_POST_INIT */

#define LX_STM32_QSPI_POST_INIT()                     do { \
                                                         if (tx_semaphore_create(&qspi_tx_semaphore, "qspi tx semaphore", 0) != TX_SUCCESS) \
                                                         { \
                                                           return LX_ERROR; \
                                                         } \
                                                         if (tx_semaphore_create(&qspi_rx_semaphore, "qspi rx semaphore", 0) != TX_SUCCESS) \
                                                         { \
                                                           return LX_ERROR; \
                                                         } \
                                                       } while(0)

/* USER CODE END LX_STM32_QSPI_POST_INIT */

/* Macro called before performing read operation */
/* USER CODE BEGIN LX_STM32_QSPI_PRE_READ_TRANSFER */

#define LX_STM32_QSPI_PRE_READ_TRANSFER(__status__)

/* USER CODE END LX_STM32_QSPI_PRE_READ_TRANSFER */

/* Define how to notify about Read completion operation */
/* USER CODE BEGIN LX_STM32_QSPI_READ_CPLT_NOTIFY */

#define LX_STM32_QSPI_READ_CPLT_NOTIFY(__status__)      do { \
                                                          if(tx_semaphore_get(&qspi_rx_semaphore, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != TX_SUCCESS) \
                                                          { \
                                                            __status__ = LX_ERROR; \
                                                          } \
                                                        } while(0)

/* USER CODE END LX_STM32_QSPI_READ_CPLT_NOTIFY */

/* Macro called after performing read operation */
/* USER CODE BEGIN LX_STM32_QSPI_POST_READ_TRANSFER */

#define LX_STM32_QSPI_POST_READ_TRANSFER(__status__)

/* USER CODE END LX_STM32_QSPI_POST_READ_TRANSFER */

/* Macro for read error handling */
/* USER CODE BEGIN LX_STM32_QSPI_READ_TRANSFER_ERROR */

#define LX_STM32_QSPI_READ_TRANSFER_ERROR(__status__)

/* USER CODE END LX_STM32_QSPI_READ_TRANSFER_ERROR */

/* Macro called before performing write operation */
/* USER CODE BEGIN LX_STM32_QSPI_PRE_WRITE_TRANSFER */

#define LX_STM32_QSPI_PRE_WRITE_TRANSFER(__status__)

/* USER CODE END LX_STM32_QSPI_PRE_WRITE_TRANSFER */

/* Define how to notify about write completion operation */
/* USER CODE BEGIN LX_STM32_QSPI_WRITE_CPLT_NOTIFY */

#define LX_STM32_QSPI_WRITE_CPLT_NOTIFY(__status__)      do { \
                                                          if(tx_semaphore_get(&qspi_tx_semaphore, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != TX_SUCCESS) \
                                                          { \
                                                            __status__ = LX_ERROR; \
                                                          } \
                                                         } while(0)
/* USER CODE END LX_STM32_QSPI_WRITE_CPLT_NOTIFY */

/* Macro called after performing write operation */
/* USER CODE BEGIN LX_STM32_QSPI_POST_WRITE_TRANSFER */

#define LX_STM32_QSPI_POST_WRITE_TRANSFER(__status__)

/* USER CODE END LX_STM32_QSPI_POST_WRITE_TRANSFER */

/* Macro for write error handling */
/* USER CODE BEGIN LX_STM32_QSPI_WRITE_TRANSFER_ERROR */

#define LX_STM32_QSPI_WRITE_TRANSFER_ERROR(__status__)

/* USER CODE END LX_STM32_QSPI_WRITE_TRANSFER_ERROR */

/* Exported functions prototypes ---------------------------------------------*/
INT lx_stm32_qspi_lowlevel_init(UINT instance);
INT lx_stm32_qspi_lowlevel_deinit(UINT instance);

INT lx_stm32_qspi_get_status(UINT instance);
INT lx_stm32_qspi_get_info(UINT instance, ULONG *block_size, ULONG *total_blocks);

INT lx_stm32_qspi_read(UINT instance, ULONG *address, ULONG *buffer, ULONG words);
INT lx_stm32_qspi_write(UINT instance, ULONG *address, ULONG *buffer, ULONG words);

INT lx_stm32_qspi_erase(UINT instance, ULONG block, ULONG erase_count, UINT full_chip_erase);
INT lx_stm32_qspi_is_block_erased(UINT instance, ULONG block);

UINT lx_qspi_driver_system_error(UINT error_code);

UINT lx_stm32_qspi_initialize(LX_NOR_FLASH *nor_flash);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

#define LX_STM32_QSPI_SECTOR_SIZE                 MT25QL512ABB_SECTOR_64K
#define LX_STM32_QSPI_FLASH_SIZE                  MT25QL512ABB_FLASH_SIZE
#define LX_STM32_QSPI_DUMMY_CYCLES_READ_QUAD      4
#define LX_STM32_QSPI_PAGE_SIZE                   MT25QL512ABB_PAGE_SIZE
#define LX_STM32_QSPI_BULK_ERASE_MAX_TIME         MT25QL512ABB_BULK_ERASE_MAX_TIME
#define LX_STM32_QSPI_SECTOR_ERASE_MAX_TIME       MT25QL512ABB_SECTOR_ERASE_MAX_TIME
#define LX_STM32_QSPI_SR_WEN                      MT25QL512ABB_SR_WEN
#define LX_STM32_QSPI_SR_WIP                      MT25QL512ABB_SR_WIP

#define LX_STM32_QSPI_QUAD_INOUT_FAST_READ_CMD   MT25QL512ABB_4IO_FAST_READ_CMD
#define LX_STM32_QSPI_BULK_ERASE_CMD             MT25QL512ABB_BULK_ERASE_CMD
#define LX_STM32_QSPI_SECTOR_ERASE_CMD           MT25QL512ABB_SECTOR_ERASE_64K_CMD
#define LX_STM32_QSPI_WRITE_ENABLE_CMD           MT25QL512ABB_WRITE_ENABLE_CMD
#define LX_STM32_QSPI_READ_STATUS_REG_CMD        MT25QL512ABB_READ_STATUS_REG_CMD
#define LX_STM32_QSPI_QUAD_IN_FAST_PROG_CMD      MT25QL512ABB_EXTENDED_QUAD_INPUT_FAST_PROG_CMD
#define LX_STM32_QSPI_RESET_MEMORY_CMD           MT25QL512ABB_RESET_MEMORY_CMD
#define LX_STM32_QSPI_RESET_MAX_TIME             MT25QL512ABB_RESET_MAX_TIME
#define LX_STM32_QSPI_AUTOPOLLING_INTERVAL_TIME  MT25QL512ABB_AUTOPOLLING_INTERVAL_TIME
#define LX_STM32_QSPI_RESET_ENABLE_CMD           MT25QL512ABB_RESET_ENABLE_CMD
#define LX_STM32_QSPI_READ_VOL_CFG_REG_CMD       MT25QL512ABB_READ_VOL_CFG_REG_CMD
#define LX_STM32_QSPI_WRITE_VOL_CFG_REG_CMD      MT25QL512ABB_WRITE_VOL_CFG_REG_CMD

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

#ifdef __cplusplus
}
#endif
#endif /* LX_STM32_QSPI_DRIVER_H */
