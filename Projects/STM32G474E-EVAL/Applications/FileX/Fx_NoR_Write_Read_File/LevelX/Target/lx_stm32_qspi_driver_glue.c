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

#include "lx_stm32_qspi_driver.h"

/* USER CODE BEGIN COMMENT */
/* HAL DMA mode based implementation for QuadSPI component MT25QL512ABB
 * The present implementation assumes the following settings are set for single/dual bank:

  QuadSPI single bank:
  QuadSPI Mode: Bank1 with Quad SPI Lines
  ClockPrescaler = 1;
  FifoThreshold = 4;
  SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  FlashSize = 25;
  ChipSelectHighTime = QSPI_CS_HIGH_TIME_4_CYCLE;
  ClockMode = QSPI_CLOCK_MODE_0;
  FlashID = QSPI_FLASH_ID_1;
  DualFlash = QSPI_DUALFLASH_DISABLE;

  QuadSPI dual bank:
  QuadSPI Mode: Dual Bank with Quad SPI Lines
  ClockPrescaler = 1;
  FifoThreshold = 4;
  SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  FlashSize = 26;
  ChipSelectHighTime = QSPI_CS_HIGH_TIME_4_CYCLE;
  ClockMode = QSPI_CLOCK_MODE_0;
  FlashID = QSPI_FLASH_ID_1;
  DualFlash = QSPI_DUALFLASH_ENABLE;

 * Different configuration can be used but need to be reflected in
 * the implementation guarded with QSPI_HAL_CFG_xxx user tags.
 */
 /* USER CODE END COMMENT */

TX_SEMAPHORE qspi_tx_semaphore;
TX_SEMAPHORE qspi_rx_semaphore;

extern QSPI_HandleTypeDef hqspi1;

#if (LX_STM32_QSPI_INIT == 1)
extern void MX_QUADSPI1_Init(void);
#endif

/* USER CODE BEGIN DUAL_BANK_FLAG */
/* The following flag is being to notify if QSPI is single or dual bank */
static int8_t is_dual_bank_enabled = 0;
/* USER CODE BEGIN DUAL_BANK_FLAG */

static uint8_t qspi_dummy_cyles_configure   (QSPI_HandleTypeDef *quadspi_handle);
static uint8_t qspi_set_write_enable        (QSPI_HandleTypeDef *quadspi_handle);
static uint8_t qspi_auto_polling_ready      (QSPI_HandleTypeDef *quadspi_handle, uint32_t timeout);

/* USER CODE BEGIN SECTOR_BUFFER */
ULONG qspi_sector_buffer[LX_STM32_QSPI_SECTOR_SIZE / sizeof(ULONG)];
/* USER CODE END SECTOR_BUFFER */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
* @brief Initializes the QSPI IP instance
* @param UINT instance QSPI instance to initialize
* @retval 0 on success error value otherwise
*/
INT lx_stm32_qspi_lowlevel_init(UINT instance)
{
  INT status = 0;

  /* USER CODE BEGIN PRE_QSPI_Init */
  UNUSED(instance);
  /* USER CODE END PRE_QSPI_Init */

#if (LX_STM32_QSPI_INIT == 1)
  /* Init the QSPI */
  MX_QUADSPI1_Init();
#endif

  /* Set Flag to notify if QSPI dual bank is enabled */
  if(QSPI_DUALFLASH_ENABLE == hqspi1.Init.DualFlash)
  {
    is_dual_bank_enabled = 1;
  }

  /* USER CODE BEGIN POST_QSPI_Init */

  /* USER CODE END POST_QSPI_Init */

  return status;
}

/**
  * @brief  De-Initializes the QSPI interface.
  * @param  Instance QSPI Instance
  * @retval  0 on Success 1 on Failure
  */
INT lx_stm32_qspi_lowlevel_deinit(UINT instance)
{
  INT status = 0;

  /* USER CODE BEGIN PRE_QSPI_DeInit */
  UNUSED(instance);
  /* USER CODE END PRE_QSPI_DeInit */

  /*Delete TX/RX semaphores*/
  tx_semaphore_delete(&qspi_tx_semaphore);
  tx_semaphore_delete(&qspi_rx_semaphore);

  /* Call the DeInit function to reset the driver */
  if (HAL_QSPI_DeInit(&hqspi1) != HAL_OK)
  {
    status = 1;
  }

  /* USER CODE BEGIN POST_QSPI_DeInit */

  /* USER CODE END POST_QSPI_DeInit */

  return status;
}

/**
* @brief Get the status of the QSPI instance
* @param UINT instance QSPI instance
* @retval 0 if the QSPI is ready 1 otherwise
*/
INT lx_stm32_qspi_get_status(UINT instance)
{
  INT status = 0;

  /* USER CODE BEGIN PRE_QSPI_Get_Status */
  UNUSED(instance);
  /* USER CODE END PRE_QSPI_Get_Status */

  QSPI_CommandTypeDef s_command;
  uint8_t reg[2];

  /* Initialize the read flag status register command */
  /* USER CODE BEGIN QSPI_HAL_CFG_GetStatus */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = LX_STM32_QSPI_READ_STATUS_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  if (is_dual_bank_enabled)
  {
    s_command.NbData          = 2U;
  }
  else {
    s_command.NbData          = 1U;
  }
  /* USER CODE END QSPI_HAL_CFG_GetStatus */

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi1, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    status = 1;
  }
  /* Receive the data */
  else if (HAL_QSPI_Receive(&hqspi1, reg, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    status = 1;
  }
  /* Check the value of the register */
  else if ((reg[0] & LX_STM32_QSPI_SR_WIP) != 0U)
  {
    status = 1;
  }
  else if ((is_dual_bank_enabled) && ((reg[1] & LX_STM32_QSPI_SR_WIP) != 0U))
  {
    status = 1;
  }
  else
  {
    status = 0;
  }

  /* USER CODE BEGIN POST_QSPI_Get_Status */

  /* USER CODE END POST_QSPI_Get_Status */

  return status;
}

/**
* @brief Get size info of the flash memory
* @param UINT instance QSPI instance
* @param ULONG * block_size pointer to be filled with Flash block size
* @param ULONG * total_blocks pointer to be filled with Flash total number of blocks
* @retval 0 on Success and block_size and total_blocks are correctly filled
          1 on Failure, block_size = 0, total_blocks = 0
*/
INT lx_stm32_qspi_get_info(UINT instance, ULONG *block_size, ULONG *total_blocks)
{
  INT status = 0;

  /* USER CODE BEGIN PRE_QSPI_Get_Info */
  UNUSED(instance);
  /* USER CODE END PRE_QSPI_Get_Info */

  *block_size = LX_STM32_QSPI_SECTOR_SIZE;
  *total_blocks = (LX_STM32_QSPI_FLASH_SIZE / LX_STM32_QSPI_SECTOR_SIZE);

  /* USER CODE BEGIN POST_QSPI_Get_Info */

  /* USER CODE END POST_QSPI_Get_Info */

  return status;
}

/**
* @brief Read data from the QSPI memory into a buffer
* @param UINT instance QSPI instance
* @param ULONG * address the start address to read from
* @param ULONG * buffer the destination buffer
* @param ULONG words the total number of words to be read
* @retval 0 on Success 1 on Failure
*/
INT lx_stm32_qspi_read(UINT instance, ULONG *address, ULONG *buffer, ULONG words)
{
  INT status = 0;

  /* USER CODE BEGIN PRE_QSPI_Read */
  UNUSED(instance);
  /* USER CODE END PRE_QSPI_Read */

  QSPI_CommandTypeDef s_command;

  /* Initialize the read command */
  /* USER CODE BEGIN QSPI_HAL_CFG_READ_1 */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = LX_STM32_QSPI_QUAD_INOUT_FAST_READ_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
  s_command.DummyCycles       = LX_STM32_QSPI_DUMMY_CYCLES_READ_QUAD;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
  s_command.Address           = (uint32_t)address;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.NbData            = (uint32_t) words * sizeof(ULONG);
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  /* USER CODE END QSPI_HAL_CFG_READ_1 */

  /* Configure Volatile Configuration register (with new dummy cycles) */
  qspi_dummy_cyles_configure(&hqspi1);

  /* Configure the command */
  if (HAL_QSPI_Command(&hqspi1, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    status = 1;
  }
  /* Reception of the data */
  else if (HAL_QSPI_Receive_DMA(&hqspi1, (uint8_t *)buffer) != HAL_OK)
  {
    status = 1;
  }
  /*Reading success */
  else
  {
    status = 0;
  }

  /* USER CODE BEGIN POST_QSPI_Read */

  /* USER CODE END POST_QSPI_Read */

  return status;
}

/**
* @brief write a data buffer into the QSPI memory
* @param UINT instance QSPI instance
* @param ULONG * address the start address to write into
* @param ULONG * buffer the data source buffer
* @param ULONG words the total number of words to be written
* @retval 0 on Success 1 on Failure
*/
INT lx_stm32_qspi_write(UINT instance, ULONG *address, ULONG *buffer, ULONG words)
{
  INT status = 0;

  /* USER CODE BEGIN PRE_QSPI_Write */
  UNUSED(instance);
  /* USER CODE END PRE_QSPI_Write */

  QSPI_CommandTypeDef s_command;
  uint32_t end_addr, current_size, current_addr;
  uint8_t* data_buffer = (uint8_t *) buffer;

  /* Calculation of the size between the write address and the end of the page */
  current_size = LX_STM32_QSPI_PAGE_SIZE - ((uint32_t) address % LX_STM32_QSPI_PAGE_SIZE);

  /* Check if the size of the data is less than the remaining place in the page */
  if (current_size > ((uint32_t) words * sizeof(ULONG)))
  {
    current_size = (uint32_t) words * sizeof(ULONG);
  }

  /* Initialize the address variables */
  current_addr = (uint32_t) address;
  end_addr = (uint32_t) address + (uint32_t) words * sizeof(ULONG);

  /* Initialize the program command */
  /* USER CODE BEGIN QSPI_HAL_CFG_WRITE */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = LX_STM32_QSPI_QUAD_IN_FAST_PROG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_4_LINES;
  s_command.DataMode          = QSPI_DATA_4_LINES;
  s_command.AddressSize       =  QSPI_ADDRESS_24_BITS;
  s_command.Address           = (uint32_t) address;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.NbData            = (uint32_t)words * sizeof(ULONG);
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  /* USER CODE END QSPI_HAL_CFG_WRITE */

  /* Perform the write page by page */
  do
  {
    s_command.Address = current_addr;
    s_command.NbData  = current_size;

    /* Check flash busy */
    if(qspi_auto_polling_ready(&hqspi1,HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != 0)
    {
      status = 1;
    }
    /* Enable write operations */
    else if (qspi_set_write_enable(&hqspi1) != 0)
    {
      status = 1;
    }
    /* Configure the command */
    else if (HAL_QSPI_Command(&hqspi1, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      status = 1;
    }
    /* Transmission of the data */
    else if (HAL_QSPI_Transmit_DMA(&hqspi1, data_buffer) != HAL_OK)
    {
      status = 1;
    }
    /* Check success of the transmission of the data */
    else if(tx_semaphore_get(&qspi_tx_semaphore, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != TX_SUCCESS)
    {
     status = 1;
    }
    /* Configure automatic polling mode to wait for end of program */
    else if (qspi_auto_polling_ready(&hqspi1, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != 0)
    {
      status = 1;
    }
    /* Update the address and size variables for next page programming */
    else
    {
      current_addr += current_size;
      data_buffer += current_size;
      current_size = ((current_addr + LX_STM32_QSPI_PAGE_SIZE) > end_addr) ? (end_addr - current_addr) : LX_STM32_QSPI_PAGE_SIZE;
    }
  } while ((current_addr < end_addr) && (!status));

  /* Release qspi_tx_semaphore in case of writing success */
  if (status == 0)
  {
    tx_semaphore_put(&qspi_tx_semaphore);
  }

  /* USER CODE BEGIN POST_QSPI_Write */

  /* USER CODE END POST_QSPI_Write */

  return status;
}

/**
* @brief Erase the whole flash or a single block
* @param UINT instance QSPI instance
* @param ULONG  block the block to be erased
* @param ULONG  erase_count the number of times the block was erased
* @param UINT full_chip_erase if set to 0 a single block is erased otherwise the whole flash is erased
* @retval 0 on Success 1 on Failure
*/
INT lx_stm32_qspi_erase(UINT instance, ULONG block, ULONG erase_count, UINT full_chip_erase)
{
  INT status = 0;

  /* USER CODE BEGIN PRE_QSPI_Erase */
  UNUSED(instance);
  /* USER CODE END PRE_QSPI_Erase */

  QSPI_CommandTypeDef s_command;
  uint32_t eraseTimeout;

  /* Initialize the erase command */
  /* USER CODE BEGIN QSPI_HAL_CFG_ERASE */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;

  if(full_chip_erase) {
    s_command.Instruction       = LX_STM32_QSPI_BULK_ERASE_CMD;
    s_command.AddressMode       = QSPI_ADDRESS_NONE;
    eraseTimeout                = LX_STM32_QSPI_BULK_ERASE_MAX_TIME;
  } else {
    s_command.Instruction       = LX_STM32_QSPI_SECTOR_ERASE_CMD;
    s_command.AddressMode       = QSPI_ADDRESS_1_LINE;
    s_command.AddressSize       = QSPI_ADDRESS_24_BITS;
    s_command.Address           = block * LX_STM32_QSPI_SECTOR_SIZE;
    eraseTimeout                = LX_STM32_QSPI_SECTOR_ERASE_MAX_TIME;
  }
  /* USER CODE END QSPI_HAL_CFG_ERASE */

  /* Check Flash busy */
  if(qspi_auto_polling_ready(&hqspi1,HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != 0)
  {
    status = 1;
  }
  /* Enable write operations */
  else if (qspi_set_write_enable(&hqspi1) != 0)
  {
    status = 1;
  }
  /* Send the command */
  else if (HAL_QSPI_Command(&hqspi1, &s_command, eraseTimeout)!= HAL_OK)
  {
    status = 1;
  }
  /* Configure automatic polling mode to wait for end of erase */
  else if (qspi_auto_polling_ready(&hqspi1, eraseTimeout) != 0)
  {
    status = 1;
  }

  /* USER CODE BEGIN POST_QSPI_Erase */

  /* USER CODE END POST_QSPI_Erase */

  return status;
}

/**
* @brief Check that a block was actually erased
* @param UINT instance QSPI instance
* @param ULONG  block the block to be checked
* @retval 0 on Success 1 on Failure
*/
INT lx_stm32_qspi_is_block_erased(UINT instance, ULONG block)
{
  INT status = 0;

  /* USER CODE BEGIN QSPI_Block_Erased */

  /* USER CODE END QSPI_Block_Erased */

  return status;
}

/**
* @brief Return QSPI driver system error code
* @param UINT error_code the QSPI driver system error code
* @retval error_code on Failure and 0 on Success
*/
UINT  lx_qspi_driver_system_error(UINT error_code)
{
  UINT status = LX_ERROR;

  /* USER CODE BEGIN QSPI_System_Error */

  /* USER CODE END QSPI_System_Error */

  return status;
}

/**
  * @brief  Configure the dummy cycles on memory side.
  * @param  quadspi_handle: QSPI handle pointer
  */
static uint8_t qspi_dummy_cyles_configure(QSPI_HandleTypeDef *quadspi_handle)
{
  uint8_t status = 0;
  QSPI_CommandTypeDef s_command;
  uint8_t reg[2];

  /* Initialize the read volatile configuration register command */
  /* USER CODE BEGIN QSPI_HAL_CFG_DUMMY_CYCLE_1 */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = LX_STM32_QSPI_READ_VOL_CFG_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DummyCycles       = 0;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  /* USER CODE END QSPI_HAL_CFG_DUMMY_CYCLE_1 */
  if (is_dual_bank_enabled)
  {
    s_command.NbData          = 2U;
  }
  else
  {
    s_command.NbData          = 1U;
  }
  /* Configure the command */
  if (HAL_QSPI_Command(quadspi_handle, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    status = 1;
  }
  /* Reception of the data */
  else if (HAL_QSPI_Receive(quadspi_handle, reg, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    status = 1;
  }
  /* Enable write operations */
  else if (qspi_set_write_enable(quadspi_handle) != 0)
  {
    status = 1;
  }
  else
  {
    /* Write Volatile Configuration register (with new dummy cycles) */
    /* USER CODE BEGIN QSPI_HAL_CFG_DUMMY_CYCLE_2 */
    s_command.Instruction = LX_STM32_QSPI_WRITE_VOL_CFG_REG_CMD;
    MODIFY_REG(reg[0], 0xF0, (4 << POSITION_VAL(0xF0)));

    if (is_dual_bank_enabled)
    {
      MODIFY_REG(reg[1], 0xF0, (4 << POSITION_VAL(0xF0)));
    }
    /* USER CODE END QSPI_HAL_CFG_DUMMY_CYCLE_2 */

    /* Configure the write volatile configuration register command */
    if (HAL_QSPI_Command(quadspi_handle, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      status = 1;
    }
    /* Transmission of the data */
    else if (HAL_QSPI_Transmit(quadspi_handle, reg, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      status = 1;
    }
  }

  return status;
}

/**
  * @brief  Send a Write Enable command and wait it is effective.
  * @param  quadspi_handle: QSPI handle pointer
  */
static uint8_t qspi_set_write_enable(QSPI_HandleTypeDef *quadspi_handle)
{
  uint8_t status = 0;
  QSPI_CommandTypeDef     s_command;
  QSPI_AutoPollingTypeDef s_config;

  /* Enable write operations */
  /* USER CODE BEGIN QSPI_HAL_CFG_WRITE_ENABLE_1 */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = LX_STM32_QSPI_WRITE_ENABLE_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0;
  s_command.DataMode          = QSPI_DATA_NONE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  /* USER CODE END QSPI_HAL_CFG_WRITE_ENABLE_1 */

  if (HAL_QSPI_Command(quadspi_handle, &s_command, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    status = 1;
  }

  /* Configure automatic polling mode to wait for write enabling */
  /* USER CODE BEGIN QSPI_HAL_CFG_WRITE_ENABLE_2 */
  /* Configure automatic polling mode to wait for write enabling */
  s_config.MatchMode       = QSPI_MATCH_MODE_AND;
  s_config.Interval        = LX_STM32_QSPI_AUTOPOLLING_INTERVAL_TIME;
  s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;

  if (is_dual_bank_enabled)
  {
    s_config.Match           = (LX_STM32_QSPI_SR_WEN << 8) | LX_STM32_QSPI_SR_WEN;
    s_config.Mask            = (LX_STM32_QSPI_SR_WEN << 8) | LX_STM32_QSPI_SR_WEN;
    s_config.StatusBytesSize = 2U;
  }
  else
  {
    s_config.Match           = LX_STM32_QSPI_SR_WEN;
    s_config.Mask            = LX_STM32_QSPI_SR_WEN;
    s_config.StatusBytesSize = 1U;
  }

  s_command.Instruction    = LX_STM32_QSPI_READ_STATUS_REG_CMD;
  s_command.DataMode       = QSPI_DATA_1_LINE;
  /* USER CODE END QSPI_HAL_CFG_WRITE_ENABLE_2 */

  if (HAL_QSPI_AutoPolling(quadspi_handle, &s_command, &s_config, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    status = 1;
  }

  return status;
}

/**
  * @brief  Read the SR of the memory and wait the EOP.
  * @param  quadspi_handle: QSPI handle pointer
  * @param  timeout: timeout value before returning an error
  */
static uint8_t qspi_auto_polling_ready(QSPI_HandleTypeDef *quadspi_handle, uint32_t timeout)
{
  uint8_t status = 0;
  QSPI_CommandTypeDef     s_command;
  QSPI_AutoPollingTypeDef s_config;

  /* Configure automatic polling mode to wait for memory ready */
  /* USER CODE BEGIN QSPI_HAL_CFG_MEM_READY */
  s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;
  s_command.Instruction       = LX_STM32_QSPI_READ_STATUS_REG_CMD;
  s_command.AddressMode       = QSPI_ADDRESS_NONE;
  s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  s_command.DummyCycles       = 0U;
  s_command.DataMode          = QSPI_DATA_1_LINE;
  s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;
  s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;
  s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;
  /* USER CODE END QSPI_HAL_CFG_MEM_READY */

  s_config.Match              = 0;
  s_config.MatchMode          = QSPI_MATCH_MODE_AND;
  s_config.Interval           = LX_STM32_QSPI_AUTOPOLLING_INTERVAL_TIME;
  s_config.AutomaticStop      = QSPI_AUTOMATIC_STOP_ENABLE;

  if (is_dual_bank_enabled)
  {
    s_config.Mask             = ((LX_STM32_QSPI_SR_WIP << 8) | LX_STM32_QSPI_SR_WIP);
    s_config.StatusBytesSize  = 2U;
  }
  else
  {
    s_config.Mask             = LX_STM32_QSPI_SR_WIP;
    s_config.StatusBytesSize  = 1U;
  }

  if (HAL_QSPI_AutoPolling(quadspi_handle, &s_command, &s_config, timeout) != HAL_OK)
  {
    status = 1;
  }

  return status;
}

/**
  * @brief  Rx Transfer completed callbacks.
  * @param  hqspi QSPI handle
  * @retval None
  */
void HAL_QSPI_RxCpltCallback(QSPI_HandleTypeDef *quadspi_handle)
{
  /* USER CODE BEGIN PRE_RX_CMPLT */

  /* USER CODE END PRE_RX_CMPLT */

  tx_semaphore_put(&qspi_rx_semaphore);

  /* USER CODE BEGIN POST_RX_CMPLT */

  /* USER CODE END POST_RX_CMPLT */
}

/**
  * @brief  Tx Transfer completed callbacks.
  * @param  hqspi QSPI handle
  * @retval None
  */
void HAL_QSPI_TxCpltCallback(QSPI_HandleTypeDef *quadspi_handle)
{
  /* USER CODE BEGIN PRE_TX_CMPLT */

  /* USER CODE END PRE_TX_CMPLT */

  tx_semaphore_put(&qspi_tx_semaphore);

  /* USER CODE BEGIN POST_TX_CMPLT */

  /* USER CODE END POST_TX_CMPLT */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
