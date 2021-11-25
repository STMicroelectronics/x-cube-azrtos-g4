## <b>Fx_NoR_Write_Read_File application description</b>

This application provides an example of Azure RTOS FileX and LevelX stacks usage on STM32G474E-EVAL board, it demonstrates how to create

a Fat File system on the NOR flash using FileX alongside LevelX. The application is designed to execute file operations on the MT25QL512ABB

NOR flash device, the code provides all required software code for properly managing it.

The application starts by calling the ThreadX's initialization routine which executes the main thread that handles file operations. At this stage,

all FileX resources are created, the MT25QL512ABB driver is initialized and a single thread is created:

  - fx_thread (Prio : 10; PreemptionPrio : 10) used for file operations.

The fx_thread will start by erasing the NOR flash then formatting it using FileX services. The resulting file system is a FAT32 compatible, with 512 bytes per sector

and 8 sectors per cluster.

The NOR Flash erase operation may take a while, do not interrupt it.

The following flags are enabled in the lx_stm32_qspi_driver.h:

  - LX_STM32_QSPI_INIT
  - LX_STM32_QSPI_ERASE

Upon successful opening of the flash media, FileX continue with creating a file called "STM32.TXT" into the root directory, then write into it

some dummy data. Then file is re-opened in read only mode and content is checked.

Through all the steps, FileX/LevelX services are called to print the flash size available before and after the example file is written into the flash.

The number of occupied sectors is also shown.

#### <b>Expected success behavior</b>

Successful operation is marked by a toggeling green LED light.

Also, information regarding the total and available size of the flash media is printed to the serial port.

#### <b>Error behaviors</b>

On failure, the red LED starts toggeling while the green LED is switched OFF.

#### <b>Assumptions if any</b>
None

#### <b>Known limitations</b>
When using the STM32CubeProgrammer to erase the NOR flash, make sure to power-off, power-on the board before running the application.

#### <b>ThreadX usage hints</b>

 - ThreadX uses the Systick as time base, thus it is mandatory that the HAL uses a separate time base through the TIM IPs.
 - ThreadX is configured with 100 ticks/sec by default, this should be taken into account when using delays or timeouts at application. It is always possible to reconfigure it in the "tx_user.h", the "TX_TIMER_TICKS_PER_SECOND" define,but this should be reflected in "tx_initialize_low_level.s" file too.
 - ThreadX is disabling all interrupts during kernel start-up to avoid any unexpected behavior, therefore all system related calls (HAL, BSP) should be done either at the beginning of the application or inside the thread entry functions.
 - ThreadX offers the "tx_application_define()" function, that is automatically called by the tx_kernel_enter() API.
   It is highly recommended to use it to create all applications ThreadX related resources (threads, semaphores, memory pools...)  but it should not in any way contain a system API call (HAL or BSP).
 - Using dynamic memory allocation requires to apply some changes to the linker file. ThreadX needs to pass a pointer to the first free memory location in RAM to the tx_application_define() function, using the "first_unused_memory" argument. This require changes in the linker files to expose this memory location.
    + For EWARM add the following section into the .icf file:
   ```
	 place in RAM_region    { last section FREE_MEM };
	 ```
    + For MDK-ARM:
	```
    either define the RW_IRAM1 region in the ".sct" file
    or modify the line below in "tx_low_level_initilize.s to match the memory region being used
        LDR r1, =|Image$$RW_IRAM1$$ZI$$Limit|
	```
    + For STM32CubeIDE add the following section into the .ld file:
	```
    ._threadx_heap :
      {
         . = ALIGN(8);
         __RAM_segment_used_end__ = .;
         . = . + 64K;
         . = ALIGN(8);
       } >RAM_D1 AT> RAM_D1
	```

       The simplest way to provide memory for ThreadX is to define a new section, see ._threadx_heap above.
       In the example above the ThreadX heap size is set to 64KBytes.
       The ._threadx_heap must be located between the .bss and the ._user_heap_stack sections in the linker script.
       Caution: Make sure that ThreadX does not need more than the provided heap memory (64KBytes in this example).
       Read more in STM32CubeIDE User Guide, chapter: "Linker script".

    + The "tx_initialize_low_level.s" should be also modified to enable the "USE_DYNAMIC_MEMORY_ALLOCATION" flag.

#### <b>FileX/LevelX usage hints</b>

- When calling the fx_media_format() API, it is highly recommended to understand all the parameters used by the API to correctly generate a valid filesystem.
- FileX is using data buffers, passed as arguments to fx_media_open(), fx_media_read() and fx_media_write() API it is recommended that these buffers are multiple of sector size and "4 bytes" aligned to avoid unaligned access issues.

### <b>Keywords</b>

RTOS, ThreadX, FileX, LevelX, File System, NOR, QSPI, FAT32

### <b>Hardware and Software environment</b>

  - This application runs on STM32G4xx devices.
  - This application has been tested with STMicroelectronics STM32G474E-EVAL board MB1397 Revision B-05
    and can be easily tailored to any other supported device and development board.

  - This application uses USART1 to display logs, the hyperterminal configuration is as follows:

      - BaudRate = 115200 baud
      - Word Length = 8 Bits
      - Stop Bit = 1
      - Parity = none
      - Flow control = None

### <b>How to use it ?</b>

In order to make the program work, you must do the following :

 - Open your preferred toolchain
 - Rebuild all files and load your image into target memory
 - Run the application
