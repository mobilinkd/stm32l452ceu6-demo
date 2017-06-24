stm32l452ceu6 demo
====

Demo program to show problem with STM32L452CEU6 running code in SRAM2.

====

This is an Eclipse project generated via STM32CubeMX.  It uses FreeRTOS to 
spawn a number of threads, only two of which are doing anything.  One thread
blinks a green LED on the PCB.  The other starts the ADC DMA process reading
into a circular buffer at 26400 sps.  These are placed on a message queue in
the DMA interrupt.  The thread then each block of these samples from the
queue and sends them to a FIR filter running in SRAM2.  The filter results
are discarded.

This code uses the ITM_SendChar to send updates from the ADC DMA interrupt
and from the thread running the filter.

This code will eventually end up in the hard fault handler, at which point
it will turn off the green LED and start flashing a red LED.

Versions of this code seems to run fine on an STM32L452RE Nucleo64 board and
on the same PCB this was tested with using an STM32L433CCU6 chip.  The code
will run indefinitely when the FIR filter code is placed in Flash.

Placing Code in SRAM2

====

1. The section "bss2" (yes, poorly named) is defined in the
STM32L452CE_FLASH.ld linker script.
2. Code and data is copied to SRAM2 by the ./Src/startup_stm32l452xx.S
startup code.
3. The FIR filter code is placed in the bss2 section by modifying
./Drivers/CMSIS/Include/arm_math.h:

    void arm_fir_f32(
      const arm_fir_instance_f32 * S,
      float32_t * pSrc,
      float32_t * pDst,
      uint32_t blockSize) __attribute__((section(".bss2")));

