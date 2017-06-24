stm32l452ceu6 demo
====

Demo program to show problem with STM32L452CEU6 running code in SRAM2.

====

This is an Eclipse project generated via STM32CubeMX.  It uses FreeRTOS to 
spawn a number of threads, only two of which are doing anything.  One thread
blinks a green LED on the PCB.  The other starts the ADC DMA process reading
into a circular buffer at 26400 sps.  These it then reads blocks of these
samples from a message queue and sends them to a FIR filter running in SRAM2.
The filter results are discarded.

This code also uses the ITM_SendChar to send updates from the ADC DMA
interrupt and from the thread running the filter.

This code will eventually end up in the hard fault handler, at which point
it will turn off the green LED and start flashing a red LED.

