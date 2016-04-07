/*
 * Copyright (c) 2015, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== main.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h> 				//header file for statically defined objects/handles

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

/* TI-RTOS Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
// #include <ti/drivers/SDSPI.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

/* Board Header file */
#include "Board.h"
//#include "msp.h"
//#include <driverlib.h>

#include "Crystalfontz128x128_ST7735.h"
#include "grlib.h"

#include <stdio.h>
#include "uart_task.h"
#include "lcd_task.h"


void ADC_Handler(void);
void ADC_Process_Data(void);
/*
 *  ======== main ========
 */
int main(void)

{
    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initUART();

    //Board_initI2C();
    // Board_initSDSPI();
    // Board_initSPI();
    // Board_initWatchdog();
    // Board_initWiFi();

    /*UART Task*/
    init_uart_task();

    /*LCD Task*/
    init_lcd_task();

    MAP_WDT_A_holdTimer();
    MAP_Interrupt_disableMaster();

   /* Initializes Clock System - This is required for the LCD */
   MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
   MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
   MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
   MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
   MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

   /* Initializing ADC (ADCOSC/64/8) */
       MAP_ADC14_enableModule();
       MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_64, ADC_DIVIDER_8,
               0);

       /* Configuring ADC Memory (ADC_MEM0 - ADC_MEM2 (A11, A13, A14)  with no repeat)
                * with internal 2.5v reference */
           MAP_ADC14_configureMultiSequenceMode(ADC_MEM0, ADC_MEM2, true);
           MAP_ADC14_configureConversionMemory(ADC_MEM0,
                   ADC_VREFPOS_AVCC_VREFNEG_VSS,
                   ADC_INPUT_A14, ADC_NONDIFFERENTIAL_INPUTS);

           MAP_ADC14_configureConversionMemory(ADC_MEM1,
                   ADC_VREFPOS_AVCC_VREFNEG_VSS,
                   ADC_INPUT_A13, ADC_NONDIFFERENTIAL_INPUTS);

           MAP_ADC14_configureConversionMemory(ADC_MEM2,
                   ADC_VREFPOS_AVCC_VREFNEG_VSS,
                   ADC_INPUT_A11, ADC_NONDIFFERENTIAL_INPUTS);

           /* Enabling the interrupt when a conversion on channel 2 (end of sequence)
            *  is complete and enabling conversions */
           MAP_ADC14_enableInterrupt(ADC_INT2);

           /* Enabling Interrupts */
           MAP_Interrupt_enableInterrupt(INT_ADC14);

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    MAP_Interrupt_enableMaster();

    /* Setting up the sample timer to automatically step through the sequence
        * convert.
        */
       MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);

       /* Triggering the start of the sample */
       MAP_ADC14_enableConversion();
       MAP_ADC14_toggleConversionTrigger();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

void ADC_Handler(void)
{
	uint64_t status;
	status = MAP_ADC14_getEnabledInterruptStatus();
	MAP_ADC14_clearInterruptFlag(status);

	 if(status & ADC_INT2)
		 Swi_post(swi0);
}

void ADC_Process_Data(void)
{
	/* Make the buffer here local, pass it on through the Mailbox*/
	/* Apparently you can call draw title here? */

	/* Store ADC14 conversion results */
	resultsBuffer[0] = ADC14_getResult(ADC_MEM0);
	resultsBuffer[1] = ADC14_getResult(ADC_MEM1);
	resultsBuffer[2] = ADC14_getResult(ADC_MEM2);
}
