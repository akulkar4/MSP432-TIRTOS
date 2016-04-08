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
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
// #include <ti/drivers/Watchdog.h>
// #include <ti/drivers/WiFi.h>

/* Board Header file */
#include "Board.h"

#include "Crystalfontz128x128_ST7735.h"
#include "grlib.h"

#include <stdio.h>
#include <stdlib.h>
#include "uart_task.h"
#include "lcd_task.h"

void ADC_Handler(void);
void ADC_Process_Data(void);

uint32_t count = 0;
/* Timer_A Continuous Mode Configuration Parameter */
const Timer_A_UpModeConfig upModeConfig =
{
        TIMER_A_CLOCKSOURCE_ACLK,            // MCLK Clock Source
        TIMER_A_CLOCKSOURCE_DIVIDER_1,       // MCLK/1 = 3MHz
        1024,
        TIMER_A_TAIE_INTERRUPT_DISABLE,      // Disable Timer ISR
        TIMER_A_CCIE_CCR0_INTERRUPT_DISABLE, // Disable CCR0
        TIMER_A_DO_CLEAR                     // Clear Counter
};

/* Timer_A Compare Configuration Parameter */
const Timer_A_CompareModeConfig compareConfig =
{
        TIMER_A_CAPTURECOMPARE_REGISTER_1,          // Use CCR1
        TIMER_A_CAPTURECOMPARE_INTERRUPT_DISABLE,   // Disable CCR interrupt
        TIMER_A_OUTPUTMODE_SET_RESET,               // Toggle output but
        1024,                                       // 16000 Period
};


/*
 *  ======== main ========
 */
int main(void)

{
    // Call board init functions
    Board_initGeneral();
    Board_initGPIO();
    Board_initUART();

    // Setup LED Pin
	MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
	MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

	//Set up push-button
	MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);

    /*UART Task*/
    init_uart_task();

    /*LCD Task*/
    init_lcd_task();

    //Watchdog Timer Stop!!
    MAP_WDT_A_holdTimer();

    //Disable interrupts before you start configuring
    MAP_Interrupt_disableMaster();

   /* Initializes Clock System - This is required for the LCD */
   MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
   MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
   MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
   MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
   MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

   // Configures Pin 4.0, 4.2, and 6.1 (A11,A13,A14)as ADC input
   MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN0 | GPIO_PIN2, GPIO_TERTIARY_MODULE_FUNCTION);
   MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P6, GPIO_PIN1, GPIO_TERTIARY_MODULE_FUNCTION);

   //Initializing ADC (ADCOSC/64/8)
   MAP_ADC14_enableModule();
   MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,0);

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

   //Configuring Timer_A1 in continuous mode and sourced from ACLK
   MAP_Timer_A_configureUpMode(TIMER_A1_BASE, &upModeConfig);

   // Configuring Timer_A1 in CCR1 to trigger at 16000 (0.5s)
   MAP_Timer_A_initCompare(TIMER_A1_BASE, &compareConfig);

   // Configuring the sample trigger to be sourced from Timer_A1  and setting it
   // to automatic iteration after it is triggereds
   MAP_ADC14_setSampleHoldTrigger(ADC_TRIGGER_SOURCE3, false);

   //Enabling the interrupt when a conversion on channel 2 (end of sequence)
   //is complete and enabling conversions
   MAP_ADC14_enableInterrupt(ADC_INT2);

   // Triggering the start of the sample
   MAP_ADC14_enableConversion();

   // Enabling Interrupts for the ADC14
   MAP_Interrupt_enableInterrupt(INT_ADC14);

   // Configuration is done, let the interrupts commence
      MAP_Interrupt_enableMaster();

   System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");

    // SysMin will only print to the console when you call flush or exit
   System_flush();

    // Starting the Timer_A0
   MAP_Timer_A_startCounter(TIMER_A1_BASE, TIMER_A_UP_MODE);

    // Start BIOS on push button press, TI-RTOS takes control after this call
   while(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) != GPIO_INPUT_PIN_LOW){}
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
	static AccData acc_avg;
	static uint32_t acc_x_raw,acc_y_raw,acc_z_raw;
	static uint32_t prev_x,prev_y,prev_z;
	//Fresh start of a second, go ahead and change all the values.
	if(count == 0)
	{
		acc_avg.x_value = 0;
		acc_avg.y_value = 0;
		acc_avg.z_value = 0;
	}

	acc_x_raw = ADC14_getResult(ADC_MEM0);
	acc_y_raw = ADC14_getResult(ADC_MEM1);
	acc_z_raw = ADC14_getResult(ADC_MEM2);

	if(count != 0)
	{
		acc_avg.x_value += abs(prev_x - acc_x_raw);
		acc_avg.y_value += abs(prev_y - acc_y_raw);
		acc_avg.z_value += abs(prev_z - acc_z_raw);
	}

	count++;
	prev_x = acc_x_raw;
	prev_y = acc_y_raw;
	prev_z = acc_z_raw;
	if(count == 16)
	{
		acc_avg.x_value = acc_avg.x_value/15;
		acc_avg.y_value = acc_avg.y_value/15;
		acc_avg.z_value = acc_avg.z_value/15;
		count = 0;

		Mailbox_post (mailbox1, &acc_avg, BIOS_NO_WAIT);
	}
}
