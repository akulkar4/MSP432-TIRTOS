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
 *  ======== empty.c ========
 */
/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

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
#include "grlib.h"
#include "Crystalfontz128x128_ST7735.h"
#include <stdio.h>

#define TASKSTACKSIZE   768

/*Task 0 Struct declaration*/
Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

/*Task 1 Struct declaration*/
Task_Struct task1Struct;
Char task1Stack[TASKSTACKSIZE];

/* ADC results buffer */
static uint16_t resultsBuffer[3];
void drawTitle(void);

/* Graphic library context */
Graphics_Context g_sContext;

/*
 *  ======== uartFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
Void uartFxn(UArg arg0, UArg arg1)
{
	/*Yet to figure out clock settings. Read won't work till then*/
    char input;
    UART_Handle uart;
    UART_Params uartParams;

    /* Create a UART with data processing off. */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL)
    {
        System_abort("Error opening the UART");
    }

    /* Loop forever reading */
    while (1)
    {
         UART_read(uart, &input, 1);
        //post input to mqueue
    }
}

/*
 *  ======== lcdFxn ========
 *  Controls LCD
 *
 */
Void lcdFxn(UArg arg0, UArg arg1)
{
	/*Init Lcd*/
	Crystalfontz128x128_Init();

	/* Set default screen orientation */
	Crystalfontz128x128_SetOrientation(LCD_ORIENTATION_UP);

	Graphics_initContext(&g_sContext, &g_sCrystalfontz128x128);
	Graphics_setForegroundColor(&g_sContext, GRAPHICS_COLOR_RED);
	Graphics_setBackgroundColor(&g_sContext, GRAPHICS_COLOR_WHITE);
	GrContextFontSet(&g_sContext, &g_sFontFixed6x8);
	drawTitle();

    /* Loop forever */
    while (1)
    {
    	//read from message queue and display
    }
}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;
    Task_Params taskParams_1;

    /* Call board init functions */
    Board_initGeneral();
    Board_initGPIO();
    Board_initUART();

    //Board_initI2C();
    // Board_initSDSPI();
    // Board_initSPI();
    // Board_initWatchdog();
    // Board_initWiFi();

    /* Construct uart Task  thread */
    Task_Params_init(&taskParams);
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    taskParams.instance->name = "echo";
    Task_construct(&task0Struct, (Task_FuncPtr)uartFxn, &taskParams, NULL);

    /* Construct uart Task  thread */
    Task_Params_init(&taskParams_1);
    taskParams_1.stackSize = TASKSTACKSIZE;
    taskParams_1.stack = &task1Stack;
    taskParams_1.instance->name = "lcd";
    //Task_construct(&task1Struct, (Task_FuncPtr)lcdFxn, &taskParams_1, NULL);

    MAP_WDT_A_holdTimer();
    MAP_Interrupt_disableMaster();

   /* Initializes Clock System - This is required for the LCD */
   MAP_CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);
   MAP_CS_initClockSignal(CS_MCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
   MAP_CS_initClockSignal(CS_HSMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
   MAP_CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1 );
   MAP_CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1);

    /* Turn on user LED */
    GPIO_write(Board_LED0, Board_LED_ON);

    System_printf("Starting the example\nSystem provider is set to SysMin. "
                  "Halt the target to view any SysMin contents in ROV.\n");
    /* SysMin will only print to the console when you call flush or exit */
    System_flush();

    MAP_Interrupt_enableMaster();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

void drawTitle()
{
    Graphics_clearDisplay(&g_sContext);
    Graphics_drawStringCentered(&g_sContext,
                                    "Accelerometer:",
                                    AUTO_STRING_LENGTH,
                                    64,
                                    30,
                                    OPAQUE_TEXT);
    //drawAccelData();
}

void drawAccelData()
{
    char string[8];
    sprintf(string, "X: %5d", resultsBuffer[0]);
    Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    8,
                                    64,
                                    50,
                                    OPAQUE_TEXT);

    sprintf(string, "Y: %5d", resultsBuffer[1]);
    Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    8,
                                    64,
                                    70,
                                    OPAQUE_TEXT);

    sprintf(string, "Z: %5d", resultsBuffer[2]);
    Graphics_drawStringCentered(&g_sContext,
                                    (int8_t *)string,
                                    8,
                                    64,
                                    90,
                                    OPAQUE_TEXT);

}
