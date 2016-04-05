/*
 * lcd_task.c
 *
 *  Created on: 05-Apr-2016
 *      Author: Alok
 */

#include <ti/drivers/GPIO.h>
#include <ti/drivers/SPI.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include "grlib.h"
#include "Crystalfontz128x128_ST7735.h"

#include "lcd_task.h"

/*Task 1 Struct declaration*/
Task_Struct task1Struct;
Char task1Stack[LCD_TASKSTACKSIZE];

/* ADC results buffer */
static uint16_t resultsBuffer[3];

/* Graphic library context */
Graphics_Context g_sContext;

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

void init_lcd_task()
{
	Task_Params taskParams_1;
    /* Construct uart Task  thread */
    Task_Params_init(&taskParams_1);
    taskParams_1.stackSize = LCD_TASKSTACKSIZE;
    taskParams_1.priority = 1;
    taskParams_1.stack = &task1Stack;
    taskParams_1.instance->name = "lcd";
    Task_construct(&task1Struct, (Task_FuncPtr)lcdFxn, &taskParams_1, NULL);
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
