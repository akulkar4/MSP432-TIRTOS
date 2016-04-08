/*
 * uart_task.c
 *
 *  Created on: 05-Apr-2016
 *      Author: Alok
 */

#include "Board.h"
#include <string.h>

#include <ti/drivers/UART.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <xdc/cfg/global.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Mailbox.h>

#include "uart_task.h"
#include "lcd_task.h"

#define UARTBUFFERSIZE	3
/*Task 0 Struct declaration*/
Task_Struct task0Struct;
Char task0Stack[UART_TASKSTACKSIZE];

/*
 *  ======== uartFxn ========
 *  Toggle the Board_LED0. The Task_sleep is determined by arg0 which
 *  is configured for the heartBeat Task instance.
 */
Void uartFxn(UArg arg0, UArg arg1)
{
	MsgObj msg;
	/*Yet to figure out clock settings. Read won't work till then*/
    char rcvBuffer[UARTBUFFERSIZE];
    char rcvChar, clearToSend = 'c';
    uint16_t i, ten_count = 0;
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

    UART_write(uart, &clearToSend, sizeof(clearToSend));

    /* Loop forever reading */
    while (1)
    {
    	ten_count = 0;
    	memset(rcvBuffer, 10 ,sizeof(rcvBuffer));

     	for(i=0; i<=UARTBUFFERSIZE; i++)
     	{
     		UART_read(uart, &rcvChar, 1);
     		rcvChar = rcvChar - 0x30;

    		if(rcvChar != 241)
    		{
    			rcvBuffer[i] = rcvChar;
    		}

    		else
    			break;
     	}

     	for(i=0; i<UARTBUFFERSIZE; i++)
     	{
    		if(rcvBuffer[i] == 10)
    			ten_count++;
     	}

     	if(ten_count == 2)
     		msg.fatigue_val = (uint16_t)rcvBuffer[0];															//one digit sensor reading

     	else if(ten_count == 1)
     		msg.fatigue_val = (uint16_t)rcvBuffer[0]*10 + (uint16_t)rcvBuffer[1];								//two digit sensor reading

     	else if(ten_count == 0)
     		msg.fatigue_val = (uint16_t)rcvBuffer[0]*100 + (uint16_t)rcvBuffer[1]*10 + (uint16_t)rcvBuffer[2];   //three digit sensor reading

     	else
     		msg.fatigue_val = 0;  																			//failsafe

     	Mailbox_post (mailbox0, &msg, BIOS_WAIT_FOREVER);
        //post input to mqueue
    }
}

void init_uart_task()
{
	Task_Params taskParams;

    /* Construct uart Task  thread */
    Task_Params_init(&taskParams);
    taskParams.stackSize = UART_TASKSTACKSIZE;
    taskParams.priority = 2;
    taskParams.stack = &task0Stack;
    taskParams.instance->name = "echo";
    Task_construct(&task0Struct, (Task_FuncPtr)uartFxn, &taskParams, NULL);

}
