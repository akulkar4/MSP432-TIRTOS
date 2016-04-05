/*
 * uart_task.c
 *
 *  Created on: 05-Apr-2016
 *      Author: Alok
 */

#include "Board.h"
#include <ti/drivers/UART.h>

/* XDCtools Header files */
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>

#include "uart_task.h"

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
	/*Yet to figure out clock settings. Read won't work till then*/
    char rcvBuffer[UARTBUFFERSIZE];
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
    	UART_read(uart, &rcvBuffer, UARTBUFFERSIZE);
    	rcvBuffer[0] = rcvBuffer[0] - 0x30;

    	if (rcvBuffer[0] != 218)
    	{
    		System_printf("%d ",rcvBuffer[0]);
    		System_printf("\n");
    		System_flush();
    	}


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
