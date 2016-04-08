/*
 * lcd_task.h
 *
 *  Created on: 05-Apr-2016
 *      Author: Alok
 */

#ifndef LCD_TASK_H_
#define LCD_TASK_H_

#define LCD_TASKSTACKSIZE 1024

//extern Graphics_Context g_sContext;
void drawTitle(void);
void init_lcd_task();

typedef struct MsgObj
{
	uint16_t fatigue_val;            		// message value
} MsgObj;

typedef struct AccelData
{
	uint16_t x_value;            		// message value
	uint16_t y_value;            		// message value
	uint16_t z_value;            		// message value
} AccData;


#endif /* LCD_TASK_H_ */
