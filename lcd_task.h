/*
 * lcd_task.h
 *
 *  Created on: 05-Apr-2016
 *      Author: Alok
 */

#ifndef LCD_TASK_H_
#define LCD_TASK_H_

#define LCD_TASKSTACKSIZE 512

extern uint16_t resultsBuffer[3];
extern Graphics_Context g_sContext;
void drawTitle(void);
void init_lcd_task();
void drawAccelData();

#endif /* LCD_TASK_H_ */
