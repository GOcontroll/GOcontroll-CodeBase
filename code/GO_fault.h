/***************************************************************************************
** file         GO_fault.h
** brief        FreeRTOS fault hooks for stack overflow and malloc failure detection.
**
**---------------------------------------------------------------------------------------
**                          C O P Y R I G H T
**---------------------------------------------------------------------------------------
**  Copyright 2024 (c) by GOcontroll      http://www.gocontroll.com     All rights reserved
**
**---------------------------------------------------------------------------------------
**                            L I C E N S E
**---------------------------------------------------------------------------------------
** Permission is hereby granted, free of charge, to any person obtaining a copy of this
** software and associated documentation files (the "Software"), to deal in the Software
** without restriction, including without limitation the rights to use, copy, modify, merge,
** publish, distribute, sublicense, and/or sell copies of the Software, and to permit
** persons to whom the Software is furnished to do so, subject to the following conditions:
**
** The above copyright notice and this permission notice shall be included in all copies or
** substantial portions of the Software.
**
** THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
** INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
** PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE
** FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
** OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
** DEALINGS IN THE SOFTWARE.
**
***************************************************************************************/

#ifndef GO_FAULT_H
#define GO_FAULT_H

#ifdef GOCONTROLL_IOT

#include "FreeRTOS.h"
#include "task.h"

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName);
void vApplicationMallocFailedHook(void);

#endif /* GOCONTROLL_IOT */

#endif /* GO_FAULT_H */
