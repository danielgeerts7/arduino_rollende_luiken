/*
 * schedular.h
 *
 *  Author: Hanzehogeschool
 */

#include <avr/io.h>
#include <avr/interrupt.h>

// Maximum number of tasks
#define SCH_MAX_TASKS (5)

#ifndef SCHEDULAR_H_INCLUDED
#define SCHEDULAR_H_INCLUDED

// Scheduler data structure for storing task data
typedef struct
{
	// Pointer to task
	void (* pTask)(void);
	// Initial delay in ticks
	unsigned int Delay;
	// Periodic interval in ticks
	unsigned int Period;
	// Runme flag (indicating when the task is due to run)
	unsigned char RunMe;
} sTask;

// Function prototypes
//-------------------------------------------------------------------

void SCH_Init_T0(void);
void SCH_Start(void);
// Core scheduler functions
void SCH_Dispatch_Tasks(void);
unsigned char SCH_Add_Task(void (*)(void), const unsigned int, const unsigned int);
unsigned char SCH_Delete_Task(const unsigned char);

#endif