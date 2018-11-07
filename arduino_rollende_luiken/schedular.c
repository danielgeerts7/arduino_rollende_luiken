/*
 * schedular.c
 *
 *  Author: Hanzehogeschool
 */

#include "schedular.h"


// The array of tasks
sTask SCH_tasks_G[SCH_MAX_TASKS];

void SCH_Dispatch_Tasks(void)
{
	unsigned char Index;

	// Dispatches (runs) the next task (if one is ready)
	for(Index = 0; Index < SCH_MAX_TASKS; Index++)
	{
		if((SCH_tasks_G[Index].RunMe > 0) && (SCH_tasks_G[Index].pTask != 0))
		{
			(*SCH_tasks_G[Index].pTask)();  // Run the task
			SCH_tasks_G[Index].RunMe -= 1;   // Reset / reduce RunMe flag

			// Periodic tasks will automatically run again
			// - if this is a 'one shot' task, remove it from the array
			if(SCH_tasks_G[Index].Period == 0)
			{
				SCH_Delete_Task(Index);
			}
		}
	}
}

unsigned char SCH_Add_Task(void (*pFunction)(), const unsigned int DELAY, const unsigned int PERIOD)
{
	unsigned char Index = 0;

	// First find a gap in the array (if there is one)
	while((SCH_tasks_G[Index].pTask != 0) && (Index < SCH_MAX_TASKS))
	{
		Index++;
	}

	// Have we reached the end of the list?
	if(Index == SCH_MAX_TASKS)
	{
		// Task list is full, return an error code
		return SCH_MAX_TASKS;
	}

	// If we're here, there is a space in the task array
	SCH_tasks_G[Index].pTask = pFunction;
	SCH_tasks_G[Index].Delay =DELAY;
	SCH_tasks_G[Index].Period = PERIOD;
	SCH_tasks_G[Index].RunMe = 0;

	// return position of task (to allow later deletion)
	return Index;
}


unsigned char SCH_Delete_Task(const unsigned char TASK_INDEX)
{
	// Return_code can be used for error reporting, NOT USED HERE THOUGH!
	unsigned char Return_code = 0;

	SCH_tasks_G[TASK_INDEX].pTask = 0;
	SCH_tasks_G[TASK_INDEX].Delay = 0;
	SCH_tasks_G[TASK_INDEX].Period = 0;
	SCH_tasks_G[TASK_INDEX].RunMe = 0;

	return Return_code;
}

void SCH_Init_T0(void)
{
	unsigned char i;

	for(i = 0; i < SCH_MAX_TASKS; i++)
	{
		SCH_Delete_Task(i);
	}
	
	// Set up Timer 0
	// prescale op 1024
	TCCR0B = (1 << CS02) | (1 << CS00);
	TCCR0A = (1 << WGM01);
	TIMSK0 = (1 << OCIE0A); // Timer 0 Output Compare A Match Interrupt Enable
	OCR0A = (uint8_t)156; // 10 mili sec = (1024/16.000.000)*156
	
}

ISR(TIMER0_COMPA_vect)
{
	unsigned char Index;
	for(Index = 0; Index < SCH_MAX_TASKS; Index++)
	{
		// Check if there is a task at this location
		if(SCH_tasks_G[Index].pTask)
		{
			if(SCH_tasks_G[Index].Delay == 0)
			{
				// The task is due to run, Inc. the 'RunMe' flag
				SCH_tasks_G[Index].RunMe += 1;

				if(SCH_tasks_G[Index].Period)
				{
					// Schedule periodic tasks to run again
					SCH_tasks_G[Index].Delay = SCH_tasks_G[Index].Period;
					SCH_tasks_G[Index].Delay -= 1;
				}
			}
			else
			{
				// Not yet ready to run: just decrement the delay
				SCH_tasks_G[Index].Delay -= 1;
			}
		}
	}
}
