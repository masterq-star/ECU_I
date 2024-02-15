/*

Provides a pre-emptive asynchoronous task scheduler for a number of tasks defined by MAX_NUMBER_OF_TASKS. 
Requires a timer tick input from which individual task frequencies are subdivided.

Tasks are added using the addTask() method. Once all tasks are added, the scheduler is started by a call to startScheduler().
Each task must call completed() when it is finished processing in its current time slot, using its task number (or index)
provided in the addTask() method.



This software/firmware source code or executable program is copyright of
Just Technology (North West) Ltd (http://www.just-technology.co.uk) 2020

This software/firmware source code or executable program is provided as free software:
you can redistribute it and/or modify it under the terms of the GNU General Public License
as published by the Free Software Foundation, either version 3 of the License, or (at your
option) any later version. The license is available at https://www.gnu.org/licenses/gpl-3.0.html

The software/firmware source code or executable program is distributed in the hope that
it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE.
*/

#include "scheduler.h"
#include "math.h"

scTaskDescription scTasks[SCH_MAX_NUMBER_OF_TASKS];
int scNumberOfTasksRegistered;
int scTimerTickPeriod;

int scSchedulerStarted = 0;


// initialise the schedule with the timer tick period specified in milliseconds
void scInitialise(int timerTickPeriod_){
	scTimerTickPeriod = timerTickPeriod_;
	scNumberOfTasksRegistered = 0;
	for (int i=0; i < SCH_MAX_NUMBER_OF_TASKS; i++) {
		scTasks[i].state = SCH_UNDEFINED;
	}
}

void scStartScheduler(){
	scSchedulerStarted = 1;
}

void scStopScheduler(){
	scSchedulerStarted = 0;
}

void scCompleted(int index){
	scTasks[index].state = SCH_READY;
}

// adds a task to the list. index is a unique task number, (*f)() is the callback, period is the task period in milliseconds
void scAddTask(int index, void (*f)(), float period) {
	if (scNumberOfTasksRegistered < SCH_MAX_NUMBER_OF_TASKS) {
		if (index >= 0 && index < SCH_MAX_NUMBER_OF_TASKS) {
			scTasks[index].period = roundf(period / scTimerTickPeriod);
			scTasks[index].periodCount = 0;
			scTasks[index].function = f;
			scTasks[index].state = SCH_READY;
			scNumberOfTasksRegistered++;
		}
	}
}

void scTimerTick(){
	//	sei();		// re-enable global interrupt flag. Must allow called tasks to be interrupted
	if (scSchedulerStarted != 0) {
		for (int t = 0; t < scNumberOfTasksRegistered; t++) {
			if (scTasks[t].state == SCH_READY) {
				if (++scTasks[t].periodCount >= scTasks[t].period) {
					scTasks[t].periodCount = 0;
					scTasks[t].overrunCount = 0;
					scTasks[t].state = SCH_STARTED;
					scTasks[t].function();
				}
			}
			if (scTasks[t].state == SCH_STARTED) {
				scTasks[t].overrunCount++;
			}
		}
	}
}

/*+++REVISION_HISTORY+++
1) 12 Feb 2021 Changed math.h round() to roundf() as this is required for float types.
+++REVISION_HISTORY_ENDS+++*/
