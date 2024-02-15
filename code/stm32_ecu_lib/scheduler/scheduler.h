#ifndef scheduler_
#define scheduler_

/*
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

#define SCH_MAX_NUMBER_OF_TASKS 5

void scInitialise(int timerTickPeriod_);
void scTimerTick(void);
void scStartScheduler(void);
void scStopScheduler(void);
void scCompleted(int index);
void scAddTask(int index, void (*f)(), float period);

typedef enum { SCH_UNDEFINED, SCH_READY, SCH_STARTED, SCH_BLOCKING } scTaskStatus;

typedef struct {
  void (*function)();			// the task function
  scTaskStatus state;			// task state
  unsigned int period;			// task period timer units
  unsigned int periodCount;		// period counter
  unsigned int overrunCount;	// counts the number of timer tick events that the task is not in a READY state
} scTaskDescription ;

extern int scNumberOfTasksRegistered;							// the number of tasks added
extern scTaskDescription scTasks[SCH_MAX_NUMBER_OF_TASKS];		// an array of task descriptions
extern int scTimerTickPeriod;									// the scheduler timer tick period in milli-seconds


#endif
