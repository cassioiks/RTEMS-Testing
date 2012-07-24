/*
 *  This file contains an example of a simple RTEMS
 *  application.  It instantiates the RTEMS Configuration
 *  Information using confdef.h and contains two tasks:
 *  a *  user initialization task and a simple task.
 *
 *  This example assumes that a board support package exists.
 */

#include <stdlib.h>
#include <stdio.h>
#include <bsp.h>
#include <rtems.h>

rtems_task task_1(rtems_task_argument argument);
rtems_task task_2(rtems_task_argument argument);
rtems_task task_3(rtems_task_argument argument);

rtems_id semaphoreId;

rtems_task Init(
		rtems_task_argument ignored
)
{

	rtems_id          tid;
	rtems_status_code status;
	rtems_name        name;

	name = rtems_build_name( 'A', 'P', 'P', '1' );

	status = rtems_task_create(
			name, 1, RTEMS_MINIMUM_STACK_SIZE,
			RTEMS_NO_PREEMPT, RTEMS_FLOATING_POINT, &tid
	);
	if ( status != RTEMS_SUCCESSFUL ) {
		printf( "1) rtems_task_create failed with status of %d.\n", status );
		exit(1);
	}

	status = rtems_task_start( tid, task_1, 0 );
	if ( status != RTEMS_SUCCESSFUL ) {
		printf( "1) rtems_task_start failed with status of %d.\n", status );
		exit( 1 );
	}

	/*
	 * SECOND TASK
	 * */

	rtems_id          tid_2;
	rtems_status_code status_2;
	rtems_name        name_2;

	name_2 = rtems_build_name( 'A', 'P', 'P', '2' );

	status_2 = rtems_task_create(
			name_2, 1, RTEMS_MINIMUM_STACK_SIZE,
			RTEMS_NO_PREEMPT, RTEMS_FLOATING_POINT, &tid_2
	);
	if ( status_2 != RTEMS_SUCCESSFUL ) {
		printf( "2) rtems_task_create failed with status of %d.\n", status );
		exit(1);
	}

	status_2 = rtems_task_start( tid_2, task_2, 0 );
	if ( status_2 != RTEMS_SUCCESSFUL ) {
		printf( "2) rtems_task_start failed with status of %d.\n", status );
		exit( 1 );
	}

	/*
	 * END OF SECOND TASK
	 * */

	/*
	 * THIRD TASK
	 * */
	rtems_id          tid_3;
	rtems_status_code status_3;
	rtems_name        name_3;

	name_3 = rtems_build_name( 'A', 'P', 'P', '3' );

	status_3 = rtems_task_create(
			name_3, 1, RTEMS_MINIMUM_STACK_SIZE,
			RTEMS_NO_PREEMPT, RTEMS_FLOATING_POINT, &tid_3
	);
	if ( status_3 != RTEMS_SUCCESSFUL ) {
		printf( "2) rtems_task_create failed with status of %d.\n", status_3 );
		exit(1);
	}

	status_3 = rtems_task_start( tid_3, task_3, 0 );
	if ( status_3 != RTEMS_SUCCESSFUL ) {
		printf( "2) rtems_task_start failed with status of %d.\n", status_3 );
		exit( 1 );
	}
	/*
	 * END OF THIRD TASK
	 * */

	createSemaphore();

	status = rtems_task_delete( RTEMS_SELF );    /* should not return */
	printf( "rtems_task_delete returned with status of %d.\n", status );
	exit( 1 );
}

void createSemaphore(void){
	rtems_status_code status;

	status = rtems_semaphore_create(
			rtems_build_name('S','M','P','R'),
			1,
			RTEMS_PRIORITY | RTEMS_BINARY_SEMAPHORE | RTEMS_GLOBAL,
			0,
			&semaphoreId
	);

	if(status != RTEMS_SUCCESSFUL){
		printf("Creation of Semaphore failed...\n");
		exit (1);
	}
	else
		printf("Creation of Semaphore succeeded...\n");

}


rtems_task task_1(rtems_task_argument argument)
{
	while ( 1 )  {
		rtems_status_code status;
		rtems_task_priority task_priority = RTEMS_CURRENT_PRIORITY;

		status = rtems_task_set_priority( RTEMS_SELF,
				1,
				&task_priority
		);

		if(status == RTEMS_SUCCESSFUL)
			printf("Task 1: Priority set sucessfully...\n");
		else
			printf("Task 1: Priority not set...\n");

		status = rtems_semaphore_obtain(semaphoreId, RTEMS_WAIT, 0);

		switch(status){
		case RTEMS_SUCCESSFUL: printf("Task 1: Semaphore obtained...\n");
		break;
		default: printf("Task 1: Semaphore not obtained...\n"); exit(1);
		break;
		}

		printf("Task 1: Waking after 5 seconds...\n");
		rtems_task_wake_after(5000);
		printf("Task 1: Task is now awake, releasing semaphore...\n");
		rtems_semaphore_release(semaphoreId);


	}
}

rtems_task task_2(rtems_task_argument argument){
	while(1){
		rtems_status_code status;
		rtems_task_priority task_priority = RTEMS_CURRENT_PRIORITY;

		status = rtems_task_set_priority( RTEMS_SELF,
				3,
				&task_priority
		);
		if(status == RTEMS_SUCCESSFUL)
			printf("Task 2: Priority set sucessfully...\n");
		else
			printf("Task 2: Priority not set...\n");

		printf("Task 2: trying to obtain the semaphore...\n ");

		status = rtems_semaphore_obtain(
				semaphoreId,
				RTEMS_WAIT,
				0
		);

		switch(status){
		case RTEMS_SUCCESSFUL: printf("Task 2: Semaphore obtained...\n");
		//exit(1);
		break;
		default: printf("Semaphore not obtained...\n");
		break;
		}

		rtems_task_wake_after(1000);
		printf("Task 2: Releasing semaphore...\n");
		rtems_semaphore_release(semaphoreId);

	}
}

rtems_task task_3(rtems_task_argument argument){

	while(1){
		rtems_status_code status;

		rtems_task_priority task_priority = RTEMS_CURRENT_PRIORITY;

		status = rtems_task_set_priority( RTEMS_SELF,
				2,
				&task_priority
		);
		if(status == RTEMS_SUCCESSFUL)
			printf("Task 3: Priority set sucessfully...\n");
		else
			printf("Task 3: Priority not set...\n");

		printf("Task 3: trying to obtain the semaphore...\n ");

		status = rtems_semaphore_obtain(
				semaphoreId,
				RTEMS_WAIT,
				0
		);

		switch(status){
		case RTEMS_SUCCESSFUL: printf("Task 3: Semaphore obtained...\n");
		//exit(1);
		break;
		default: printf("Semaphore not obtained...\n");
		break;
		}

		rtems_task_wake_after(1000);
		printf("Task 3: Releasing semaphore...\n");
		rtems_semaphore_release(semaphoreId);


	}
	//rtems_task_delete( RTEMS_SELF );
}

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER  /* for stdio */
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER    /* for time services */

#define CONFIGURE_MAXIMUM_TIMERS 1
#define CONFIGURE_MAXIMUM_TASKS 4
#define CONFIGURE_MAXIMUM_PARTITIONS 1
#define maximum_global_objects 10

#define CONFIGURE_INIT_TASK_NAME rtems_build_name( 'E', 'X', 'A', 'M' )
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

#define CONFIGURE_INIT

#include <rtems/confdefs.h>
