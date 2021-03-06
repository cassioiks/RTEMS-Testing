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
#include "test.h"

int test = 0;
rtems_id partitionID;

rtems_task user_application(rtems_task_argument argument);
rtems_task task_1(rtems_task_argument argument);
rtems_task task_partition(rtems_task_argument argument);

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

  status = rtems_task_start( tid, user_application, 0 );
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

   status_2 = rtems_task_start( tid_2, task_1, 0 );
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

   status_3 = rtems_task_start( tid_3, task_partition, 0 );
   if ( status_3 != RTEMS_SUCCESSFUL ) {
     printf( "2) rtems_task_start failed with status of %d.\n", status_3 );
     exit( 1 );
   }
   /*
    * END OF THIRD TASK
    * */

  status = rtems_task_delete( RTEMS_SELF );    /* should not return */
  printf( "rtems_task_delete returned with status of %d.\n", status );
  exit( 1 );
}


rtems_task user_application(rtems_task_argument argument)
{
	/* application specific initialization goes here */
	while ( 1 )  {              /* infinite loop */
		/*  APPLICATION CODE GOES HERE
		 *
		 *  This code will typically include at least one
		 *  directive which causes the calling task to
		 *  give up the processor.
		 */
		test = test + 1;
		printf("User App: %i\n", test);

		//Testing Partition Buffer
		rtems_status_code returnCode;
		double *buff;
		returnCode = rtems_partition_get_buffer(partitionID, &buff);
		printf("User App: The get buffer code is %i\n", returnCode);

		if(returnCode == 0)
			printf("User App: The buffer address is %d\n", buff);

		if(returnCode == 0){
			returnCode = rtems_partition_return_buffer(partitionID, buff);
			if(returnCode == 0)
				printf("User app: Buffer returned successfully!\n");
			else{
				printf("User app: Not able to return buffer! Status %i.\n", returnCode);
				exit(0);
			}
		}

		//
		rtems_task_wake_after(1000);
	}
}

rtems_task task_1(rtems_task_argument argument){
	while(1){
		test = test + 2;
		printf("Task 1: %i\n", test);

		//Testing Partition Buffer
		rtems_status_code returnCode;
		double *buff;
		returnCode = rtems_partition_get_buffer(partitionID, &buff);
		printf("Task 1: The get buffer code is %i\n", returnCode);

		if(returnCode == 0)
			printf("Task 1: The buffer address is %d\n", buff);

		if(returnCode == 0){
			returnCode = rtems_partition_return_buffer(partitionID, buff);

			if(returnCode == 0)
				printf("Task 1: Buffer returned successfully!\n");
			else{
				printf("Task 1: Not able to return buffer! Status %i.\n", returnCode);
				exit(0);
			}
		}
		//
		rtems_task_wake_after(500);
	}
}

rtems_task task_partition(rtems_task_argument argument){
	rtems_status_code status;

	rtems_name partName = rtems_build_name( 'P','A','R','T');
	int address = 0x001;
	uint32_t length = 64;
	uint32_t buffer_size = 64;
	rtems_attribute partAttr = RTEMS_LOCAL;
	rtems_id pid;

	status = rtems_partition_create(partName, &address, length, buffer_size, partAttr, &pid);
	partitionID = pid;
	//Check for errors
	switch (status) {
	case RTEMS_SUCCESSFUL:
		printf("partition created successfully\n");
		break;
	case RTEMS_INVALID_NAME:
		printf("invalid partition name\n");
		break;
	case RTEMS_INVALID_ADDRESS:
		printf("address not on four byte boundary\n");
		break;
	case RTEMS_INVALID_SIZE:
		printf("length or buffer size is 0\n");
		break;
	case RTEMS_MP_NOT_CONFIGURED:
		printf("multiprocessing not configured\n");
		break;
	case RTEMS_TOO_MANY:
		printf("too many global objects\n");
		break;
	default: printf("none of the above\n"); break;
	}
	printf("The Partition creation returned with the following code %d.\n", status);

	if(status != 0)
		exit(0);

	rtems_task_delete( RTEMS_SELF );
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
