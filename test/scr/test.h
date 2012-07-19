/*
 * test.h
 *
 *  Created on: Apr 7, 2012
 *      Author: root
 */

#ifndef TEST_H_
#define TEST_H_
#include <rtems.h>

inline int soma(int a, int b){
	return a+b;
}

void checkStatus(int status){
	switch (status) {
	case RTEMS_SUCCESSFUL:
		printf("partition created successfully");
		break;
	case RTEMS_INVALID_NAME:
		printf("invalid partition name");
		break;
	case RTEMS_INVALID_ADDRESS:
		printf("address not on four byte boundary");
		break;
	case RTEMS_INVALID_SIZE:
		printf("length or buffer size is 0");
		break;
	case RTEMS_MP_NOT_CONFIGURED:
		printf("multiprocessing not configured");
		break;
	case RTEMS_TOO_MANY:
		printf("too many global objects");
		break;
	}
}



#endif /* TEST_H_ */
