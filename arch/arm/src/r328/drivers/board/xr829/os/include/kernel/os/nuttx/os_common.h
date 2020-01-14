/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _KERNEL_OS_NUTTX_OS_COMMON_H_
#define _KERNEL_OS_NUTTX_OS_COMMON_H_
#include <sys/types.h>
#ifdef __cplusplus
extern "C" {
#endif
#define PRIORIY_BASE  90
//TODO
typedef enum  {
	OS_PRIORITY_IDLE        	= (PRIORIY_BASE + 1),
	OS_PRIORITY_LOW         	= (PRIORIY_BASE + 2),
	OS_PRIORITY_BELOW_NORMAL	= (PRIORIY_BASE + 3),
	OS_PRIORITY_NORMAL     		= (PRIORIY_BASE + 4),
	OS_PRIORITY_ABOVE_NORMAL 	= (PRIORIY_BASE + 5),
	OS_PRIORITY_HIGH        	= (PRIORIY_BASE + 6),
	OS_PRIORITY_REAL_TIME    	= (PRIORIY_BASE + 7),
} OS_Priority;

typedef enum {
	OS_OK			= 0,
	OS_FAIL			= -1,
	OS_E_NOMEM		= -2,
	OS_E_PARAM		= -3,
	OS_E_TIMEOUT	= -4,
	OS_E_ISR		= -5,
} OS_Status;

typedef unsigned long OS_Time_t;

#define OS_WAIT_FOREVER		0xffffffffU
#define OS_SEMAPHORE_MAX_COUNT	0xffffffffU
#define OS_INVALID_HANDLE	NULL

#ifdef __cplusplus
}
#endif

#endif /* _KERNEL_OS_NUTTX_OS_COMMON_H_ */
