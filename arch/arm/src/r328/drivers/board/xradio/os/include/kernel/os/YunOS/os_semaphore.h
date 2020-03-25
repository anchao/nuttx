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
#ifndef _KERNEL_OS_YUNOS_OS_SEMAPHORE_H_
#define _KERNEL_OS_YUNOS_OS_SEMAPHORE_H_

#include "kernel/os/YunOS/os_common.h"
#include "kernel/os/os_time.h"

#ifdef __cplusplus
extern "C" {
#endif

#if 0
typedef struct {
    void *hdl;
} aos_hdl_t;

typedef aos_hdl_t aos_sem_t;
#endif

typedef void* SemaphoreHandle_t;

typedef struct OS_Semaphore {
    SemaphoreHandle_t   handle;
} OS_Semaphore_t;

typedef struct OS_Semaphore_XR{
	aos_sem_t sem;
	uint32_t count;
	uint32_t max_count;
} OS_Semaphore_XR_t;

inline static OS_Status OS_SemaphoreCreate(OS_Semaphore_t *sem, uint32_t initCount, uint32_t maxCount)
{
	//int aos_sem_new(aos_sem_t *sem, int count);
	OS_Semaphore_XR_t *sem_t = NULL;

    sem_t = aos_malloc(sizeof(OS_Semaphore_XR_t));
    if (sem_t == NULL) {
        return OS_E_NOMEM;
    }

	memset(sem_t, 0, sizeof(OS_Semaphore_XR_t));

	if (RHINO_SUCCESS == aos_sem_new(&(sem_t->sem), initCount)) {
		sem_t->count = initCount;
		sem_t->max_count = maxCount;
		sem->handle = sem_t;
		return OS_OK;
	} else {
		aos_free(sem_t);
		sem->handle = NULL;
		return OS_FAIL;
	}
}

inline static OS_Status OS_SemaphoreCreateBinary(OS_Semaphore_t *sem)
{

	//int aos_sem_new(aos_sem_t *sem, int count);
	OS_Semaphore_XR_t *sem_t;
    sem_t = aos_malloc(sizeof(OS_Semaphore_XR_t));
    if (sem_t == NULL) {
        return OS_E_NOMEM;
    }

	if (RHINO_SUCCESS == aos_sem_new(&(sem_t->sem), 0)) {
		sem_t->count = 0;
		sem_t->max_count = 1;
		sem->handle = sem_t;
		return OS_OK;
	} else {
		aos_free(sem_t);
		sem->handle = NULL;
		return OS_FAIL;
	}

}

inline static OS_Status OS_SemaphoreDelete(OS_Semaphore_t *sem)
{

	//void aos_sem_free(aos_sem_t *sem)
	OS_Semaphore_XR_t *sem_t;
	sem_t = (OS_Semaphore_XR_t *)(sem->handle);

	aos_sem_free(&(sem_t->sem));
	aos_free(sem_t);

	sem->handle = NULL;
	return OS_OK;
}

inline static OS_Status OS_SemaphoreWait(OS_Semaphore_t *sem, OS_Time_t waitMS)//xradio irq call,can not in xip
{

	OS_Semaphore_XR_t *sem_t;
	sem_t = (OS_Semaphore_XR_t *)(sem->handle);
	//int aos_sem_wait(aos_sem_t *sem, unsigned int timeout)


	//tick_t ticks = waitMS == OS_WAIT_FOREVER ? OS_WAIT_FOREVER : OS_MSecsToTicks(waitMS);
	OS_Status ret;
	int sta;
	//uint32_t time_tick = OS_GetTicks();

	if (sem == NULL) {
		return OS_FAIL;
	}

	sta = aos_sem_wait(&(sem_t->sem), waitMS);
	if(sta == RHINO_SUCCESS) {
		if (sem_t->count)
			sem_t->count--;
		ret = OS_OK;
	} else {
		ret = OS_E_TIMEOUT;
	}

	return ret;
}

inline static OS_Status OS_SemaphoreRelease(OS_Semaphore_t *sem)//xradio irq, can not xin xip
{
	//void aos_sem_signal(aos_sem_t *sem)
	OS_Semaphore_XR_t *sem_t;
	sem_t = (OS_Semaphore_XR_t *)(sem->handle);

	if (sem_t->max_count == 1 && sem_t->count == 1) {
		//printf("sem release fail, max_count %d\n", sem_t->max_count);
		return OS_FAIL;
	}
	aos_sem_signal(&(sem_t->sem));

	if (sem_t->count < sem_t->max_count) {
		sem_t->count++;
	}
	return OS_OK;
}

inline static int OS_SemaphoreIsValid(OS_Semaphore_t *sem)
{
	OS_Semaphore_XR_t *sem_t;
	sem_t = (OS_Semaphore_XR_t *)(sem->handle);

	//return (sem->sem.blk_obj.obj_type == RHINO_SEM_OBJ_TYPE);
	if(sem_t == NULL) {
		return 0;
	}

	if((aos_sem_is_valid(&(sem_t->sem))) == 0) {
		return 0;
	} else {
		return 1;
	}
}

inline static void OS_SemaphoreSetInvalid(OS_Semaphore_t *sem)
{
	OS_Semaphore_XR_t *sem_t;
	sem_t = (OS_Semaphore_XR_t *)(sem->handle);

	//sem->sem.blk_obj.obj_type = RHINO_OBJ_TYPE_NONE;
	sem_t->sem.hdl = NULL;

	sem->handle = OS_INVALID_HANDLE;
}
#ifdef __cplusplus
}
#endif

#endif /* _KERNEL_OS_YUNOS_OS_SEMAPHORE_H_ */
