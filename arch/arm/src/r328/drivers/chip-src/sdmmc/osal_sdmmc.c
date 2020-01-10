#include <sdmmc/osal_sdmmc.h>
#include <sdmmc/sdmmc_log.h>

#ifdef OS_RESOURCE_TRACE

extern int g_r_msgqueue_cnt;
extern int g_r_mutex_cnt;
extern int g_r_semaphore_cnt;
extern int g_r_thread_cnt;
extern int g_r_timer_cnt;

void r_os_resource_info(void)
{
/*
	OS_LOG(1, "<<< r os resource info >>>\n");
	OS_LOG(1, "g_r_msgqueue_cnt  %d\n", g_r_msgqueue_cnt);
	OS_LOG(1, "g_r_mutex_cnt     %d\n", g_r_mutex_cnt);
	OS_LOG(1, "g_r_semaphore_cnt %d\n", g_r_semaphore_cnt);
	OS_LOG(1, "g_r_thread_cnt    %d\n", g_r_thread_cnt);
	OS_LOG(1, "g_r_timer_cnt     %d\n", g_r_timer_cnt);
*/
}

#endif /* OS_RESOURCE_TRACE */

void print_hex_dump_words(const void *addr, unsigned int len)
{
	unsigned int i;
	const unsigned int *p = addr;

	if ((unsigned int)p & 0x03) {
		SDMMC_LOG_TAG("addr should be align 4B!\n");
		p =  (const void *)(((unsigned int)p) & ~0x03);
		return ;
	}
	len = DIV_ROUND_UP(len, 4);

	for (i = 0; i < len; i++) {
		if ((i & 0x03) == 0x0)
			SDMMC_LOG_TAG("\n[%p]: ", p);
		SDMMC_LOG_TAG("0x%08x ", *p++);
	}
	SDMMC_LOG_TAG("\n");
}

void print_hex_dump_bytes(const void *addr, unsigned int len)
{
	unsigned int i;
	const unsigned char *p = addr;
	len++;

	for (i = 1; i < len; ++i) {
		SDMMC_LOG_TAG("%02x ", *p++);
		if (i % 16 == 0) {
			SDMMC_LOG_TAG("\n");
		}
	}
	SDMMC_LOG_TAG("\n");
}
/*
 *
 * osal muttex.
 *
 */
/**
 * @brief Create and initialize a mutex object
 * @note A mutex can only be locked by a single thread at any given time.
 * @param[in] mutex Pointer to the mutex object
 * @retval osal_sdmmc_status_t, OS_OK on success
 */
osal_sdmmc_status_t osal_sdmmc_mutex_init(osal_sdmmc_mutex_t *mutex)
{
	return nxsem_init(&mutex->handle,0,1);
}

/**
 * @brief Delete the mutex object
 * @param[in] mutex Pointer to the mutex object
 * @retval osal_sdmmc_status_t, OS_OK on success
 */
osal_sdmmc_status_t osal_sdmmc_mutex_delete(osal_sdmmc_mutex_t *mutex)
{
//	vSemaphoreDelete(mutex->handle);
	return nxsem_destroy(&mutex->handle);
}

/**
 * @brief Lock the mutex object
 * @note A mutex can only be locked by a single thread at any given time. If
 *       the mutex is already locked, the caller will be blocked for the
 *       specified time duration.
 * @param[in] mutex Pointer to the mutex object
 * @param[in] waitMS The maximum amount of time (in millisecond) the thread
 *                   should remain in the blocked state to wait for the mutex
 *                   to become unlocked.
 *                   OS_WAIT_FOREVER for waiting forever, zero for no waiting.
 * @retval osal_sdmmc_status_t, OS_OK on success
 */
//osal_sdmmc_status_t SDC_MutexLock(osal_sdmmc_mutex_t *mutex, OS_Time_t waitMS)
osal_sdmmc_status_t osal_sdmmc_mutex_lock(osal_sdmmc_mutex_t *mutex,
		osal_sdmmc_wait_timems_t waitMS)
{
#if 0
	//TODO : wait time out,
	return nxsem_wait(&mutex->handle);
#else
	struct timespec abstime;
	unsigned int timeout_sec;

	if(waitMS == OS_WAIT_FOREVER) {
		waitMS = 0x0fffffffU;
	}

  /* Get the current time */

	(void)clock_gettime(CLOCK_REALTIME, &abstime);

	timeout_sec      = waitMS / 1000;
	abstime.tv_sec  += timeout_sec;
	abstime.tv_nsec += 1000 * 1000 * (waitMS % 1000);

	if (abstime.tv_nsec >= 1000 * 1000 * 1000) {
	    abstime.tv_sec++;
	    abstime.tv_nsec -= 1000 * 1000 * 1000;
	}

	if(nxsem_timedwait(&mutex->handle, &abstime) == OK)
		return OSAL_STATUS_OK;

	return OSAL_STATUS_FAIL;
#endif
}

/**
 * @brief Unlock the mutex object previously locked using OS_MutexLock()
 * @note The mutex should be unlocked from the same thread context from which
 *       it was locked.
 * @param[in] mutex Pointer to the mutex object
 * @retval osal_sdmmc_status_t, OS_OK on success
 */
//osal_sdmmc_status_t SDC_MutexUnlock(osal_sdmmc_mutex_t *mutex)
osal_sdmmc_status_t osal_sdmmc_mutex_unlock(osal_sdmmc_mutex_t *mutex)
{
	return nxsem_post(&mutex->handle);
}
/*
 *
 * osal semaphore.
 *
 */

/**
 * @brief Create and initialize a counting semaphore object
 * @param[in] sem Pointer to the semaphore object
 * @param[in] initCount The count value assigned to the semaphore when it is
 *                      created.
 * @param[in] maxCount The maximum count value that can be reached. When the
 *                     semaphore reaches this value it can no longer be
 *                     released.
 * @retval osal_sdmmc_status_t, OS_OK on success
 */
//osal_sdmmc_status_t SDC_SemCreate(osal_sdmmc_sem_t *sem, uint32_t initCount)
osal_sdmmc_status_t osal_sdmmc_sem_create(osal_sdmmc_sem_t *sem, uint32_t initCount)
{
	return nxsem_init(sem,0,initCount);
}
/**
 * @brief Delete the semaphore object
 * @param[in] sem Pointer to the semaphore object
 * @retval osal_sdmmc_status_t, OS_OK on success
 */
//osal_sdmmc_status_t OS_SemaphoreDelete(osal_sdmmc_sem_t *sem)
osal_sdmmc_status_t osal_sdmmc_sem_delete(osal_sdmmc_sem_t *sem)
{

	return nxsem_destroy(sem);
}

/**
 * @brief Wait until the semaphore object becomes available
 * @param[in] sem Pointer to the semaphore object
 * @param[in] waitMS The maximum amount of time (in millisecond) the thread
 *                   should remain in the blocked state to wait for the
 *                   semaphore to become available.
 *                   OS_WAIT_FOREVER for waiting forever, zero for no waiting.
 * @retval osal_sdmmc_status_t, OS_OK on success
 */
//osal_sdmmc_status_t OS_SemaphoreWait(osal_sdmmc_sem_t *sem, OS_Time_t waitMS)
osal_sdmmc_status_t osal_sdmmc_sem_wait(osal_sdmmc_sem_t *sem,
		osal_sdmmc_wait_timems_t waitMS)
{
#if 0
	return nxsem_wait(sem);
#else
	struct timespec abstime;
	unsigned int timeout_sec;

	if(waitMS == OS_WAIT_FOREVER) {
		waitMS = 0x0fffffffU;
	}

  /* Get the current time */

	(void)clock_gettime(CLOCK_REALTIME, &abstime);

	timeout_sec      = waitMS / 1000;
	abstime.tv_sec  += timeout_sec;
	abstime.tv_nsec += 1000 * 1000 * (waitMS % 1000);

	if (abstime.tv_nsec >= 1000 * 1000 * 1000) {
	    abstime.tv_sec++;
	    abstime.tv_nsec -= 1000 * 1000 * 1000;
	}

	if(nxsem_timedwait(sem, &abstime) == OK)
		return OSAL_STATUS_OK;

	return OSAL_STATUS_FAIL;

#endif
}

/**
 * @brief Release the semaphore object
 * @param[in] sem Pointer to the semaphore object
 * @retval osal_sdmmc_status_t, OS_OK on success
 */
//osal_sdmmc_status_t OS_SemaphoreRelease(osal_sdmmc_sem_t *sem)
osal_sdmmc_status_t osal_sdmmc_sem_give(osal_sdmmc_sem_t *sem)
{
	return nxsem_post(sem);
}

#ifdef CONFIG_SDIO_IRQ
/*
 *
 * osal thread.
 *
 */

/**
 * @brief Create and start a thread
 *
 * This function starts a new thread. The new thread starts execution by
 * invoking entry(). The argument arg is passed as the sole argument of entry().
 *
 * @note After finishing execution, the new thread should call OS_ThreadDelete()
 *       to delete itself. Failing to do this and just returning from entry()
 *       will result in undefined behavior.
 *
 * @param[in] thread Pointer to the thread object
 * @param[in] name A descriptive name for the thread. This is mainly used to
 *                 facilitate debugging.
 * @param[in] entry Entry, which is a function pointer, to the thread function
 * @param[in] arg The sole argument passed to entry()
 * @param[in] priority The priority at which the thread will execute
 * @param[in] stackSize The number of bytes the thread stack can hold
 * @retval osal_sdmmc_status_t, OS_OK on success
 */
//osal_sdmmc_status_t OS_ThreadCreate(OS_Thread_t *thread, const char *name,
osal_sdmmc_status_t osal_sdmmc_thread_create(osal_sdmmc_task_handle_t *thread, const char *name,
                          osal_sdmmc_thread_t entry, void *arg,
                          int priority, uint32_t stackSize);
{
	return kthread_create(name,priority,stackSize,entry,arg);
}

/**
 * @brief Terminate the thread
 * @note Only memory that is allocated to a thread by the kernel itself is
 *       automatically freed when a thread is deleted. Memory, or any other
 *       resource, that the application (rather than the kernel) allocates
 *       to a thread must be explicitly freed by the application when the task
 *       is deleted.
 * @param[in] thread Pointer to the thread object to be deleted.
 *                   A thread can delete itself by passing NULL in place of a
 *                   valid thread object.
 * @retval osal_sdmmc_status_t, OS_OK on success
 */
//osal_sdmmc_status_t OS_ThreadDelete(OS_Thread_t *thread)
osal_sdmmc_status_t osal_sdmmc_thread_delete(osal_sdmmc_task_handle_t *thread);
{
	return OS_OK;
}
#endif
/*
 *
 * osal timer.
 *
 */

#if defined(CONFIG_DETECT_BY_D3) || defined(CONFIG_DETECT_BY_GPIO_IRQ)
static void _os_time_priv_callback(TimerHandle_t xTimer)
{
}

osal_sdmmc_status_t osal_time_create(OS_Timer_t *timer, OS_TimerType type,
                         OS_TimerCallback_t cb, void *arg, uint32_t periodMS)
{
}

/**
 * @brief Delete the timer object
 * @param[in] timer Pointer to the timer object
 * @retval osal_sdmmc_status_t, OS_OK on success
 */
osal_sdmmc_status_t osal_time_del(OS_Timer_t *timer)
{
}

/**
 * @brief Start a timer running.
 * @note If the timer is already running, this function will re-start the timer.
 * @param[in] timer Pointer to the timer object
 * @retval osal_sdmmc_status_t, OS_OK on success
 */
osal_sdmmc_status_t osal_time_start(OS_Timer_t *timer)
{
}

/**
 * @brief Change the period of a timer
 *
 * If osal_time_change_period() is used to change the period of a timer that is
 * already running, then the timer will use the new period value to recalculate
 * its expiry time. The recalculated expiry time will then be relative to when
 * osal_time_change_period() was called, and not relative to when the timer was
 * originally started.

 * If osal_time_change_period() is used to change the period of a timer that is
 * not already running, then the timer will use the new period value to
 * calculate an expiry time, and the timer will start running.
 *
 * @param[in] timer Pointer to the timer object
 * @retval osal_sdmmc_status_t, OS_OK on success
 */
osal_sdmmc_status_t osal_time_change_period(OS_Timer_t *timer, uint32_t periodMS)
{
	return OS_OK;
}

/**
 * @brief Stop a timer running.
 * @param[in] timer Pointer to the timer object
 * @retval osal_sdmmc_status_t, OS_OK on success
 */
osal_sdmmc_status_t osal_time_stop(OS_Timer_t *timer)
{

	return OS_OK;
}
#endif
