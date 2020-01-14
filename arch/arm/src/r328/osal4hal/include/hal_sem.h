#ifndef SUNXI_HAL_SEM_H
#define SUNXI_HAL_SEM_H
#include <nuttx/semaphore.h>
#include <time.h>

typedef sem_t hal_sem_t;

#define hal_sem_create(sem, cnt) 	nxsem_init(sem, 0, cnt)

#define hal_sem_delete(sem) 		nxsem_destroy(sem)

#define hal_sem_post(sem) 		nxsem_post(sem)

#define hal_sem_wait(sem) 		nxsem_wait(sem)

static inline int hal_sem_timewait(hal_sem_t *sem, int ms)
{
	struct timespec tp;

	if (ms < 0)
		return -EINVAL;
	else if (ms == 0)
		return nxsem_wait(sem);
	clock_gettime(CLOCK_REALTIME, &tp);
	tp.tv_sec += ms/1000;
	tp.tv_nsec += ms%1000*1000;
	return nxsem_timedwait(sem, (const struct timespec *)&tp);
}

#define hal_sem_trywait(sem) 		nxsem_trywait(sem)

#endif /* SUNXI_HAL_SEM_H */
