#ifndef SUNXI_HAL_MUTEX_H
#define SUNXI_HAL_MUTEX_H
#include <nuttx/mutex.h>

typedef mutex_t hal_mutex_t;

#define hal_mutex_init(mutex) 		nxmutex_init(mutex)
#define hal_mutex_destroy(mutex) 	nxmutex_destroy(mutex)
#define hal_mutex_lock(mutex)   	nxmutex_lock(mutex)
#define hal_mutex_trylock(mutex) 	nxmutex_trylock(mutex)
#define hal_mutex_is_lock(mutex) 	nxmutex_is_locked(mutex)
#define hal_mutex_unlock(mutex) 	nxmutex_unlock(mutex)

#endif /* SUNXI_HAL_MUTEX_H */
