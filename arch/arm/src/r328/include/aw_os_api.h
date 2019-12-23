#ifndef AW_OS_API_H
#define AW_OS_API_H
#include <nuttx/mutex.h>

typedef mutex_t aw_mutex_t;

#define aw_mutex_init(mutex) 	nxmutex_init(mutex)
#define aw_mutex_destroy(mutex) nxmutex_destroy(mutex)
#define aw_mutex_lock(mutex)   	nxmutex_lock(mutex)
#define aw_mutex_trylock(mutex) nxmutex_trylock(mutex)
#define aw_mutex_is_lock(mutex) nxmutex_is_locked(mutex)
#define aw_mutex_unlock(mutex) 	nxmutex_unlock(mutex)


#endif  /*AW_OS_API_H*/
