#ifndef _WIRELESS_XRADIO_PORT_XR_DRV_MEM_H_
#define _WIRELESS_XRADIO_PORT_XR_DRV_MEM_H_
#ifndef CONFIG_OS_NUTTX
#include "types.h"
#include <string.h>
#include <stdlib.h>
#else
#include <sys/types.h>
#include <nuttx/kmalloc.h>
#endif

#define XR_DRV_MEM_TRACE	0

#if XR_DRV_MEM_TRACE
void *xr_drv_malloc(size_t size);
void xr_drv_free(void *ptr);
#else /* XR_DRV_MEM_TRACE */
#ifndef CONFIG_OS_NUTTX
#define xr_drv_malloc(l)	malloc(l)
#define xr_drv_free(p)		free(p)
#else
#define xr_drv_malloc(l)	kmm_malloc(l)
#define xr_drv_free(p)		kmm_free(p)
#endif /*CONFIG_OS_NUTTX*/
#endif /* XR_DRV_MEM_TRACE */

#define xr_memcpy(d, s, l)	memcpy(d, s, l)
#define xr_memset(d, c, l)	memset(d, c, l)
#define xr_memcmp(a, b, l)	memcmp(a, b, l)
#define xr_memmove(d, s, n)	memmove(d, s, n)
#endif /* _WIRELESS_XRADIO_PORT_XR_DRV_MEM_H_ */
