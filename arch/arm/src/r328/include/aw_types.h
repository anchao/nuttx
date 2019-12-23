#ifndef __TYPES_H__
#define __TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif


//#include <types.h>

#define min(a, b)  ((a) < (b) ? (a) : (b))
#define max(a,b)   ((a) < (b) ? (b) : (a))

/* return value defines */
#if 0
#define	OK	(0)
#define	FAIL	(-1)
#define TRUE	(1)
#define	FALSE	(0)
#define true     1
#define false    0
#endif

#ifndef NULL
#define NULL    0
#endif

/* general data type defines */
typedef	void * 		    HANDLE;
typedef unsigned long long  u64;
typedef unsigned int        u32;
typedef unsigned short      u16;
typedef unsigned char       u8;
typedef signed long long    s64;
typedef signed int          s32;
typedef signed short        s16;
typedef signed char         s8;
//typedef signed char         bool;
typedef unsigned int        size_t;
typedef unsigned int        uint;

typedef u8 		u_int8_t;
typedef u16 		u_int16_t;
typedef u32 		u_int32_t;
typedef u64 		u_int64_t;

#ifdef __cplusplus
}
#endif


#endif /* __TYPES_H__ */
