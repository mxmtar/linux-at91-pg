/******************************************************************************/
/* polygator-types.h                                                          */
/******************************************************************************/

#ifndef __POLYGATOR_TYPES_H__
#define __POLYGATOR_TYPES_H__

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <sys/types.h>
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,26)
typedef unsigned long int uintptr_t;
#endif

#endif //__POLYGATOR_TYPES_H__

/******************************************************************************/
/* end of polygator-types.h                                                   */
/******************************************************************************/
