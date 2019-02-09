/******************************************************************************/
/* vinetic-ioctl.h                                                            */
/******************************************************************************/

#ifndef __VINETIC_IOCTL_H__
#define __VINETIC_IOCTL_H__

#include <linux/types.h>
#include <linux/ioctl.h>

#define VINETIC_MAGIC			'v'
#define VINETIC_RESET			_IO(VINETIC_MAGIC, 0)
#define VINETIC_RESET_RDYQ		_IO(VINETIC_MAGIC, 1)
#define VINETIC_FLUSH_MBOX		_IO(VINETIC_MAGIC, 2)
#define VINETIC_DISABLE_IRQ		_IO(VINETIC_MAGIC, 3)
#define VINETIC_GET_NOT_READY	_IOR(VINETIC_MAGIC, 4, int)
#define VINETIC_READ_DIA		_IOR(VINETIC_MAGIC, 5, u_int16_t)
#define VINETIC_REVISION		_IOR(VINETIC_MAGIC, 6, u_int16_t)
#define VINETIC_CHECKSUM		_IOR(VINETIC_MAGIC, 7, u_int16_t)
#define VINETIC_SET_POLL		_IOW(VINETIC_MAGIC, 8, int)
#define VINETIC_RESET_STATUS	_IO(VINETIC_MAGIC, 9)

#endif //__VINETIC_IOCTL_H__

/******************************************************************************/
/* end of vinetic-ioctl.h                                                     */
/******************************************************************************/
