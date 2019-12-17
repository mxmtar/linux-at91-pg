/******************************************************************************/
/* polygator-base.h                                                           */
/******************************************************************************/

#ifndef __POLYGATOR_BASE_H__
#define __POLYGATOR_BASE_H__

#define POLYGATOR_DEVICE_MAXCOUNT 256
#define POLYGATOR_DEVNAME_MAXLEN 256

#define POLYGATOR_BOARD_MAXCOUNT 32
#define POLYGATOR_BRDNAME_MAXLEN 256

#define POLYGATOR_TTY_DEVICE_MAXCOUNT 256

enum {
	POLYGATOR_MODULE_TYPE_UNKNOWN = 0,
	POLYGATOR_MODULE_TYPE_SIM300 = 1,
	POLYGATOR_MODULE_TYPE_SIM900 = 2,
	POLYGATOR_MODULE_TYPE_M10 = 3,
	POLYGATOR_MODULE_TYPE_SIM5215 = 4,
	POLYGATOR_MODULE_TYPE_SIM5215A2 = 5,
	POLYGATOR_MODULE_TYPE_M95 = 6,
};

#ifdef __KERNEL__

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/types.h>
#include <linux/wait.h>

struct polygator_board {
	char name[POLYGATOR_BRDNAME_MAXLEN];
	int devno;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	struct device *device;
#else
	struct class_device *device;
#endif
	struct cdev *cdev;
};

struct polygator_tty_device {
	int tty_minor;
	struct tty_operations *tty_ops;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	struct device *device;
#else
	struct class_device *device;
#endif
	void *data;
};

struct polygator_board *polygator_board_register(struct device *device, struct module *owner, char *name, struct cdev *cdef, struct file_operations *fops);
void polygator_board_unregister(struct polygator_board *brd);

char *polygator_print_gsm_module_type(int type);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
struct polygator_tty_device *polygator_tty_device_register(struct device *device, void *data, struct tty_port *port, struct tty_operations *tty_ops);
#else
struct polygator_tty_device *polygator_tty_device_register(struct device *device, void *data, struct tty_operations *tty_ops);
#endif
void polygator_tty_device_unregister(struct polygator_tty_device *ptd);

int polygator_power_on_schedule(void (* callback)(void *data), void *data);
void polygator_power_on_cancel(int id);

#endif //__KERNEL__

#endif //__POLYGATOR_BASE_H__

/******************************************************************************/
/* end of polygator-base.h                                                    */
/******************************************************************************/
