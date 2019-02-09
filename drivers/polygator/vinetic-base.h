/******************************************************************************/
/* vinetic-base.h                                                             */
/******************************************************************************/

#ifndef __VINETIC_BASE_H__
#define __VINETIC_BASE_H__

#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "polygator-types.h"
#include "vinetic-def.h"

#define VINETIC_DEVICE_MAXCOUNT 256
#define VINETIC_DEVNAME_MAXLEN 256

#define VINETIC_PACKETSLOT_MAXCOUNT 32

enum {
	VINETIC_DEVTYPE_UNKNOWN = 0,
	VINETIC_DEVTYPE_VINETIC = 1,
	VINETIC_DEVTYPE_RTPCHAN = 2,
};

struct vinetic_rtp_channel;
struct vinetic {
	char *name;
	int devno;
	spinlock_t lock;
	size_t usage;
// 	struct file *filp;
	struct timer_list poll_timer;
	int poll;

	// status
	int status_ready;
	struct vin_status_registers status;
	struct vin_status_registers status_old;
	struct vin_status_registers status_mask;
	wait_queue_head_t status_waitq;

	// FIBXMS
	size_t free_cbox_space;
	size_t free_pbox_space;
	// OBXML
	size_t cdata_size;
	size_t pdata_size;
	wait_queue_head_t free_cbox_waitq;
	wait_queue_head_t seek_cbox_waitq;
	wait_queue_head_t read_cbox_waitq;
	u_int16_t read_cbox_data[32];
	size_t read_cbox_length;

	struct vinetic_rtp_channel *rtp_channels[8];

	uintptr_t cbdata;
	void (* reset)(uintptr_t cbdata);
	size_t (* is_not_ready)(uintptr_t cbdata);
	void (* write_nwd)(uintptr_t cbdata, u_int16_t value);
	void (* write_eom)(uintptr_t cbdata, u_int16_t value);
	u_int16_t (* read_nwd)(uintptr_t cbdata);
	u_int16_t (* read_eom)(uintptr_t cbdata);
	u_int16_t (* read_dia)(uintptr_t cbdata);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	struct device *device;
#else
	struct class_device *device;
#endif
	struct cdev cdev;
};

struct rtp_packet_slot {
	u_int16_t data[256];
	size_t length;
};

struct vinetic_rtp_channel {
	char *name;
	int devno;
	spinlock_t lock;
	size_t usage;

	wait_queue_head_t poll_waitq;
	wait_queue_head_t read_slot_count_waitq;
	wait_queue_head_t write_slot_count_waitq;

	struct vinetic *vinetic;
	int index;

	struct rtp_packet_slot read_slot[VINETIC_PACKETSLOT_MAXCOUNT];
	size_t read_slot_read;
	size_t read_slot_write;
	size_t read_slot_count;

	struct rtp_packet_slot write_slot[VINETIC_PACKETSLOT_MAXCOUNT];
	size_t write_slot_read;
	size_t write_slot_write;
	size_t write_slot_count;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	struct device *device;
#else
	struct class_device *device;
#endif
	struct cdev cdev;
};

struct vinetic *vinetic_device_register(struct module *owner,
							char *name,
							uintptr_t cbdata,
							void (* reset)(uintptr_t cbdata),
							size_t (* is_not_ready)(uintptr_t cbdata),
							void (* write_nwd)(uintptr_t cbdata, u_int16_t value),
							void (* write_eom)(uintptr_t cbdata, u_int16_t value),
							u_int16_t (* read_nwd)(uintptr_t cbdata),
							u_int16_t (* read_eom)(uintptr_t cbdata),
							u_int16_t (* read_dia)(uintptr_t cbdata));
void vinetic_device_unregister(struct vinetic *vin);

struct vinetic_rtp_channel *vinetic_rtp_channel_register(struct module *owner, char *name, struct vinetic *vin, int index);
void vinetic_rtp_channel_unregister(struct vinetic_rtp_channel *rtp);

#endif //__VINETIC_BASE_H__

/******************************************************************************/
/* end of vinetic-base.h                                                      */
/******************************************************************************/
