
#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/vmalloc.h>

#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/io.h>
#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
#include <asm/compat.h>
#endif

#include "vinetic-base.h"
#include "vinetic-def.h"
#include "vinetic-ioctl.h"

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@gmail.com>");
MODULE_DESCRIPTION("Polygator Linux module VINETIC support");
MODULE_LICENSE("GPL");

static int vinetic_major = 0;
module_param(vinetic_major, int, 0);
MODULE_PARM_DESC(vinetic_major, "Major number for Polygator Linux module VINETIC support");

EXPORT_SYMBOL(vinetic_device_register);
EXPORT_SYMBOL(vinetic_device_unregister);
EXPORT_SYMBOL(vinetic_rtp_channel_register);
EXPORT_SYMBOL(vinetic_rtp_channel_unregister);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) device_create(_class, _device, _devt, NULL, "%s", _name)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) device_create(_class, _device, _devt, _name)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,15)
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) class_device_create(_class, NULL, _devt, _device, _name)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) class_device_create(_class, _devt, _device, _name)
#else
	#define CLASS_DEV_CREATE(_class, _devt, _device, _name) class_simple_device_add(_class, _devt, _device, _name)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	#define CLASS_DEV_DESTROY(_class, _devt) device_destroy(_class, _devt)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
	#define CLASS_DEV_DESTROY(_class, _devt) class_device_destroy(_class, _devt)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,9)
	#define CLASS_DEV_DESTROY(_class, _devt) class_simple_device_remove(_devt)
#else
	#define CLASS_DEV_DESTROY(_class, _devt) class_simple_device_remove(_class, _devt)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,13)
	static struct class *vinetic_class = NULL;
#else
	static struct class_simple *vinetic_class = NULL;
	#define class_create(_a, _b) class_simple_create(_a, _b)
	#define class_destroy(_a) class_simple_destroy(_a)
#endif

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, "vinetic", ## _args)
#define log(_level, _fmt, _args...) printk(_level "polygator-%s] %s:%d - %s(): " _fmt, "vinetic", "vinetic.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, "vinetic", "vinetic.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

#define VINETIC_WAIT_COUNT 10
#define VINETIC_WAIT_TIMEOUT 1

struct vinetic_device {
	char name[VINETIC_DEVNAME_MAXLEN];
	int devno;
	u_int32_t type;
	void *data;
};

static struct vinetic_device vinetic_device_list[VINETIC_DEVICE_MAXCOUNT];
static DEFINE_MUTEX(vinetic_device_list_lock);

static void vinetic_poll_proc(struct timer_list *timer)
{
    union vin_cmd cmd;

    u_int16_t res;

    size_t ch;
    size_t pkt_write[8];

    struct vin_status_registers status_changed;

    size_t wr;

    size_t len;

    size_t cnt = 0;

    size_t wait_count;

    u_int16_t *datap;

    struct vinetic_rtp_channel *rtp;
    struct vinetic *vin = container_of(timer, struct vinetic, poll_timer);

	// lock vinetic
	spin_lock(&vin->lock);

	// check data for read
	for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
		if (!vin->is_not_ready(vin->cbdata)) {
			break;
		}
		udelay(VINETIC_WAIT_TIMEOUT);
	}
	if (wait_count >= VINETIC_WAIT_COUNT) {
		log(KERN_ERR, "%s: timeout\n", vin->name);
		goto vinetic_poll_proc_error;
	}
	vin->write_nwd(vin->cbdata, VIN_rOBXML);
	for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
		if (!vin->is_not_ready(vin->cbdata)) {
			break;
		}
		udelay(VINETIC_WAIT_TIMEOUT);
	}
	if (wait_count >= VINETIC_WAIT_COUNT) {
		log(KERN_ERR, "%s: timeout\n", vin->name);
		goto vinetic_poll_proc_error;
	}
	res = vin->read_eom(vin->cbdata);
	vin->pdata_size = res & 0xff;
	vin->cdata_size = (res >> 8) & 0x1f;

	// read voice packet from mailbox
	if (vin->pdata_size) {
		// write short commands rPOBX
		for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
			if (!vin->is_not_ready(vin->cbdata)) {
				break;
			}
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count >= VINETIC_WAIT_COUNT) {
			log(KERN_ERR, "%s: timeout\n", vin->name);
			goto vinetic_poll_proc_error;
		}
		vin->write_nwd(vin->cbdata, VIN_rPOBX);
		while (vin->pdata_size) {
			// read vop/evt packet first part
			vin->pdata_size--;
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				log(KERN_ERR, "%s: timeout\n", vin->name);
				goto vinetic_poll_proc_error;
			}
			cmd.parts.first.full = vin->read_nwd(vin->cbdata);
			// check for read marker
			if (cmd.parts.first.bits.rw != VIN_READ) {
				log(KERN_ERR, "%s: write type is wrong\n", vin->name);
				goto vinetic_poll_proc_error;
			}
			// check for is not short command
			if (cmd.parts.first.bits.sc != VIN_SC_NO) {
				log(KERN_ERR, "%s: short command is wrong\n", vin->name);
				goto vinetic_poll_proc_error;
			}
			// check for is not broadcast command
			if (cmd.parts.first.bits.bc != VIN_BC_NO) {
				log(KERN_ERR, "%s: broadcast command is wrong\n", vin->name);
				goto vinetic_poll_proc_error;
			}
			// check for vop/evt packet type
			if ((cmd.parts.first.bits.cmd != VIN_CMD_VOP) && (cmd.parts.first.bits.cmd != VIN_CMD_EVT)) {
				log(KERN_ERR, "%s: wrong packet type=%lu\n", vin->name, (long unsigned int)cmd.parts.first.bits.cmd);
				goto vinetic_poll_proc_error;
			}
			// check for valid channel number
			if (cmd.parts.first.bits.chan > 7) {
				log(KERN_ERR, "%s: wrong channel number=%lu\n", vin->name, (long unsigned int)cmd.parts.first.bits.chan);
				goto vinetic_poll_proc_error;
			}
			// read vop/evt packet second part
			vin->pdata_size--;
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				log(KERN_ERR, "%s: timeout\n", vin->name);
				goto vinetic_poll_proc_error;
			}
			cmd.parts.second.full = vin->read_nwd(vin->cbdata);
			// get packet data length
			cnt = cmd.parts.second.vop.bits.length;
			if (cnt > 253) {
				log(KERN_ERR, "%s: wrong voice packet length=%lu\n", vin->name, (long unsigned int)cnt);
				goto vinetic_poll_proc_error;
			}
			// sort packet by channel
			rtp = vin->rtp_channels[cmd.parts.first.bits.chan];
			if (rtp) {
				spin_lock(&rtp->lock);
				rtp->read_slot[rtp->read_slot_write].length = cnt * 2;
				if (cmd.parts.second.vop.bits.odd) {
					rtp->read_slot[rtp->read_slot_write].length--;
				}
				datap = rtp->read_slot[rtp->read_slot_write].data;
				while (vin->pdata_size && cnt) {
					vin->pdata_size--;
					cnt--;
					for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
						if (!vin->is_not_ready(vin->cbdata)) {
							break;
						}
						udelay(VINETIC_WAIT_TIMEOUT);
					}
					if (wait_count >= VINETIC_WAIT_COUNT) {
						spin_unlock(&rtp->lock);
						log(KERN_ERR, "%s: timeout\n", vin->name);
						goto vinetic_poll_proc_error;
					}
					if (vin->pdata_size) {
						*datap++ = vin->read_nwd(vin->cbdata);
					} else {
						*datap++ = vin->read_eom(vin->cbdata);
					}
				}
				// adjust read packet slot index
				rtp->read_slot_count++;
				if (rtp->read_slot_count > VINETIC_PACKETSLOT_MAXCOUNT) {
					rtp->read_slot_count = VINETIC_PACKETSLOT_MAXCOUNT;
					rtp->read_slot_read++;
					if (rtp->read_slot_read >= VINETIC_PACKETSLOT_MAXCOUNT) {
						rtp->read_slot_read = 0;
					}
				}
				rtp->read_slot_write++;
				if (rtp->read_slot_write >= VINETIC_PACKETSLOT_MAXCOUNT) {
					rtp->read_slot_write = 0;
				}
				// wake read_slot_count waitqueue
				wake_up_interruptible(&rtp->read_slot_count_waitq);
				// wake poll waitqueue
				wake_up_interruptible(&rtp->poll_waitq);
				spin_unlock(&rtp->lock);
			} else {
				// unknown data packet - flush out mailbox
				while (vin->pdata_size && cnt) {
					vin->pdata_size--;
					cnt--;
					for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
						if (!vin->is_not_ready(vin->cbdata)) {
							break;
						}
						udelay(VINETIC_WAIT_TIMEOUT);
					}
					if (wait_count >= VINETIC_WAIT_COUNT) {
						log(KERN_ERR, "%s: timeout\n", vin->name);
						goto vinetic_poll_proc_error;
					}
					if (vin->pdata_size) {
						vin->read_nwd(vin->cbdata);
					} else {
						vin->read_eom(vin->cbdata);
					}
				}
			}
		}
	}

	// read command data from mailbox
	if (vin->cdata_size) {
// 		debug("cdata_size=%lu\n", vin->cdata_size);
		// write short commands rCOBX
		for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
			if (!vin->is_not_ready(vin->cbdata)) {
				break;
			}
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count >= VINETIC_WAIT_COUNT) {
			log(KERN_ERR, "%s: timeout\n", vin->name);
			goto vinetic_poll_proc_error;
		}
		vin->write_nwd(vin->cbdata, VIN_rCOBX);
		// read rest data
		vin->read_cbox_length = vin->cdata_size;
		datap = vin->read_cbox_data;
		while (vin->cdata_size) {
			vin->cdata_size--;
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				log(KERN_ERR, "%s: timeout\n", vin->name);
				goto vinetic_poll_proc_error;
			}
			if (vin->cdata_size) {
				*datap++ = vin->read_nwd(vin->cbdata);
			} else {
				*datap++ = vin->read_eom(vin->cbdata);
			}
		}
		wake_up_interruptible(&vin->read_cbox_waitq);
	}
	// write voice packet into vinetic
	for (ch = 0; ch < 8; ch++) {
		pkt_write[ch] = 1;
	}
	wr = 1;
	while (wr) {
		// test for write is actual
		wr = 0;
		for (ch = 0; ch < 8; ch++) {
			if (pkt_write[ch]) {
				wr = 1;
				break;
			}
		}
		// check free mailbox space
		for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
			if (!vin->is_not_ready(vin->cbdata)) {
				break;
			}
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count >= VINETIC_WAIT_COUNT) {
			log(KERN_ERR, "%s: timeout\n", vin->name);
			goto vinetic_poll_proc_error;
		}
		vin->write_nwd(vin->cbdata, VIN_rFIBXMS);
		for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
			if (!vin->is_not_ready(vin->cbdata)) {
				break;
			}
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count >= VINETIC_WAIT_COUNT) {
			log(KERN_ERR, "%s: timeout\n", vin->name);
			goto vinetic_poll_proc_error;
		}
		res = vin->read_eom(vin->cbdata);
		vin->free_pbox_space = res & 0xff;
		for (ch = 0; ch < 8; ch++) {
			if (pkt_write[ch]) {
				rtp = vin->rtp_channels[ch];
				if (rtp) {
					spin_lock(&rtp->lock);
					if (rtp->write_slot_count) {
						len = rtp->write_slot[rtp->write_slot_read].length / 2;
						if (rtp->write_slot[rtp->write_slot_read].length % 2) {
							len++;
						}
						if (vin->free_pbox_space >= (len + 2)) {
							cmd.parts.first.bits.rw = VIN_WRITE;
							cmd.parts.first.bits.sc = VIN_SC_NO;
							cmd.parts.first.bits.bc = VIN_BC_NO;
							cmd.parts.first.bits.cmd = VIN_CMD_VOP;
							cmd.parts.first.bits.res = 0;
							cmd.parts.first.bits.chan = ch;
							cmd.parts.second.vop.bits.res1 = 0;
							cmd.parts.second.vop.bits.odd = rtp->write_slot[rtp->write_slot_read].length%2;
							cmd.parts.second.vop.bits.res0 = 0;
							cmd.parts.second.vop.bits.length = len;
							// write voice packet header to vinetic
							for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
								if (!vin->is_not_ready(vin->cbdata)) {
									break;
								}
								udelay(VINETIC_WAIT_TIMEOUT);
							}
							if (wait_count >= VINETIC_WAIT_COUNT) {
								spin_unlock(&rtp->lock);
								log(KERN_ERR, "%s: timeout\n", vin->name);
								goto vinetic_poll_proc_error;
							}
							vin->write_nwd(vin->cbdata, cmd.parts.first.full);
							for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
								if (!vin->is_not_ready(vin->cbdata)) {
									break;
								}
								udelay(VINETIC_WAIT_TIMEOUT);
							}
							if (wait_count >= VINETIC_WAIT_COUNT) {
								spin_unlock(&rtp->lock);
								log(KERN_ERR, "%s: timeout\n", vin->name);
								goto vinetic_poll_proc_error;
							}
							vin->write_nwd(vin->cbdata, cmd.parts.second.full);
							// write voice packet data to vinetic
							for (cnt = 0; cnt < len; cnt++) {
								for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
									if (!vin->is_not_ready(vin->cbdata)) {
										break;
									}
									udelay(VINETIC_WAIT_TIMEOUT);
								}
								if (wait_count >= VINETIC_WAIT_COUNT) {
									spin_unlock(&rtp->lock);
									log(KERN_ERR, "%s: timeout\n", vin->name);
									goto vinetic_poll_proc_error;
								}
								if (cnt == (len - 1)) {
									vin->write_eom(vin->cbdata, rtp->write_slot[rtp->write_slot_read].data[cnt]);
								} else {
									vin->write_nwd(vin->cbdata, rtp->write_slot[rtp->write_slot_read].data[cnt]);
								}
							}
							// adjust free pbox space
							vin->free_pbox_space -= (len + 2);
							// adjust write packet slot index
							rtp->write_slot_read++;
							if (rtp->write_slot_read >= VINETIC_PACKETSLOT_MAXCOUNT) {
								rtp->write_slot_read = 0;
							}
							rtp->write_slot_count--;
							// wake write_slot_count waitqueue
							wake_up_interruptible(&rtp->write_slot_count_waitq);
							// wake poll waitqueue
							wake_up_interruptible(&rtp->poll_waitq);
						}
						pkt_write[ch]--;
					} else {
						pkt_write[ch] = 0;
					}
					spin_unlock(&rtp->lock);
				} else {
					pkt_write[ch] = 0;
				}
			}
		}
	}
	// read edsp channel status
	for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
		if (!vin->is_not_ready(vin->cbdata)) {
			break;
		}
		udelay(VINETIC_WAIT_TIMEOUT);
	}
	if (wait_count >= VINETIC_WAIT_COUNT) {
		log(KERN_ERR, "%s: timeout\n", vin->name);
		goto vinetic_poll_proc_error;
	}
	vin->write_nwd(vin->cbdata, VIN_rSR(VIN_BC, 0));
	datap = (u_int16_t *)&vin->status.sr;
	for (cnt = 0; cnt < 24; cnt++) {
		for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
			if (!vin->is_not_ready(vin->cbdata)) {
				break;
			}
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count >= VINETIC_WAIT_COUNT) {
			log(KERN_ERR, "%s: timeout\n", vin->name);
			goto vinetic_poll_proc_error;
		}
		if (cnt == 23) {
			*datap++ = vin->read_eom(vin->cbdata);
		} else {
			*datap++ = vin->read_nwd(vin->cbdata);
		}
	}
	// read hardware status
	for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
		if (!vin->is_not_ready(vin->cbdata)) {
			break;
		}
		udelay(VINETIC_WAIT_TIMEOUT);
	}
	if (wait_count >= VINETIC_WAIT_COUNT) {
		log(KERN_ERR, "%s: timeout\n", vin->name);
		goto vinetic_poll_proc_error;
	}
	vin->write_nwd(vin->cbdata, VIN_rHWSR);
	datap = (u_int16_t *)&vin->status.hwsr;
	for (cnt = 0; cnt < 2; cnt++) {
		for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
			if (!vin->is_not_ready(vin->cbdata)) {
				break;
			}
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count >= VINETIC_WAIT_COUNT) {
			log(KERN_ERR, "%s: timeout\n", vin->name);
			goto vinetic_poll_proc_error;
		}
		if (cnt == 1) {
			*datap++ = vin->read_eom(vin->cbdata);
		} else {
			*datap++ = vin->read_nwd(vin->cbdata);
		}
	}
	// read mailbox status
	for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
		if (!vin->is_not_ready(vin->cbdata)) {
			break;
		}
		udelay(VINETIC_WAIT_TIMEOUT);
	}
	if (wait_count >= VINETIC_WAIT_COUNT) {
		log(KERN_ERR, "%s: timeout\n", vin->name);
		goto vinetic_poll_proc_error;
	}
	vin->write_nwd(vin->cbdata, VIN_rBXSR);
	datap = (u_int16_t *)&vin->status.bxsr;
	for (cnt = 0; cnt < 2; cnt++) {
		for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
			if (!vin->is_not_ready(vin->cbdata)) {
				break;
			}
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count >= VINETIC_WAIT_COUNT) {
			log(KERN_ERR, "%s: timeout\n", vin->name);
			goto vinetic_poll_proc_error;
		}
		if (cnt == 1) {
			*datap++ = vin->read_eom(vin->cbdata);
		} else {
			*datap++ = vin->read_nwd(vin->cbdata);
		}
	}
	// get changes
	status_changed.sr.sre1_0.full = (vin->status.sr.sre1_0.full ^ vin->status_old.sr.sre1_0.full) & vin->status_mask.sr.sre1_0.full;
	status_changed.sr.sre2_0.full = (vin->status.sr.sre2_0.full ^ vin->status_old.sr.sre2_0.full) & vin->status_mask.sr.sre2_0.full;
	status_changed.sr.srs1_0.full = (vin->status.sr.srs1_0.full ^ vin->status_old.sr.srs1_0.full) & vin->status_mask.sr.srs1_0.full;
	status_changed.sr.srs2_0.full = (vin->status.sr.srs2_0.full ^ vin->status_old.sr.srs2_0.full) & vin->status_mask.sr.srs2_0.full;
	status_changed.sr.sre1_1.full = (vin->status.sr.sre1_1.full ^ vin->status_old.sr.sre1_1.full) & vin->status_mask.sr.sre1_1.full;
	status_changed.sr.sre2_1.full = (vin->status.sr.sre2_1.full ^ vin->status_old.sr.sre2_1.full) & vin->status_mask.sr.sre2_1.full;
	status_changed.sr.srs1_1.full = (vin->status.sr.srs1_1.full ^ vin->status_old.sr.srs1_1.full) & vin->status_mask.sr.srs1_1.full;
	status_changed.sr.srs2_1.full = (vin->status.sr.srs2_1.full ^ vin->status_old.sr.srs2_1.full) & vin->status_mask.sr.srs2_1.full;
	status_changed.sr.sre1_2.full = (vin->status.sr.sre1_2.full ^ vin->status_old.sr.sre1_2.full) & vin->status_mask.sr.sre1_2.full;
	status_changed.sr.sre2_2.full = (vin->status.sr.sre2_2.full ^ vin->status_old.sr.sre2_2.full) & vin->status_mask.sr.sre2_2.full;
	status_changed.sr.srs1_2.full = (vin->status.sr.srs1_2.full ^ vin->status_old.sr.srs1_2.full) & vin->status_mask.sr.srs1_2.full;
	status_changed.sr.srs2_2.full = (vin->status.sr.srs2_2.full ^ vin->status_old.sr.srs2_2.full) & vin->status_mask.sr.srs2_2.full;
	status_changed.sr.sre1_3.full = (vin->status.sr.sre1_3.full ^ vin->status_old.sr.sre1_3.full) & vin->status_mask.sr.sre1_3.full;
	status_changed.sr.sre2_3.full = (vin->status.sr.sre2_3.full ^ vin->status_old.sr.sre2_3.full) & vin->status_mask.sr.sre2_3.full;
	status_changed.sr.srs1_3.full = (vin->status.sr.srs1_3.full ^ vin->status_old.sr.srs1_3.full) & vin->status_mask.sr.srs1_3.full;
	status_changed.sr.srs2_3.full = (vin->status.sr.srs2_3.full ^ vin->status_old.sr.srs2_3.full) & vin->status_mask.sr.srs2_3.full;
	status_changed.sr.sre1_4.full = (vin->status.sr.sre1_4.full ^ vin->status_old.sr.sre1_4.full) & vin->status_mask.sr.sre1_4.full;
	status_changed.sr.sre2_4.full = (vin->status.sr.sre2_4.full ^ vin->status_old.sr.sre2_4.full) & vin->status_mask.sr.sre2_4.full;
	status_changed.sr.sre1_5.full = (vin->status.sr.sre1_5.full ^ vin->status_old.sr.sre1_5.full) & vin->status_mask.sr.sre1_5.full;
	status_changed.sr.sre2_5.full = (vin->status.sr.sre2_5.full ^ vin->status_old.sr.sre2_5.full) & vin->status_mask.sr.sre2_5.full;
	status_changed.sr.sre1_6.full = (vin->status.sr.sre1_6.full ^ vin->status_old.sr.sre1_6.full) & vin->status_mask.sr.sre1_6.full;
	status_changed.sr.sre2_6.full = (vin->status.sr.sre2_6.full ^ vin->status_old.sr.sre2_6.full) & vin->status_mask.sr.sre2_6.full;
	status_changed.sr.sre1_7.full = (vin->status.sr.sre1_7.full ^ vin->status_old.sr.sre1_7.full) & vin->status_mask.sr.sre1_7.full;
	status_changed.sr.sre2_7.full = (vin->status.sr.sre2_7.full ^ vin->status_old.sr.sre2_7.full) & vin->status_mask.sr.sre2_7.full;
	status_changed.hwsr.hwsr1.full = (vin->status.hwsr.hwsr1.full ^ vin->status_old.hwsr.hwsr1.full) & vin->status_mask.hwsr.hwsr1.full;
	status_changed.hwsr.hwsr2.full = (vin->status.hwsr.hwsr2.full ^ vin->status_old.hwsr.hwsr2.full) & vin->status_mask.hwsr.hwsr2.full;
	status_changed.bxsr.bxsr1.full = (vin->status.bxsr.bxsr1.full ^ vin->status_old.bxsr.bxsr1.full) & vin->status_mask.bxsr.bxsr1.full;
	status_changed.bxsr.bxsr2.full = (vin->status.bxsr.bxsr2.full ^ vin->status_old.bxsr.bxsr2.full) & vin->status_mask.bxsr.bxsr2.full;
	// store current status as old
	memcpy(&vin->status_old, &vin->status, sizeof(struct vin_status_registers));
	// check status
	if (status_changed.sr.sre1_0.full || status_changed.sr.sre2_0.full || status_changed.sr.srs1_0.full || status_changed.sr.srs2_0.full ||
		status_changed.sr.sre1_1.full || status_changed.sr.sre2_1.full || status_changed.sr.srs1_1.full || status_changed.sr.srs2_1.full ||
		status_changed.sr.sre1_2.full || status_changed.sr.sre2_2.full || status_changed.sr.srs1_2.full || status_changed.sr.srs2_2.full ||
		status_changed.sr.sre1_3.full || status_changed.sr.sre2_3.full || status_changed.sr.srs1_3.full || status_changed.sr.srs2_3.full ||
		status_changed.sr.sre1_4.full || status_changed.sr.sre2_4.full ||
		status_changed.sr.sre1_5.full || status_changed.sr.sre2_5.full ||
		status_changed.sr.sre1_6.full || status_changed.sr.sre2_6.full ||
		status_changed.sr.sre1_7.full || status_changed.sr.sre2_7.full ||
		status_changed.hwsr.hwsr1.full ||
		status_changed.hwsr.hwsr2.full ||
		status_changed.bxsr.bxsr1.full ||
		status_changed.bxsr.bxsr2.full ) {
		vin->status_ready = 1;
		wake_up_interruptible(&vin->status_waitq);
	}
	// check free mailbox space
	for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
		if (!vin->is_not_ready(vin->cbdata)) {
			break;
		}
		udelay(VINETIC_WAIT_TIMEOUT);
	}
	if (wait_count >= VINETIC_WAIT_COUNT) {
		log(KERN_ERR, "%s: timeout\n", vin->name);
		goto vinetic_poll_proc_error;
	}
	vin->write_nwd(vin->cbdata, VIN_rFIBXMS);
	for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
		if (!vin->is_not_ready(vin->cbdata)) {
			break;
		}
		udelay(VINETIC_WAIT_TIMEOUT);
	}
	if (wait_count >= VINETIC_WAIT_COUNT) {
		log(KERN_ERR, "%s: timeout\n", vin->name);
		goto vinetic_poll_proc_error;
	}
	res = vin->read_eom(vin->cbdata);
	vin->free_cbox_space = (res >> 8) & 0xff;
	if (vin->free_cbox_space) {
		wake_up_interruptible(&vin->free_cbox_waitq);
		wake_up_interruptible(&vin->seek_cbox_waitq);
	}

	spin_unlock(&vin->lock);
	if (vin->poll) {
#if HZ >= 200
		mod_timer(&vin->poll_timer, jiffies + (5 * HZ) / 1000);
#else
		mod_timer(&vin->poll_timer, jiffies + 1);
#endif
	}
	return;

vinetic_poll_proc_error:
	vin->status.custom.bits.timeout = 1;
	wake_up_interruptible(&vin->free_cbox_waitq);
	vin->read_cbox_length = 0x100;
	wake_up_interruptible(&vin->read_cbox_waitq);
// 	vin->filp->f_pos = 0;
	wake_up_interruptible(&vin->seek_cbox_waitq);
	vin->status_ready = 1;
	wake_up_interruptible(&vin->status_waitq);
	spin_unlock(&vin->lock);
}

static int vinetic_open(struct inode *inode, struct file *filp)
{
	struct vinetic *vin;

	vin = container_of(inode->i_cdev, struct vinetic, cdev);
	filp->private_data = vin;

	spin_lock_bh(&vin->lock);
// 	if (!vin->usage) {
// 		vin->filp = filp;
// 	}
	vin->usage++;
	spin_unlock_bh(&vin->lock);
	return 0;
}

static int vinetic_release(struct inode *inode, struct file *filp)
{
	size_t usage;
	struct vinetic *vin = filp->private_data;

	spin_lock_bh(&vin->lock);
	usage = --vin->usage;
	if (!usage) {
// 		vin->filp = NULL;
		vin->poll = 0;
	}
	spin_unlock_bh(&vin->lock);

	if (!usage) {
		del_timer_sync(&vin->poll_timer);
	}
	return 0;
}

static ssize_t vinetic_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	u_int16_t data[32];
	size_t wait_count;
	size_t cnt;
	ssize_t res;

	union vin_cmd cmd;
	union vin_cmd_short cmd_short;

	struct vin_status_registers status;

	struct vinetic *vin = filp->private_data;

	res = 0;

	cmd.full = filp->f_pos & 0xffffffff;

	// read vinetic status
	if (cmd.full == 0xffffffff) {
		// sleeping
		wait_event_interruptible(vin->status_waitq, vin->status_ready != 0);
		// copy status
		spin_lock_bh(&vin->lock);
		memcpy(&status, &vin->status, sizeof(struct vin_status_registers));
		vin->status_ready = 0;
		spin_unlock_bh(&vin->lock);
		cnt = sizeof(struct vin_status_registers);
		cnt = min(cnt, count);
		// copy data to user
		if (copy_to_user(buff, &status, cnt)) {
			res = -EFAULT;
		} else {
			res = cnt;
		}
		goto vinetic_read_end;
	}
	// check for is read command
	if (cmd.parts.first.bits.rw != VIN_READ) {
		log(KERN_ERR, "\"%s\": is write command=0x%08x\n", vin->name, cmd.full);
		res = -EINVAL;
		goto vinetic_read_end;
	}
	
	if (cmd.parts.first.bits.sc) {
		// short command
		cmd_short.full = cmd.parts.first.full;
		spin_lock_bh(&vin->lock);
		// write command to vinetic
		for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
			if (!vin->is_not_ready(vin->cbdata)) {
				break;
			}
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count >= VINETIC_WAIT_COUNT) {
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_read_end;
		}
		vin->write_nwd(vin->cbdata, cmd_short.full);
		// get actual data length to read
		switch ((cmd_short.full >> 4) & 0x1f) {
			case VIN_SH_CMD_CODE_rIR:
				res = 1;
				break;
			case VIN_SH_CMD_CODE_rSR:
			case VIN_SH_CMD_CODE_rI_SR:
				if (cmd_short.bits.bc) {
					res = 24;
					break;
				} else {
					switch (cmd_short.bits.chan) {
						case 0: case 1: case 2: case 3:
							res = 4;
							break;
						case 4: case 5: case 6: case 7:
							res = 2;
							break;
						default:
							spin_unlock_bh(&vin->lock);
							log(KERN_ERR, "\"%s\": rSR/rI_SR wrong channel index=%u\n", vin->name, cmd_short.bits.chan);
							res = -EINVAL;
							goto vinetic_read_end;
					}
				}
				break;
			case VIN_SH_CMD_CODE_rSRS:
			case VIN_SH_CMD_CODE_rI_SRS:
				if (cmd_short.bits.bc) {
					res = 16;
				} else {
					res = 2;
				}
				break;
			case VIN_SH_CMD_CODE_rHWSR:
			case VIN_SH_CMD_CODE_rI_HWSR:
				res = 2;
				break;
			case VIN_SH_CMD_CODE_rBXSR:
			case VIN_SH_CMD_CODE_rI_BXSR:
				res = 2;
				break;
			case VIN_SH_CMD_CODE_rSRGPIO:
			case VIN_SH_CMD_CODE_rI_SRGPIO:
				res =1;
				break;
			case VIN_SH_CMD_CODE_rFIBXMS:
				res = 1;
				break;
			case VIN_SH_CMD_CODE_rOBXML:
				res = 1;
				break;
			case VIN_SH_CMD_CODE_rPOBX:
				spin_unlock_bh(&vin->lock);
				log(KERN_ERR, "\"%s\": rPOBX unsupported from userspace\n", vin->name);
				res = -EINVAL;
				goto vinetic_read_end;
			case VIN_SH_CMD_CODE_rCOBX:
				spin_unlock_bh(&vin->lock);
				log(KERN_ERR, "\"%s\": rCOBX unsupported from userspace\n", vin->name);
				res = -EINVAL;
				goto vinetic_read_end;
			default:
				spin_unlock_bh(&vin->lock);
				log(KERN_ERR, "\"%s\": unknown short command=0x%04x\n", vin->name, cmd_short.full);
				res = -EINVAL;
				goto vinetic_read_end;
		}
		// read data from vinetic
		for (cnt = 0; cnt < res; cnt++) {
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				goto vinetic_read_end;
			}
			if (cnt == res-1) {
				data[cnt] = vin->read_eom(vin->cbdata);
			} else {
				data[cnt] = vin->read_nwd(vin->cbdata);
			}
		}
		// check data count
		if (count < res*2) {
			spin_unlock_bh(&vin->lock);
			log(KERN_ERR, "\"%s\": data count=%lu less than actual readed=%lu\n", vin->name, (long unsigned int)count, (long unsigned int)res*2);
			res = -EINVAL;
			goto vinetic_read_end;
		}
		spin_unlock_bh(&vin->lock);
		// copy data to user
		if (copy_to_user(buff, data, res*2)) {
			res = -EFAULT;
			goto vinetic_read_end;
		}
		res *= 2;
	} else {
		// regular command
		spin_lock_bh(&vin->lock);
		// check for free space in mailbox
		cnt = HZ;
		for (;;) {
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				goto vinetic_read_end;
			}
			vin->write_nwd(vin->cbdata, VIN_rFIBXMS);
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				goto vinetic_read_end;
			}
			vin->free_cbox_space = (vin->read_eom(vin->cbdata) >> 8) & 0xff;
			if (vin->free_cbox_space >= 2) {
				break;
			}
			if (filp->f_flags & O_NONBLOCK) {
				spin_unlock_bh(&vin->lock);
				res = -EAGAIN;
				goto vinetic_read_end;
			}
			// sleeping
			spin_unlock_bh(&vin->lock);
			wait_event_interruptible_timeout(vin->free_cbox_waitq, vin->free_cbox_space >= 2, 1);
			cnt--;
			if (!cnt) {
				res = -EIO;
				goto vinetic_read_end;
			}
			spin_lock_bh(&vin->lock);
		}
		// write command to vinetic
		for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
			if (!vin->is_not_ready(vin->cbdata)) {
				break;
			}
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count >= VINETIC_WAIT_COUNT) {
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_read_end;
		}
		vin->write_nwd(vin->cbdata, cmd.parts.first.full);
		for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
			if (!vin->is_not_ready(vin->cbdata)) {
				break;
			}
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count >= VINETIC_WAIT_COUNT) {
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_read_end;
		}
		vin->write_eom(vin->cbdata, cmd.parts.second.full);
		vin->read_cbox_length = 0;
		spin_unlock_bh(&vin->lock);
		res = wait_event_interruptible(vin->read_cbox_waitq, vin->read_cbox_length != 0);
		spin_lock_bh(&vin->lock);
		if (vin->status.custom.bits.timeout) {
			res = -EIO;
		}
		spin_unlock_bh(&vin->lock);
		if (res) {
			goto vinetic_read_end;
		}
		spin_lock_bh(&vin->lock);
		res = vin->read_cbox_length * 2;
		memcpy(data, vin->read_cbox_data, vin->read_cbox_length * 2);
		spin_unlock_bh(&vin->lock);
		// check data count
		if (count < res) {
			log(KERN_ERR, "\"%s\": data count=%lu less than actual readed=%lu\n", vin->name, (long unsigned int)count, (long unsigned int)res*2);
			res = -EINVAL;
			goto vinetic_read_end;
		}
		// copy data to user
		if (copy_to_user(buff, data, res)) {
			res = -EFAULT;
			goto vinetic_read_end;
		}
	}

vinetic_read_end:
	spin_lock_bh(&vin->lock);
	*offp = 0;
	wake_up_interruptible(&vin->seek_cbox_waitq);
	if (res == -EIO) {
		vin->status.custom.bits.timeout = 1;
		vin->status_ready = 1;
		wake_up_interruptible(&vin->status_waitq);
	}
	spin_unlock_bh(&vin->lock);
	return res;
}

static ssize_t vinetic_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	u_int16_t data[256];
	size_t cnt, length;
	size_t wait_count;
	ssize_t res;

	union vin_cmd cmd;

	struct vin_status_registers mask;

	struct vinetic *vin = filp->private_data;

	res = 0;

	cmd.full = filp->f_pos & 0xffffffff;

	// set vinetic status mask
	if (cmd.full == 0xffffffff) {
		// copy data from user
		cnt = sizeof(struct vin_status_registers);
		cnt = min(cnt, count);
		if (copy_from_user(&mask, buff, cnt)) {
			res = -EFAULT;
			goto vinetic_write_end;
		}
		// copy status mask
		spin_lock_bh(&vin->lock);
		memcpy(&vin->status_mask, &mask, sizeof(struct vin_status_registers));
		spin_unlock_bh(&vin->lock);
		res = cnt;
		goto vinetic_write_end;
	}
	// check for is write command
	if (cmd.parts.first.bits.rw != VIN_WRITE) {
		log(KERN_ERR, "\"%s\": is read command=0x%08x\n", vin->name, cmd.full);
		res = -EINVAL;
		goto vinetic_write_end;
	}

	if (cmd.parts.first.bits.sc) {
		// short command
		memcpy(data, &cmd.parts.first.full, sizeof(u_int16_t));
		res = 0;
		length = 1;
	} else {
		// regular command
		switch (cmd.parts.first.bits.cmd) {
			case VIN_CMD_SOP:
				memcpy(data, &cmd.full, sizeof(u_int32_t));
				res = cmd.parts.second.sop.bits.length;
				length = cmd.parts.second.sop.bits.length + 2;
				break;
			case VIN_CMD_COP:
				memcpy(data, &cmd.full, sizeof(u_int32_t));
				res = cmd.parts.second.sop.bits.length;
				length = cmd.parts.second.cop.bits.length + 2;
				break;
			case VIN_CMD_IOP:
				memcpy(data, &cmd.full, sizeof(u_int32_t));
				res = cmd.parts.second.sop.bits.length;
				length = cmd.parts.second.iop.bits.length + 2;
				break;
			case VIN_CMD_EOP:
				memcpy(data, &cmd.full, sizeof(u_int32_t));
				res = cmd.parts.second.sop.bits.length;
				length = cmd.parts.second.eop.bits.length + 2;
				break;
			default:
				log(KERN_ERR, "\"%s\": unknown cmd=%d\n", vin->name, cmd.parts.first.bits.cmd);
				res = -EINVAL;
				goto vinetic_write_end;
		}
	}
	// check data count
	if (count != res*2) {
		log(KERN_ERR, "\"%s\": data count=%lu mismatch to %lu\n", vin->name, (long unsigned int)count, (long unsigned int)res*2);
		res = -EINVAL;
		goto vinetic_write_end;
	}

	// get user space data
	if (res > 0) {
		if (copy_from_user(&data[2], buff, res*2)) {
			res = -EFAULT;
			goto vinetic_write_end;
		}
	}
	// check for free space in mailbox
	spin_lock_bh(&vin->lock);
	cnt = HZ;
	for (;;) {
		for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
			if (!vin->is_not_ready(vin->cbdata)) {
				break;
			}
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count >= VINETIC_WAIT_COUNT) {
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_write_end;
		}
		vin->write_nwd(vin->cbdata, VIN_rFIBXMS);
		for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
			if (!vin->is_not_ready(vin->cbdata)) {
				break;
			}
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count >= VINETIC_WAIT_COUNT) {
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_write_end;
		}
		vin->free_cbox_space = (vin->read_eom(vin->cbdata) >> 8) & 0xff;
		if (vin->free_cbox_space >= length) {
			break;
		}
		if (filp->f_flags & O_NONBLOCK) {
			spin_unlock_bh(&vin->lock);
			res = -EAGAIN;
			goto vinetic_write_end;
		}
		// sleeping
		spin_unlock_bh(&vin->lock);
		wait_event_interruptible_timeout(vin->free_cbox_waitq, vin->free_cbox_space >= length, 1);
		cnt--;
		if (!cnt) {
			res = -EIO;
			goto vinetic_write_end;
		}
		spin_lock_bh(&vin->lock);
	}
	// write data to vinetic
	for (cnt = 0; cnt < length; cnt++) {
		for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
			if (!vin->is_not_ready(vin->cbdata)) {
				break;
			}
			udelay(VINETIC_WAIT_TIMEOUT);
		}
		if (wait_count >= VINETIC_WAIT_COUNT) {
			spin_unlock_bh(&vin->lock);
			res = -EIO;
			goto vinetic_write_end;
		}
		if (cnt == (length - 1)) {
			vin->write_eom(vin->cbdata, data[cnt]);
		} else {
			vin->write_nwd(vin->cbdata, data[cnt]);
		}
	}
	spin_unlock_bh(&vin->lock);
	res *= 2;

vinetic_write_end:
	spin_lock_bh(&vin->lock);
	*offp = 0;
	wake_up_interruptible(&vin->seek_cbox_waitq);
	if (res == -EIO) {
		vin->status.custom.bits.timeout = 1;
		vin->status_ready = 1;
		wake_up_interruptible(&vin->status_waitq);
	}
	spin_unlock_bh(&vin->lock);
	return res;
}

static int vinetic_generic_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	u_int16_t phi;
	size_t wait_count;
	size_t cnt;
	int not_ready;
	int res = 0;
	struct vinetic *vin = filp->private_data;
	void __user *argp = (void __user *)data;

// 	debug("%s cmd=0x%08x\n", vin->name, cmd);

	switch (cmd) {
		case VINETIC_RESET:
			spin_lock_bh(&vin->lock);
			vin->reset(vin->cbdata);
			memset(&vin->status, 0, sizeof(struct vin_status_registers));
			vin->status_ready = 0;
			spin_unlock_bh(&vin->lock);
			break;
		case VINETIC_RESET_RDYQ:
			spin_lock_bh(&vin->lock);
			vin->write_nwd(vin->cbdata, VIN_rIR);
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->read_eom(vin->cbdata);
			spin_unlock_bh(&vin->lock);
			break;
		case VINETIC_FLUSH_MBOX:
			spin_lock_bh(&vin->lock);
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, VIN_rOBXML);
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			phi = vin->read_eom(vin->cbdata);
			vin->pdata_size = phi & 0xff;
			vin->cdata_size = (phi >> 8) & 0x1f;
			if (vin->pdata_size) {
				// write short commands rPOBX
				for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
					if (!vin->is_not_ready(vin->cbdata)) {
						break;
					}
					udelay(VINETIC_WAIT_TIMEOUT);
				}
				if (wait_count >= VINETIC_WAIT_COUNT) {
					spin_unlock_bh(&vin->lock);
					res = -EIO;
					break;
				}
				vin->write_nwd(vin->cbdata, VIN_rPOBX);
				while (vin->pdata_size) {
					vin->pdata_size--;
					for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
						if (!vin->is_not_ready(vin->cbdata)) {
							break;
						}
						udelay(VINETIC_WAIT_TIMEOUT);
					}
					if (wait_count >= VINETIC_WAIT_COUNT) {
						spin_unlock_bh(&vin->lock);
						res = -EIO;
						break;
					}
					if (vin->pdata_size) {
						vin->read_nwd(vin->cbdata);
					} else {
						vin->read_eom(vin->cbdata);
					}
				}
			}
			if (vin->cdata_size) {
				// write short commands rCOBX
				for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
					if (!vin->is_not_ready(vin->cbdata)) {
						break;
					}
					udelay(VINETIC_WAIT_TIMEOUT);
				}
				if (wait_count >= VINETIC_WAIT_COUNT) {
					spin_unlock_bh(&vin->lock);
					res = -EIO;
					break;
				}
				vin->write_nwd(vin->cbdata, VIN_rCOBX);
				while (vin->cdata_size) {
					vin->cdata_size--;
					for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
						if (!vin->is_not_ready(vin->cbdata)) {
							break;
						}
						udelay(VINETIC_WAIT_TIMEOUT);
					}
					if (wait_count >= VINETIC_WAIT_COUNT) {
						spin_unlock_bh(&vin->lock);
						res = -EIO;
						break;
					}
					if (vin->cdata_size) {
						vin->read_nwd(vin->cbdata);
					} else {
						vin->read_eom(vin->cbdata);
					}
				}
			}
			spin_unlock_bh(&vin->lock);
			break;
		case VINETIC_DISABLE_IRQ:
			spin_lock_bh(&vin->lock);
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x0801);
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x4020);
			// write data to vinetic
			for (cnt = 0; cnt < 32; cnt++) {
				for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
					if (!vin->is_not_ready(vin->cbdata)) {
						break;
					}
					udelay(VINETIC_WAIT_TIMEOUT);
				}
				if (wait_count >= VINETIC_WAIT_COUNT) {
					spin_unlock_bh(&vin->lock);
					res = -EIO;
					goto vinetic_generic_ioctl_end;
				}
				if (cnt == 31) {
					vin->write_eom(vin->cbdata, 0xffff);
				} else {
					vin->write_nwd(vin->cbdata, 0xffff);
				}
			}
			spin_unlock_bh(&vin->lock);
			break;
		case VINETIC_GET_NOT_READY:
			not_ready = vin->is_not_ready(vin->cbdata);
			if (copy_to_user(argp, &not_ready, sizeof(int))) {
				res = -EINVAL;
			}
			break;
		case VINETIC_READ_DIA:
			phi = vin->read_dia(vin->cbdata);
			if (copy_to_user(argp, &phi, sizeof(u_int16_t))) {
				res = -EINVAL;
			}
			break;
		case VINETIC_REVISION:
			spin_lock_bh(&vin->lock);
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8801);
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8001);
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			phi = vin->read_eom(vin->cbdata);
			spin_unlock_bh(&vin->lock);
			if (copy_to_user(argp, &phi, sizeof(u_int16_t))) {
				res = -EINVAL;
			}
			break;
		case VINETIC_CHECKSUM:
			spin_lock_bh(&vin->lock);
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8801);
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			vin->write_nwd(vin->cbdata, 0x8901);
			for (wait_count = 0; wait_count < VINETIC_WAIT_COUNT; wait_count++) {
				if (!vin->is_not_ready(vin->cbdata)) {
					break;
				}
				udelay(VINETIC_WAIT_TIMEOUT);
			}
			if (wait_count >= VINETIC_WAIT_COUNT) {
				spin_unlock_bh(&vin->lock);
				res = -EIO;
				break;
			}
			phi = vin->read_eom(vin->cbdata);
			spin_unlock_bh(&vin->lock);
			if (copy_to_user(argp, &phi, sizeof(u_int16_t))) {
				res = -EINVAL;
			}
			break;
		case VINETIC_SET_POLL:
			if (copy_from_user(&vin->poll, argp, sizeof(int))) {
				res = -EINVAL;
			}
			del_timer_sync(&vin->poll_timer);
			if (vin->poll) {
                mod_timer(&vin->poll_timer, jiffies + 1);
			}
			break;
		case VINETIC_RESET_STATUS:
			spin_lock_bh(&vin->lock);
			vin->status_ready = 1;
			wake_up_interruptible(&vin->status_waitq);
			spin_unlock_bh(&vin->lock);
			break;
		default:
			res = -ENOIOCTLCMD;
			break;
	}
vinetic_generic_ioctl_end:
	return res;
}

#if defined(HAVE_UNLOCKED_IOCTL)
static long vinetic_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	return (long)vinetic_generic_ioctl(filp, cmd, data);
}
#else
static int vinetic_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long data)
{
	return (long)vinetic_generic_ioctl(filp, cmd, data);
}
#endif

#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
static long vinetic_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	return (long)vinetic_generic_ioctl(filp, cmd, data);
}
#endif

loff_t vinetic_llseek(struct file *filp, loff_t off, int whence)
{
	loff_t res;
	loff_t newpos;
	struct vinetic *vin = filp->private_data;

	switch (whence) {
		case 0: /* SEEK_SET */
			newpos = off;
			break;
		case 1: /* SEEK_CUR */
			newpos = off;
			break;
		case 2: /* SEEK_END */
			newpos = off;
			break;
		default: /* can't happen */
			res = -EINVAL;
			goto vinetic_llseek_end;
	}
	if (newpos < 0) {
		return -EINVAL;
	}

	spin_lock_bh(&vin->lock);
	for (;;) {
		if (filp->f_pos == 0) {
			break;
		}
		if (filp->f_flags & O_NONBLOCK) {
			spin_unlock_bh(&vin->lock);
			res = -EAGAIN;
			goto vinetic_llseek_end;
		}
		// sleeping
		spin_unlock_bh(&vin->lock);
		res = wait_event_interruptible(vin->seek_cbox_waitq, filp->f_pos == 0);
		spin_lock_bh(&vin->lock);
		if (vin->status.custom.bits.timeout) {
			res = -EIO;
		}
		spin_unlock_bh(&vin->lock);
		if (res) {
			goto vinetic_llseek_end;
		}
		spin_lock_bh(&vin->lock);
	}
	filp->f_pos = res = newpos;
	spin_unlock_bh(&vin->lock);

vinetic_llseek_end:
	if (res == 0) {
		log(KERN_ERR, "bad offset=%ld\n", (long int)res);
		res = -EINVAL;
	}
	return res;
}

static struct file_operations vinetic_fops = {
	.owner   = THIS_MODULE,
	.open    = vinetic_open,
	.release = vinetic_release,
	.read    = vinetic_read,
	.write   = vinetic_write,
#if defined(HAVE_UNLOCKED_IOCTL)
	.unlocked_ioctl = vinetic_unlocked_ioctl,
#else
	.ioctl = vinetic_ioctl,
#endif
#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
	.compat_ioctl = vinetic_compat_ioctl,
#endif
	.llseek = vinetic_llseek,
};

static int vinetic_rtp_channel_open(struct inode *inode, struct file *filp)
{
	struct vinetic_rtp_channel *rtp;

	rtp = container_of(inode->i_cdev, struct vinetic_rtp_channel, cdev);
	filp->private_data = rtp;

	spin_lock_bh(&rtp->lock);

	if (!rtp->usage++) {
		// reset read packet slot counters
		rtp->read_slot_read = 0;
		rtp->read_slot_write = 0;
		rtp->read_slot_count = 0;

		// reset write packet slot counters
		rtp->write_slot_read = 0;
		rtp->write_slot_write = 0;
		rtp->write_slot_count = 0;
	}

	spin_unlock_bh(&rtp->lock);

	return 0;
}

static int vinetic_rtp_channel_release(struct inode *inode, struct file *filp)
{
	struct vinetic_rtp_channel *rtp = filp->private_data;

	spin_lock_bh(&rtp->lock);
	rtp->usage--;
	spin_unlock_bh(&rtp->lock);
	return 0;
}

static ssize_t vinetic_rtp_channel_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	struct rtp_packet_slot read_slot;
	struct vinetic_rtp_channel *rtp = filp->private_data;

	spin_lock_bh(&rtp->lock);

	for (;;) {
		// successfull return
		if (rtp->read_slot_count > 0) {
			break;
		}

		spin_unlock_bh(&rtp->lock);

		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}

		// sleeping
		res = wait_event_interruptible(rtp->read_slot_count_waitq, rtp->read_slot_count > 0);
		if (res) {
			return res;
		}

		spin_lock_bh(&rtp->lock);
	}
	// copy data out of lock area
	read_slot.length = rtp->read_slot[rtp->read_slot_read].length;
	memcpy(read_slot.data, rtp->read_slot[rtp->read_slot_read].data, read_slot.length);
	// adjust read position of receiving buffer
	rtp->read_slot_read++;
	if (rtp->read_slot_read >= VINETIC_PACKETSLOT_MAXCOUNT) {
		rtp->read_slot_read = 0;
	}

	rtp->read_slot_count--;

	spin_unlock_bh(&rtp->lock);

	// copy to user
	read_slot.length = min(read_slot.length, count);
	if (copy_to_user(buff, read_slot.data, read_slot.length)) {
		return -EFAULT;
	}

	return read_slot.length;
}

static ssize_t vinetic_rtp_channel_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	struct rtp_packet_slot write_slot;
	struct vinetic_rtp_channel *rtp = filp->private_data;

	// copy from user
	write_slot.length = 506;
	write_slot.length = min(write_slot.length, count);
	if (copy_from_user(write_slot.data, buff, write_slot.length)) {
		res = -EFAULT;
		goto vinetic_rtp_channel_write_end;
	}

	spin_lock_bh(&rtp->lock);

	for (;;) {
		// successfull return
		if (rtp->write_slot_count < VINETIC_PACKETSLOT_MAXCOUNT) {
			break;
		}

		spin_unlock_bh(&rtp->lock);

		if (filp->f_flags & O_NONBLOCK) {
			return -EAGAIN;
		}

		// sleeping
		res = wait_event_interruptible(rtp->write_slot_count_waitq, rtp->write_slot_count < VINETIC_PACKETSLOT_MAXCOUNT);
		if (res) {
			return res;
		}

		spin_lock_bh(&rtp->lock);
	}

	// copy data to lock area
	rtp->write_slot[rtp->write_slot_write].length = write_slot.length;
	memcpy(rtp->write_slot[rtp->write_slot_write].data, write_slot.data, write_slot.length);

	// adjust write position of transmiting buffer
	rtp->write_slot_write++;
	if (rtp->write_slot_write >= VINETIC_PACKETSLOT_MAXCOUNT) {
		rtp->write_slot_write = 0;
	}

	rtp->write_slot_count++;
	if (rtp->write_slot_count > VINETIC_PACKETSLOT_MAXCOUNT) {
		rtp->write_slot_count = VINETIC_PACKETSLOT_MAXCOUNT;
		rtp->write_slot_read++;
		if (rtp->write_slot_read >= VINETIC_PACKETSLOT_MAXCOUNT) {
			rtp->write_slot_read = 0;
		}
	}

	spin_unlock_bh(&rtp->lock);

	res = write_slot.length;

vinetic_rtp_channel_write_end:
	return res;
}

static unsigned int vinetic_rtp_channel_poll(struct file *filp, struct poll_table_struct *wait_table)
{
	unsigned int res;
	struct vinetic_rtp_channel *rtp = filp->private_data;

	res = 0;

	poll_wait(filp, &rtp->poll_waitq, wait_table);

	spin_lock_bh(&rtp->lock);

	if (rtp->read_slot_count > 0) {
		res |= POLLIN | POLLRDNORM;
	}

	if (rtp->write_slot_count < VINETIC_PACKETSLOT_MAXCOUNT) {
		res |= POLLOUT | POLLWRNORM;
	}

	spin_unlock_bh(&rtp->lock);

	return res;
}

static struct file_operations vinetic_rtp_channel_fops = {
	.owner   = THIS_MODULE,
	.open    = vinetic_rtp_channel_open,
	.release = vinetic_rtp_channel_release,
	.read    = vinetic_rtp_channel_read,
	.write   = vinetic_rtp_channel_write,
	.poll   = vinetic_rtp_channel_poll,
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
							u_int16_t (* read_dia)(uintptr_t cbdata))
{
	struct vinetic *vin;
	size_t i;
	int rc;
	char devname[64];
	int devno = 0;
	int slot_alloc = 0;

	if (!(vin = kmalloc(sizeof(struct vinetic), GFP_KERNEL))) {
		log(KERN_ERR, "\"%s\" - can't get memory\n", name);
		goto vinetic_device_register_error;
	}
	memset(vin, 0, sizeof(struct vinetic));

	mutex_lock(&vinetic_device_list_lock);
	// check for name is not used
	for (i = 0; i < VINETIC_DEVICE_MAXCOUNT; i++) {
		if (!strcmp(vinetic_device_list[i].name, name)) {
			mutex_unlock(&vinetic_device_list_lock);
			log(KERN_ERR, "\"%s\" already registered\n", name);
			goto vinetic_device_register_error;
		}
	}
	// get free slot
	for (i = 0; i < VINETIC_DEVICE_MAXCOUNT; i++) {
		if (!vinetic_device_list[i].data) {
			devno = MKDEV(vinetic_major, i);
			vinetic_device_list[i].devno = devno;
			snprintf(vinetic_device_list[i].name, VINETIC_DEVNAME_MAXLEN, "%s", name);
			vinetic_device_list[i].type = VINETIC_DEVTYPE_VINETIC;
			vinetic_device_list[i].data = vin;
			vin->name = vinetic_device_list[i].name;
			vin->devno = devno;
			break;
		}
	}
	mutex_unlock(&vinetic_device_list_lock);
	
	if (!devno) {
		log(KERN_ERR, "\"%s\" - can't get free slot\n", name);
		goto vinetic_device_register_error;
	}
	slot_alloc = 1;
	
	// init vinetic private data
	for (i = 0; i < 8; i++) {
		vin->rtp_channels[i] = NULL;
	}
	spin_lock_init(&vin->lock);
	init_waitqueue_head(&vin->free_cbox_waitq);
	init_waitqueue_head(&vin->read_cbox_waitq);
	init_waitqueue_head(&vin->seek_cbox_waitq);
    init_waitqueue_head(&vin->status_waitq);
    timer_setup(&vin->poll_timer, vinetic_poll_proc, 0);
    vin->poll = 0;

    vin->cbdata = cbdata;

	if (!reset) {
		log(KERN_ERR, "\"%s\" - reset callback not present\n", name);
		goto vinetic_device_register_error;
	}
	vin->reset = reset;

	if (!is_not_ready) {
		log(KERN_ERR, "\"%s\" - is_not_ready callback not present\n", name);
		goto vinetic_device_register_error;
	}
	vin->is_not_ready = is_not_ready;

	if (!write_nwd) {
		log(KERN_ERR, "\"%s\" - write_nwd callback not present\n", name);
		goto vinetic_device_register_error;
	}
	vin->write_nwd = write_nwd;

	if (!write_eom) {
		log(KERN_ERR, "\"%s\" - write_eom callback not present\n", name);
		goto vinetic_device_register_error;
	}
	vin->write_eom = write_eom;

	if (!read_nwd) {
		log(KERN_ERR, "\"%s\" - read_nwd callback not present\n", name);
		goto vinetic_device_register_error;
	}
	vin->read_nwd = read_nwd;

	if (!read_eom) {
		log(KERN_ERR, "\"%s\" - read_eom callback not present\n", name);
		goto vinetic_device_register_error;
	}
	vin->read_eom = read_eom;

	if (!read_dia) {
		log(KERN_ERR, "\"%s\" - read_dia callback not present\n", name);
		goto vinetic_device_register_error;
	}
	vin->read_dia = read_dia;

	// Add char device to system
	cdev_init(&vin->cdev, &vinetic_fops);
	vin->cdev.owner = owner;
	vin->cdev.ops = &vinetic_fops;
	if ((rc = cdev_add(&vin->cdev, devno, 1)) < 0) {
		log(KERN_ERR, "\"%s\" - cdev_add() error=%d\n", name, rc);
		goto vinetic_device_register_error;
	}
	snprintf(devname, sizeof(devname), "polygator!%s", name);
	if (!(vin->device = CLASS_DEV_CREATE(vinetic_class, devno, NULL, devname))) {
		log(KERN_ERR, "\"%s\" - class_dev_create() error\n", name);
		goto vinetic_device_register_error;
	}

	return vin;

vinetic_device_register_error:
	if (slot_alloc) {
		mutex_lock(&vinetic_device_list_lock);
		for (i = 0; i < VINETIC_DEVICE_MAXCOUNT; i++) {
			if (!strcmp(vinetic_device_list[i].name, name)) {
				vinetic_device_list[i].name[0] = '\0';
				vinetic_device_list[i].devno = 0;
				vinetic_device_list[i].type = VINETIC_DEVTYPE_UNKNOWN;
				vinetic_device_list[i].data = NULL;
				break;
			}
		}
		mutex_unlock(&vinetic_device_list_lock);
	}
	if (vin) {
		kfree(vin);
	}
	return NULL;
}

void vinetic_device_unregister(struct vinetic *vin)
{
	size_t i;

	CLASS_DEV_DESTROY(vinetic_class, vin->devno);
	cdev_del(&vin->cdev);

	mutex_lock(&vinetic_device_list_lock);

	for (i = 0; i < VINETIC_DEVICE_MAXCOUNT; i++) {
		if ((vinetic_device_list[i].type == VINETIC_DEVTYPE_VINETIC) &&
				(!strcmp(vinetic_device_list[i].name, vin->name))) {
			vinetic_device_list[i].name[0] = '\0';
			vinetic_device_list[i].devno = 0;
			vinetic_device_list[i].type = VINETIC_DEVTYPE_UNKNOWN;
			vinetic_device_list[i].data = NULL;
			break;
		}
	}
	mutex_unlock(&vinetic_device_list_lock);

	// deleting vinetic polling timer
	del_timer_sync(&vin->poll_timer);

	kfree(vin);
}

struct vinetic_rtp_channel *vinetic_rtp_channel_register(struct module *owner, char *name, struct vinetic *vin, int index)
{
	struct vinetic_rtp_channel *rtp;
	size_t i;
	int rc;
	char devname[64];
	int devno = 0;
	int slot_alloc = 0;

	if (!(rtp = kmalloc(sizeof(struct vinetic_rtp_channel), GFP_KERNEL))) {
		log(KERN_ERR, "\"%s\" - can't get memory\n", name);
		goto vinetic_rtp_channel_register_error;
	}
	memset(rtp, 0, sizeof(struct vinetic_rtp_channel));

	mutex_lock(&vinetic_device_list_lock);
	// check for name is not used
	for (i = 0; i < VINETIC_DEVICE_MAXCOUNT; i++) {
		if (!strcmp(vinetic_device_list[i].name, name)) {
			mutex_unlock(&vinetic_device_list_lock);
			log(KERN_ERR, "\"%s\" already registered\n", name);
			goto vinetic_rtp_channel_register_error;
		}
	}
	// get free slot
	for (i = 0; i < VINETIC_DEVICE_MAXCOUNT; i++) {
		if (!vinetic_device_list[i].data) {
			devno = MKDEV(vinetic_major, i);
			vinetic_device_list[i].devno = devno;
			snprintf(vinetic_device_list[i].name, VINETIC_DEVNAME_MAXLEN, "%s", name);
			vinetic_device_list[i].type = VINETIC_DEVTYPE_RTPCHAN;
			vinetic_device_list[i].data = rtp;
			rtp->name = vinetic_device_list[i].name;
			rtp->devno = devno;
			break;
		}
	}
	mutex_unlock(&vinetic_device_list_lock);
	
	if (!devno) {
		log(KERN_ERR, "\"%s\" - can't get free slot\n", name);
		goto vinetic_rtp_channel_register_error;
	}
	slot_alloc = 1;

	if (!vin) {
		log(KERN_ERR, "\"%s\" - vinetic not specified\n", name);
		goto vinetic_rtp_channel_register_error;
	}

	if ((index < 0) || (index > 7)) {
		log(KERN_ERR, "\"%s\" - index %d out of range 0-7\n", name, index);
		goto vinetic_rtp_channel_register_error;
	}

	spin_lock_bh(&vin->lock);
		
	if (vin->rtp_channels[index]) {
		spin_unlock_bh(&vin->lock);
		log(KERN_ERR, "\"%s\" - vinetic rtp channel=%d already used\n", name, index);
		goto vinetic_rtp_channel_register_error;
	}

	vin->rtp_channels[index] = rtp;

	spin_unlock_bh(&vin->lock);

	// init rtp data
	spin_lock_init(&rtp->lock);

	init_waitqueue_head(&rtp->poll_waitq);
	init_waitqueue_head(&rtp->read_slot_count_waitq);
	init_waitqueue_head(&rtp->write_slot_count_waitq);
	
	rtp->index = index;
	rtp->vinetic = vin;

	// Add char device to system
	cdev_init(&rtp->cdev, &vinetic_rtp_channel_fops);
	rtp->cdev.owner = owner;
	rtp->cdev.ops = &vinetic_rtp_channel_fops;
	if ((rc = cdev_add(&rtp->cdev, devno, 1)) < 0) {
		log(KERN_ERR, "\"%s\" - cdev_add() error=%d\n", name, rc);
		goto vinetic_rtp_channel_register_error;
	}
	snprintf(devname, sizeof(devname), "polygator!%s", name);
	if (!(rtp->device = CLASS_DEV_CREATE(vinetic_class, devno, NULL, devname))) {
		log(KERN_ERR, "\"%s\" - class_dev_create() error\n", name);
		goto vinetic_rtp_channel_register_error;
	}

	return rtp;

vinetic_rtp_channel_register_error:
	if (slot_alloc) {
		mutex_lock(&vinetic_device_list_lock);
		for (i = 0; i < VINETIC_DEVICE_MAXCOUNT; i++) {
			if (!strcmp(vinetic_device_list[i].name, name)) {
				vinetic_device_list[i].name[0] = '\0';
				vinetic_device_list[i].devno = 0;
				vinetic_device_list[i].type = VINETIC_DEVTYPE_UNKNOWN;
				vinetic_device_list[i].data = NULL;
				break;
			}
		}
		mutex_unlock(&vinetic_device_list_lock);
	}
	if (rtp) {
		kfree(rtp);
	}
	return NULL;
}

void vinetic_rtp_channel_unregister(struct vinetic_rtp_channel *rtp)
{
	size_t i;
	
	if (!rtp) {
		return;
	}

	CLASS_DEV_DESTROY(vinetic_class, rtp->devno);
	cdev_del(&rtp->cdev);

	mutex_lock(&vinetic_device_list_lock);

	for (i = 0; i < VINETIC_DEVICE_MAXCOUNT; i++) {
		if ((vinetic_device_list[i].type == VINETIC_DEVTYPE_RTPCHAN) &&
				(!strcmp(vinetic_device_list[i].name, rtp->name))) {
			vinetic_device_list[i].name[0] = '\0';
			vinetic_device_list[i].devno = 0;
			vinetic_device_list[i].type = VINETIC_DEVTYPE_UNKNOWN;
			vinetic_device_list[i].data = NULL;
			break;
		}
	}
	mutex_unlock(&vinetic_device_list_lock);

	spin_lock_bh(&rtp->vinetic->lock);
	rtp->vinetic->rtp_channels[rtp->index] = NULL;
	spin_unlock_bh(&rtp->vinetic->lock);

	kfree(rtp);
}

static int __init vinetic_init(void)
{
	size_t i;
	dev_t devno;
	int vinetic_major_reg = 0;
	int rc = -1;

	verbose("loading ...\n");

	// Init vinetic device list
	for (i = 0; i < VINETIC_DEVICE_MAXCOUNT; i++) {
		vinetic_device_list[i].name[0] = '\0';
		vinetic_device_list[i].devno = 0;
		vinetic_device_list[i].type = VINETIC_DEVTYPE_UNKNOWN;
		vinetic_device_list[i].data = NULL;
	}

	// Registering vinetic device class
	if (!(vinetic_class = class_create(THIS_MODULE, "vinetic"))) {
		log(KERN_ERR, "class_create() error\n");
		goto vinetic_init_error;
	}
	// Register char device region
	if (vinetic_major) {
		devno = MKDEV(vinetic_major, 0);
		rc = register_chrdev_region(devno, VINETIC_DEVICE_MAXCOUNT, "vinetic");
	} else {
		rc = alloc_chrdev_region(&devno, 0, VINETIC_DEVICE_MAXCOUNT, "vinetic");
		if(rc >= 0) vinetic_major = MAJOR(devno);
	}
	if (rc < 0) {
		log(KERN_ERR, "register chrdev region error=%d\n", rc);
		goto vinetic_init_error;
	}
	vinetic_major_reg = 1;

	verbose("loaded successfull\n");
	return 0;

vinetic_init_error:
	if (vinetic_major_reg) {
		unregister_chrdev_region(MKDEV(vinetic_major, 0), VINETIC_DEVICE_MAXCOUNT);
	}
	if (vinetic_class) {
		class_destroy(vinetic_class);
	}
	return rc;
}

static void __exit vinetic_exit(void)
{
	// Unregister char device region
	unregister_chrdev_region(MKDEV(vinetic_major, 0), VINETIC_DEVICE_MAXCOUNT);
	// Destroy vinetic device class
	class_destroy(vinetic_class);

	verbose("stopped\n");
}

module_init(vinetic_init);
module_exit(vinetic_exit);
