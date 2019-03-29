#ifndef __SIMCARD_BASE_H__
#define __SIMCARD_BASE_H__

#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/wait.h>

#include "polygator-types.h"

#include "simcard-def.h"

#define SIMCARD_DEVICE_MAXCOUNT 256
#define SIMCARD_DEVNAME_MAXLEN 256

union simcard_data_status {
    struct {
        u_int32_t data:1;
        u_int32_t reset:1;
        u_int32_t speed:1;
        u_int32_t reserved:29;
    }  __attribute__((packed)) bits;
    u_int32_t full;
} __attribute__((packed));

struct simcard_device {
    int devno;
    spinlock_t lock;

    int api;

    size_t usage;

    struct timer_list poll_timer;
    size_t poll;

    wait_queue_head_t poll_waitq;
    wait_queue_head_t read_waitq;
    wait_queue_head_t write_waitq;

    void *cbdata;

    uint8_t (* read)(void *cbdata);
    void (* write)(void *cbdata, uint8_t value);
    int (* is_read_ready)(void *cbdata);
    int (* is_write_ready)(void *cbdata);
    int (* is_reset_requested)(void *cbdata);
    void (* set_speed)(void *cbdata, int speed);
    void (* do_after_reset)(void *cbdata);

    size_t (* get_write_room)(void *cbdata);
    size_t (* write2)(void *cbdata, uint8_t *data, size_t length);
    size_t (* read2)(void *cbdata, uint8_t *data, size_t length);
    void (* set_etu_count)(void *cbdata, uint32_t etu);


    int reset;
    bool reset_toggled;

    size_t write_room;

    uint8_t read_data[256];
    size_t read_data_length;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
    struct device *device;
#else
    struct class_device *device;
#endif
    struct cdev cdev;
};

struct simcard_device *simcard_device_register(struct module *owner,
                                                void *cbdata,
												u_int8_t (* read)(void *cbdata),
												void (* write)(void *cbdata, u_int8_t value),
												int (* is_read_ready)(void *cbdata),
												int (* is_write_ready)(void *cbdata),
												int (* is_reset_request)(void *cbdata),
												void (* set_speed)(void *cbdata, int speed),
												void (* do_after_reset)(void *cbdata));

struct simcard_device *simcard_device_register2(struct module *owner,
                                                void *cbdata);

void simcard_device_unregister(struct simcard_device *sim);

void simcard_device_set_is_reset_requested(struct simcard_device *sim, int (* is_reset_requested)(void *cbdata));
void simcard_device_set_get_write_room(struct simcard_device *sim, size_t (* get_write_room)(void *cbdata));
void simcard_device_set_write2(struct simcard_device *sim, size_t (* write2)(void *cbdata, uint8_t *data, size_t length));
void simcard_device_set_read2(struct simcard_device *sim, size_t (* read2)(void *cbdata, uint8_t *data, size_t length));
void simcard_device_set_etu_count(struct simcard_device *sim, void (* set_etu_count)(void *cbdata, uint32_t etu));

#endif //__SIMCARD_BASE_H__
