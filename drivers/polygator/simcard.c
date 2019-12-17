
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

#include "simcard-base.h"
#include "simcard-def.h"

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@gmail.com>");
MODULE_DESCRIPTION("Polygator Linux module for virtual SIM card support");
MODULE_LICENSE("GPL");

static int simcard_major = 0;
module_param(simcard_major, int, 0);
MODULE_PARM_DESC(simcard_major, "Major number for Polygator Linux simcard module");

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
    static struct class *simcard_class = NULL;
#else
    static struct class_simple *simcard_class = NULL;
    #define class_create(_a, _b) class_simple_create(_a, _b)
    #define class_destroy(_a) class_simple_destroy(_a)
#endif

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, "simcard", ## _args)
#define log(_level, _fmt, _args...) printk(_level "polygator-%s] %s:%d - %s(): " _fmt, "simcard", "simcard.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, "simcard", "simcard.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

static struct simcard_device *simcard_device_list[SIMCARD_DEVICE_MAXCOUNT];
static DEFINE_MUTEX(simcard_device_list_lock);

static void simcard_poll_proc(struct timer_list *timer)
{
    uint8_t buf[256];
    size_t len = 0;
    int reset = 0;
    struct simcard_device *sim = container_of(timer, struct simcard_device, poll_timer);

    spin_lock(&sim->lock);

    // reset
    if (sim->is_reset_requested) {
        reset = sim->is_reset_requested(sim->cbdata);
    }
    if (reset != sim->reset) {
        // set reset status bit
        sim->reset_toggled = true;
        // do after reset action
        if (reset && sim->do_after_reset) {
            sim->do_after_reset(sim->cbdata);
        }
    } else {
        if (reset) {
            if (sim->api == 2) {
                // read
                if (sim->read2) {
                    len = sim->read2(sim->cbdata, buf, sizeof(buf));
                }
            } else if (sim->api == 1) {
                // read
                if (sim->is_read_ready(sim->cbdata)) {
                    while ((len < sizeof(buf)) && (sim->is_read_ready(sim->cbdata))) {
                        buf[len++] = sim->read(sim->cbdata);
                    }
                }
            }
        }
    }

    sim->reset = reset;

    if (len) {
        memcpy(sim->read_data + sim->read_data_length, buf, min(len, sizeof(sim->read_data) - sim->read_data_length));
        sim->read_data_length  += min(len, sizeof(sim->read_data) - sim->read_data_length);
    }

    if (sim->reset_toggled || sim->read_data_length) {
        wake_up_interruptible(&sim->read_waitq);
        wake_up_interruptible(&sim->poll_waitq);
    }

    spin_unlock(&sim->lock);

    if (sim->poll) {
        mod_timer(&sim->poll_timer, jiffies + 1);
    }
}

static int simcard_open(struct inode *inode, struct file *filp)
{
    size_t usage;
    struct simcard_device *sim;

    sim = container_of(inode->i_cdev, struct simcard_device, cdev);
    filp->private_data = sim;

    spin_lock_bh(&sim->lock);
    usage = sim->usage++;
    if (!usage) {
        sim->read_data_length = 0;
        sim->write_room = 512;
        sim->poll = 1;
    }
    spin_unlock_bh(&sim->lock);

    if (!usage) {
        mod_timer(&sim->poll_timer, jiffies + 1);
    }

    return 0;
}

static int simcard_release(struct inode *inode, struct file *filp)
{
    size_t usage;
    struct simcard_device *sim = filp->private_data;

    spin_lock_bh(&sim->lock);
    usage = --sim->usage;
    if (!usage) {
        sim->poll = 0;
    }
    spin_unlock_bh(&sim->lock);

    if (!usage) {
        del_timer_sync(&sim->poll_timer);
    }

    return 0;
}

static ssize_t simcard_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
    ssize_t res;
    size_t length;
    struct simcard_data data;
    struct simcard_device *sim = filp->private_data;

    res = 0;
    length = 0;

    spin_lock_bh(&sim->lock);

    for (;;) {
        if (sim->reset_toggled || sim->read_data_length) {
            break;
        }

        if (filp->f_flags & O_NONBLOCK) {
            spin_unlock_bh(&sim->lock);
            res = -EAGAIN;
            goto simcard_read_end;
        }
        // sleeping
        spin_unlock_bh(&sim->lock);
        if ((res = wait_event_interruptible(sim->read_waitq, (sim->reset_toggled || sim->read_data_length)))) {
            goto simcard_read_end;
        }
        spin_lock_bh(&sim->lock);
    }

    // select body type
    if (sim->reset_toggled) {
        data.header.type = SIMCARD_CONTAINER_TYPE_RESET;
        data.header.length = sizeof(data.body.reset);
        length = sizeof(data.header) + data.header.length;
        data.body.reset = sim->reset;
        sim->reset_toggled = false;
    } else if (sim->read_data_length) {
        data.header.type = SIMCARD_CONTAINER_TYPE_DATA;
        data.header.length = sim->read_data_length;
        length = sizeof(data.header) + data.header.length;
        memcpy(data.body.data, sim->read_data, sim->read_data_length);
        sim->read_data_length = 0;
    } else {
        spin_unlock_bh(&sim->lock);
        res = -EAGAIN;
        goto simcard_read_end;
    }

    spin_unlock_bh(&sim->lock);

    length = min(length, count);
    if (copy_to_user(buff, &data, length)) {
        res = -EFAULT;
        goto simcard_read_end;
    }

    res = length;

simcard_read_end:
    return res;
}

static ssize_t simcard_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
    ssize_t res;
    size_t length;
    size_t i;
    struct simcard_data data;

    struct simcard_device *sim = filp->private_data;

    res = 0;

    length = sizeof(struct simcard_data);
    length = min(length, count);
    if (copy_from_user(&data, buff, length)) {
        res = -EFAULT;
        goto simcard_write_end;
    }

    spin_lock_bh(&sim->lock);

    for (;;) {
        if ((sim->api == 2) && (sim->get_write_room)) {
            sim->write_room = sim->get_write_room(sim->cbdata);
        }
        if (sim->write_room == 512) {
            break;
        }

        if (filp->f_flags & O_NONBLOCK) {
            spin_unlock_bh(&sim->lock);
            res = -EAGAIN;
            goto simcard_write_end;
        }
        // sleeping
        spin_unlock_bh(&sim->lock);
        if ((res = wait_event_interruptible(sim->write_waitq, (sim->write_room == 512)))) {
            goto simcard_write_end;
        }
        spin_lock_bh(&sim->lock);
    }

    switch (data.header.type) {
        case SIMCARD_CONTAINER_TYPE_DATA:
            if (sim->api == 2) {
                if (sim->write2) {
                    length = data.header.length;
                    length = min(length, sim->write_room);
                    res = sim->write2(sim->cbdata, data.body.data, length);
                    if (res > 0) {
                        res += sizeof(struct simcard_data_header);
                    }
                }
            } else if (sim->api == 1) {
                i = 0;
                while (data.header.length) {
                    if (sim->is_write_ready(sim->cbdata)) {
                        sim->write(sim->cbdata, data.body.data[i++]);
                        data.header.length--;
                    }
                }
                res = length;
            } else {
                res = -EINVAL;
            }
            break;
        case SIMCARD_CONTAINER_TYPE_SPEED:
            if (sim->set_etu_count) {
                sim->set_etu_count(sim->cbdata, data.body.speed);
            } else if (sim->set_speed) {
                sim->set_speed(sim->cbdata, data.body.speed);
            }
            res = length;
            break;
        default:
            res = -EINVAL;
            break;
    }

    spin_unlock_bh(&sim->lock);

simcard_write_end:
    return res;
}

static unsigned int simcard_poll(struct file *filp, struct poll_table_struct *wait_table)
{
    unsigned int res;
    struct simcard_device *sim = filp->private_data;

    res = 0;

    poll_wait(filp, &sim->poll_waitq, wait_table);

    spin_lock_bh(&sim->lock);

    if (sim->reset_toggled || sim->read_data_length) {
        res |= POLLIN | POLLRDNORM;
    }

    if (sim->write_room == 512) {
        res |= POLLOUT | POLLWRNORM;
    }

    spin_unlock_bh(&sim->lock);

    return res;
}

static struct file_operations simcard_fops = {
    .owner      = THIS_MODULE,
    .open       = simcard_open,
    .release    = simcard_release,
    .read       = simcard_read,
    .write      = simcard_write,
    .poll       = simcard_poll,
};

struct simcard_device *simcard_device_register(struct module *owner,
                            void *cbdata,
                            uint8_t (* read)(void *cbdata),
                            void (* write)(void *cbdata, uint8_t value),
                            int (* is_read_ready)(void *cbdata),
                            int (* is_write_ready)(void *cbdata),
                            int (* is_reset_requested)(void *cbdata),
                            void (* set_speed)(void *cbdata, int speed),
                            void (* do_after_reset)(void *cbdata))
{
    struct simcard_device *sim;
    size_t i;
    int rc;
    char devname[64];
    int devno = 0;
    int slot_alloc = 0;

    if (!(sim = kmalloc(sizeof(struct simcard_device), GFP_KERNEL))) {
        log(KERN_ERR, "can't get memory for struct simcard_device\n");
        goto simcard_device_register_error;
    }
    memset(sim, 0, sizeof(struct simcard_device));

    mutex_lock(&simcard_device_list_lock);
    // get free slot
    for (i = 0; i < SIMCARD_DEVICE_MAXCOUNT; ++i) {
        if (!simcard_device_list[i]) {
            sim->devno = devno = MKDEV(simcard_major, i);
            simcard_device_list[i] = sim;
            break;
        }
    }
    mutex_unlock(&simcard_device_list_lock);

    if (!devno) {
        log(KERN_ERR, "can't get free slot for simcard\n");
        goto simcard_device_register_error;
    }
    slot_alloc = 1;

    // init simcard data
    spin_lock_init(&sim->lock);
    init_waitqueue_head(&sim->poll_waitq);
    init_waitqueue_head(&sim->read_waitq);
    init_waitqueue_head(&sim->write_waitq);
    timer_setup(&sim->poll_timer, simcard_poll_proc, 0);
    sim->poll = 0;

    // set callback data pointer
    sim->cbdata = cbdata;

    if (!read) {
        log(KERN_ERR, "reset callback not present\n");
        goto simcard_device_register_error;
    }
    sim->read = read;

    if (!write) {
        log(KERN_ERR, "write callback not present\n");
        goto simcard_device_register_error;
    }
    sim->write = write;

    if (!is_read_ready) {
        log(KERN_ERR, "is_read_ready callback not present\n");
        goto simcard_device_register_error;
    }
    sim->is_read_ready = is_read_ready;

    if (!is_write_ready) {
        log(KERN_ERR, "is_write_ready callback not present\n");
        goto simcard_device_register_error;
    }
    sim->is_write_ready = is_write_ready;

    if (!is_reset_requested) {
        log(KERN_ERR, "is_reset_requested callback not present\n");
        goto simcard_device_register_error;
    }
    sim->is_reset_requested = is_reset_requested;

    if (!set_speed) {
        log(KERN_ERR, "set_speed callback not present\n");
        goto simcard_device_register_error;
    }
    sim->set_speed = set_speed;

    if (!do_after_reset) {
        log(KERN_ERR, "do_after_reset callback not present\n");
        goto simcard_device_register_error;
    }
    sim->do_after_reset = do_after_reset;

    // Add char device to system
    cdev_init(&sim->cdev, &simcard_fops);
    sim->cdev.owner = owner;
    sim->cdev.ops = &simcard_fops;
    if ((rc = cdev_add(&sim->cdev, devno, 1)) < 0) {
        log(KERN_ERR, "cdev_add() error=%d\n", rc);
        goto simcard_device_register_error;
    }
    snprintf(devname, sizeof(devname), "polygator!sim%d", MINOR(sim->devno));
    if (!(sim->device = CLASS_DEV_CREATE(simcard_class, devno, NULL, devname))) {
        log(KERN_ERR, "class_dev_create() error\n");
        goto simcard_device_register_error;
    }

    sim->api = 1;

    return sim;

simcard_device_register_error:
    if (slot_alloc) {
        mutex_lock(&simcard_device_list_lock);
        simcard_device_list[MINOR(sim->devno)] = NULL;
        mutex_unlock(&simcard_device_list_lock);
    }
    if (sim) {
        kfree(sim);
    }
    return NULL;
}
EXPORT_SYMBOL(simcard_device_register);

struct simcard_device *simcard_device_register2(struct module *owner,
                                                void *cbdata)
{
    struct simcard_device *sim;
    size_t i;
    int rc;
    char devname[64];
    int devno = 0;
    int slot_alloc = 0;

    if (!(sim = kmalloc(sizeof(struct simcard_device), GFP_KERNEL))) {
        log(KERN_ERR, "can't get memory for struct simcard_device\n");
        goto simcard_device_register_error;
    }
    memset(sim, 0, sizeof(struct simcard_device));

    mutex_lock(&simcard_device_list_lock);
    // get free slot
    for (i = 0; i < SIMCARD_DEVICE_MAXCOUNT; ++i) {
        if (!simcard_device_list[i]) {
            sim->devno = devno = MKDEV(simcard_major, i);
            simcard_device_list[i] = sim;
            break;
        }
    }
    mutex_unlock(&simcard_device_list_lock);

    if (!devno) {
        log(KERN_ERR, "can't get free slot for simcard\n");
        goto simcard_device_register_error;
    }
    slot_alloc = 1;

    // init simcard data
    spin_lock_init(&sim->lock);
    init_waitqueue_head(&sim->poll_waitq);
    init_waitqueue_head(&sim->read_waitq);
    init_waitqueue_head(&sim->write_waitq);
    timer_setup(&sim->poll_timer, simcard_poll_proc, 0);
    sim->poll = 0;

    // set callback data pointer
    sim->cbdata = cbdata;

    // Add char device to system
    cdev_init(&sim->cdev, &simcard_fops);
    sim->cdev.owner = owner;
    sim->cdev.ops = &simcard_fops;
    if ((rc = cdev_add(&sim->cdev, devno, 1)) < 0) {
        log(KERN_ERR, "cdev_add() error=%d\n", rc);
        goto simcard_device_register_error;
    }
    snprintf(devname, sizeof(devname), "polygator!sim%d", MINOR(sim->devno));
    if (!(sim->device = CLASS_DEV_CREATE(simcard_class, devno, NULL, devname))) {
        log(KERN_ERR, "class_dev_create() error\n");
        goto simcard_device_register_error;
    }

    sim->api = 2;

    return sim;

simcard_device_register_error:
    if (slot_alloc) {
        mutex_lock(&simcard_device_list_lock);
        simcard_device_list[MINOR(sim->devno)] = NULL;
        mutex_unlock(&simcard_device_list_lock);
    }
    if (sim) {
        kfree(sim);
    }
    return NULL;
}
EXPORT_SYMBOL(simcard_device_register2);

void simcard_device_unregister(struct simcard_device *sim)
{
    CLASS_DEV_DESTROY(simcard_class, sim->devno);
    cdev_del(&sim->cdev);

    mutex_lock(&simcard_device_list_lock);
    simcard_device_list[MINOR(sim->devno)] = NULL;
    mutex_unlock(&simcard_device_list_lock);

    kfree(sim);
}
EXPORT_SYMBOL(simcard_device_unregister);

void simcard_device_set_is_reset_requested(struct simcard_device *sim, int (* is_reset_requested)(void *cbdata))
{
    sim->is_reset_requested = is_reset_requested;
}
EXPORT_SYMBOL(simcard_device_set_is_reset_requested);

void simcard_device_set_get_write_room(struct simcard_device *sim, size_t (* get_write_room)(void *cbdata))
{
    sim->get_write_room = get_write_room;
}
EXPORT_SYMBOL(simcard_device_set_get_write_room);

void simcard_device_set_write2(struct simcard_device *sim, size_t (* write2)(void *cbdata, uint8_t *data, size_t length))
{
    sim->write2 = write2;
}
EXPORT_SYMBOL(simcard_device_set_write2);

void simcard_device_set_read2(struct simcard_device *sim, size_t (* read2)(void *cbdata, uint8_t *data, size_t length))
{
    sim->read2 = read2;
}
EXPORT_SYMBOL(simcard_device_set_read2);

void simcard_device_set_etu_count(struct simcard_device *sim, void (* set_etu_count)(void *cbdata, uint32_t etu))
{
    sim->set_etu_count = set_etu_count;
}
EXPORT_SYMBOL(simcard_device_set_etu_count);

static int __init simcard_init(void)
{
    size_t i;
    dev_t devno;
    int simcard_major_reg = 0;
    int rc = -1;

    verbose("loading ...\n");

    // Init simcard device list
    for (i = 0; i < SIMCARD_DEVICE_MAXCOUNT; ++i) {
        simcard_device_list[i] = NULL;
    }

    // Registering simcard device class
    if (!(simcard_class = class_create(THIS_MODULE, "simcard"))) {
        log(KERN_ERR, "class_create() error\n");
        goto simcard_init_error;
    }
    // Register simcard char device region
    if (simcard_major) {
        devno = MKDEV(simcard_major, 0);
        rc = register_chrdev_region(devno, SIMCARD_DEVICE_MAXCOUNT, "simcard");
    } else {
        rc = alloc_chrdev_region(&devno, 0, SIMCARD_DEVICE_MAXCOUNT, "simcard");
        if (rc >= 0) {
            simcard_major = MAJOR(devno);
        }
    }
    if (rc < 0) {
        log(KERN_ERR, "register chrdev region error=%d\n", rc);
        goto simcard_init_error;
    }
    simcard_major_reg = 1;

    verbose("loaded successfull\n");
    return 0;

simcard_init_error:
    if (simcard_major_reg) {
        unregister_chrdev_region(MKDEV(simcard_major, 0), SIMCARD_DEVICE_MAXCOUNT);
    }
    if (simcard_class) {
        class_destroy(simcard_class);
    }
    return rc;
}

static void __exit simcard_exit(void)
{
    // Unregister simcard char device region
    unregister_chrdev_region(MKDEV(simcard_major, 0), SIMCARD_DEVICE_MAXCOUNT);
    // Destroy simcard device class
    class_destroy(simcard_class);

    verbose("stopped\n");
}

module_init(simcard_init);
module_exit(simcard_exit);
