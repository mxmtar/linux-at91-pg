
#include <linux/kobject.h>
#include <linux/fs.h>
#include <linux/cdev.h>
// #include <linux/delay.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/slab.h>
// #include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/vmalloc.h>

#include <asm/uaccess.h>
#include <asm/atomic.h>
#include <asm/io.h>
#if defined(CONFIG_COMPAT) && defined(HAVE_COMPAT_IOCTL) && (HAVE_COMPAT_IOCTL == 1)
#include <asm/compat.h>
#endif

#include "polygator-base.h"
#include "version.h"

MODULE_AUTHOR("Maksym Tarasevych <mxmtar@gmail.com>");
MODULE_DESCRIPTION("Polygator Linux base module");
MODULE_LICENSE("GPL");

static int polygator_subsystem_major = 0;
module_param(polygator_subsystem_major, int, 0);
MODULE_PARM_DESC(polygator_subsystem_major, "Major number for Polygator subsystem device");

static int polygator_tty_major = 0;
module_param(polygator_tty_major, int, 0);
MODULE_PARM_DESC(polygator_tty_major, "Major number for Polygator TTY device");

EXPORT_SYMBOL(polygator_print_gsm_module_type);

EXPORT_SYMBOL(polygator_board_register);
EXPORT_SYMBOL(polygator_board_unregister);

EXPORT_SYMBOL(polygator_tty_device_register);
EXPORT_SYMBOL(polygator_tty_device_unregister);

EXPORT_SYMBOL(polygator_power_on_schedule);
EXPORT_SYMBOL(polygator_power_on_cancel);

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
	static struct class *polygator_class = NULL;
#else
	static struct class_simple *polygator_class = NULL;
	#define class_create(_a, _b) class_simple_create(_a, _b)
	#define class_destroy(_a) class_simple_destroy(_a)
#endif

#define verbose(_fmt, _args...) printk(KERN_INFO "[%s] " _fmt, "polygator", ## _args)
#define log(_level, _fmt, _args...) printk(_level "[%s] %s:%d - %s(): " _fmt, "polygator", "subsystem.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[%s] %s:%d - %s(): " _fmt, "polygator", "subsystem.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

struct subsystem_private_data {
	char buff[0x8000];
	size_t length;
};

static struct cdev polygator_subsystem_cdev;

static struct polygator_board *polygator_board_list[POLYGATOR_BOARD_MAXCOUNT];
static DEFINE_MUTEX(polygator_board_list_lock);

struct polygator_power_on_entry {
    int id;
    void (* callback)(void *data);
    void *data;
    struct list_head list;
};
static LIST_HEAD(polygator_power_on_list);
static spinlock_t polygator_power_on_list_lock;
static struct timer_list polygator_power_on_timer;

static struct tty_driver *polygator_tty_device_driver = NULL;

struct polygator_tty_device *polygator_tty_device_list[POLYGATOR_TTY_DEVICE_MAXCOUNT];
static DEFINE_MUTEX(polygator_tty_device_list_lock);

static int polygator_tty_device_open(struct tty_struct *tty, struct file *filp);
static void polygator_tty_device_close(struct tty_struct *tty, struct file *filp);
static int polygator_tty_device_write(struct tty_struct *tty, const unsigned char *buf, int count);
static int polygator_tty_device_write_room(struct tty_struct *tty);
static int polygator_tty_device_chars_in_buffer(struct tty_struct *tty);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void polygator_tty_device_set_termios(struct tty_struct *tty, struct ktermios *old_termios);
#else
static void polygator_tty_device_set_termios(struct tty_struct *tty, struct termios *old_termios);
#endif
static void polygator_tty_device_flush_buffer(struct tty_struct *tty);
static void polygator_tty_device_hangup(struct tty_struct *tty);

static struct tty_operations polygator_tty_device_ops = {
	.open = polygator_tty_device_open,
	.close = polygator_tty_device_close,
	.write = polygator_tty_device_write,
	.write_room = polygator_tty_device_write_room,
	.chars_in_buffer = polygator_tty_device_chars_in_buffer,
	.set_termios = polygator_tty_device_set_termios,
	.flush_buffer = polygator_tty_device_flush_buffer,
	.hangup = polygator_tty_device_hangup,
};

static int polygator_tty_device_open(struct tty_struct *tty, struct file *filp)
{
	size_t i;
	struct polygator_tty_device *ptd = NULL;

	if (mutex_lock_interruptible(&polygator_tty_device_list_lock)) {
		return -ERESTARTSYS;
	}

	for (i = 0; i < POLYGATOR_TTY_DEVICE_MAXCOUNT; i++) {
		if ((polygator_tty_device_list[i]) && (polygator_tty_device_list[i]->tty_minor == tty->index)) {
			ptd = polygator_tty_device_list[i];
			break;
		}
	}

	if (!ptd) {
		mutex_unlock(&polygator_tty_device_list_lock);
		return -ENODEV;
	}

	tty->driver_data = ptd;

	mutex_unlock(&polygator_tty_device_list_lock);

	return ptd->tty_ops->open(tty, filp);
}

static void polygator_tty_device_close(struct tty_struct *tty, struct file *filp)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	ptd->tty_ops->close(tty, filp);
}

static int polygator_tty_device_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	return ptd->tty_ops->write(tty, buf, count);
}

static int polygator_tty_device_write_room(struct tty_struct *tty)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	return ptd->tty_ops->write_room(tty);
}

static int polygator_tty_device_chars_in_buffer(struct tty_struct *tty)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	return ptd->tty_ops->chars_in_buffer(tty);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void polygator_tty_device_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
#else
static void polygator_tty_device_set_termios(struct tty_struct *tty, struct termios *old_termios)
#endif
{
	struct polygator_tty_device *ptd = tty->driver_data;
	ptd->tty_ops->set_termios(tty, old_termios);
}

static void polygator_tty_device_flush_buffer(struct tty_struct *tty)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	ptd->tty_ops->flush_buffer(tty);
}

static void polygator_tty_device_hangup(struct tty_struct *tty)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	ptd->tty_ops->hangup(tty);
}

static int polygator_subsystem_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i;
	size_t len;

	struct subsystem_private_data *private_data;

	if (!(private_data = kmalloc(sizeof(struct subsystem_private_data), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory=%lu bytes\n", (unsigned long int)sizeof(struct subsystem_private_data));
		res = -ENOMEM;
		goto polygator_subsystem_open_error;
	}
	memset(private_data, 0, sizeof(struct subsystem_private_data));

	mutex_lock(&polygator_board_list_lock);
	len = 0;
	len += sprintf(private_data->buff + len, "{");
	len += sprintf(private_data->buff + len, "\r\n\t\"version\": \"%s\",", POLYGATOR_LINUX_VERSION);
	len += sprintf(private_data->buff + len, "\r\n\t\"boards\": [");
	for (i = 0; i < POLYGATOR_BOARD_MAXCOUNT; i++) {
		if (polygator_board_list[i]) {
			len += sprintf(private_data->buff + len, "%s\r\n\t\t{\r\n\t\t\t\"driver\": \"%s\",\r\n\t\t\t\"path\": \"%s\"\r\n\t\t}",
							i ? "," : "",
							polygator_board_list[i]->cdev->owner->name,
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
							dev_name(polygator_board_list[i]->device)
#else
							polygator_board_list[i]->device->class_id
#endif
						  );
		}
	}
	len += sprintf(private_data->buff + len, "\r\n\t]");
	len += sprintf(private_data->buff + len, "\r\n}\r\n");
	mutex_unlock(&polygator_board_list_lock);

	private_data->length = len;

	filp->private_data = private_data;

	return 0;

polygator_subsystem_open_error:
	if (private_data) {
		kfree(private_data);
	}
	return res;
}

static int polygator_subsystem_release(struct inode *inode, struct file *filp)
{
	struct subsystem_private_data *private_data = filp->private_data;

	kfree(private_data);
	return 0;
}

static ssize_t polygator_subsystem_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	size_t len;
	ssize_t res;
	struct subsystem_private_data *private_data = filp->private_data;

	res = (private_data->length > filp->f_pos)?(private_data->length - filp->f_pos):(0);

	if (res) {
		len = res;
		len = min(count, len);
		if (copy_to_user(buff, private_data->buff + filp->f_pos, len)) {
			res = -EINVAL;
			goto polygator_subsystem_read_end;
		}
		*offp = filp->f_pos + len;
	}

polygator_subsystem_read_end:
	return res;
}

static struct file_operations polygator_subsystem_fops = {
	.owner			= THIS_MODULE,
	.open			= polygator_subsystem_open,
	.release		= polygator_subsystem_release,
	.read			= polygator_subsystem_read,
};

char *polygator_print_gsm_module_type(int type)
{
	switch (type) {
		case POLYGATOR_MODULE_TYPE_SIM300: return "SIM300";
		case POLYGATOR_MODULE_TYPE_SIM900: return "SIM900";
		case POLYGATOR_MODULE_TYPE_M10: return "M10";
		case POLYGATOR_MODULE_TYPE_SIM5215: return "SIM5215";
		case POLYGATOR_MODULE_TYPE_SIM5215A2: return "SIM5215A2";
		case POLYGATOR_MODULE_TYPE_M95: return "M95";
		default: return "UNKNOWN";
	}
}

struct polygator_board *polygator_board_register(struct device *device, struct module *owner, char *name, struct cdev *cdev, struct file_operations *fops)
{
	size_t i;
	char brdname[POLYGATOR_BRDNAME_MAXLEN];
	int rc;
	int devno = -1;
	struct polygator_board *brd;

	if (!(brd = kmalloc(sizeof(struct polygator_board), GFP_KERNEL))) {
		log(KERN_ERR, "\"%s\" - can't get memory for struct polygator_board\n", name);
		goto polygator_board_register_error;
	}

	mutex_lock(&polygator_board_list_lock);
	// check for name is not used
	for (i = 0; i < POLYGATOR_BOARD_MAXCOUNT; i++) {
		if ((polygator_board_list[i]) && (!strcmp(polygator_board_list[i]->name, name))) {
			mutex_unlock(&polygator_board_list_lock);
			log(KERN_ERR, "\"%s\" already registered\n", name);
			goto polygator_board_register_error;
		}
	}
	// get free slot
	for (i = 0; i < POLYGATOR_BOARD_MAXCOUNT; i++) {
		if (!polygator_board_list[i]) {
			devno = MKDEV(polygator_subsystem_major, i);
			polygator_board_list[i] = brd;
			brd->devno = devno;
			snprintf(brd->name, POLYGATOR_BRDNAME_MAXLEN, "%s", name);
			break;
		}
	}
	mutex_unlock(&polygator_board_list_lock);

	if (devno < 0) {
		log(KERN_ERR, "\"%s\" - can't get free slot\n", name);
		goto polygator_board_register_error;
	}

	// Add char device to system
	cdev_init(cdev, fops);
	cdev->owner = owner;
	cdev->ops = fops;
	brd->cdev = cdev;
	if ((rc = cdev_add(cdev, devno, 1)) < 0) {
		log(KERN_ERR, "\"%s\" - cdev_add() error=%d\n", name, rc);
		goto polygator_board_register_error;
	}
	snprintf(brdname, POLYGATOR_BRDNAME_MAXLEN, "polygator!%s", name);
	if (!(brd->device = CLASS_DEV_CREATE(polygator_class, devno, device, brdname))) {
		log(KERN_ERR, "\"%s\" - class_dev_create() error\n", name);
		goto polygator_board_register_error;
	}

	verbose("\"%s\" registered\n", name);
	return brd;

polygator_board_register_error:
	if (devno >= 0) {
		mutex_lock(&polygator_board_list_lock);
		for (i = 0; i < POLYGATOR_BOARD_MAXCOUNT; i++) {
			if ((polygator_board_list[i]) && (!strcmp(polygator_board_list[i]->name, name))) {
				polygator_board_list[i] = NULL;
				break;
			}
		}
		mutex_unlock(&polygator_board_list_lock);
	}
	if (brd) {
		kfree(brd);
	}
	return NULL;
}

void polygator_board_unregister(struct polygator_board *brd)
{
	size_t i;

	CLASS_DEV_DESTROY(polygator_class, brd->devno);
	cdev_del(brd->cdev);

	verbose("\"%s\" unregistered\n", brd->name);

	mutex_lock(&polygator_board_list_lock);

	for (i = 0; i < POLYGATOR_BOARD_MAXCOUNT; i++) {
		if ((polygator_board_list[i]) && (!strcmp(polygator_board_list[i]->name, brd->name))) {
			kfree(polygator_board_list[i]);
			polygator_board_list[i] = NULL;
			break;
		}
	}
	mutex_unlock(&polygator_board_list_lock);
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
struct polygator_tty_device *polygator_tty_device_register(struct device *device, void *data, struct tty_port *port, struct tty_operations *tty_ops)
#else
struct polygator_tty_device *polygator_tty_device_register(struct device *device, void *data, struct tty_operations *tty_ops)
#endif
{
	size_t i;
	struct polygator_tty_device *ptd;
	
	if (!(ptd = kmalloc(sizeof(struct polygator_tty_device), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory for struct polygator_tty_device\n");
		goto polygator_tty_device_register_error;
	}
	ptd->tty_minor = -1;

	mutex_lock(&polygator_tty_device_list_lock);
	// get free slot
	for (i = 0; i < POLYGATOR_TTY_DEVICE_MAXCOUNT; i++) {
		if (!polygator_tty_device_list[i]) {
			polygator_tty_device_list[i] = ptd;
			break;
		}
	}
	mutex_unlock(&polygator_tty_device_list_lock);

	if (i >= POLYGATOR_TTY_DEVICE_MAXCOUNT) {
		log(KERN_ERR, "can't get free slot for polygator tty device\n");
		goto polygator_tty_device_register_error;
	}
	ptd->tty_minor = i;
	ptd->tty_ops = tty_ops;

	// register device on sysfs
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
	ptd->device = tty_port_register_device(port, polygator_tty_device_driver, ptd->tty_minor, device);
#else
	ptd->device = tty_register_device(polygator_tty_device_driver, ptd->tty_minor, device);
#endif
	if (IS_ERR(ptd->device)) {
		log(KERN_ERR, "can't register tty device\n");
		goto polygator_tty_device_register_error;
	}
	// set data
	ptd->data = data;

	return ptd;

polygator_tty_device_register_error:
	if (ptd) {
		if (ptd->tty_minor >= 0) {
			mutex_lock(&polygator_tty_device_list_lock);
			polygator_tty_device_list[ptd->tty_minor] = NULL;
			mutex_unlock(&polygator_tty_device_list_lock);
		}
		kfree(ptd);
	}
	return NULL;
}

void polygator_tty_device_unregister(struct polygator_tty_device *ptd)
{
	if (ptd) {
		tty_unregister_device(polygator_tty_device_driver, ptd->tty_minor);
		mutex_lock(&polygator_tty_device_list_lock);
		polygator_tty_device_list[ptd->tty_minor] = NULL;
		mutex_unlock(&polygator_tty_device_list_lock);
		kfree(ptd);
	}
}

static void polygator_power_on_worker(struct timer_list *timer)
{
    int id = 0;
    struct polygator_power_on_entry *entry = NULL, *iter;

    spin_lock(&polygator_power_on_list_lock);

    list_for_each_entry (iter, &polygator_power_on_list, list) {
        id = max(id, iter->id);
    }

    list_for_each_entry (iter, &polygator_power_on_list, list) {
        if (iter->id == id) {
            entry = iter;
            break;
        }
    }

    if (entry) {
        list_del(&entry->list);
        mod_timer(&polygator_power_on_timer, jiffies + HZ);
    }


    spin_unlock(&polygator_power_on_list_lock);

    /* call power on function */
    if (entry) {
        entry->callback(entry->data);
        kfree(entry);
    }
}

int polygator_power_on_schedule(void (* callback)(void *data), void *data)
{
	int id;
	int done;
	struct polygator_power_on_entry *entry;

	do {
		done = 0;
		id = get_random_int() & 0x7fffffff;
		spin_lock_bh(&polygator_power_on_list_lock);
		list_for_each_entry (entry, &polygator_power_on_list, list) {
			if (entry->id == id) {
				done = 1;
				break;
			}
		}
		spin_unlock_bh(&polygator_power_on_list_lock);
	} while (done);

	if (!(entry = kmalloc(sizeof(struct polygator_power_on_entry), GFP_KERNEL))) {
		log(KERN_ERR, "can't alloc memory=%lu bytes\n", (unsigned long int)sizeof(struct polygator_power_on_entry));
		id = -ENOMEM;
		goto polygator_power_on_schedule_end;
	}
	entry->id = id;
	entry->callback = callback;
	entry->data = data;

	spin_lock_bh(&polygator_power_on_list_lock);

	list_add_tail(&entry->list, &polygator_power_on_list);

	if (!timer_pending(&polygator_power_on_timer)) {
		mod_timer(&polygator_power_on_timer, jiffies + 1);
	}

	spin_unlock_bh(&polygator_power_on_list_lock);

polygator_power_on_schedule_end:
	return id;
}

void polygator_power_on_cancel(int id)
{
	struct polygator_power_on_entry *entry = NULL, *iter;

	spin_lock_bh(&polygator_power_on_list_lock);

	list_for_each_entry (iter, &polygator_power_on_list, list) {
		if (iter->id == id) {
			entry = iter;
			break;
		}
	}

	if (entry) {
		list_del(&entry->list);
	}

	if (list_empty(&polygator_power_on_list)) {
		del_timer_sync(&polygator_power_on_timer);
	}

	spin_unlock_bh(&polygator_power_on_list_lock);

	if (entry) {
		kfree(entry);
	}
}

static int __init polygator_init(void)
{
	size_t i;
	dev_t devno;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	struct device *device = NULL;
#else
	struct class_device *device = NULL;
#endif
	int polygator_subsystem_major_reg = 0;
	int rc = -1;

	verbose("loading version \"%s\"...\n", POLYGATOR_LINUX_VERSION);

	for (i = 0; i < POLYGATOR_BOARD_MAXCOUNT; i++) {
		polygator_board_list[i] = NULL;
	}

	// Registering polygator device class
	if (!(polygator_class = class_create(THIS_MODULE, "polygator"))) {
		log(KERN_ERR, "class_create() error\n");
		goto polygator_init_error;
	}
	// Register char device region
	if (polygator_subsystem_major) {
		devno = MKDEV(polygator_subsystem_major, 0);
		rc = register_chrdev_region(devno, POLYGATOR_DEVICE_MAXCOUNT, "polygator");
	} else {
		rc = alloc_chrdev_region(&devno, 0, POLYGATOR_DEVICE_MAXCOUNT, "polygator");
		if(rc >= 0) polygator_subsystem_major = MAJOR(devno);
	}
	if (rc < 0) {
		log(KERN_ERR, "register chrdev region error=%d\n", rc);
		goto polygator_init_error;
	}
	polygator_subsystem_major_reg = 1;

	// Add subsystem device
	cdev_init(&polygator_subsystem_cdev, &polygator_subsystem_fops);
	polygator_subsystem_cdev.owner = THIS_MODULE;
	polygator_subsystem_cdev.ops = &polygator_subsystem_fops;
	devno = MKDEV(polygator_subsystem_major, POLYGATOR_DEVICE_MAXCOUNT - 1);
	if ((rc = cdev_add(&polygator_subsystem_cdev, devno, 1)) < 0) {
		log(KERN_ERR, "\"subsystem\" - cdev_add() error=%d\n", rc);
		goto polygator_init_error;
	}
	if (!(device = CLASS_DEV_CREATE(polygator_class, devno, NULL, "polygator!subsystem"))) {
		log(KERN_ERR, "\"subsystem\" - class_dev_create() error\n");
		goto polygator_init_error;
	}

	// Register polygator tty driver
	polygator_tty_device_driver = alloc_tty_driver(POLYGATOR_TTY_DEVICE_MAXCOUNT);
	if (!polygator_tty_device_driver) {
		log(KERN_ERR, "can't allocated memory for tty driver\n");
		rc = -ENOMEM;
		goto polygator_init_error;
	}

	polygator_tty_device_driver->owner = THIS_MODULE;
	polygator_tty_device_driver->driver_name = "polygator";
	polygator_tty_device_driver->name = "ttyPG";
	polygator_tty_device_driver->major = polygator_tty_major;
	polygator_tty_device_driver->minor_start = 0;
	polygator_tty_device_driver->type = TTY_DRIVER_TYPE_SERIAL;
	polygator_tty_device_driver->subtype = SERIAL_TYPE_NORMAL;
	polygator_tty_device_driver->init_termios = tty_std_termios;
	polygator_tty_device_driver->init_termios.c_iflag &= ~ICRNL;
	polygator_tty_device_driver->init_termios.c_cflag = B115200 | CS8 | HUPCL | CLOCAL | CREAD;
	polygator_tty_device_driver->init_termios.c_lflag &= ~ECHO;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
	polygator_tty_device_driver->init_termios.c_ispeed = 115200;
	polygator_tty_device_driver->init_termios.c_ospeed = 115200;
#endif
	polygator_tty_device_driver->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	tty_set_operations(polygator_tty_device_driver, &polygator_tty_device_ops);

	if ((rc = tty_register_driver(polygator_tty_device_driver))) {
		log(KERN_ERR, "can't register polygator_tty_device driver: rc=%d\n", rc);
		// release allocated tty driver environment
		put_tty_driver(polygator_tty_device_driver);
		polygator_tty_device_driver = NULL;
		goto polygator_init_error;
	}

    /* init power on functionality */
    spin_lock_init(&polygator_power_on_list_lock);
    timer_setup(&polygator_power_on_timer, polygator_power_on_worker, 0);

    verbose("loaded successfull\n");
    return 0;

polygator_init_error:
	if (polygator_tty_device_driver) {
		tty_unregister_driver(polygator_tty_device_driver);
		put_tty_driver(polygator_tty_device_driver);
	}
	if (device) {
		CLASS_DEV_DESTROY(polygator_class, MKDEV(polygator_subsystem_major, POLYGATOR_DEVICE_MAXCOUNT - 1));
	}
	if (polygator_subsystem_major_reg) {
		unregister_chrdev_region(MKDEV(polygator_subsystem_major, 0), POLYGATOR_DEVICE_MAXCOUNT);
	}
	if (polygator_class) {
		class_destroy(polygator_class);
	}
	return rc;
}

static void __exit polygator_exit(void)
{
	// destroy power on functionality
	del_timer_sync(&polygator_power_on_timer);

	// Unregister polygator tty driver
	tty_unregister_driver(polygator_tty_device_driver);
	put_tty_driver(polygator_tty_device_driver);
	// Destroy subsystem device
	CLASS_DEV_DESTROY(polygator_class, MKDEV(polygator_subsystem_major, POLYGATOR_DEVICE_MAXCOUNT - 1));
	cdev_del(&polygator_subsystem_cdev);
	// Unregister char device region
	unregister_chrdev_region(MKDEV(polygator_subsystem_major, 0), POLYGATOR_DEVICE_MAXCOUNT);
	// Destroy polygator device class
	class_destroy(polygator_class);

	verbose("stopped\n");
}

module_init(polygator_init);
module_exit(polygator_exit);

/******************************************************************************/
/* end of polygator-base.c                                                    */
/******************************************************************************/
