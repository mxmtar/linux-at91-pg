
#include <linux/delay.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/version.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include <asm/io.h>

#include "polygator-base.h"

#include "vinetic-base.h"
#include "vinetic-def.h"

#include "simcard-base.h"

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,36) // 2,6,30 - orig
#define TTY_PORT
#endif

#define verbose(_fmt, _args...) printk(KERN_INFO "[polygator-%s] " _fmt, "gx", ## _args)
#define log(_level, _fmt, _args...) printk(_level "[polygator-%s] %s:%d - %s(): " _fmt, "gx", "gx.c", __LINE__, __PRETTY_FUNCTION__, ## _args)
#define debug(_fmt, _args...) printk(KERN_DEBUG "[polygator-%s] %s:%d - %s(): " _fmt, "gx", "gx.c", __LINE__, __PRETTY_FUNCTION__, ## _args)

#define MAX_GX_BOARD_COUNT		5

#define BOARD_TYPE_UNKNOWN		0
#define BOARD_TYPE_G4			4
#define BOARD_TYPE_G8			8

#define MB_MODE_AUTONOM			0x2300	// 0 - bank, 1 -standalone
#define MB_CS_ROM_KROSS			0x4000
#define MB_RESET_ROM			0x4800

#define GX_CS_VINETIC			0x1100
#define GX_CS_STATUS_VINETIC	0x1120
#define GX_PRESENCE_TEST1		0x1180
#define GX_MODE_AUTONOM			0x1190
#define GX_SIP_ONLY				0x11a0
#define GX_PRESENCE_TEST0		0x11b0
#define GX_RESET_VINETIC		0x11c0
#define GX_CS_ROM				0x11d0
#define GX_CS_PRESENCE			0x11e0
#define GX_RESET_BOARD			0x11f0

union gx_gsm_mod_status_reg {
	struct {
		u_int8_t status:1;
		u_int8_t at_rd_empty:1;
		u_int8_t at_wr_empty:1;
		u_int8_t sim_rd_ready:1;
		u_int8_t sim_wr_ready:1;
		u_int8_t sim_rst_req:1;
		u_int8_t imei_rd_empty:1;
		u_int8_t imei_wr_empty:1;
	} __attribute__((packed)) bits;
	u_int8_t full;
} __attribute__((packed));

union gx_gsm_mod_control_reg {
	struct {
		u_int8_t vbat:1; // 1 - disable, 0 - enable
		u_int8_t pkey:1;
		u_int8_t gap:2;
		u_int8_t cn_speed_a:1;
		u_int8_t cn_speed_b:1;
		u_int8_t at_baudrate:2;
	} __attribute__((packed)) bits;
	u_int8_t full;
} __attribute__((packed));

struct gx_board;
struct gx_gsm_module_data {

    int type;
    size_t pos_on_board;
    struct gx_board *board;

    spinlock_t lock;
    union gx_gsm_mod_control_reg control;
    int power_on_id;

    uintptr_t cbdata;

	void (* set_control)(uintptr_t cbdata, size_t pos, u_int8_t reg);
	u_int8_t (* get_status)(uintptr_t cbdata, size_t pos);
	void (* at_write)(uintptr_t cbdata, size_t pos, u_int8_t reg);
	u_int8_t (* at_read)(uintptr_t cbdata, size_t pos);
	void (* at_write_sim16)(uintptr_t cbdata, size_t pos, u_int8_t reg);
	u_int16_t (* at_read_sim16)(uintptr_t cbdata, size_t pos);
	void (* sim_write)(uintptr_t cbdata, size_t pos, u_int8_t reg);
	u_int8_t (* sim_read)(uintptr_t cbdata, size_t pos);
	void (* sim_do_after_reset)(uintptr_t cbdata, size_t pos);
	void (* imei_write)(uintptr_t cbdata, size_t pos, u_int8_t reg);
	u_int8_t (* imei_read)(uintptr_t cbdata, size_t pos);

    /* at section */
    int at_port_select;
    spinlock_t at_lock;
#ifdef TTY_PORT
	struct tty_port at_port;
#else
	size_t at_count;
	struct tty_struct *at_tty;
	unsigned char *at_xmit_buf;
#endif
	size_t at_xmit_count;
	size_t at_xmit_head;
	size_t at_xmit_tail;
	struct timer_list at_poll_timer;
};

struct gx_board {

	struct polygator_board *pg_board;
	struct cdev cdev;

	char name[POLYGATOR_BRDNAME_MAXLEN];

	u_int32_t type;
	u_int32_t pos;

	u_int8_t rom[256];
	size_t romsize;

	u_int32_t ver_maj;
	u_int32_t ver_min;

	int sim16;

	size_t vinetics_count;
	struct vinetic *vinetics[2];

	size_t channels_count;
	struct gx_gsm_module_data *gsm_modules[8];
	struct polygator_tty_device *tty_at_channels[8];
	struct simcard_device *simcard_channels[8];
};

struct gx_board_private_data {
	struct gx_board *board;
	char buff[0x10000];
	size_t length;
};

static int gx_tty_at_open(struct tty_struct *tty, struct file *filp);
static void gx_tty_at_close(struct tty_struct *tty, struct file *filp);
static int gx_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count);
static int gx_tty_at_write_room(struct tty_struct *tty);
static int gx_tty_at_chars_in_buffer(struct tty_struct *tty);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void gx_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios);
#else
static void gx_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios);
#endif
static void gx_tty_at_flush_buffer(struct tty_struct *tty);
static void gx_tty_at_hangup(struct tty_struct *tty);

static struct tty_operations gx_tty_at_ops = {
	.open = gx_tty_at_open,
	.close = gx_tty_at_close,
	.write = gx_tty_at_write,
	.write_room = gx_tty_at_write_room,
	.chars_in_buffer = gx_tty_at_chars_in_buffer,
	.set_termios = gx_tty_at_set_termios,
	.flush_buffer = gx_tty_at_flush_buffer,
	.hangup = gx_tty_at_hangup,
};

static int gx_tty_at_port_carrier_raised(struct tty_port *port);
static void gx_tty_at_port_dtr_rts(struct tty_port *port, int onoff);
static int gx_tty_at_port_activate(struct tty_port *tport, struct tty_struct *tty);
static void gx_tty_at_port_shutdown(struct tty_port *port);

static const struct tty_port_operations gx_tty_at_port_ops = {
	.carrier_raised = gx_tty_at_port_carrier_raised,
	.dtr_rts = gx_tty_at_port_dtr_rts,
	.activate = gx_tty_at_port_activate,
	.shutdown = gx_tty_at_port_shutdown,
};

static char mainboard_rom[256];
static struct gx_board *gx_boards[5];

static struct resource *gx_cs3_iomem_reg = NULL;
static void __iomem *gx_cs3_base_ptr = NULL;

static void gx_vinetic_reset(uintptr_t cbdata)
{
	void __iomem *addr;

	addr = gx_cs3_base_ptr;
	addr += cbdata + GX_RESET_VINETIC;
	iowrite8(0, addr);
	mdelay(10);
	iowrite8(1,  addr);
	mdelay(2);
// 	log(KERN_INFO, "%08lx\n", addr);
}
static void gx_vinetic_write_nwd(uintptr_t cbdata, u_int16_t value)
{
	void __iomem *addr;

	addr = gx_cs3_base_ptr;
	addr += cbdata + GX_CS_VINETIC + 0x04;
	iowrite16(value, addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
}
static void gx_vinetic_write_eom(uintptr_t cbdata, u_int16_t value)
{
	void __iomem *addr;

	addr = gx_cs3_base_ptr;
	addr += cbdata + GX_CS_VINETIC + 0x06;
	iowrite16(value, addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
}
static u_int16_t gx_vinetic_read_nwd(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr;

	addr = gx_cs3_base_ptr;
	addr += cbdata + GX_CS_VINETIC + 0x04;
	value = ioread16(addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
	return value;
}
static u_int16_t gx_vinetic_read_eom(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr;

	addr = gx_cs3_base_ptr;
	addr += cbdata + GX_CS_VINETIC + 0x06;
	value = ioread16(addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, value);
	return value;
}
#if 0
static size_t gx_vinetic_is_not_ready(uintptr_t cbdata)
{
	size_t status;
	uintptr_t addr;

	addr = (uintptr_t)gx_cs3_base_ptr;
	addr += cbdata + GX_CS_STATUS_VINETIC;
	status = ioread8(addr);
// 	log(KERN_INFO, "%08lx: %04x\n", addr, (unsigned int)status);
	status &= 1;
// 	log(KERN_INFO, "%08lx: %lu\n", addr, (long unsigned int)status);
	return status;
}
#else
static size_t gx_vinetic_is_not_ready(uintptr_t cbdata)
{
	void __iomem *addr;
	union vin_reg_ir reg_ir;

	addr = gx_cs3_base_ptr;
	addr += cbdata + GX_CS_VINETIC + 0x18;
	reg_ir.full = ioread16(addr);
	return reg_ir.bits.rdyq;
}
#endif
static u_int16_t gx_vinetic_read_dia(uintptr_t cbdata)
{
	u_int16_t value;
	void __iomem *addr;

	addr = gx_cs3_base_ptr;
	addr += cbdata + GX_CS_VINETIC + 0x18;
	value = ioread16(addr);
	return value;
}

static void gx_gsm_mod_set_control(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	void __iomem *addr = (void __iomem *)cbdata;

	iowrite8(reg, addr);
}

static u_int8_t gx_gsm_mod_get_status(uintptr_t cbdata, size_t pos)
{
	void __iomem *addr = (void __iomem *)cbdata;

	return ioread8(addr);
}

static void gx_gsm_mod_init_counters(uintptr_t cbdata, size_t pos)
{
    void __iomem *addr = (void __iomem *)cbdata;

    iowrite8(0, addr + 0x3c);
    iowrite8(0, addr + 0x3a);
    iowrite8(0, addr + 0x38);
    iowrite8(0, addr + 0x36);
    iowrite8(0, addr + 0x34);
}

static void gx_gsm_mod_at_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	void __iomem *addr = (void __iomem *)cbdata;

	iowrite8(reg, addr + 0x10);
	iowrite8(0, addr + 0x3c);
	iowrite8(1, addr + 0x3c);
	iowrite8(0, addr + 0x3c);
}

static u_int8_t gx_gsm_mod_at_read(uintptr_t cbdata, size_t pos)
{
	u_int8_t data;
	void __iomem *addr = (void __iomem *)cbdata;

	data = ioread8(addr + 0x10);
	iowrite8(0, addr + 0x3a);
	iowrite8(1, addr + 0x3a);
	iowrite8(0, addr + 0x3a);

	return data;
}

static void gx_gsm_mod_at_write_sim16(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
	void __iomem *addr = (void __iomem *)cbdata;

	iowrite8(reg, addr + 0x10);
	iowrite8(0, addr + 0x3c);
	iowrite8(1, addr + 0x3c);
	iowrite8(0, addr + 0x3c);
}

static u_int16_t gx_gsm_mod_at_read_sim16(uintptr_t cbdata, size_t pos)
{
	u_int16_t data;
	void __iomem *addr = (void __iomem *)cbdata;

	data = ioread16(addr + 0x10);
	if ((data & 0x0200) == 0) {
		iowrite8(0, addr + 0x3a);
		iowrite8(1, addr + 0x3a);
		iowrite8(0, addr + 0x3a);
	}

	return data;
}

static void gx_gsm_mod_sim_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
    void __iomem *addr = (void __iomem *)cbdata;

    iowrite8(reg, addr + 0x20);
    iowrite8(0, addr + 0x38);
    iowrite8(1, addr + 0x38);
}

static u_int8_t gx_gsm_mod_sim_read(uintptr_t cbdata, size_t pos)
{
    u_int8_t data;
    void __iomem *addr = (void __iomem *)cbdata;

    data = ioread8(addr + 0x20);
    iowrite8(0, addr + 0x36);
    iowrite8(1, addr + 0x36);

    udelay(200);

    return data;
}

static void gx_gsm_mod_sim_do_after_reset(uintptr_t cbdata, size_t pos)
{
    void __iomem *addr = (void __iomem *)cbdata;

    iowrite8(0x10, addr + 0x34);
    iowrite8(0x00, addr + 0x34);
    iowrite8(0x10, addr + 0x34);
}

static void gx_gsm_mod_imei_write(uintptr_t cbdata, size_t pos, u_int8_t reg)
{
    void __iomem *addr = (void __iomem *)cbdata;

    iowrite8(reg, addr + 0x30);
}

static u_int8_t gx_gsm_mod_imei_read(uintptr_t cbdata, size_t pos)
{
    void __iomem *addr = (void __iomem *)cbdata;

    return ioread8(addr + 0x30);
}

static u_int8_t gx_sim_read(void *data)
{
    struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)data;

    return mod->sim_read(mod->cbdata, mod->pos_on_board);
}

static void gx_sim_write(void *data, u_int8_t value)
{
    struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)data;

    mod->sim_write(mod->cbdata, mod->pos_on_board, value);
}

static int gx_sim_is_read_ready(void *data)
{
    union gx_gsm_mod_status_reg status;
    struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)data;

    status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

    return status.bits.sim_rd_ready;
}

static int gx_sim_is_write_ready(void *data)
{
    union gx_gsm_mod_status_reg status;
    struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)data;

    status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

    return status.bits.sim_wr_ready;
}

static int gx_sim_is_reset_request(void *data)
{
    union gx_gsm_mod_status_reg status;
    struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)data;

    status.full = mod->get_status(mod->cbdata, mod->pos_on_board);

    return status.bits.sim_rst_req;
}

static void gx_sim_set_speed(void *data, int speed)
{
    struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)data;

    switch (speed) {
        case 0x94: case 57600:
            mod->control.bits.cn_speed_a = 1;
            mod->control.bits.cn_speed_b = 0;
            break;
        case 0x95: case 115200:
            mod->control.bits.cn_speed_a = 0;
            mod->control.bits.cn_speed_b = 1;
            break;
        case 0x96: case 230400:
            mod->control.bits.cn_speed_a = 1;
            mod->control.bits.cn_speed_b = 1;
            break;
        case 0x11: case 9600: default:
            mod->control.bits.cn_speed_a = 0;
            mod->control.bits.cn_speed_b = 0;
            break;
    }

    mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
}

static void gx_sim_do_after_reset(void *data)
{
    struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)data;

    mod->sim_do_after_reset(mod->cbdata, mod->pos_on_board);
}

static void gx_power_on(void *cbdata)
{
    struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)cbdata;

    spin_lock(&mod->lock);

    mod->power_on_id = -1;

    mod->control.bits.vbat = 0;
    mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);

    spin_unlock(&mod->lock);
}

static void gx_tty_at_poll(struct timer_list *timer)
{
	char buff[512];
	size_t len;
	u_int16_t rd16;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,9,0)
	struct tty_struct *tty;
#endif
	union gx_gsm_mod_status_reg status;
	struct gx_gsm_module_data *mod = container_of(timer, struct gx_gsm_module_data, at_poll_timer);

	len = 0;
#if 0
	// read received data
	while (len < sizeof(buff))
	{
		// read status register
		at->status.full = at->mod_status(at->cbdata, at->pos_on_board);
		if (at->status.bits.at_rd_empty)
			break;
		// put char to receiving buffer
		buff[len++] = at->mod_at_read(at->cbdata, at->pos_on_board);
	}
#else
	// read received data
	while (len < sizeof(buff)) {
		// read status register
		status.full = mod->get_status(mod->cbdata, mod->pos_on_board);
		// select port
		if (mod->at_port_select) {
			// auxilary
			if (status.bits.imei_rd_empty) {
				break;
			}
			// put char to receiving buffer
			buff[len++] = mod->imei_read(mod->cbdata, mod->pos_on_board);
		} else {
			// main
			if (mod->board->sim16) {
				rd16 = mod->at_read_sim16(mod->cbdata, mod->pos_on_board);
				if (rd16 & 0x0200) {
					break;
				}
				// put char to receiving buffer
				buff[len++] = rd16 & 0xff;
			} else {
				if (status.bits.at_rd_empty) {
					break;
				}
				// put char to receiving buffer
				buff[len++] = mod->at_read(mod->cbdata, mod->pos_on_board);
			}
		}
	}
#endif

	spin_lock(&mod->at_lock);

	 while (mod->at_xmit_count) {
		// read status register
		status.full = mod->get_status(mod->cbdata, mod->pos_on_board);
		// select port
		if (mod->at_port_select) {
			// auxilary
			// check for transmitter is ready
			if (status.bits.imei_wr_empty) {
				// put char to transmitter buffer
#ifdef TTY_PORT
				mod->imei_write(mod->cbdata, mod->pos_on_board, mod->at_port.xmit_buf[mod->at_xmit_tail]);
#else
				mod->imei_write(mod->cbdata, mod->pos_on_board, mod->at_xmit_buf[mod->at_xmit_tail]);
#endif
				++mod->at_xmit_tail;
				if (mod->at_xmit_tail == SERIAL_XMIT_SIZE) {
					mod->at_xmit_tail = 0;
				}
				--mod->at_xmit_count;
			}
		} else {
			// main
			// check for transmitter is ready
			if (status.bits.at_wr_empty) {
				// put char to transmitter buffer
#ifdef TTY_PORT
				if (mod->board->sim16) {
					mod->at_write_sim16(mod->cbdata, mod->pos_on_board, mod->at_port.xmit_buf[mod->at_xmit_tail]);
				} else {
					mod->at_write(mod->cbdata, mod->pos_on_board, mod->at_port.xmit_buf[mod->at_xmit_tail]);
				}
#else
				if (mod->board->sim16) {
					mod->at_write_sim16(mod->cbdata, mod->pos_on_board, mod->at_xmit_buf[mod->at_xmit_tail]);
				} else {
					mod->at_write(mod->cbdata, mod->pos_on_board, mod->at_xmit_buf[mod->at_xmit_tail]);
				}
#endif
				++mod->at_xmit_tail;
				if (mod->at_xmit_tail == SERIAL_XMIT_SIZE) {
					mod->at_xmit_tail = 0;
				}
				--mod->at_xmit_count;
			}
		}
	}

	spin_unlock(&mod->at_lock);

	if (len) {
#ifdef TTY_PORT
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
		tty_insert_flip_string(&mod->at_port, buff, len);
		tty_flip_buffer_push(&mod->at_port);
#else
		tty = tty_port_tty_get(&mod->at_port);
		tty_insert_flip_string(tty, buff, len);
		tty_flip_buffer_push(tty);
		tty_kref_put(tty);
#endif
#else
		tty = mod->at_tty;
		tty_insert_flip_string(tty, buff, len);
		tty_flip_buffer_push(tty);
#endif
	}

	mod_timer(&mod->at_poll_timer, jiffies + 1);
}

static int gx_board_open(struct inode *inode, struct file *filp)
{
	ssize_t res;
	size_t i, j;
	size_t len;

	struct gx_board *brd;
	struct gx_board_private_data *private_data;
	union gx_gsm_mod_status_reg status;
	struct gx_gsm_module_data *mod;

	brd = container_of(inode->i_cdev, struct gx_board, cdev);

	if (!(private_data = kmalloc(sizeof(struct gx_board_private_data), GFP_KERNEL))) {
		log(KERN_ERR, "can't get memory=%lu bytes\n", (unsigned long int)sizeof(struct gx_board_private_data));
		res = -ENOMEM;
		goto gx_open_error;
	}
	private_data->board = brd;

    len = 0;

    len += sprintf(private_data->buff + len, "{\r\n\t\"hardware\": \"gx\",");

    len += sprintf(private_data->buff + len, "\r\n\t\"rom\": \"%s\",", &brd->rom[2]);

    len += sprintf(private_data->buff + len, "\r\n\t\"channels\": [");
    for (i = 0; i < brd->channels_count; ++i) {
        if (brd->tty_at_channels[i]) {
            mod = brd->gsm_modules[i];
            status.full = mod->get_status(mod->cbdata, mod->pos_on_board);
            len += sprintf(private_data->buff + len, "%s\r\n\t\t{\
                                                        \r\n\t\t\t\"tty\": \"%s\",\
                                                        \r\n\t\t\t\"sim\": \"%s\",\
                                                        \r\n\t\t\t\"module\": \"%s\",\
                                                        \r\n\t\t\t\"power\": \"%s\",\
                                                        \r\n\t\t\t\"key\": \"%s\",\
                                                        \r\n\t\t\t\"status\": \"%s\",\
                                                        \r\n\t\t\t\"audio\": {\
                                                        \r\n\t\t\t\t\"vinetic\": %lu,\
                                                        \r\n\t\t\t\t\"rtp\": %lu\
                                                        \r\n\t\t\t}\
                                                        \r\n\t\t}",
                            i ? "," : "",
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
                            brd->tty_at_channels[i] ? dev_name(brd->tty_at_channels[i]->device) : "unknown",
                            brd->simcard_channels[i] ? dev_name(brd->simcard_channels[i]->device) : "unknown",
#else
                            brd->tty_at_channels[i] ? brd->tty_at_channels[i]->device->class_id : "unknown",
                            brd->simcard_channels[i] ? brd->simcard_channels[i]->device->class_id : "unknown",
#endif
                            polygator_print_gsm_module_type(mod->type),
                            (mod->control.bits.vbat == 0) ? "on" : "off",
                            (mod->control.bits.pkey == 0) ? "on" : "off",
                            status.bits.status ? "on" : "off",
                            (unsigned long int)(i / 4),
                            (unsigned long int)(i % 4));
        }
    }
    len += sprintf(private_data->buff + len, "\r\n\t],");
    /* vinetic */
    len += sprintf(private_data->buff + len, "\r\n\t\"vinetic\": [");
    for (i = 0; i < 2; ++i) {
        if (brd->vinetics[i]) {
            len += sprintf(private_data->buff + len, "%s\r\n\t\t{\r\n\t\t\t\"path\": \"%s\",",
                            i ? "," : "",
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
                            dev_name(brd->vinetics[i]->device)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
                            dev_name(brd->vinetics[i]->device)
#else
                            brd->vinetics[i]->device->class_id
#endif
                            );
            len += sprintf(private_data->buff + len, "\r\n\t\t\t\"rtp\": [");
            for (j = 0; j < 4; ++j) {
                if (brd->vinetics[i]->rtp_channels[j])
                    len += sprintf(private_data->buff + len, "%s\r\n\t\t\t\t{\r\n\t\t\t\t\t\"path\": \"%s\"\r\n\t\t\t\t}",
                                j ? "," : "",
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
                                dev_name(brd->vinetics[i]->rtp_channels[j]->device)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,26)
                                dev_name(brd->vinetics[i]->rtp_channels[j]->device)
#else
                                brd->vinetics[i]->rtp_channels[j]->device->class_id
#endif
                                );
            }
            len += sprintf(private_data->buff + len, "\r\n\t\t\t]");
            len += sprintf(private_data->buff + len, "\r\n\t\t}");
        }
    }
    len += sprintf(private_data->buff + len, "\r\n\t]");

    len += sprintf(private_data->buff + len, "\r\n}\r\n");

    private_data->length = len;

    filp->private_data = private_data;

    return 0;

gx_open_error:
	if (private_data) {
		kfree(private_data);
	}
	return res;
}

static int gx_board_release(struct inode *inode, struct file *filp)
{
	struct gx_board_private_data *private_data = filp->private_data;

	kfree(private_data);
	return 0;
}

static ssize_t gx_board_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
	size_t len;
	ssize_t res;
	struct gx_board_private_data *private_data = filp->private_data;

	res = (private_data->length > filp->f_pos)?(private_data->length - filp->f_pos):(0);

	if (res) {
		len = res;
		len = min(count, len);
		if (copy_to_user(buff, private_data->buff + filp->f_pos, len)) {
			res = -EINVAL;
			goto gx_board_read_end;
		}
		*offp = filp->f_pos + len;
	}

gx_board_read_end:
	return res;
}

static ssize_t gx_board_write(struct file *filp, const char __user *buff, size_t count, loff_t *offp)
{
	ssize_t res;
	char cmd[256];
	size_t len;

	u_int32_t chan;
	u_int32_t value;
	struct gx_gsm_module_data *mod;
	struct gx_board_private_data *private_data = filp->private_data;

	memset(cmd, 0, sizeof(cmd));
	len = sizeof(cmd) - 1;
	len = min(len, count);

	if (copy_from_user(cmd, buff, len)) {
		res = -EINVAL;
		goto gx_board_write_end;
	}

    if (sscanf(cmd, "channel[%u].power_supply(%u)", &chan, &value) == 2) {
        if ((chan >= 0) && (chan < private_data->board->channels_count) && (private_data->board->gsm_modules[chan])) {

            mod = private_data->board->gsm_modules[chan];

            spin_lock_bh(&mod->lock);

            if (value) {
                if (mod->control.bits.vbat == 1) {
                    if (mod->power_on_id == -1) {
                        res = polygator_power_on_schedule(gx_power_on, mod);
                        if (res >= 0) {
                            mod->power_on_id = res;
                            res = len;
                        }
                    } else {
                        res = -EAGAIN;
                    }
                } else {
                    res = len;
                }
            } else {
                if (mod->power_on_id != -1) {
                    polygator_power_on_cancel(mod->power_on_id);
                    mod->power_on_id = -1;
                }
                mod->control.bits.vbat = 1;
                mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
                res = len;
            }

            spin_unlock_bh(&mod->lock);

            res = len;
        } else {
            res= -ENODEV;
        }
    } else if (sscanf(cmd, "channel[%u].power_key(%u)", &chan, &value) == 2) {
        if ((chan >= 0) && (chan < private_data->board->channels_count) && (private_data->board->gsm_modules[chan])) {

            mod = private_data->board->gsm_modules[chan];

            spin_lock_bh(&mod->lock);

            mod->control.bits.pkey = (value == 0) ? 1 : 0;
            mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);

            spin_unlock_bh(&mod->lock);

            res = len;
        } else {
            res= -ENODEV;
        }
    } else if (sscanf(cmd, "channel[%u].uart.baudrate(%u)", &chan, &value) == 2) {
        if ((chan >= 0) && (chan < private_data->board->channels_count) && (private_data->board->gsm_modules[chan])) {

            mod = private_data->board->gsm_modules[chan];

            spin_lock_bh(&mod->lock);

            if (value == 9600) {
                mod->control.bits.at_baudrate = 0;
            } else {
                mod->control.bits.at_baudrate = 2;
            }
            mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);

            spin_unlock_bh(&mod->lock);

            res = len;
        } else {
            res= -ENODEV;
        }
    } else if (sscanf(cmd, "channel[%u].uart.port(%u)", &chan, &value) == 2) {
        if ((chan >= 0) && (chan < private_data->board->channels_count) && (private_data->board->gsm_modules[chan])) {
            mod = private_data->board->gsm_modules[chan];
            mod->at_port_select = value;
            res = len;
        } else {
            res = -ENODEV;
        }
    } else if (sscanf(cmd, "channel[%u].smart_card.enable(%u)", &chan, &value) == 2) {
        if (value) {
            /* enable */
            iowrite8(0, gx_cs3_base_ptr + MB_MODE_AUTONOM);
            iowrite8(0, gx_cs3_base_ptr + GX_MODE_AUTONOM + (0x0200 * private_data->board->pos));
            if (private_data->board->type == BOARD_TYPE_G8) {
                iowrite8(0, gx_cs3_base_ptr + GX_MODE_AUTONOM + (0x0200 * (private_data->board->pos + 1)));
            }
        } else {
            /* disable */
            iowrite8(1, gx_cs3_base_ptr + MB_MODE_AUTONOM);
            iowrite8(1, gx_cs3_base_ptr + GX_MODE_AUTONOM + (0x0200 * private_data->board->pos));
            if (private_data->board->type == BOARD_TYPE_G8) {
                iowrite8(1, gx_cs3_base_ptr + GX_MODE_AUTONOM + (0x0200 * (private_data->board->pos + 1)));
            }
        }
        res = len;
    } else {
        res = -ENOMSG;
    }
gx_board_write_end:
    return res;
}

static struct file_operations gx_board_fops = {
	.owner   = THIS_MODULE,
	.open    = gx_board_open,
	.release = gx_board_release,
	.read    = gx_board_read,
	.write   = gx_board_write,
};

static int gx_tty_at_open(struct tty_struct *tty, struct file *filp)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)ptd->data;

#ifdef TTY_PORT
	return tty_port_open(&mod->at_port, tty, filp);
#else
	unsigned char *xbuf;
	
	if (!(xbuf = kmalloc(SERIAL_XMIT_SIZE, GFP_KERNEL))) {
		return -ENOMEM;
	}

	spin_lock_bh(&mod->at_lock);

	if (!mod->at_count++) {
		mod->at_xmit_buf = xbuf;
		mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;

        mod_timer(&mod->at_poll_timer, jiffies + 1);
	
		mod->at_tty = tty;
	} else {
		kfree(xbuf);
	}

	spin_unlock_bh(&mod->at_lock);

	return 0;
#endif
}

static void gx_tty_at_close(struct tty_struct *tty, struct file *filp)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)ptd->data;

#ifdef TTY_PORT
	tty_port_close(&mod->at_port, tty, filp);
#else
	unsigned char *xbuf = NULL;

	spin_lock_bh(&mod->at_lock);

	if (!--mod->at_count) {
		xbuf = mod->at_xmit_buf;
		mod->at_tty = NULL;
	}

	spin_unlock_bh(&mod->at_lock);
	
	if (xbuf) {
		del_timer_sync(&mod->at_poll_timer);
		kfree(mod->at_xmit_buf);
	}
#endif
	return;
}

static int gx_tty_at_write(struct tty_struct *tty, const unsigned char *buf, int count)
{
	int res = 0;
	size_t len;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);

	if (mod->at_xmit_count < SERIAL_XMIT_SIZE) {
		while (1) {
			if (mod->at_xmit_head == mod->at_xmit_tail) {
				if (mod->at_xmit_count) {
					len = 0;
				} else {
					len = SERIAL_XMIT_SIZE - mod->at_xmit_head;
				}
			} else if (mod->at_xmit_head > mod->at_xmit_tail) {
				len = SERIAL_XMIT_SIZE - mod->at_xmit_head;
			} else {
				len = mod->at_xmit_tail - mod->at_xmit_head;
			}
			len = min(len, (size_t)count);
			if (!len) {
				break;
			}
#ifdef TTY_PORT
			memcpy(mod->at_port.xmit_buf + mod->at_xmit_head, buf, len);
#else
			memcpy(mod->at_xmit_buf + mod->at_xmit_head, buf, len);
#endif
			mod->at_xmit_head += len;
			if (mod->at_xmit_head == SERIAL_XMIT_SIZE) {
				mod->at_xmit_head = 0;
			}
			mod->at_xmit_count += len;
			buf += len;
			count -= len;
			res += len;
		}
	}

	spin_unlock_bh(&mod->at_lock);

	return res ;
}

static int gx_tty_at_write_room(struct tty_struct *tty)
{
	int res;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);

	res = SERIAL_XMIT_SIZE - mod->at_xmit_count;

	spin_unlock_bh(&mod->at_lock);

	return res;
}

static int gx_tty_at_chars_in_buffer(struct tty_struct *tty)
{
	int res;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);

	res = mod->at_xmit_count;

	spin_unlock_bh(&mod->at_lock);

	return res;
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,20)
static void gx_tty_at_set_termios(struct tty_struct *tty, struct ktermios *old_termios)
#else
static void gx_tty_at_set_termios(struct tty_struct *tty, struct termios *old_termios)
#endif
{
	speed_t baud;
	struct polygator_tty_device *ptd = tty->driver_data;
	struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)ptd->data;

	baud = tty_get_baud_rate(tty);

	spin_lock_bh(&mod->at_lock);

	switch (baud) {
		case 9600:
			mod->control.bits.at_baudrate = 0;
			break;
		default:
			mod->control.bits.at_baudrate = 2;
			break;
	}
	
	mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);

	spin_unlock_bh(&mod->at_lock);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
	tty_encode_baud_rate(tty, baud, baud);
#endif
}

static void gx_tty_at_flush_buffer(struct tty_struct *tty)
{
	struct polygator_tty_device *ptd = tty->driver_data;
	struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)ptd->data;

	spin_lock_bh(&mod->at_lock);
	mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;
	spin_unlock_bh(&mod->at_lock);
	tty_wakeup(tty);
}

static void gx_tty_at_hangup(struct tty_struct *tty)
{
#ifdef TTY_PORT
	struct polygator_tty_device *ptd = tty->driver_data;
	struct gx_gsm_module_data *mod = (struct gx_gsm_module_data *)ptd->data;
	tty_port_hangup(&mod->at_port);
#endif
}
#ifdef TTY_PORT
static int gx_tty_at_port_carrier_raised(struct tty_port *port)
{
	return 1;
}

static void gx_tty_at_port_dtr_rts(struct tty_port *port, int onoff)
{
}

static int gx_tty_at_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct gx_gsm_module_data *mod = container_of(port, struct gx_gsm_module_data, at_port);

	if (tty_port_alloc_xmit_buf(port) < 0) {
		return -ENOMEM;
	}
	mod->at_xmit_count = mod->at_xmit_head = mod->at_xmit_tail = 0;

    mod_timer(&mod->at_poll_timer, jiffies + 1);

	return 0;
}

static void gx_tty_at_port_shutdown(struct tty_port *port)
{
	struct gx_gsm_module_data *mod = container_of(port, struct gx_gsm_module_data, at_port);

	del_timer_sync(&mod->at_poll_timer);

	tty_port_free_xmit_buf(port);
}
#endif

static const struct of_device_id gx_of_ids[] = {
	{
		.compatible = "polygator,gx",
	},
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, gx_of_ids);

static int gx_driver_probe(struct platform_device *pdev)
{
    size_t i, j, k;
    struct gx_board *brd;
    struct vinetic *vin;
    unsigned char *str, *dp;
    u8 modtype;
    char devname[VINETIC_DEVNAME_MAXLEN];
    struct gx_gsm_module_data *mod;
    int rc = 0;

    verbose("loading ...\n");

    if (!(gx_cs3_iomem_reg = platform_get_resource(pdev, IORESOURCE_MEM, 0))) {
        rc = -ENOMEM;
        goto gx_driver_probe_error;
    }

    if (!request_mem_region(gx_cs3_iomem_reg->start, resource_size(gx_cs3_iomem_reg), pdev->name)) {
        rc = -EBUSY;
        goto gx_driver_probe_error;
    }

    if (!(gx_cs3_base_ptr = ioremap_nocache(gx_cs3_iomem_reg->start, resource_size(gx_cs3_iomem_reg)))) {
        rc = -ENOMEM;
        goto gx_driver_probe_error;
    }

    for (k = 0; k < MAX_GX_BOARD_COUNT; ++k) {
        gx_boards[k] = NULL;
    }

    /* Read ROM from mainboard */
    iowrite8(0, gx_cs3_base_ptr + MB_RESET_ROM);
    mdelay(1);
    iowrite8(1, gx_cs3_base_ptr + MB_RESET_ROM);
    mdelay(1);
    iowrite8(0, gx_cs3_base_ptr + MB_RESET_ROM);
    for (i = 0; i < sizeof(mainboard_rom); ++i) {
        mainboard_rom[i] = ioread8(gx_cs3_base_ptr + MB_CS_ROM_KROSS);
    }
    verbose("mainboard: \"%s\"\n", &mainboard_rom[2]);
    /* set MainBoard SIM standalone mode */
    iowrite8(1, gx_cs3_base_ptr + MB_MODE_AUTONOM);

    /* Search for gx boards */
    for (k = 0; k < MAX_GX_BOARD_COUNT; ++k) {
        /* alloc memory for board data */
        if (!(brd = kmalloc(sizeof(struct gx_board), GFP_KERNEL))) {
            log(KERN_ERR, "can't get memory for struct gx_board\n");
            rc = -1;
            goto gx_driver_probe_error;
        }
        memset(brd, 0, sizeof(struct gx_board));
        brd->pos = k;
        /* reset gx board */
        iowrite8(0, gx_cs3_base_ptr + GX_RESET_BOARD + (0x0200 * k));
        iowrite8(0, gx_cs3_base_ptr + GX_RESET_BOARD + (0x0200 * (k + 1)));
        mdelay(1);
        iowrite8(1, gx_cs3_base_ptr + GX_RESET_BOARD + (0x0200 * k));
        iowrite8(1, gx_cs3_base_ptr + GX_RESET_BOARD + (0x0200 * (k + 1)));
        /* check gx board for present */
        iowrite8(0x55, gx_cs3_base_ptr + GX_PRESENCE_TEST0 + 0x0200 * k);
        iowrite8(0xaa, gx_cs3_base_ptr + GX_PRESENCE_TEST1 + 0x0200 * k);
        iowrite8(0x5a, gx_cs3_base_ptr + GX_PRESENCE_TEST0 + 0x0200 * (k + 1));
        iowrite8(0xa5, gx_cs3_base_ptr + GX_PRESENCE_TEST1 + 0x0200 * (k + 1));
        if ((ioread8(gx_cs3_base_ptr + GX_PRESENCE_TEST0 + 0x0200 * k) == 0x5a) && (ioread8(gx_cs3_base_ptr + GX_PRESENCE_TEST1 + 0x0200 * k) == 0xa5) &&
            (ioread8(gx_cs3_base_ptr + GX_PRESENCE_TEST0 + 0x0200 * (k + 1)) == 0x5a) && (ioread8(gx_cs3_base_ptr + GX_PRESENCE_TEST1 + 0x0200 * (k + 1)) == 0xa5)) {
            /* board g8 */
            snprintf(brd->name, POLYGATOR_BRDNAME_MAXLEN, "board-g8-cx");
            brd->type = BOARD_TYPE_G8;
            brd->channels_count = 8;
            brd->vinetics_count = 2;
        } else if ((ioread8(gx_cs3_base_ptr + GX_PRESENCE_TEST0 + (0x0200 * k)) == 0x55) &&
            (ioread8(gx_cs3_base_ptr + GX_PRESENCE_TEST1 + (0x0200 * k)) == 0xaa)) {
            /* board g4 */
            if (k == 0) {
                snprintf(brd->name, POLYGATOR_BRDNAME_MAXLEN, "board-g4-ll");
            } else if (k == 1) {
                snprintf(brd->name, POLYGATOR_BRDNAME_MAXLEN, "board-g4-lr");
            } else if (k == 2) {
                snprintf(brd->name, POLYGATOR_BRDNAME_MAXLEN, "board-g4-rl");
            } else if (k == 3) {
                snprintf(brd->name, POLYGATOR_BRDNAME_MAXLEN, "board-g4-rr");
            } else {
                snprintf(brd->name, POLYGATOR_BRDNAME_MAXLEN, "board-g4-cx");
            }
            brd->type = BOARD_TYPE_G4;
            brd->channels_count = 4;
            brd->vinetics_count = 1;
        } else {
            /* board not present */
            kfree(brd);
            continue;
        }
        for (i = 0; i < sizeof(brd->rom); ++i) {
            iowrite8(i, gx_cs3_base_ptr + GX_CS_ROM + (0x0200 * k));
            brd->rom[i] = ioread8(gx_cs3_base_ptr + GX_CS_ROM + (0x0200 * k));
        }
        str = strstr(&brd->rom[2], "ver");
        dp = devname;
        while ((*str) && (*str != 0x20)) {
            *dp++ = *str++;
        }
        *dp = '\0';
        sscanf(devname, "ver.%u.%u", &brd->ver_maj, &brd->ver_min);
        verbose("found %s: firmware version %u.%u\n", brd->name, brd->ver_maj, brd->ver_min);
        if (strstr(&brd->rom[2], "sim16")) {
            brd->sim16 = 1;
        }
        gx_boards[k] = brd;
        /* set autonom */
        iowrite8(1, gx_cs3_base_ptr + GX_MODE_AUTONOM + (0x0200 * k));
        if (brd->type == BOARD_TYPE_G8) {
            iowrite8(1, gx_cs3_base_ptr + GX_MODE_AUTONOM + (0x0200 * (k + 1)));
        }
        /* set sip only */
        iowrite8(0, gx_cs3_base_ptr + GX_SIP_ONLY + (0x0200 * k));
        if (brd->type == BOARD_TYPE_G8) {
            iowrite8(0, gx_cs3_base_ptr + GX_SIP_ONLY + (0x0200 * (k + 1)));
        }
        /* vinetics */
        for (j = 0; j < brd->vinetics_count; ++j) {
            snprintf(devname, VINETIC_DEVNAME_MAXLEN, "%s-vin%lu", brd->name, (unsigned long int)j);
            if (!(brd->vinetics[j] = vinetic_device_register(THIS_MODULE, devname, (0x0200 * (k + j)),
                                                                gx_vinetic_reset,
                                                                gx_vinetic_is_not_ready,
                                                                gx_vinetic_write_nwd,
                                                                gx_vinetic_write_eom,
                                                                gx_vinetic_read_nwd,
                                                                gx_vinetic_read_eom,
                                                                gx_vinetic_read_dia))) {
                rc = -1;
                goto gx_driver_probe_error;
            }
            for (i = 0; i < 4; ++i) {
                snprintf(devname, VINETIC_DEVNAME_MAXLEN, "%s-vin%lu-rtp%lu", brd->name, (unsigned long int)j, (unsigned long int)i);
                if (!(vinetic_rtp_channel_register(THIS_MODULE, devname, brd->vinetics[j], i))) {
                    rc = -1;
                    goto gx_driver_probe_error;
                }
            }
        }
        /* get board GSM module type */
        modtype = ioread8(gx_cs3_base_ptr + GX_CS_PRESENCE + (0x0200 * k));
        /* set AT command channels */
        for (i = 0; i < brd->channels_count; ++i) {
            if (!(mod= kmalloc(sizeof(struct gx_gsm_module_data), GFP_KERNEL))) {
                log(KERN_ERR, "can't get memory for struct gx_gsm_module_data\n");
                rc = -1;
                goto gx_driver_probe_error;
            }
            memset(mod, 0, sizeof(struct gx_gsm_module_data));
            /* select GSM module type */
            switch (modtype) {
                case 4:
                case 5:
                    mod->type = POLYGATOR_MODULE_TYPE_M10;
                    break;
                case 6:
                    mod->type = POLYGATOR_MODULE_TYPE_SIM5215;
                    break;
                case 7:
                    mod->type = POLYGATOR_MODULE_TYPE_SIM900;
                    break;
                default:
                    mod->type = POLYGATOR_MODULE_TYPE_UNKNOWN;
                    verbose("unsupported GSM module type=%u\n", modtype);
                    break;
            }

            if (mod->type == POLYGATOR_MODULE_TYPE_UNKNOWN) {
                kfree(mod);
                continue;
            }

            spin_lock_init(&mod->lock);
            mod->power_on_id = -1;

            mod->control.bits.vbat = 1;
            mod->control.bits.pkey = 1;
            mod->control.bits.cn_speed_a = 0;
            mod->control.bits.cn_speed_b = 0;
            mod->control.bits.at_baudrate = 2;

            mod->pos_on_board = i;
            mod->board = brd;
            mod->cbdata = (((uintptr_t)gx_cs3_base_ptr) + 0x1000 + 0x0200 * (k + (i / 4)) + 0x40 * (i % 4));
            mod->set_control = gx_gsm_mod_set_control;
            mod->get_status = gx_gsm_mod_get_status;
            mod->at_write = gx_gsm_mod_at_write;
            mod->at_read = gx_gsm_mod_at_read;
            mod->at_write_sim16 = gx_gsm_mod_at_write_sim16;
            mod->at_read_sim16 = gx_gsm_mod_at_read_sim16;
            mod->sim_write = gx_gsm_mod_sim_write;
            mod->sim_read = gx_gsm_mod_sim_read;
            mod->sim_do_after_reset = gx_gsm_mod_sim_do_after_reset;
            mod->imei_write = gx_gsm_mod_imei_write;
            mod->imei_read = gx_gsm_mod_imei_read;

            mod->set_control(mod->cbdata, mod->pos_on_board, mod->control.full);
            timer_setup(&mod->at_poll_timer, gx_tty_at_poll, 0);

            gx_gsm_mod_init_counters(mod->cbdata, mod->pos_on_board);

            spin_lock_init(&mod->at_lock);
#ifdef TTY_PORT
            tty_port_init(&mod->at_port);
            mod->at_port.ops = &gx_tty_at_port_ops;
            mod->at_port.close_delay = 0;
            mod->at_port.closing_wait = ASYNC_CLOSING_WAIT_NONE;
#endif
            brd->gsm_modules[i] = mod;
        }
        /* register polygator tty at device */
        for (i = 0; i < brd->channels_count; ++i) {
            if ((mod = brd->gsm_modules[i])) {
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,7,0)
                if (!(brd->tty_at_channels[i] = polygator_tty_device_register(&pdev->dev, brd->gsm_modules[i], &mod->at_port, &gx_tty_at_ops))) {
#else
                if (!(brd->tty_at_channels[i] = polygator_tty_device_register(&pdev->dev, brd->gsm_modules[i], &gx_tty_at_ops))) {
#endif
                    log(KERN_ERR, "can't register polygator tty device\n");
                    rc = -1;
                    goto gx_driver_probe_error;
                }
            }
        }
        /* register polygator simcard device */
        for (i = 0; i < brd->channels_count; ++i) {
            if (brd->gsm_modules[i]) {
                if (!(brd->simcard_channels[i] = simcard_device_register(THIS_MODULE,
                                                                        brd->gsm_modules[i],
                                                                        gx_sim_read,
                                                                        gx_sim_write,
                                                                        gx_sim_is_read_ready,
                                                                        gx_sim_is_write_ready,
                                                                        gx_sim_is_reset_request,
                                                                        gx_sim_set_speed,
                                                                        gx_sim_do_after_reset))) {
                    log(KERN_ERR, "can't register polygator simcard device\n");
                    rc = -1;
                    goto gx_driver_probe_error;
                }
            }
        }
    }
    /* change board cyclic position 4 -> 2, 2 -> 3, 3 -> 4 */
    brd = gx_boards[4];
    gx_boards[4] = gx_boards[3];
    gx_boards[3] = gx_boards[2];
    gx_boards[2] = brd;
    /* register board */
    for (k = 0; k < MAX_GX_BOARD_COUNT; ++k) {
        if ((brd = gx_boards[k])) {
            if (!(brd->pg_board =  polygator_board_register(&pdev->dev, THIS_MODULE, brd->name, &brd->cdev, &gx_board_fops))) {
                rc = -1;
                goto gx_driver_probe_error;
            }
        }
    }
    verbose("loaded successfull\n");
    return rc;

    return 0;

gx_driver_probe_error:
    /* boards */
    for (k = 0; k < MAX_GX_BOARD_COUNT; ++k) {
        if ((brd = gx_boards[k])) {
            /* channels */
            for (i = 0; i < brd->channels_count; ++i) {
                if (brd->simcard_channels[i]) {
                    simcard_device_unregister(brd->simcard_channels[i]);
                }
                if (brd->tty_at_channels[i]) {
                    polygator_tty_device_unregister(brd->tty_at_channels[i]);
                }
                if (brd->gsm_modules[i]) {
                    del_timer_sync(&brd->gsm_modules[i]->at_poll_timer);
                    kfree(brd->gsm_modules[i]);
                }
            }
            /* vinetics */
            for (j = 0; j < brd->vinetics_count; ++j) {
                if ((vin = brd->vinetics[j])) {
                    /* rtp_channels */
                    for (i = 0; i < 4; ++i) {
                        vinetic_rtp_channel_unregister(vin->rtp_channels[i]);
                    }
                    vinetic_device_unregister(vin);
                }
            }
            if (brd->pg_board) {
                polygator_board_unregister(brd->pg_board);
            }
            kfree(gx_boards[k]);
        }
    }

    if (gx_cs3_base_ptr) {
        iounmap(gx_cs3_base_ptr);
    }

    if (gx_cs3_iomem_reg) {
        release_mem_region(gx_cs3_iomem_reg->start, resource_size(gx_cs3_iomem_reg));
    }

    return rc;
}

static int gx_driver_remove(struct platform_device *pdev)
{
    size_t i, j, k;
    struct gx_board *brd;
    struct vinetic *vin;

    /* boards */
    for (k = 0; k < MAX_GX_BOARD_COUNT; ++k) {
        if ((brd = gx_boards[k])) {
            /* channels */
            for (i = 0; i < brd->channels_count; ++i) {
                if (brd->simcard_channels[i]) {
                    simcard_device_unregister(brd->simcard_channels[i]);
                }
                if (brd->tty_at_channels[i]) {
                    polygator_tty_device_unregister(brd->tty_at_channels[i]);
                }
                if (brd->gsm_modules[i]) {
                    del_timer_sync(&brd->gsm_modules[i]->at_poll_timer);
                    kfree(brd->gsm_modules[i]);
                }
            }
            /* vinetics */
            for (j = 0; j < brd->vinetics_count; ++j) {
                if ((vin = brd->vinetics[j])) {
                    /* rtp_channels */
                    for (i = 0; i < 4; ++i) {
                        vinetic_rtp_channel_unregister(vin->rtp_channels[i]);
                    }
                    vinetic_device_unregister(vin);
                }
            }
            polygator_board_unregister(brd->pg_board);
            kfree(brd);
        }
    }

    iounmap(gx_cs3_base_ptr);

    release_mem_region(gx_cs3_iomem_reg->start, resource_size(gx_cs3_iomem_reg));

    verbose("remove\n");

    return 0;
}

static struct platform_driver gx_driver = {
    .driver = {
        .name = "polygator-gx",
        .of_match_table = of_match_ptr(gx_of_ids),
    },
    .probe = gx_driver_probe,
    .remove = gx_driver_remove,
};
module_platform_driver(gx_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maksym Tarasevych <mxmtar@gmail.com>");
MODULE_DESCRIPTION("Polygator Linux module for GX devices");
MODULE_ALIAS("platform:polygator-gx");
