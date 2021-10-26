/* Copyright(C) 2019 Hex Five Security, Inc. */

#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/semaphore.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/tty_flip.h>
#include <linux/tty.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define VALID_BIT 63
#define LOCK_BIT 0
#define MAX_ZONES 4
#define TRANSFER_SIZE 4

static struct of_device_id multizone_match_table[] = { { .compatible =
        "hexfive,multizone", }, { } };
MODULE_DEVICE_TABLE(of, multizone_match_table);

struct mbox_entry {
    unsigned long valid;
    u32 msg[4];
};

struct multizone_zone;

struct multizone {
    unsigned int irq;
    unsigned int n_zones;
    struct mbox_entry *send;
    struct mbox_entry *recv;
    struct multizone_zone *zones[MAX_ZONES + 1];
};

struct multizone_zone {
    struct uart_port port;
    spinlock_t lock;
    int kicked;
    struct multizone *m;
    struct task_struct *zone_thread;
    bool irq_registered;
    unsigned int zone;
};

static void multizone_serial_stop_tx(struct uart_port *port);

static void multizone_serial_transmit_chars(struct multizone_zone *mz) {
    struct multizone *m = mz->m;
    volatile struct mbox_entry *send_entry = &m->send[mz->zone - 1];
    volatile char *send_buf = (char*) (&send_entry->msg[0]);
    struct circ_buf *xmit = &mz->port.state->xmit;
    int i;
    unsigned long flags;
    volatile unsigned long lock = 0x01;

    spin_lock_irqsave(&mz->lock, flags);

    /* Atomic: Aquire lock and update send_entry->valid */
    asm volatile("amoswap.d.aq %0, %1, (%2)" : "=r" (lock) : "r" (lock), "r" (&send_entry->valid) : "memory");

    if (!test_bit(LOCK_BIT, &lock)) {

        if (!test_bit(VALID_BIT, &lock)) {
            if (uart_circ_empty(xmit) || uart_tx_stopped(&mz->port)) {
                multizone_serial_stop_tx(&mz->port);
                /* Atomic: Release lock and update send_entry->valid */
                asm volatile("amoswap.d.rl %0, %1, (%2)" : "=r" (lock) : "r" (lock), "r" (&send_entry->valid) : "memory");
                spin_unlock_irqrestore(&mz->lock, flags);
                return;
            }
            for (i = 0;
                    i < TRANSFER_SIZE && i < mz->port.fifosize
                            && !uart_circ_empty(xmit); i++) {
                send_buf[i] = xmit->buf[xmit->tail];
                xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
                mz->port.icount.tx++;
            }
            for (; i < TRANSFER_SIZE; i++) {
                send_buf[i] = 0;
            }
            set_bit(VALID_BIT, &lock);
        }

        /* Atomic: Release lock and update send_entry->valid */
        asm volatile("amoswap.d.rl %0, %1, (%2)" : "=r" (lock) : "r" (lock), "r" (&send_entry->valid) : "memory");

        if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
            uart_write_wakeup(&mz->port);

        if (uart_circ_empty(xmit))
            multizone_serial_stop_tx(&mz->port);
    }

    spin_unlock_irqrestore(&mz->lock, flags);
}

void multizone_check_buffers(struct multizone_zone *mz) {
    struct multizone *m = mz->m;
    volatile struct mbox_entry *recv_entry = &m->recv[mz->zone - 1];
    struct uart_port *port = &mz->port;
    int c;
    volatile char *buf = (char*) &recv_entry->msg[0];
    unsigned long flags;
    volatile unsigned long lock = 0x01;

    spin_lock_irqsave(&mz->lock, flags);

    /* Atomic: Aquire lock and update recv_entry->valid */
    asm volatile("amoswap.d.aq %0, %1, (%2)" : "=r" (lock) : "r" (lock), "r" (&recv_entry->valid) : "memory");

    if (!test_bit(LOCK_BIT, &lock)) {

        if (test_bit(VALID_BIT, &lock)) {
            for (c = 0; buf[c] != 0 && c < TRANSFER_SIZE; c++) {
                port->icount.rx++;
                uart_insert_char(port, 0, 0, buf[c], TTY_NORMAL);
            }
            tty_flip_buffer_push(&port->state->port);
            clear_bit(VALID_BIT, &lock);
        }
        /* Atomic: Release lock and update recv_entry->valid */
        asm volatile("amoswap.d.rl %0, %1, (%2)" : "=r" (lock) : "r" (lock), "r" (&recv_entry->valid) : "memory");
    }

    spin_unlock_irqrestore(&mz->lock, flags);

    multizone_serial_transmit_chars(mz);
}

irqreturn_t multizone_irq(int irq, void *data) {
    struct multizone *m = (struct multizone*) data;
    int i;

    for (i = 1; i <= m->n_zones; i++) {
        struct multizone_zone *mz = m->zones[i];

        mz->kicked = 1;

        if (mz->zone_thread) {
            wake_up_process(mz->zone_thread);
        }
    }

    return IRQ_HANDLED;
}

static int zone_thread(void *data) {
    struct multizone_zone *mz = (struct multizone_zone*) data;
    while (!kthread_should_stop()) {
        mz->kicked = 0;
        multizone_check_buffers(mz);
        if (mz->kicked)
            continue;

        schedule_timeout_interruptible(1);
    }

    return 0;
}

static unsigned int multizone_serial_tx_empty(struct uart_port *port) {
    return 0;
}

static void multizone_serial_set_mctrl(struct uart_port *port,
        unsigned int mctrl) {

}

static unsigned int multizone_serial_get_mctrl(struct uart_port *port) {
    return 0;
}

static void multizone_serial_stop_tx(struct uart_port *port) {

}

static void multizone_serial_start_tx(struct uart_port *port) {
    struct multizone_zone
    *mz = container_of(port, struct multizone_zone, port);

    multizone_serial_transmit_chars (mz);
}

static void multizone_serial_stop_rx(struct uart_port *port) {

}

static void multizone_serial_enable_ms(struct uart_port *port) {

}

static void multizone_serial_break_ctl(struct uart_port *port,
        int break_state) {

}

static int multizone_serial_startup(struct uart_port *port) {
    struct multizone_zone
    *mz = container_of(port, struct multizone_zone, port);
    mz->zone_thread = kthread_run(zone_thread, mz, "zone%d", mz->zone);
    return 0;
}

static void multizone_serial_shutdown(struct uart_port *port) {
    struct multizone_zone
    *mz = container_of(port, struct multizone_zone, port);
    kthread_stop(mz->zone_thread);
}

static void multizone_serial_set_termios(struct uart_port *port, struct ktermios *new, struct ktermios *old){
    int rate;

    rate = uart_get_baud_rate(port, new, old, 0, 115200);
    uart_update_timeout(port, new->c_cflag, rate);
}

static const char* multizone_serial_type(struct uart_port *port) {
    return "multizone";
}

static void multizone_serial_release_port(struct uart_port *port) {

}

static int multizone_serial_request_port(struct uart_port *port) {
    return 0;
}

static void multizone_serial_config_port(struct uart_port *port, int flags) {

}

static int multizone_serial_verify_port(struct uart_port *port,
        struct serial_struct *ser) {
    return 0;
}

static struct uart_ops multizone_serial_ops = { .tx_empty =
        multizone_serial_tx_empty, .set_mctrl = multizone_serial_set_mctrl,
        .get_mctrl = multizone_serial_get_mctrl, .stop_tx =
                multizone_serial_stop_tx, .start_tx = multizone_serial_start_tx,
        .stop_rx = multizone_serial_stop_rx, .enable_ms =
                multizone_serial_enable_ms, .break_ctl =
                multizone_serial_break_ctl, .startup = multizone_serial_startup,
        .shutdown = multizone_serial_shutdown, .set_termios =
                multizone_serial_set_termios, .type = multizone_serial_type,
        .release_port = multizone_serial_release_port, .request_port =
                multizone_serial_request_port, .config_port =
                multizone_serial_config_port, .verify_port =
                multizone_serial_verify_port, };

static struct uart_driver multizone_serial_driver = { .owner = THIS_MODULE,
        .driver_name = "multizone", .dev_name = "multizone", .minor = 0, .nr =
        MAX_ZONES + 1, };

static int multizone_probe(struct platform_device *pdev) {
    struct multizone *m;
    struct resource *send_mem = &pdev->resource[0];
    struct resource *recv_mem = &pdev->resource[1];
    int i, ret = 0;

    m = devm_kzalloc(&pdev->dev, sizeof(struct multizone), GFP_KERNEL);
    if (!m) {
        ret = -ENOMEM;
        goto fail;
    }

    platform_set_drvdata(pdev, m);

    ret = of_property_read_u32(pdev->dev.of_node, "zones", &m->n_zones);
    if (ret < 0) {
        dev_warn(&pdev->dev,
                "MultiZone #zones not specified, defaulting to 4\n");
        m->n_zones = 4;
    }

    m->send = devm_memremap(&pdev->dev, send_mem->start,
            resource_size(send_mem), MEMREMAP_WT);
    m->recv = devm_memremap(&pdev->dev, recv_mem->start,
            resource_size(recv_mem), MEMREMAP_WT);
    if (!m->send || !m->recv) {
        ret = -ENOMEM;
        goto fail;
    }

    for (i = 1; i <= m->n_zones; i++) {
        struct multizone_zone *mz;

        mz = devm_kzalloc(&pdev->dev, sizeof(struct multizone_zone),
                GFP_KERNEL);
        if (!mz)
            return -ENOMEM;

        m->zones[i] = mz;
        mz->m = m;
        mz->zone = i;
        mz->port.dev = &pdev->dev;
        mz->port.fifosize = TRANSFER_SIZE;
        mz->port.line = i;
        mz->port.ops = &multizone_serial_ops;
        mz->port.type = PORT_8250;
        spin_lock_init(&mz->lock);

        ret = uart_add_one_port(&multizone_serial_driver, &mz->port);
        if (ret != 0) {
            dev_err(&pdev->dev, "Zone %d, could not add UART port, error %d\n",
                    i, ret);
        }
    }

    m->irq = 16;
    ret = request_irq(m->irq, multizone_irq, IRQF_SHARED, dev_name(&pdev->dev),
            m);
    if (ret) {
        m->irq = 0;
        dev_err(&pdev->dev, "Could not register interrupts, error %d\n", ret);
    }

    dev_info(&pdev->dev,
            "MultiZone loaded, zones = %d, send buffer @ %llx, recv buffer @ %llx\n",
            m->n_zones, send_mem->start, recv_mem->start);

    return 0;

    fail: return ret;
}

static int multizone_remove(struct platform_device *pdev) {
    struct multizone *m = (struct multizone*) platform_get_drvdata(pdev);
    int i;

    if (m->irq)
        free_irq(m->irq, m);

    for (i = 1; i <= m->n_zones; i++) {
        uart_remove_one_port(&multizone_serial_driver, &m->zones[i]->port);
    }

    dev_info(&pdev->dev, "MultiZone unloaded!\n");
    return 0;
}

static struct platform_driver multizone_platform_driver = { .probe =
        multizone_probe, .remove = multizone_remove, .driver = { .owner =
        THIS_MODULE, .name = "multizone_driver", .of_match_table = of_match_ptr(
        multizone_match_table), }, };

int __init multizone_init(void)
{
    struct tty_driver *tty_drv;
    int ret;

    ret = uart_register_driver(&multizone_serial_driver);
    if (ret) goto fail;

    tty_drv = multizone_serial_driver.tty_driver;

    tty_drv->init_termios = tty_std_termios;
    tty_drv->init_termios.c_iflag = 0;
    tty_drv->init_termios.c_oflag = 0;
    tty_drv->init_termios.c_lflag = 0;
    tty_drv->flags = TTY_DRIVER_REAL_RAW;

    ret = platform_driver_register(&multizone_platform_driver);
    if (ret) goto fail_platform;

    return 0;

    fail_platform:
    uart_unregister_driver(&multizone_serial_driver);
    fail:
    return ret;
}

void __exit multizone_exit(void)
{
    platform_driver_unregister(&multizone_platform_driver);
    uart_unregister_driver(&multizone_serial_driver);
}

module_init (multizone_init);
module_exit (multizone_exit);

MODULE_DESCRIPTION("MultiZone Linux Driver");
MODULE_LICENSE("ISC");
MODULE_AUTHOR("Hex Five Security, Inc. <info@hex-five.com>");
