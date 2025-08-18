#define pr_fmt(fmt) "%s:%s: " fmt, KBUILD_MODNAME, __func__

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/cdev.h>
#include <linux/dma-mapping.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/mm.h>
#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include "edu.h"

// See https://github.com/qemu/qemu/blob/stable-7.2/docs/specs/edu.txt
#define PCI_VENDOR_ID_QEMU     0x1234
#define PCI_DEVICE_ID_QEMU_EDU 0x11e8

// The number of bits be changed in QEMU via '-device edu,dma_mask=<mask>'
#define EDU_DMA_BITS 28
#define EDU_DMA_BUF_DEVICE_OFFSET 0x40000
#define EDU_DMA_CMD_START_XFER    1
#define EDU_DMA_CMD_RAM_TO_DEVICE 0
#define EDU_DMA_CMD_DEVICE_TO_RAM 2
#define EDU_DMA_CMD_RAISE_IRQ     4
#define EDU_STATUS_COMPUTING 0x01
#define EDU_STATUS_RAISE_IRQ 0x80

#define EDU_ADDR_IDENT      0x0
#define EDU_ADDR_LIVENESS   0x04
#define EDU_ADDR_FACTORIAL  0x08
#define EDU_ADDR_STATUS     0x20
#define EDU_ADDR_IRQ_STATUS 0x24
#define EDU_ADDR_IRQ_RAISE  0x60
#define EDU_ADDR_IRQ_ACK    0x64
#define EDU_ADDR_DMA_SRC    0x80
#define EDU_ADDR_DMA_DST    0x88
#define EDU_ADDR_DMA_XFER   0x90
#define EDU_ADDR_DMA_CMD    0x98

static const struct pci_device_id edu_pci_tbl[] = {
    { PCI_DEVICE(PCI_VENDOR_ID_QEMU, PCI_DEVICE_ID_QEMU_EDU) },
    { }
};

// e.g. insmod edu.ko debug=1
// e.g. echo 1 > /sys/module/edu/parameters/debug
static bool param_debug;
module_param_named(debug, param_debug, bool, S_IRUGO | S_IWUSR);
#define edu_log(...) if (param_debug) pr_info(__VA_ARGS__)

// Load with msi=1 to use MSI instead of INTx
static bool param_msi;
module_param_named(msi, param_msi, bool, S_IRUGO);

struct edu_device {
    bool registered_irq_handler;
    bool added_cdev;
    struct cdev cdev;
    char __iomem *iomem;
    unsigned int irq;
    u32 irq_value;
    wait_queue_head_t irq_wait_queue;
    dma_addr_t dma_bus_addr;
    void *dma_virt_addr;
};

static dev_t devno;
static const int minor = 0;
static struct edu_device *edu_dev;

static int edu_open(struct inode *inode, struct file *filp) {
    struct edu_device *dev;

    nonseekable_open(inode, filp);
    dev = container_of(inode->i_cdev, struct edu_device, cdev);
    filp->private_data = dev;
    return 0;
}

static int edu_release(struct inode *inode, struct file *filp) {
    return 0;
}

static int edu_mmap(struct file *filp, struct vm_area_struct *vma) {
    struct edu_device *dev = filp->private_data;
    unsigned long len = vma->vm_end - vma->vm_start;

    if (len > EDU_DMA_BUF_SIZE) {
        return -EINVAL;
    }
    if (vma->vm_pgoff) {
        return -EINVAL;
    }
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    return vm_iomap_memory(vma, __pa(dev->dma_virt_addr), len);
}

static int ioctl_ident(struct edu_device *dev, u32 __user *arg) {
    u32 val = readl(dev->iomem + EDU_ADDR_IDENT);
    return put_user(val, arg);
}

static int ioctl_liveness(struct edu_device *dev, u32 __user *arg) {
    u32 val;
    if (get_user(val, arg)) {
        return -EFAULT;
    }
    writel(val, dev->iomem + EDU_ADDR_LIVENESS);
    val = readl(dev->iomem + EDU_ADDR_LIVENESS);
    return put_user(val, arg);
}

static bool is_computing_factorial(struct edu_device *dev) {
    return readl(dev->iomem + EDU_ADDR_STATUS) & EDU_STATUS_COMPUTING;
}

static int ioctl_factorial(struct edu_device *dev, u32 __user *arg) {
    u32 val;
    if (get_user(val, arg)) {
        return -EFAULT;
    }
    writel(EDU_STATUS_RAISE_IRQ, dev->iomem + EDU_ADDR_STATUS);
    edu_log("Writing %u to register\n", val);
    writel(val, dev->iomem + EDU_ADDR_FACTORIAL);
    if (wait_event_interruptible(dev->irq_wait_queue, !is_computing_factorial(dev))) {
        return -ERESTARTSYS;
    }
    val = readl(dev->iomem + EDU_ADDR_FACTORIAL);
    edu_log("Got factorial result: %u\n", val);
    return put_user(val, arg);
}

static int ioctl_wait_irq(struct edu_device *dev, u32 __user *arg) {
    DEFINE_WAIT(wait);
    prepare_to_wait(&dev->irq_wait_queue, &wait, TASK_INTERRUPTIBLE);
    schedule();
    finish_wait(&dev->irq_wait_queue, &wait);
    if (signal_pending(current)) {
        return -ERESTARTSYS;
    }
    return put_user(READ_ONCE(dev->irq_value), arg);
}

static int ioctl_raise_irq(struct edu_device *dev, u32 arg) {
    writel(arg, dev->iomem + EDU_ADDR_IRQ_RAISE);
    return 0;
}

static bool is_doing_dma(struct edu_device *dev) {
    return readl(dev->iomem + EDU_ADDR_DMA_CMD) & EDU_DMA_CMD_START_XFER;
}

static int do_dma(struct edu_device *dev, u32 len, bool to_device) {
    u32 src, dst, cmd;
    if (len == 0 || len > EDU_DMA_BUF_SIZE) {
        return -EINVAL;
    }
    if (dev->dma_bus_addr > ~(u32)0) {
        pr_warn("DMA bus addr is greater than 32 bits, cannot use writel\n");
        return -EOPNOTSUPP;
    }
    if (to_device) {
        src = (u32)dev->dma_bus_addr;
        dst = EDU_DMA_BUF_DEVICE_OFFSET;
        cmd = EDU_DMA_CMD_START_XFER | EDU_DMA_CMD_RAM_TO_DEVICE | EDU_DMA_CMD_RAISE_IRQ;
    } else {
        src = EDU_DMA_BUF_DEVICE_OFFSET;
        dst = (u32)dev->dma_bus_addr;
        cmd = EDU_DMA_CMD_START_XFER | EDU_DMA_CMD_DEVICE_TO_RAM | EDU_DMA_CMD_RAISE_IRQ;
    }
    edu_log("src=0x%08x dst=0x%08x len=%u\n", src, dst, len);
    writel(src, dev->iomem + EDU_ADDR_DMA_SRC);
    writel(dst, dev->iomem + EDU_ADDR_DMA_DST);
    writel(len, dev->iomem + EDU_ADDR_DMA_XFER);
    writel(cmd, dev->iomem + EDU_ADDR_DMA_CMD);
    if (wait_event_interruptible(dev->irq_wait_queue, !is_doing_dma(dev))) {
        return -ERESTARTSYS;
    }
    return 0;
}

static int ioctl_dma_to_device(struct edu_device *dev, u32 arg) {
    return do_dma(dev, arg, true);
}

static int ioctl_dma_from_device(struct edu_device *dev, u32 arg) {
    return do_dma(dev, arg, false);
}

static long edu_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    struct edu_device *dev = filp->private_data;
    switch (cmd) {
        case EDU_IOCTL_IDENT:
            return ioctl_ident(dev, (u32 __user*)arg);
        case EDU_IOCTL_LIVENESS:
            return ioctl_liveness(dev, (u32 __user*)arg);
        case EDU_IOCTL_FACTORIAL:
            return ioctl_factorial(dev, (u32 __user*)arg);
        case EDU_IOCTL_WAIT_IRQ:
            return ioctl_wait_irq(dev, (u32 __user*)arg);
        case EDU_IOCTL_RAISE_IRQ:
            return ioctl_raise_irq(dev, (u32)arg);
        case EDU_IOCTL_DMA_TO_DEVICE:
            return ioctl_dma_to_device(dev, (u32)arg);
        case EDU_IOCTL_DMA_FROM_DEVICE:
            return ioctl_dma_from_device(dev, (u32)arg);
        default:
            return -ENOTTY;
    }
}

struct file_operations edu_fops = {
    .owner = THIS_MODULE,
    .open = edu_open,
    .release = edu_release,
    .unlocked_ioctl = edu_ioctl,
    .mmap = edu_mmap,
};

static void edu_dev_init(struct edu_device *dev) {
    cdev_init(&dev->cdev, &edu_fops);
    dev->cdev.owner = THIS_MODULE;
    init_waitqueue_head(&dev->irq_wait_queue);
}

static irqreturn_t edu_irq_handler(int irq, void *dev_id) {
    struct edu_device *dev = dev_id;
    u32 irq_value;

    irq_value = readl(dev->iomem + EDU_ADDR_IRQ_STATUS);
    edu_log("irq_value = %u\n", irq_value);
    writel(irq_value, dev->iomem + EDU_ADDR_IRQ_ACK);
    WRITE_ONCE(dev->irq_value, irq_value);
    wake_up_interruptible(&dev->irq_wait_queue);
    return IRQ_HANDLED;
}

static void edu_pci_cleanup(struct pci_dev *pdev) {
    if (!edu_dev) {
        return;
    }
    if (edu_dev->added_cdev) {
        cdev_del(&edu_dev->cdev);
        edu_dev->added_cdev = false;
    }

