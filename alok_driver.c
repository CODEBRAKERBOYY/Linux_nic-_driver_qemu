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
#include "alok.h"

#define PCI_VENDOR_ID_QEMU     0x1234
#define PCI_DEVICE_ID_QEMU_ALOK 0x11e8

#define ALOK_DMA_BITS 28
#define ALOK_DMA_BUF_DEVICE_OFFSET 0x40000
#define ALOK_DMA_CMD_START_XFER    1
#define ALOK_DMA_CMD_RAM_TO_DEVICE 0
#define ALOK_DMA_CMD_DEVICE_TO_RAM 2
#define ALOK_DMA_CMD_RAISE_IRQ     4
#define ALOK_STATUS_COMPUTING 0x01
#define ALOK_STATUS_RAISE_IRQ 0x80

#define ALOK_ADDR_IDENT      0x0
#define ALOK_ADDR_LIVENESS   0x04
#define ALOK_ADDR_FACTORIAL  0x08
#define ALOK_ADDR_STATUS     0x20
#define ALOK_ADDR_IRQ_STATUS 0x24
#define ALOK_ADDR_IRQ_RAISE  0x60
#define ALOK_ADDR_IRQ_ACK    0x64
#define ALOK_ADDR_DMA_SRC    0x80
#define ALOK_ADDR_DMA_DST    0x88
#define ALOK_ADDR_DMA_XFER   0x90
#define ALOK_ADDR_DMA_CMD    0x98

static const struct pci_device_id alok_pci_tbl[] = {
    { PCI_DEVICE(PCI_VENDOR_ID_QEMU, PCI_DEVICE_ID_QEMU_ALOK) },
    { }
};

// e.g. insmod alok_driver.ko debug=1

static bool param_debug;
module_param_named(debug, param_debug, bool, S_IRUGO | S_IWUSR);
#define alok_log(...) if (param_debug) pr_info(__VA_ARGS__)

// Load with msi=1 to use MSI instead of INTx
static bool param_msi;
module_param_named(msi, param_msi, bool, S_IRUGO);

struct alok_device {
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
static struct alok_device *alok_dev;

static int alok_open(struct inode *inode, struct file *filp) {
    struct alok_device *dev;

    nonseekable_open(inode, filp);
    dev = container_of(inode->i_cdev, struct alok_device, cdev);
    filp->private_data = dev;
    return 0;
}

static int alok_release(struct inode *inode, struct file *filp) {
    return 0;
}

static int alok_mmap(struct file *filp, struct vm_area_struct *vma) {
    struct alok_device *dev = filp->private_data;
    unsigned long len = vma->vm_end - vma->vm_start;

    if (len > ALOK_DMA_BUF_SIZE) {
        return -EINVAL;
    }
    if (vma->vm_pgoff) {
        return -EINVAL;
    }
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);
    return vm_iomap_memory(vma, __pa(dev->dma_virt_addr), len);
}

static int ioctl_ident(struct alok_device *dev, u32 __user *arg) {
    u32 val = readl(dev->iomem + ALOK_ADDR_IDENT);
    return put_user(val, arg);
}

static int ioctl_liveness(struct alok_device *dev, u32 __user *arg) {
    u32 val;
    if (get_user(val, arg)) {
        return -EFAULT;
    }
    writel(val, dev->iomem + ALOK_ADDR_LIVENESS);
    val = readl(dev->iomem + ALOK_ADDR_LIVENESS);
    return put_user(val, arg);
}

static bool is_computing_factorial(struct alok_device *dev) {
    return readl(dev->iomem + ALOK_ADDR_STATUS) & ALOK_STATUS_COMPUTING;
}

static int ioctl_factorial(struct alok_device *dev, u32 __user *arg) {
    u32 val;
    if (get_user(val, arg)) {
        return -EFAULT;
    }
    writel(ALOK_STATUS_RAISE_IRQ, dev->iomem + ALOK_ADDR_STATUS);
    alok_log("Writing %u to register\n", val);
    writel(val, dev->iomem + ALOK_ADDR_FACTORIAL);
    if (wait_event_interruptible(dev->irq_wait_queue, !is_computing_factorial(dev))) {
        return -ERESTARTSYS;
    }
    val = readl(dev->iomem + ALOK_ADDR_FACTORIAL);
    alok_log("Got factorial result: %u\n", val);
    return put_user(val, arg);
}

static int ioctl_wait_irq(struct alok_device *dev, u32 __user *arg) {
    DEFINE_WAIT(wait);
    prepare_to_wait(&dev->irq_wait_queue, &wait, TASK_INTERRUPTIBLE);
    schedule();
    finish_wait(&dev->irq_wait_queue, &wait);
    if (signal_pending(current)) {
        return -ERESTARTSYS;
    }
    return put_user(READ_ONCE(dev->irq_value), arg);
}

static int ioctl_raise_irq(struct alok_device *dev, u32 arg) {
    writel(arg, dev->iomem + ALOK_ADDR_IRQ_RAISE);
    return 0;
}

static bool is_doing_dma(struct alok_device *dev) {
    return readl(dev->iomem + ALOK_ADDR_DMA_CMD) & ALOK_DMA_CMD_START_XFER;
}

static int do_dma(struct alok_device *dev, u32 len, bool to_device) {
    u32 src, dst, cmd;
    if (len == 0 || len > ALOK_DMA_BUF_SIZE) {
        return -EINVAL;
    }
    if (dev->dma_bus_addr > ~(u32)0) {
        pr_warn("DMA bus addr is greater than 32 bits, cannot use writel\n");
        return -EOPNOTSUPP;
    }
    if (to_device) {
        src = (u32)dev->dma_bus_addr;
        dst = ALOK_DMA_BUF_DEVICE_OFFSET;
        cmd = ALOK_DMA_CMD_START_XFER | ALOK_DMA_CMD_RAM_TO_DEVICE | ALOK_DMA_CMD_RAISE_IRQ;
    } else {
        src = ALOK_DMA_BUF_DEVICE_OFFSET;
        dst = (u32)dev->dma_bus_addr;
        cmd = ALOK_DMA_CMD_START_XFER | ALOK_DMA_CMD_DEVICE_TO_RAM | ALOK_DMA_CMD_RAISE_IRQ;
    }
    alok_log("src=0x%08x dst=0x%08x len=%u\n", src, dst, len);
    writel(src, dev->iomem + ALOK_ADDR_DMA_SRC);
    writel(dst, dev->iomem + ALOK_ADDR_DMA_DST);
    writel(len, dev->iomem + ALOK_ADDR_DMA_XFER);
    writel(cmd, dev->iomem + ALOK_ADDR_DMA_CMD);
    if (wait_event_interruptible(dev->irq_wait_queue, !is_doing_dma(dev))) {
        return -ERESTARTSYS;
    }
    return 0;
}

static int ioctl_dma_to_device(struct alok_device *dev, u32 arg) {
    return do_dma(dev, arg, true);
}

static int ioctl_dma_from_device(struct alok_device *dev, u32 arg) {
    return do_dma(dev, arg, false);
}

static long alok_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
    struct alok_device *dev = filp->private_data;
    switch (cmd) {
        case ALOK_IOCTL_IDENT:
            return ioctl_ident(dev, (u32 __user*)arg);
        case ALOK_IOCTL_LIVENESS:
            return ioctl_liveness(dev, (u32 __user*)arg);
        case ALOK_IOCTL_FACTORIAL:
            return ioctl_factorial(dev, (u32 __user*)arg);
        case ALOK_IOCTL_WAIT_IRQ:
            return ioctl_wait_irq(dev, (u32 __user*)arg);
        case ALOK_IOCTL_RAISE_IRQ:
            return ioctl_raise_irq(dev, (u32)arg);
        case ALOK_IOCTL_DMA_TO_DEVICE:
            return ioctl_dma_to_device(dev, (u32)arg);
        case ALOK_IOCTL_DMA_FROM_DEVICE:
            return ioctl_dma_from_device(dev, (u32)arg);
        default:
            return -ENOTTY;
    }
}

struct file_operations alok_fops = {
    .owner = THIS_MODULE,
    .open = alok_open,
    .release = alok_release,
    .unlocked_ioctl = alok_ioctl,
    .mmap = alok_mmap,
};

static void alok_dev_init(struct alok_device *dev) {
    cdev_init(&dev->cdev, &alok_fops);
    dev->cdev.owner = THIS_MODULE;
    init_waitqueue_head(&dev->irq_wait_queue);
}

static irqreturn_t alok_irq_handler(int irq, void *dev_id) {
    struct alok_device *dev = dev_id;
    u32 irq_value;

    irq_value = readl(dev->iomem + ALOK_ADDR_IRQ_STATUS);
    alok_log("irq_value = %u\n", irq_value);
    writel(irq_value, dev->iomem + ALOK_ADDR_IRQ_ACK);
    WRITE_ONCE(dev->irq_value, irq_value);
    wake_up_interruptible(&dev->irq_wait_queue);
    return IRQ_HANDLED;
}

static void alok_pci_cleanup(struct pci_dev *pdev) {
    if (!alok_dev) {
        return;
    }
    if (alok_dev->added_cdev) {
        cdev_del(&alok_dev->cdev);
        alok_dev->added_cdev = false;
    }
    if (alok_dev->registered_irq_handler) {
        free_irq(alok_dev->irq, alok_dev);
        alok_dev->registered_irq_handler = false;
    }
    if (pci_dev_msi_enabled(pdev)) {
        pci_free_irq_vectors(pdev);
    }
}

static int alok_pci_probe(struct pci_dev *pdev, const struct pci_device_id *ent) {
    int err;
    int nvec;

    err = pcim_enable_device(pdev);
    if (err) {
        goto fail;
    }

    pci_set_master(pdev);
    err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(ALOK_DMA_BITS));
    if (err) {
        goto fail;
    }

    alok_dev->dma_virt_addr = dmam_alloc_coherent(
        &pdev->dev, ALOK_DMA_BUF_SIZE, &alok_dev->dma_bus_addr, GFP_KERNEL);
    if (!alok_dev->dma_virt_addr) {
        err = -ENOMEM;
        goto fail;
    }
    alok_log("DMA bus addr = 0x%08lx\n", (unsigned long)alok_dev->dma_bus_addr);
    alok_log("DMA virt addr = %p\n", alok_dev->dma_virt_addr);

    alok_log("resource 0: start=0x%08llx end=0x%08llx\n", pci_resource_start(pdev, 0), pci_resource_end(pdev, 0));
    err = pcim_iomap_regions(pdev, BIT(0), KBUILD_MODNAME);
    if (err) {
        goto fail;
    }
    alok_dev->iomem = pcim_iomap_table(pdev)[0];

    if (param_msi) {
        nvec = pci_alloc_irq_vectors(pdev, 1, 1, PCI_IRQ_ALL_TYPES);
        if (nvec < 0) {
            err = nvec;
            goto fail;
        }
        alok_dev->irq = pci_irq_vector(pdev, 0);
    } else {
        alok_dev->irq = pdev->irq;
    }
    alok_log("irq = %u\n", alok_dev->irq);
    err = request_irq(alok_dev->irq, alok_irq_handler, IRQF_SHARED, KBUILD_MODNAME, alok_dev);
    if (err) {
        goto fail;
    }
    alok_dev->registered_irq_handler = true;

    err = cdev_add(&alok_dev->cdev, devno, 1);
    if (err) {
        goto fail;
    }
    alok_dev->added_cdev = true;

    return 0;
fail:
    alok_pci_cleanup(pdev);
    return err;
}

static void alok_pci_remove(struct pci_dev *pdev) {
    pr_info("removing\n");
    alok_pci_cleanup(pdev);
}

static struct pci_driver alok_pci_driver = {
    .name = KBUILD_MODNAME,
    .id_table = alok_pci_tbl,
    .probe = alok_pci_probe,
    .remove = alok_pci_remove,
};

static void alok_driver_cleanup(void) {
    if (alok_dev) {
        kfree(alok_dev);
        alok_dev = NULL;
    }
    if (devno) {
        unregister_chrdev_region(devno, 1);
        devno = 0;
    }
}

static int alok_init(void) {
    int err;

    err = alloc_chrdev_region(&devno, minor, 1, KBUILD_MODNAME);
    if (err) {
        return err;
    }
    pr_alert("device number is %d:%d\n", MAJOR(devno), minor);

    alok_dev = kzalloc(sizeof(*alok_dev), GFP_KERNEL);
    if (!alok_dev) {
        err = -ENOMEM;
        goto fail;
    }
    alok_dev_init(alok_dev);

    err = pci_register_driver(&alok_pci_driver);
    if (err) {
        goto fail;
    }
    return 0;

fail:
    alok_driver_cleanup();
    return err;
}

static void alok_exit(void) {
    pci_unregister_driver(&alok_pci_driver);
    alok_driver_cleanup();
}

module_init(alok_init);
module_exit(alok_exit);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("QEMU ALOK device driver");
