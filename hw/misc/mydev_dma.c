/*
 * Simple sysbus device that performs DMA to/from guest memory.
 * DMA transactions are tagged with device_wid and routed through
 * the WGC IOMMU (address_space_memory), simulating a WGmarker in
 * front of the device — just like CPU harts have WGmarkers.
 */

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "qapi/error.h"
#include "qemu/log.h"
#include "hw/hw.h"
#include "exec/memory.h"
#include "exec/address-spaces.h"  /* address_space_memory */
#include "qemu/module.h"
#include "qemu/typedefs.h"
#include "hw/irq.h"
#include "hw/registerfields.h"
#include "hw/qdev-properties.h"
#include "qom/object.h"


//-----------------------------------------------------------------------
// OFFSET | SIZE | R/W |     NAME     | Description |
//-----------------------------------------------------------------------
//  0x00  | 8bit | WO  | CMD          | 1: device to dma
//                                    | 2: dma to device
//                                    | raise an interrupt after competion
//-----------------------------------------------------------------------
//  0x08  | 8bit | RW  | Addr low32   | lower 32-bit of dma address
//-----------------------------------------------------------------------
//  0x0c  | 8bit | RW  | Addr high32  | high 32-bit of dma address
//-----------------------------------------------------------------------
//  0x10  | 8bit | RW  | LENGTH       | Size of dma area
//-----------------------------------------------------------------------
//  0x18  | 8bit | RO  | Status       | 0: operation competed
//                                      1: Wrting operation
//                                      2: Reading operation
//                                      3: Error
//                                      4: Unknown CMD
//-----------------------------------------------------------------------

#define TYPE_MYDEVICE "mydev-dma"
#define MYDEVICE(obj) OBJECT_CHECK(MyDevDMAState, (obj), TYPE_MYDEVICE)

typedef struct {
    SysBusDevice parent_obj;

    /* MMIO */
    MemoryRegion iomem;
    uint8_t *host_buf;        /* device internal host buffer */
    size_t host_buf_size;

    /* simple registers */
    hwaddr dma_gpa;           /* guest physical address (destination/source) */
    uint32_t dma_len;
    uint32_t cmd;
    uint32_t status;

    qemu_irq irq;             /* irq line to guest */

    /* WGmarker: WID stamped on all DMA bus transactions */
    uint32_t device_wid;

    /* convenience */
    SysBusDevice *sbd;
} MyDevDMAState;

static void mydev_set_irq(MyDevDMAState *s)
{
    /* assert IRQ for a short time (edge-like) */
    qemu_set_irq(s->irq, 1);
}

static void mydev_clr_irq(MyDevDMAState *s)
{
    /* assert IRQ for a short time (edge-like) */
    qemu_set_irq(s->irq, 0);
}


static uint64_t mydev_read(void *opaque, hwaddr offset, unsigned size)
{
    MyDevDMAState *s = opaque;
    uint64_t val = 0;

    switch (offset) {
    case 0x00: /* CMD (read returns 0) */ break;
    case 0x18: /* status */
        val = s->status;
        mydev_clr_irq(s);
        break;
    case 0x08: /* dma_gpa low 32 */
        val = (uint32_t)(s->dma_gpa & 0xffffffff);
        break;
    case 0x0c: /* dma_gpa high 32 */
        val = (uint32_t)((s->dma_gpa >> 32) & 0xffffffff);
        break;
    case 0x10: /* dma_len */
        val = s->dma_len;
        break;
    default:
        printf("[QEMU] mydev_dma: read offset %#lx size %#x val %#lx\n", offset, size, val);
        qemu_log_mask(LOG_GUEST_ERROR, "mydev_dma: read offset %#"HWADDR_PRIx"\n", offset);
    }
    //printf("[QEMU] mydev_dma: read offset %#lx size %#x val %#lx\n", offset, size, val);
    return val;
}

static void perform_dma_device_to_guest(MyDevDMAState *s)
{
    s->status = 1;
    hwaddr gpa = s->dma_gpa;
    size_t len = s->dma_len;
    if (!len || len > s->host_buf_size) {
        mydev_set_irq(s);
        return;
    }

    /*
     * WGmarker: tag the DMA transaction with device_wid.
     * address_space_memory routes DRAM accesses through the WGC IOMMU
     * (create_wgc replaced the raw DRAM region with the WGC upstream).
     * riscv_wgc_translate() will check: does device_wid have write
     * permission for gpa?  If not → WGC fault, MEMTX_ERROR returned.
     */
    MemTxAttrs attrs = {};
    attrs.world_id = s->device_wid;
    MemTxResult res = address_space_write(&address_space_memory,
                                         gpa, attrs, s->host_buf, len);
    if (res != MEMTX_OK) {
        printf("[QEMU] mydev_dma: WGC BLOCKED device->guest DMA "
               "(WID=%u gpa=0x%"HWADDR_PRIx")\n", s->device_wid, gpa);
        s->status = 3; /* error */
        mydev_set_irq(s);
        return;
    }

    s->status = 0;
    mydev_set_irq(s);
}

static void perform_dma_guest_to_device(MyDevDMAState *s)
{
    s->status = 2;
    hwaddr gpa = s->dma_gpa;
    size_t len = s->dma_len;
    if (!len || len > s->host_buf_size) {
        s->status = 3;
        return;
    }

    /* WGmarker: tag read transaction with device_wid, go through WGC */
    MemTxAttrs attrs = {};
    attrs.world_id = s->device_wid;
    MemTxResult res = address_space_read(&address_space_memory,
                                        gpa, attrs, s->host_buf, len);
    if (res != MEMTX_OK) {
        printf("[QEMU] mydev_dma: WGC BLOCKED guest->device DMA "
               "(WID=%u gpa=0x%"HWADDR_PRIx")\n", s->device_wid, gpa);
        s->status = 3;
        return;
    }

    s->status = 0;
}

static void mydev_start_command(MyDevDMAState *s, uint32_t cmd)
{
    /* Synchronous (blocking) DMA for simplicity — fine for testing.
       In a real device you'd do this asynchronously. */
    if (cmd == 1) {
        /* device -> guest DMA */
        perform_dma_device_to_guest(s);
    } else if (cmd == 2) {
        /* guest -> device DMA */
        perform_dma_guest_to_device(s);
    } else {
    }
    mydev_set_irq(s);
}

/* MMIO write */
static void mydev_write(void *opaque, hwaddr offset, uint64_t val, unsigned size)
{
    printf("[QEMU] mydev_dma: write offset %#lx val %#lx\n", offset, val);
    MyDevDMAState *s = opaque;

    switch (offset) {
    case 0x00: /* CMD */
        s->cmd = (uint32_t)val;
        /* start operation immediately */
        mydev_start_command(s, s->cmd);
        break;
    case 0x08: /* dma_gpa low */
        s->dma_gpa = (s->dma_gpa & ~0xffffffffULL) | (val & 0xffffffffULL);
        break;
    case 0x0c: /* dma_gpa high */
        s->dma_gpa = (s->dma_gpa & 0xffffffffULL) | ((uint64_t)(val & 0xffffffffULL) << 32);
        break;
    case 0x10: /* dma_len */
        s->dma_len = (uint32_t)val;
        break;
    default:
        printf("[QEMU] mydev_dma: write error offset %#lx val %#lx\n", offset, val);
        qemu_log_mask(LOG_GUEST_ERROR, "mydev_dma: write error offset %#"HWADDR_PRIx" val %#"PRIx64"\n", offset, val);
    }
}

static const MemoryRegionOps mydev_mmio_ops = {
    .read = mydev_read,
    .write = mydev_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void mydevice_reset(Object *obj, ResetType type)
{
    printf("[QEMU] mydev_dma: device reset\n");
    MyDevDMAState *s = MYDEVICE(obj);
    s->dma_gpa = 0;
    s->dma_len = 0;
    s->cmd = 0;
    s->status = 0;
    memset(s->host_buf, 1, s->host_buf_size);
}


static void mydevice_init(Object *obj)
{
    printf("[QEMU] mydev_dma: device init\n");
    MyDevDMAState *s = MYDEVICE(obj);

    s->host_buf_size = 4096; /* device buffer size */
    s->host_buf = g_malloc0(s->host_buf_size);

    /* fill buffer with a recognizable pattern (for device->guest DMA tests) */
    for (unsigned i = 0; i < s->host_buf_size; ++i) {
        s->host_buf[i] = (uint8_t)(i & 0xff);
    }
    
    printf("[QEMU] mydev_dma: inititial host buf (first 16 bytes: ");
    for (int i = 0 ; i < 16; i++) printf("%#x ", s->host_buf[i]);
    printf(")\n");

    memory_region_init_io(&s->iomem, OBJECT(s), &mydev_mmio_ops, s, "mydevice-mmio", 0x1000);
}

/* SysBus glue */
static void mydevice_realize(DeviceState *dev, Error **errp)
{
    printf("[QEMU] mydev_dma: device realize\n");
    MyDevDMAState *s = MYDEVICE(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    sysbus_init_mmio(sbd, &s->iomem);
    sysbus_init_irq(sbd, &s->irq); /* single irq */

    s->status = 0;
    //device_reset_register(DEVICE(dev), mydevice_reset);
    //mydevice_reset(s);
}

static void mydevice_uninit(Object *obj)
{
    printf("[QEMU] mydev_dma: device uninit\n");
    MyDevDMAState *s = MYDEVICE(obj);
    g_free(s->host_buf);
}

static Property mydevice_properties[] = {
    /* WGmarker WID: stamped on every DMA bus transaction.
     * Default 29 = my_dev WID; SM is WID 31 (trusted), host Linux is WID 30,
     * my_dev is WID 29, enclave WIDs use 0-28. */
    DEFINE_PROP_UINT32("device-wid", MyDevDMAState, device_wid, 29),
    DEFINE_PROP_END_OF_LIST(),
};

static void mydevice_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);
    dc->realize = mydevice_realize;
    dc->desc = "Simple DMA-capable test device";
    dc->vmsd = NULL;
    device_class_set_props(dc, mydevice_properties);

    ResettableClass *rc = RESETTABLE_CLASS(oc);
    rc->phases.enter = mydevice_reset;
}

static const TypeInfo mydevice_info = {
    .name = TYPE_MYDEVICE,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(MyDevDMAState),
    .instance_init = mydevice_init,
    .class_init = mydevice_class_init,
    .instance_finalize = mydevice_uninit,
};

static void mydevice_register_types(void)
{
    type_register_static(&mydevice_info);
}

type_init(mydevice_register_types)


