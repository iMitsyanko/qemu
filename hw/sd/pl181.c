/*
 * Arm PrimeCell PL181 MultiMedia Card Interface
 *
 * Copyright (c) 2007 CodeSourcery.
 * Written by Paul Brook
 *
 * This code is licensed under the GPL.
 */

#include "sysemu/blockdev.h"
#include "hw/sysbus.h"
#include "hw/sd.h"

//#define DEBUG_PL181 1

#ifdef DEBUG_PL181
#define DPRINTF(fmt, ...) \
do { fprintf(stderr, "pl181: " fmt , ## __VA_ARGS__); } while (0)
#else
#define DPRINTF(fmt, ...) do {} while(0)
#endif

#define PL181_FIFO_LEN  64

typedef struct {
    SysBusDevice busdev;
    MemoryRegion iomem;
    SDState *card;
    uint32_t clock;
    uint32_t power;
    uint32_t cmdarg;
    uint32_t cmd;
    uint32_t datatimer;
    uint32_t datalength;
    uint32_t respcmd;
    uint32_t response[4];
    uint32_t datactrl;
    uint32_t datacnt;
    uint32_t status;
    uint32_t mask[2];
    uint32_t fifocnt;
    /* The linux 2.6.21 driver is buggy, and misbehaves if new data arrives
       while it is reading the FIFO.  We hack around this be defering
       subsequent transfers until after the driver polls the status word.
       http://www.arm.linux.org.uk/developer/patches/viewpatch.php?id=4446/1
     */
    int32_t linux_hack;
    qemu_irq irq[2];
    /* GPIO outputs for 'card is readonly' and 'card inserted' */
    qemu_irq cardstatus[2];
    qemu_irq sbit_received;
    qemu_irq dat_busy;
} pl181_state;

static const VMStateDescription vmstate_pl181 = {
    .name = "pl181",
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(clock, pl181_state),
        VMSTATE_UINT32(power, pl181_state),
        VMSTATE_UINT32(cmdarg, pl181_state),
        VMSTATE_UINT32(cmd, pl181_state),
        VMSTATE_UINT32(datatimer, pl181_state),
        VMSTATE_UINT32(datalength, pl181_state),
        VMSTATE_UINT32(respcmd, pl181_state),
        VMSTATE_UINT32_ARRAY(response, pl181_state, 4),
        VMSTATE_UINT32(datactrl, pl181_state),
        VMSTATE_UINT32(datacnt, pl181_state),
        VMSTATE_UINT32(status, pl181_state),
        VMSTATE_UINT32_ARRAY(mask, pl181_state, 2),
        VMSTATE_END_OF_LIST()
    }
};

#define PL181_CMD_INDEX     0x3f
#define PL181_CMD_RESPONSE  (1 << 6)
#define PL181_CMD_LONGRESP  (1 << 7)
#define PL181_CMD_INTERRUPT (1 << 8)
#define PL181_CMD_PENDING   (1 << 9)
#define PL181_CMD_ENABLE    (1 << 10)

#define PL181_DATA_ENABLE             (1 << 0)
#define PL181_DATA_DIRECTION          (1 << 1)
#define PL181_DATA_MODE               (1 << 2)
#define PL181_DATA_DMAENABLE          (1 << 3)

#define PL181_STATUS_CMDCRCFAIL       (1 << 0)
#define PL181_STATUS_DATACRCFAIL      (1 << 1)
#define PL181_STATUS_CMDTIMEOUT       (1 << 2)
#define PL181_STATUS_DATATIMEOUT      (1 << 3)
#define PL181_STATUS_TXUNDERRUN       (1 << 4)
#define PL181_STATUS_RXOVERRUN        (1 << 5)
#define PL181_STATUS_CMDRESPEND       (1 << 6)
#define PL181_STATUS_CMDSENT          (1 << 7)
#define PL181_STATUS_DATAEND          (1 << 8)
#define PL181_STATUS_DATABLOCKEND     (1 << 10)
#define PL181_STATUS_CMDACTIVE        (1 << 11)
#define PL181_STATUS_TXACTIVE         (1 << 12)
#define PL181_STATUS_RXACTIVE         (1 << 13)
#define PL181_STATUS_TXFIFOHALFEMPTY  (1 << 14)
#define PL181_STATUS_RXFIFOHALFFULL   (1 << 15)
#define PL181_STATUS_TXFIFOFULL       (1 << 16)
#define PL181_STATUS_RXFIFOFULL       (1 << 17)
#define PL181_STATUS_TXFIFOEMPTY      (1 << 18)
#define PL181_STATUS_RXFIFOEMPTY      (1 << 19)
#define PL181_STATUS_TXDATAAVLBL      (1 << 20)
#define PL181_STATUS_RXDATAAVLBL      (1 << 21)

#define PL181_STATUS_TX_FIFO (PL181_STATUS_TXACTIVE \
                             |PL181_STATUS_TXFIFOHALFEMPTY \
                             |PL181_STATUS_TXFIFOFULL \
                             |PL181_STATUS_TXFIFOEMPTY \
                             |PL181_STATUS_TXDATAAVLBL)
#define PL181_STATUS_RX_FIFO (PL181_STATUS_RXACTIVE \
                             |PL181_STATUS_RXFIFOHALFFULL \
                             |PL181_STATUS_RXFIFOFULL \
                             |PL181_STATUS_RXFIFOEMPTY \
                             |PL181_STATUS_RXDATAAVLBL)

static const unsigned char pl181_id[] =
{ 0x81, 0x11, 0x04, 0x00, 0x0d, 0xf0, 0x05, 0xb1 };

static inline unsigned int pl181_get_blk_size(pl181_state *s)
{
    if (s->datactrl & PL181_DATA_MODE) {
        return 1;
    } else {
        return 1 << ((s->datactrl >> 4) & 0xf);
    }
}

static void pl181_update(pl181_state *s)
{
    int i;
    for (i = 0; i < 2; i++) {
        qemu_set_irq(s->irq[i], (s->status & s->mask[i]) != 0);
    }
}

static void pl181_fifo_push(pl181_state *s, uint32_t value)
{
    unsigned i;
    unsigned int blkdat_left = s->datacnt & (pl181_get_blk_size(s) - 1);

    if (blkdat_left == 0 && s->fifocnt == 0) {
        /* Start of block */
        blkdat_left = pl181_get_blk_size(s);
    }

    for (i = 0; i < sizeof(uint32_t); ++i) {
        if (s->fifocnt >= PL181_FIFO_LEN) {
            qemu_log_mask(LOG_GUEST_ERROR, "pl181_fifo_push: fifo overflow\n");
            break;
        }

        sd_write_data_async(s->card, value >> i * 8);

        if (blkdat_left > PL181_FIFO_LEN) {
            --blkdat_left;
            --s->datacnt;
        } else {
            ++s->fifocnt;
        }
    }

    if (s->fifocnt == 0) {
        s->status &= ~(PL181_STATUS_TXDATAAVLBL | PL181_STATUS_TXFIFOFULL);
        s->status |= PL181_STATUS_TXFIFOEMPTY | PL181_STATUS_TXFIFOHALFEMPTY;
    } else {
        s->status &= ~(PL181_STATUS_TXFIFOEMPTY | PL181_STATUS_TXFIFOHALFEMPTY |
            PL181_STATUS_TXFIFOFULL);
        s->status |= PL181_STATUS_TXDATAAVLBL;

        if (s->fifocnt == PL181_FIFO_LEN) {
            s->status |= PL181_STATUS_TXFIFOFULL;
        } else if (s->fifocnt <= (PL181_FIFO_LEN / 2)) {
            s->status |= PL181_STATUS_TXFIFOHALFEMPTY;
        }
    }

    DPRINTF("FIFO push %08x: datacnt=%u, fifocnt=%u\n",
        (int)value, s->datacnt, s->fifocnt);
    pl181_update(s);
}

static uint32_t pl181_fifo_pop(pl181_state *s)
{
    uint32_t value = 0;
    uint32_t blksz_mask = pl181_get_blk_size(s) - 1;
    unsigned i;

    for (i = 0; i < sizeof(uint32_t); ++i) {
        if (s->fifocnt == 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "pl181_fifo_pop: fifo underflow\n");
            break;
        }

        value |= sd_read_data_async(s->card) << i * 8;

        if (s->datacnt & blksz_mask) {
            --s->datacnt;
        } else {
            --s->fifocnt;
        }
    }

    if (s->fifocnt == 0) {
        s->status |= PL181_STATUS_RXFIFOEMPTY | PL181_STATUS_DATABLOCKEND;
        s->status &= ~(PL181_STATUS_RXDATAAVLBL | PL181_STATUS_RXFIFOFULL |
            PL181_STATUS_RXFIFOHALFFULL);

        if (s->datacnt == 0) {
            s->status |= PL181_STATUS_DATAEND;
            s->status &= ~PL181_STATUS_RX_FIFO;
            s->datactrl &= ~PL181_DATA_ENABLE;
            DPRINTF("Data engine idle\n");
        }
    } else {
        s->status |= PL181_STATUS_RXDATAAVLBL;
        s->status &= ~PL181_STATUS_RXFIFOEMPTY;

        if (s->fifocnt == PL181_FIFO_LEN) {
            s->status |= PL181_STATUS_RXFIFOFULL;
        }

        if (s->fifocnt >= (PL181_FIFO_LEN / 2)) {
            s->status |= PL181_STATUS_RXFIFOHALFFULL;
        }
    }

    pl181_update(s);

    DPRINTF("FIFO pop %08x: datacnt=%u\n", (int)value, s->datacnt);

    return value;
}

static void pl181_send_command(pl181_state *s)
{
    SDRequest request;
    uint8_t response[16];
    int rlen;

    request.cmd = s->cmd & PL181_CMD_INDEX;
    request.arg = s->cmdarg;
    DPRINTF("Command %d %08x\n", request.cmd, request.arg);
    rlen = sd_do_command(s->card, &request, response);
    if (rlen < 0)
        goto error;
    if (s->cmd & PL181_CMD_RESPONSE) {
#define RWORD(n) ((response[n] << 24) | (response[n + 1] << 16) \
                  | (response[n + 2] << 8) | response[n + 3])
        if (rlen == 0 || (rlen == 4 && (s->cmd & PL181_CMD_LONGRESP)))
            goto error;
        if (rlen != 4 && rlen != 16)
            goto error;
        s->response[0] = RWORD(0);
        if (rlen == 4) {
            s->response[1] = s->response[2] = s->response[3] = 0;
        } else {
            s->response[1] = RWORD(4);
            s->response[2] = RWORD(8);
            s->response[3] = RWORD(12) & ~1;
        }
        DPRINTF("Response received\n");
        s->status |= PL181_STATUS_CMDRESPEND;
#undef RWORD
    } else {
        DPRINTF("Command sent\n");
        s->status |= PL181_STATUS_CMDSENT;
    }
    return;

error:
    DPRINTF("Timeout\n");
    s->status |= PL181_STATUS_CMDTIMEOUT;
}

static void pl181_transfer_start(pl181_state *s)
{
    s->datacnt = s->datalength;
    s->fifocnt = 0;
    s->status &= ~(PL181_STATUS_RX_FIFO | PL181_STATUS_TX_FIFO);

    if (s->datacnt) {
        if (s->datactrl & PL181_DATA_DIRECTION) {
            s->status |= PL181_STATUS_RXFIFOEMPTY | PL181_STATUS_RXACTIVE;
            DPRINTF("Reading %u bytes (blksz=%u): waiting for start bit\n",
                s->datacnt, pl181_get_blk_size(s));
        } else {
            s->status |= PL181_STATUS_TXFIFOEMPTY |
                PL181_STATUS_TXFIFOHALFEMPTY | PL181_STATUS_TXACTIVE;
            DPRINTF("Writing %u bytes (blksize=%u)\n", s->datacnt,
                pl181_get_blk_size(s));
        }
    } else {
        s->status |= PL181_STATUS_DATAEND;
        s->datactrl &= ~PL181_DATA_ENABLE;
    }

    pl181_update(s);
}

static void pl181_start_bit_received(void *opaque, int irq, int level)
{
    pl181_state *s = (pl181_state *)opaque;
    unsigned int blksz = pl181_get_blk_size(s);

    if (!(s->status & PL181_STATUS_RXACTIVE) || s->datacnt == 0) {
        qemu_log_mask(LOG_GUEST_ERROR, "pl181: unexpected start bit\n");
        return;
    }

    if (s->linux_hack) {
        s->linux_hack = 2;
        return;
    }

    s->status &= ~(PL181_STATUS_RXFIFOHALFFULL | PL181_STATUS_RXFIFOFULL |
        PL181_STATUS_RXFIFOEMPTY);
    s->status |= PL181_STATUS_RXDATAAVLBL;
    s->fifocnt = MIN(blksz, PL181_FIFO_LEN);
    s->fifocnt = MIN(s->fifocnt, s->datacnt);
    s->datacnt -= s->fifocnt;

    if (s->fifocnt == PL181_FIFO_LEN) {
        s->status |= PL181_STATUS_RXFIFOFULL;
    }

    if (s->fifocnt >= (PL181_FIFO_LEN / 2)) {
        s->status |= PL181_STATUS_RXFIFOHALFFULL;
    }

    DPRINTF("Start bit received: datacnt=%u, fifocnt=%u\n",
        s->datacnt, s->fifocnt);
    pl181_update(s);
}

static void pl181_busy_deasserted(void *opaque, int irq, int level)
{
    pl181_state *s = (pl181_state *)opaque;

    if (!(s->status & PL181_STATUS_TXACTIVE)) {
        qemu_log_mask(LOG_GUEST_ERROR, "pl181: unexpected busy end\n");
        return;
    }

    s->datacnt -= s->fifocnt;
    s->fifocnt = 0;

    if (s->datacnt == 0) {
        s->datactrl &= ~PL181_DATA_ENABLE;
        s->status |= PL181_STATUS_DATAEND | PL181_STATUS_DATABLOCKEND;
        s->status &= ~PL181_STATUS_TX_FIFO;
        DPRINTF("Data engine idle\n");
    } else {
        s->status &= ~PL181_STATUS_TXDATAAVLBL;
        s->status |= PL181_STATUS_TXFIFOEMPTY | PL181_STATUS_TXFIFOHALFEMPTY |
            PL181_STATUS_DATABLOCKEND;
    }

    pl181_update(s);
    DPRINTF("Busy deasserted: datacnt=%u\n", s->datacnt);
}

static uint64_t pl181_read(void *opaque, hwaddr offset,
                           unsigned size)
{
    pl181_state *s = (pl181_state *)opaque;
    uint32_t ret = 0;


    if (offset >= 0xfe0 && offset < 0x1000) {
        return pl181_id[(offset - 0xfe0) >> 2];
    }
    switch (offset) {
    case 0x00: /* Power */
        ret = s->power;
        break;
    case 0x04: /* Clock */
        ret = s->clock;
        break;
    case 0x08: /* Argument */
        ret = s->cmdarg;
        break;
    case 0x0c: /* Command */
        ret = s->cmd;
        break;
    case 0x10: /* RespCmd */
        ret = s->respcmd;
        break;
    case 0x14: /* Response0 */
        ret = s->response[0];
        break;
    case 0x18: /* Response1 */
        ret = s->response[1];
        break;
    case 0x1c: /* Response2 */
        ret = s->response[2];
        break;
    case 0x20: /* Response3 */
        ret = s->response[3];
        break;
    case 0x24: /* DataTimer */
        ret = s->datatimer;
        break;
    case 0x28: /* DataLength */
        ret = s->datalength;
        break;
    case 0x2c: /* DataCtrl */
        ret = s->datactrl;
        break;
    case 0x30: /* DataCnt */
        ret = s->datacnt;
        break;
    case 0x34: /* Status */
        ret = s->status;
        if (s->linux_hack == 2) {
            s->linux_hack = 0;
            qemu_irq_raise(s->sbit_received);
        }
        s->linux_hack = 0;
        break;
    case 0x3c: /* Mask0 */
        ret = s->mask[0];
        break;
    case 0x40: /* Mask1 */
        ret = s->mask[1];
        break;
    case 0x48: /* FifoCnt */
        /* The documentation is somewhat vague about exactly what FifoCnt
           does.  On real hardware it appears to be when decrememnted
           when a word is transferred between the FIFO and the serial
           data engine.  DataCnt is decremented after each byte is
           transferred between the serial engine and the card.
           We don't emulate this level of detail, so both can be the same.  */
        ret = (s->datacnt + 3) >> 2;
        if (s->linux_hack == 2) {
            s->linux_hack = 0;
            qemu_irq_raise(s->sbit_received);
        }
        s->linux_hack = 0;
        break;
    case 0x80: case 0x84: case 0x88: case 0x8c: /* FifoData */
    case 0x90: case 0x94: case 0x98: case 0x9c:
    case 0xa0: case 0xa4: case 0xa8: case 0xac:
    case 0xb0: case 0xb4: case 0xb8: case 0xbc:
        if (s->fifocnt == 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "pl181: Unexpected FIFO read\n");
            ret = 0;
        } else {
            s->linux_hack = 1;
            ret = pl181_fifo_pop(s);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl181_read: Bad offset %x\n", (int)offset);
        ret = 0;
        break;
    }
    DPRINTF("read 0x%" HWADDR_PRIx" --> 0x%x(%u)\n", offset, ret, ret);
    return ret;
}

static void pl181_write(void *opaque, hwaddr offset,
                        uint64_t value, unsigned size)
{
    pl181_state *s = (pl181_state *)opaque;
    DPRINTF("write 0x%" HWADDR_PRIx" <-- 0x%x(%u)\n", offset,
        (uint32_t)value, (uint32_t)value);

    switch (offset) {
    case 0x00: /* Power */
        s->power = value & 0xff;
        break;
    case 0x04: /* Clock */
        s->clock = value & 0xff;
        break;
    case 0x08: /* Argument */
        s->cmdarg = value;
        break;
    case 0x0c: /* Command */
        s->cmd = value;
        if (s->cmd & PL181_CMD_ENABLE) {
            if (s->cmd & PL181_CMD_INTERRUPT) {
                qemu_log_mask(LOG_UNIMP,
                              "pl181: Interrupt mode not implemented\n");
            } if (s->cmd & PL181_CMD_PENDING) {
                qemu_log_mask(LOG_UNIMP,
                              "pl181: Pending commands not implemented\n");
            } else {
                pl181_send_command(s);
            }
            /* The command has completed one way or the other.  */
            s->cmd &= ~PL181_CMD_ENABLE;
        }
        break;
    case 0x24: /* DataTimer */
        s->datatimer = value;
        break;
    case 0x28: /* DataLength */
        s->datalength = value & 0xffff;
        break;
    case 0x2c: /* DataCtrl */
        s->datactrl = value & 0xff;
        if (value & PL181_DATA_ENABLE) {
            pl181_transfer_start(s);
        }
        break;
    case 0x38: /* Clear */
        s->status &= ~(value & 0x7ff);
        break;
    case 0x3c: /* Mask0 */
        s->mask[0] = value;
        break;
    case 0x40: /* Mask1 */
        s->mask[1] = value;
        break;
    case 0x80: case 0x84: case 0x88: case 0x8c: /* FifoData */
    case 0x90: case 0x94: case 0x98: case 0x9c:
    case 0xa0: case 0xa4: case 0xa8: case 0xac:
    case 0xb0: case 0xb4: case 0xb8: case 0xbc:
        if (s->datacnt == 0) {
            qemu_log_mask(LOG_GUEST_ERROR, "pl181: Unexpected FIFO write\n");
        } else {
            pl181_fifo_push(s, value);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "pl181_write: Bad offset %x\n", (int)offset);
    }
    pl181_update(s);
}

static const MemoryRegionOps pl181_ops = {
    .read = pl181_read,
    .write = pl181_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void pl181_reset(DeviceState *d)
{
    pl181_state *s = DO_UPCAST(pl181_state, busdev.qdev, d);

    s->power = 0;
    s->cmdarg = 0;
    s->cmd = 0;
    s->datatimer = 0;
    s->datalength = 0;
    s->respcmd = 0;
    s->response[0] = 0;
    s->response[1] = 0;
    s->response[2] = 0;
    s->response[3] = 0;
    s->datatimer = 0;
    s->datalength = 0;
    s->datactrl = 0;
    s->datacnt = 0;
    s->status = 0;
    s->linux_hack = 0;
    s->mask[0] = 0;
    s->mask[1] = 0;
    s->fifocnt = 0;

    /* We can assume our GPIO outputs have been wired up now */
    sd_set_cb(s->card, s->cardstatus[0], s->cardstatus[1], s->sbit_received,
        s->dat_busy);
}

static int pl181_init(SysBusDevice *dev)
{
    pl181_state *s = FROM_SYSBUS(pl181_state, dev);
    DriveInfo *dinfo;

    memory_region_init_io(&s->iomem, &pl181_ops, s, "pl181", 0x1000);
    sysbus_init_mmio(dev, &s->iomem);
    sysbus_init_irq(dev, &s->irq[0]);
    sysbus_init_irq(dev, &s->irq[1]);
    qdev_init_gpio_out(&s->busdev.qdev, s->cardstatus, 2);
    dinfo = drive_get_next(IF_SD);
    s->card = sd_init(dinfo ? dinfo->bdrv : NULL, 0);
    s->sbit_received = qemu_allocate_irqs(pl181_start_bit_received, s, 1)[0];
    s->dat_busy = qemu_allocate_irqs(pl181_busy_deasserted, s, 1)[0];

    return 0;
}

static void pl181_class_init(ObjectClass *klass, void *data)
{
    SysBusDeviceClass *sdc = SYS_BUS_DEVICE_CLASS(klass);
    DeviceClass *k = DEVICE_CLASS(klass);

    sdc->init = pl181_init;
    k->vmsd = &vmstate_pl181;
    k->reset = pl181_reset;
    k->no_user = 1;
}

static const TypeInfo pl181_info = {
    .name          = "pl181",
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(pl181_state),
    .class_init    = pl181_class_init,
};

static void pl181_register_types(void)
{
    type_register_static(&pl181_info);
}

type_init(pl181_register_types)
