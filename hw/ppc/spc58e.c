/*
 * SPC58E machine skeleton for incremental emulation bring-up.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/ppc/ppc.h"
#include "exec/address-spaces.h"
#include "exec/memory.h"
#include "exec/cpu-common.h"
#include "qemu/timer.h"
#include "elf.h"
#include "qapi/error.h"
#include "target/ppc/cpu.h"

#define SPC58_BOOT_FLASH_BASE  0x00fc0000u
#define SPC58_BOOT_FLASH_SIZE  (256 * KiB)

#define SPC58_SYS_SRAM_BASE    0x400a8000u
#define SPC58_SYS_SRAM_SIZE    (384 * KiB)

#define SPC58_CORE2_SRAM_BASE  0x52800000u
#define SPC58_CORE2_SRAM_SIZE  (32 * KiB)

#define SPC58_INTC_BASE        0xfff48000u
#define SPC58_INTC_SIZE        0x4000u

#define SPC58_INTC_MCR         0x0000u
#define SPC58_INTC_CPR         0x0008u
#define SPC58_INTC_IACKR       0x0010u
#define SPC58_INTC_EOIR        0x0018u
#define SPC58_INTC_TEST_SET    0x0100u
#define SPC58_INTC_TEST_CLR    0x0104u

#define SPC58_INTC_NUM_IRQS    256u
#define SPC58_INTC_VEC_BASE    0x1000u

#define SPC58_MC_CGM_BASE      0xfffec000u
#define SPC58_MC_CGM_SIZE      0x4000u

#define SPC58_MC_CGM_CTRL      0x0000u
#define SPC58_MC_CGM_STAT      0x0004u
#define SPC58_MC_CGM_DIV       0x0030u

#define SPC58_FLASHC_BASE      0xfff44000u
#define SPC58_FLASHC_SIZE      0x4000u

#define SPC58_FLASHC_MCR       0x0000u
#define SPC58_FLASHC_MCRS      0x0004u
#define SPC58_FLASHC_LMLR      0x0008u
#define SPC58_FLASHC_HLR       0x000cu
#define SPC58_FLASHC_AR        0x0010u
#define SPC58_FLASHC_CMD       0x0014u

#define SPC58_PIT_BASE         0xfff84000u
#define SPC58_PIT_SIZE         0x4000u

#define SPC58_PIT_CH_BASE      0x0100u
#define SPC58_PIT_CH_STRIDE    0x0010u
#define SPC58_PIT_LDVAL_OFF    0x0000u
#define SPC58_PIT_CVAL_OFF     0x0004u
#define SPC58_PIT_TCTRL_OFF    0x0008u
#define SPC58_PIT_TFLG_OFF     0x000cu

#define SPC58_STM_BASE         0xfff7c000u
#define SPC58_STM_SIZE         0x4000u

#define SPC58_STM_CNT          0x0000u
#define SPC58_STM_CMP0         0x0010u
#define SPC58_STM_CR           0x0020u
#define SPC58_STM_SR           0x0024u

#define SPC58_PIT_NUM_CH       4u
#define SPC58_PIT_IRQ_BASE     32u
#define SPC58_STM_IRQ_ID       48u

#define SPC58_VALID_BASE        0xfffe8000u
#define SPC58_VALID_SIZE        0x1000u

#define SPC58_VALID_SIGNATURE   0x0000u
#define SPC58_VALID_FLAGS       0x0004u
#define SPC58_VALID_ENTRY_LO    0x0008u
#define SPC58_VALID_LAST_IRQ    0x000cu
#define SPC58_VALID_IRQ_COUNT   0x0010u
#define SPC58_VALID_WDG_TRIPS   0x0014u
#define SPC58_VALID_LAST_EVENT  0x0018u

#define SPC58_VALID_MAGIC       0x53504335u /* 'SPC5' */

#define SPC58_SWT_BASE         0xfff38000u
#define SPC58_SWT_SIZE         0x4000u

#define SPC58_SWT_CR           0x0000u
#define SPC58_SWT_IR           0x0004u
#define SPC58_SWT_TO           0x0008u
#define SPC58_SWT_CNT          0x000cu
#define SPC58_SWT_SR           0x0010u

#define SPC58_SWT_IRQ_ID       9u

typedef struct SPC58EState {
    PowerPCCPU *cpu;
    MemoryRegion boot_flash;
    MemoryRegion sys_sram;
    MemoryRegion core2_sram;
    MemoryRegion intc_mmio;
    MemoryRegion mc_cgm_mmio;
    MemoryRegion flashc_mmio;
    MemoryRegion pit_mmio;
    MemoryRegion stm_mmio;
    MemoryRegion valid_mmio;
    MemoryRegion swt_mmio;
    uint32_t intc_regs[SPC58_INTC_SIZE / sizeof(uint32_t)];
    uint32_t intc_pending[SPC58_INTC_NUM_IRQS / 32u];
    uint32_t mc_cgm_regs[SPC58_MC_CGM_SIZE / sizeof(uint32_t)];
    uint32_t flashc_regs[SPC58_FLASHC_SIZE / sizeof(uint32_t)];
    uint32_t pit_regs[SPC58_PIT_SIZE / sizeof(uint32_t)];
    uint32_t stm_regs[SPC58_STM_SIZE / sizeof(uint32_t)];
    uint32_t valid_regs[SPC58_VALID_SIZE / sizeof(uint32_t)];
    uint32_t swt_regs[SPC58_SWT_SIZE / sizeof(uint32_t)];
    uint8_t intc_enabled;
    int32_t intc_current_irq;
    uint8_t cgm_enabled;
    uint32_t cgm_core_hz;
    uint32_t cgm_periph_hz;
    uint8_t swt_enabled;
    uint32_t swt_reload;
    uint32_t swt_counter;
    uint32_t swt_service_step;
    int64_t swt_last_ns;
    uint8_t flashc_busy;
    int64_t pit_last_ns;
    int64_t stm_last_ns;
} SPC58EState;

static SPC58EState spc58e;

static void spc58e_intc_set_pending(SPC58EState *s, uint32_t irq);
static void spc58e_intc_recompute_irq(SPC58EState *s);

static void spc58e_swt_update_counter(SPC58EState *s)
{
    uint64_t elapsed_ns;
    uint64_t ticks;
    int64_t now;

    if (!s->swt_enabled) {
        s->swt_last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        return;
    }

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (s->swt_last_ns == 0) {
        s->swt_last_ns = now;
        return;
    }

    if (now <= s->swt_last_ns) {
        return;
    }

    elapsed_ns = (uint64_t)(now - s->swt_last_ns);
    s->swt_last_ns = now;

    /* lightweight model: watchdog runs from peripheral clock / 1024 */
    ticks = (elapsed_ns * ((uint64_t)s->cgm_periph_hz / 1024u)) / NANOSECONDS_PER_SECOND;
    if (ticks == 0) {
        return;
    }

    if (ticks >= s->swt_counter) {
        s->swt_counter = 0;
        spc58e_intc_set_pending(s, SPC58_SWT_IRQ_ID);
        spc58e_intc_recompute_irq(s);
        s->valid_regs[SPC58_VALID_WDG_TRIPS >> 2]++;
        s->valid_regs[SPC58_VALID_LAST_EVENT >> 2] = 0x57444730u; /* WDG0 */
    } else {
        s->swt_counter -= (uint32_t)ticks;
    }
}

static void spc58e_pit_update(SPC58EState *s)
{
    uint64_t elapsed_ns;
    uint64_t ticks;
    int64_t now;
    uint32_t ch;

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (s->pit_last_ns == 0) {
        s->pit_last_ns = now;
        return;
    }

    if (now <= s->pit_last_ns) {
        return;
    }

    elapsed_ns = (uint64_t)(now - s->pit_last_ns);
    s->pit_last_ns = now;
    ticks = (elapsed_ns * ((uint64_t)s->cgm_periph_hz / 64u)) / NANOSECONDS_PER_SECOND;
    if (ticks == 0) {
        return;
    }

    for (ch = 0; ch < SPC58_PIT_NUM_CH; ch++) {
        uint32_t base = (SPC58_PIT_CH_BASE + ch * SPC58_PIT_CH_STRIDE) >> 2;
        uint32_t ldval = s->pit_regs[base + (SPC58_PIT_LDVAL_OFF >> 2)];
        uint32_t cval = s->pit_regs[base + (SPC58_PIT_CVAL_OFF >> 2)];
        uint32_t tctrl = s->pit_regs[base + (SPC58_PIT_TCTRL_OFF >> 2)];
        uint32_t tflg = s->pit_regs[base + (SPC58_PIT_TFLG_OFF >> 2)];

        if ((tctrl & 0x1u) == 0u) {
            continue;
        }

        if (ticks >= cval) {
            s->pit_regs[base + (SPC58_PIT_CVAL_OFF >> 2)] = ldval;
            tflg |= 0x1u;
            s->pit_regs[base + (SPC58_PIT_TFLG_OFF >> 2)] = tflg;

            if (tctrl & 0x2u) {
                spc58e_intc_set_pending(s, SPC58_PIT_IRQ_BASE + ch);
            }
        } else {
            s->pit_regs[base + (SPC58_PIT_CVAL_OFF >> 2)] = cval - (uint32_t)ticks;
        }
    }

    spc58e_intc_recompute_irq(s);
}

static void spc58e_stm_update(SPC58EState *s)
{
    uint64_t elapsed_ns;
    uint64_t ticks;
    uint32_t cnt;
    uint32_t cmp0;
    uint32_t cr;
    int64_t now;

    now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    if (s->stm_last_ns == 0) {
        s->stm_last_ns = now;
        return;
    }

    if (now <= s->stm_last_ns) {
        return;
    }

    elapsed_ns = (uint64_t)(now - s->stm_last_ns);
    s->stm_last_ns = now;
    ticks = (elapsed_ns * ((uint64_t)s->cgm_periph_hz / 128u)) / NANOSECONDS_PER_SECOND;
    if (ticks == 0) {
        return;
    }

    cnt = s->stm_regs[SPC58_STM_CNT >> 2];
    cmp0 = s->stm_regs[SPC58_STM_CMP0 >> 2];
    cr = s->stm_regs[SPC58_STM_CR >> 2];
    cnt += (uint32_t)ticks;
    s->stm_regs[SPC58_STM_CNT >> 2] = cnt;

    if ((cr & 0x1u) && cnt >= cmp0) {
        s->stm_regs[SPC58_STM_SR >> 2] |= 0x1u;
        spc58e_intc_set_pending(s, SPC58_STM_IRQ_ID);
        spc58e_intc_recompute_irq(s);
    }
}

static void spc58e_cgm_recompute_clocks(SPC58EState *s)
{
    uint32_t div_raw = s->mc_cgm_regs[SPC58_MC_CGM_DIV >> 2] & 0x0fu;
    uint32_t div = div_raw == 0 ? 1 : div_raw;

    if (s->cgm_enabled) {
        s->cgm_core_hz = 160000000u / div;
        s->cgm_periph_hz = 80000000u / div;
    } else {
        s->cgm_core_hz = 16000000u;
        s->cgm_periph_hz = 8000000u;
    }
}

static bool spc58e_intc_is_pending(SPC58EState *s, uint32_t irq)
{
    uint32_t idx = irq >> 5;
    uint32_t bit = irq & 31u;

    return (s->intc_pending[idx] & (1u << bit)) != 0;
}

static void spc58e_intc_set_pending(SPC58EState *s, uint32_t irq)
{
    uint32_t idx = irq >> 5;
    uint32_t bit = irq & 31u;

    s->intc_pending[idx] |= (1u << bit);
}

static void spc58e_intc_clear_pending(SPC58EState *s, uint32_t irq)
{
    uint32_t idx = irq >> 5;
    uint32_t bit = irq & 31u;

    s->intc_pending[idx] &= ~(1u << bit);
}

static int32_t spc58e_intc_next_pending(SPC58EState *s)
{
    uint32_t irq;

    for (irq = 0; irq < SPC58_INTC_NUM_IRQS; irq++) {
        if (spc58e_intc_is_pending(s, irq)) {
            return irq;
        }
    }

    return -1;
}

static void spc58e_intc_recompute_irq(SPC58EState *s)
{
    int32_t next = spc58e_intc_next_pending(s);

    if (!s->cpu) {
        return;
    }

    if (s->intc_enabled && next >= 0) {
        ppc_set_irq(s->cpu, PPC_INTERRUPT_EXT, 1);
    } else {
        ppc_set_irq(s->cpu, PPC_INTERRUPT_EXT, 0);
    }
}

static uint64_t spc58e_intc_read(void *opaque, hwaddr addr, unsigned size)
{
    SPC58EState *s = opaque;
    int32_t irq;
    uint32_t vec;
    uint32_t idx = addr >> 2;
    uint32_t value = (idx < ARRAY_SIZE(s->intc_regs)) ? s->intc_regs[idx] : 0;

    switch (addr) {
    case SPC58_INTC_MCR:
        value = s->intc_enabled ? 0x0u : 0x1u;
        break;
    case SPC58_INTC_IACKR:
        irq = spc58e_intc_next_pending(s);
        if (irq >= 0) {
            s->intc_current_irq = irq;
            vec = SPC58_INTC_VEC_BASE + ((uint32_t)irq << 2);
            value = vec;
            s->valid_regs[SPC58_VALID_LAST_IRQ >> 2] = (uint32_t)irq;
            s->valid_regs[SPC58_VALID_IRQ_COUNT >> 2]++;
            s->valid_regs[SPC58_VALID_LAST_EVENT >> 2] = 0x49525141u; /* IRQA */
        } else {
            s->intc_current_irq = -1;
            value = 0;
        }
        break;
    default:
        break;
    }

    qemu_log_mask(LOG_UNIMP,
                  "spc58e:intc rd addr=0x%08" HWADDR_PRIx " size=%u -> 0x%08x\n",
                  addr, size, value);
    return value;
}

static void spc58e_intc_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t irq;
    uint32_t idx = addr >> 2;
    uint32_t value = (uint32_t)data;

    switch (addr) {
    case SPC58_INTC_MCR:
        s->intc_enabled = ((value & 0x1u) == 0u);
        spc58e_intc_recompute_irq(s);
        break;
    case SPC58_INTC_EOIR:
        if (s->intc_current_irq >= 0) {
            spc58e_intc_clear_pending(s, (uint32_t)s->intc_current_irq);
            s->intc_current_irq = -1;
            s->valid_regs[SPC58_VALID_LAST_EVENT >> 2] = 0x49525145u; /* IRQE */
        }
        spc58e_intc_recompute_irq(s);
        break;
    case SPC58_INTC_TEST_SET:
        irq = value & 0xffu;
        spc58e_intc_set_pending(s, irq);
        spc58e_intc_recompute_irq(s);
        break;
    case SPC58_INTC_TEST_CLR:
        irq = value & 0xffu;
        spc58e_intc_clear_pending(s, irq);
        spc58e_intc_recompute_irq(s);
        break;
    default:
        break;
    }

    if (idx < ARRAY_SIZE(s->intc_regs)) {
        s->intc_regs[idx] = value;
    }

    qemu_log_mask(LOG_UNIMP,
                  "spc58e:intc wr addr=0x%08" HWADDR_PRIx " size=%u val=0x%08" PRIx64 "\n",
                  addr, size, data);
}

static const MemoryRegionOps spc58e_intc_ops = {
    .read = spc58e_intc_read,
    .write = spc58e_intc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t spc58e_mc_cgm_read(void *opaque, hwaddr addr, unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;
    uint32_t value = (idx < ARRAY_SIZE(s->mc_cgm_regs)) ? s->mc_cgm_regs[idx] : 0;

    switch (addr) {
    case SPC58_MC_CGM_STAT:
        /* bit0: clock valid/locked (model) */
        value = s->cgm_enabled ? 0x1u : 0x0u;
        break;
    default:
        break;
    }

    qemu_log_mask(LOG_UNIMP,
                  "spc58e:mc_cgm rd addr=0x%08" HWADDR_PRIx " size=%u -> 0x%08x\n",
                  addr, size, value);
    return value;
}

static void spc58e_mc_cgm_write(void *opaque, hwaddr addr, uint64_t data,
                                unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;
    uint32_t value = (uint32_t)data;

    switch (addr) {
    case SPC58_MC_CGM_CTRL:
        s->cgm_enabled = (value & 0x1u) ? 1u : 0u;
        break;
    case SPC58_MC_CGM_DIV:
        /* keep a small divider range to avoid unrealistic values */
        value &= 0x0fu;
        if (value == 0) {
            value = 1;
        }
        break;
    default:
        break;
    }

    if (idx < ARRAY_SIZE(s->mc_cgm_regs)) {
        s->mc_cgm_regs[idx] = value;
    }

    spc58e_cgm_recompute_clocks(s);

    qemu_log_mask(LOG_UNIMP,
                  "spc58e:mc_cgm wr addr=0x%08" HWADDR_PRIx " size=%u val=0x%08" PRIx64 "\n",
                  addr, size, data);
}

static const MemoryRegionOps spc58e_mc_cgm_ops = {
    .read = spc58e_mc_cgm_read,
    .write = spc58e_mc_cgm_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t spc58e_flashc_read(void *opaque, hwaddr addr, unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;
    uint32_t value = (idx < ARRAY_SIZE(s->flashc_regs)) ? s->flashc_regs[idx] : 0;

    switch (addr) {
    case SPC58_FLASHC_MCRS:
        /* bit0: DONE, bit1: PGM/erase busy */
        value = s->flashc_busy ? 0x2u : 0x1u;
        break;
    default:
        break;
    }

    qemu_log_mask(LOG_UNIMP,
                  "spc58e:flashc rd addr=0x%08" HWADDR_PRIx " size=%u -> 0x%08x\n",
                  addr, size, value);
    return value;
}

static void spc58e_flashc_write(void *opaque, hwaddr addr, uint64_t data,
                                unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;
    uint32_t value = (uint32_t)data;

    switch (addr) {
    case SPC58_FLASHC_CMD:
        /* model command execution as immediate completion */
        s->flashc_busy = 1;
        s->flashc_busy = 0;
        break;
    default:
        break;
    }

    if (idx < ARRAY_SIZE(s->flashc_regs)) {
        s->flashc_regs[idx] = value;
    }

    qemu_log_mask(LOG_UNIMP,
                  "spc58e:flashc wr addr=0x%08" HWADDR_PRIx " size=%u val=0x%08" PRIx64 "\n",
                  addr, size, data);
}

static const MemoryRegionOps spc58e_flashc_ops = {
    .read = spc58e_flashc_read,
    .write = spc58e_flashc_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t spc58e_pit_read(void *opaque, hwaddr addr, unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;
    uint32_t value = (idx < ARRAY_SIZE(s->pit_regs)) ? s->pit_regs[idx] : 0;

    spc58e_pit_update(s);
    qemu_log_mask(LOG_UNIMP,
                  "spc58e:pit rd addr=0x%08" HWADDR_PRIx " size=%u -> 0x%08x\n",
                  addr, size, value);
    return value;
}

static void spc58e_pit_write(void *opaque, hwaddr addr, uint64_t data,
                             unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;
    uint32_t value = (uint32_t)data;

    spc58e_pit_update(s);
    if (idx < ARRAY_SIZE(s->pit_regs)) {
        s->pit_regs[idx] = value;
    }

    if ((addr & (SPC58_PIT_CH_STRIDE - 1u)) == SPC58_PIT_TFLG_OFF && (value & 0x1u)) {
        uint32_t ch = (addr - SPC58_PIT_CH_BASE) / SPC58_PIT_CH_STRIDE;
        if (ch < SPC58_PIT_NUM_CH) {
            spc58e_intc_clear_pending(s, SPC58_PIT_IRQ_BASE + ch);
            spc58e_intc_recompute_irq(s);
        }
    }

    qemu_log_mask(LOG_UNIMP,
                  "spc58e:pit wr addr=0x%08" HWADDR_PRIx " size=%u val=0x%08" PRIx64 "\n",
                  addr, size, data);
}

static const MemoryRegionOps spc58e_pit_ops = {
    .read = spc58e_pit_read,
    .write = spc58e_pit_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t spc58e_stm_read(void *opaque, hwaddr addr, unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;
    uint32_t value = (idx < ARRAY_SIZE(s->stm_regs)) ? s->stm_regs[idx] : 0;

    spc58e_stm_update(s);
    qemu_log_mask(LOG_UNIMP,
                  "spc58e:stm rd addr=0x%08" HWADDR_PRIx " size=%u -> 0x%08x\n",
                  addr, size, value);
    return value;
}

static void spc58e_stm_write(void *opaque, hwaddr addr, uint64_t data,
                             unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;
    uint32_t value = (uint32_t)data;

    spc58e_stm_update(s);
    if (idx < ARRAY_SIZE(s->stm_regs)) {
        s->stm_regs[idx] = value;
    }

    if (addr == SPC58_STM_SR && (value & 0x1u)) {
        s->stm_regs[SPC58_STM_SR >> 2] &= ~0x1u;
        spc58e_intc_clear_pending(s, SPC58_STM_IRQ_ID);
        spc58e_intc_recompute_irq(s);
    }

    qemu_log_mask(LOG_UNIMP,
                  "spc58e:stm wr addr=0x%08" HWADDR_PRIx " size=%u val=0x%08" PRIx64 "\n",
                  addr, size, data);
}

static const MemoryRegionOps spc58e_stm_ops = {
    .read = spc58e_stm_read,
    .write = spc58e_stm_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t spc58e_valid_read(void *opaque, hwaddr addr, unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;
    uint32_t value = (idx < ARRAY_SIZE(s->valid_regs)) ? s->valid_regs[idx] : 0;

    qemu_log_mask(LOG_UNIMP,
                  "spc58e:valid rd addr=0x%08" HWADDR_PRIx " size=%u -> 0x%08x\n",
                  addr, size, value);
    return value;
}

static void spc58e_valid_write(void *opaque, hwaddr addr, uint64_t data,
                               unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;
    uint32_t value = (uint32_t)data;

    if (idx < ARRAY_SIZE(s->valid_regs)) {
        s->valid_regs[idx] = value;
    }

    qemu_log_mask(LOG_UNIMP,
                  "spc58e:valid wr addr=0x%08" HWADDR_PRIx " size=%u val=0x%08" PRIx64 "\n",
                  addr, size, data);
}

static const MemoryRegionOps spc58e_valid_ops = {
    .read = spc58e_valid_read,
    .write = spc58e_valid_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static uint64_t spc58e_swt_read(void *opaque, hwaddr addr, unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;
    uint32_t value = (idx < ARRAY_SIZE(s->swt_regs)) ? s->swt_regs[idx] : 0;

    spc58e_swt_update_counter(s);

    switch (addr) {
    case SPC58_SWT_CR:
        value = s->swt_enabled ? 0x1u : 0x0u;
        break;
    case SPC58_SWT_TO:
        value = s->swt_reload;
        break;
    case SPC58_SWT_CNT:
        value = s->swt_counter;
        break;
    default:
        break;
    }

    qemu_log_mask(LOG_UNIMP,
                  "spc58e:swt rd addr=0x%08" HWADDR_PRIx " size=%u -> 0x%08x\n",
                  addr, size, value);
    return value;
}

static void spc58e_swt_write(void *opaque, hwaddr addr, uint64_t data,
                             unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;
    uint32_t value = (uint32_t)data;

    spc58e_swt_update_counter(s);

    switch (addr) {
    case SPC58_SWT_CR:
        s->swt_enabled = (value & 0x1u) ? 1u : 0u;
        if (s->swt_enabled && s->swt_counter == 0) {
            s->swt_counter = s->swt_reload;
        }
        break;
    case SPC58_SWT_TO:
        s->swt_reload = value == 0 ? 1u : value;
        if (!s->swt_enabled) {
            s->swt_counter = s->swt_reload;
        }
        break;
    case SPC58_SWT_SR:
        /* service sequence: 0xA602 then 0xB480 */
        if (s->swt_service_step == 0 && value == 0xA602u) {
            s->swt_service_step = 1;
        } else if (s->swt_service_step == 1 && value == 0xB480u) {
            s->swt_counter = s->swt_reload;
            s->swt_service_step = 0;
            spc58e_intc_clear_pending(s, SPC58_SWT_IRQ_ID);
            spc58e_intc_recompute_irq(s);
        } else {
            s->swt_service_step = 0;
        }
        break;
    default:
        break;
    }

    if (idx < ARRAY_SIZE(s->swt_regs)) {
        s->swt_regs[idx] = value;
    }

    qemu_log_mask(LOG_UNIMP,
                  "spc58e:swt wr addr=0x%08" HWADDR_PRIx " size=%u val=0x%08" PRIx64 "\n",
                  addr, size, data);
}

static const MemoryRegionOps spc58e_swt_ops = {
    .read = spc58e_swt_read,
    .write = spc58e_swt_write,
    .endianness = DEVICE_BIG_ENDIAN,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

/*
 * Custom ELF loader: loads PT_LOAD segments at their VirtAddr (p_vaddr).
 *
 * The Honda MasterMcu_Fbl.elf has all PhysAddr=0x0 but correct VirtAddr
 * (e.g. 0xfc0000 for flash). QEMU's load_elf() uses PhysAddr, so we must
 * parse the segments ourselves.
 *
 * Returns the ELF entry point on success, 0 on failure.
 */
static uint64_t spc58e_load_elf_vma(const char *filename)
{
    int fd;
    Elf32_Ehdr ehdr;
    Elf32_Phdr *phdrs = NULL;
    uint8_t *seg_data = NULL;
    uint64_t entry = 0;
    int i;

    fd = open(filename, O_RDONLY | O_BINARY);
    if (fd < 0) {
        error_report("spc58e: cannot open '%s': %s", filename, strerror(errno));
        return 0;
    }

    if (read(fd, &ehdr, sizeof(ehdr)) != sizeof(ehdr)) {
        error_report("spc58e: failed to read ELF header");
        goto out;
    }
    if (memcmp(ehdr.e_ident, ELFMAG, SELFMAG) != 0) {
        error_report("spc58e: not an ELF file");
        goto out;
    }

    /* Convert from big-endian (PowerPC) */
    bswap32s(&ehdr.e_entry);
    bswap32s((uint32_t *)&ehdr.e_phoff);
    bswap16s(&ehdr.e_phentsize);
    bswap16s(&ehdr.e_phnum);

    entry = ehdr.e_entry;

    size_t phdr_sz = (size_t)ehdr.e_phentsize * ehdr.e_phnum;
    phdrs = g_malloc(phdr_sz);

    if (lseek(fd, ehdr.e_phoff, SEEK_SET) < 0 ||
        read(fd, phdrs, phdr_sz) != (ssize_t)phdr_sz) {
        error_report("spc58e: failed to read program headers");
        entry = 0;
        goto out;
    }

    for (i = 0; i < ehdr.e_phnum; i++) {
        Elf32_Phdr *ph = &phdrs[i];
        bswap32s(&ph->p_type);
        bswap32s(&ph->p_offset);
        bswap32s(&ph->p_vaddr);
        bswap32s(&ph->p_paddr);
        bswap32s(&ph->p_filesz);
        bswap32s(&ph->p_memsz);
        bswap32s(&ph->p_flags);
        bswap32s(&ph->p_align);

        if (ph->p_type != PT_LOAD || ph->p_filesz == 0) {
            continue;
        }

        seg_data = g_malloc(ph->p_filesz);
        if (lseek(fd, ph->p_offset, SEEK_SET) < 0 ||
            read(fd, seg_data, ph->p_filesz) != (ssize_t)ph->p_filesz) {
            error_report("spc58e: failed to read segment %d", i);
            g_free(seg_data);
            seg_data = NULL;
            entry = 0;
            goto out;
        }

        cpu_physical_memory_write(ph->p_vaddr, seg_data, ph->p_filesz);
        error_report("spc58e: seg[%d] vaddr=0x%08x size=0x%x loaded", i,
                     ph->p_vaddr, ph->p_filesz);

        g_free(seg_data);
        seg_data = NULL;
    }

out:
    g_free(phdrs);
    close(fd);
    return entry;
}

static void spc58e_map_memories(void)
{
    MemoryRegion *sysmem = get_system_memory();

    memory_region_init_ram(&spc58e.boot_flash, NULL, "spc58e.boot_flash",
                           SPC58_BOOT_FLASH_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, SPC58_BOOT_FLASH_BASE, &spc58e.boot_flash);

    memory_region_init_ram(&spc58e.sys_sram, NULL, "spc58e.sys_sram",
                           SPC58_SYS_SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, SPC58_SYS_SRAM_BASE, &spc58e.sys_sram);

    memory_region_init_ram(&spc58e.core2_sram, NULL, "spc58e.core2_sram",
                           SPC58_CORE2_SRAM_SIZE, &error_fatal);
    memory_region_add_subregion(sysmem, SPC58_CORE2_SRAM_BASE, &spc58e.core2_sram);

    memory_region_init_io(&spc58e.intc_mmio, NULL, &spc58e_intc_ops, &spc58e,
                          "spc58e.intc", SPC58_INTC_SIZE);
    memory_region_add_subregion(sysmem, SPC58_INTC_BASE, &spc58e.intc_mmio);

    memory_region_init_io(&spc58e.mc_cgm_mmio, NULL, &spc58e_mc_cgm_ops, &spc58e,
                          "spc58e.mc_cgm", SPC58_MC_CGM_SIZE);
    memory_region_add_subregion(sysmem, SPC58_MC_CGM_BASE, &spc58e.mc_cgm_mmio);

    memory_region_init_io(&spc58e.flashc_mmio, NULL, &spc58e_flashc_ops, &spc58e,
                          "spc58e.flashc", SPC58_FLASHC_SIZE);
    memory_region_add_subregion(sysmem, SPC58_FLASHC_BASE, &spc58e.flashc_mmio);

    memory_region_init_io(&spc58e.pit_mmio, NULL, &spc58e_pit_ops, &spc58e,
                          "spc58e.pit", SPC58_PIT_SIZE);
    memory_region_add_subregion(sysmem, SPC58_PIT_BASE, &spc58e.pit_mmio);

    memory_region_init_io(&spc58e.stm_mmio, NULL, &spc58e_stm_ops, &spc58e,
                          "spc58e.stm", SPC58_STM_SIZE);
    memory_region_add_subregion(sysmem, SPC58_STM_BASE, &spc58e.stm_mmio);

    memory_region_init_io(&spc58e.valid_mmio, NULL, &spc58e_valid_ops, &spc58e,
                          "spc58e.valid", SPC58_VALID_SIZE);
    memory_region_add_subregion(sysmem, SPC58_VALID_BASE, &spc58e.valid_mmio);

    memory_region_init_io(&spc58e.swt_mmio, NULL, &spc58e_swt_ops, &spc58e,
                          "spc58e.swt", SPC58_SWT_SIZE);
    memory_region_add_subregion(sysmem, SPC58_SWT_BASE, &spc58e.swt_mmio);
}

static void spc58e_load_firmware(MachineState *machine, CPUPPCState *env)
{
    uint64_t entry = 0;

    if (!machine->kernel_filename) {
        error_report("spc58e: no firmware provided, use -kernel <MasterMcu_Fbl.elf>");
        return;
    }

    /*
     * Honda MasterMcu_Fbl.elf has all PhysAddr=0x0 but correct VirtAddr.
     * Use custom VMA-based loader instead of load_elf() which uses PhysAddr.
     */
    entry = spc58e_load_elf_vma(machine->kernel_filename);
    if (!entry) {
        /* Fallback: try raw binary at flash base */
        hwaddr raw = load_image_targphys(machine->kernel_filename,
                                         SPC58_BOOT_FLASH_BASE,
                                         SPC58_BOOT_FLASH_SIZE);
        if (raw == (hwaddr)-1) {
            error_report("spc58e: failed to load firmware '%s'", machine->kernel_filename);
            exit(EXIT_FAILURE);
        }
        entry = SPC58_BOOT_FLASH_BASE;
        error_report("spc58e: raw image loaded at 0x%08x (%" PRIu64 " bytes)",
                     SPC58_BOOT_FLASH_BASE, (uint64_t)raw);
    }

    env->nip = entry;
    error_report("spc58e: entry=0x%" PRIx64, entry);
    spc58e.valid_regs[SPC58_VALID_ENTRY_LO >> 2] = (uint32_t)entry;
    spc58e.valid_regs[SPC58_VALID_FLAGS >> 2] |= 0x1u; /* entry loaded */
    spc58e.valid_regs[SPC58_VALID_LAST_EVENT >> 2] = 0x454e5452u; /* ENTR */
}

static void spc58e_machine_init(MachineState *machine)
{
    PowerPCCPU *cpu = POWERPC_CPU(cpu_create(machine->cpu_type));
    CPUPPCState *env = &cpu->env;

    spc58e.cpu = cpu;
    spc58e.intc_enabled = 1;
    spc58e.intc_current_irq = -1;
    spc58e.cgm_enabled = 1;
    spc58e.mc_cgm_regs[SPC58_MC_CGM_DIV >> 2] = 1;
    spc58e_cgm_recompute_clocks(&spc58e);
    spc58e.swt_enabled = 1;
    spc58e.swt_reload = 50000u;
    spc58e.swt_counter = spc58e.swt_reload;
    spc58e.swt_service_step = 0;
    spc58e.swt_last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    spc58e.flashc_busy = 0;
    spc58e.pit_last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    spc58e.stm_last_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    spc58e.stm_regs[SPC58_STM_CMP0 >> 2] = 10000u;
    spc58e.valid_regs[SPC58_VALID_SIGNATURE >> 2] = SPC58_VALID_MAGIC;
    spc58e.valid_regs[SPC58_VALID_FLAGS >> 2] = 0x0u;
    spc58e.valid_regs[SPC58_VALID_LAST_EVENT >> 2] = 0x424f4f54u; /* BOOT */

    cpu_ppc_tb_init(env, 160000000UL);

    spc58e_map_memories();
    spc58e_load_firmware(machine, env);

    /*
     * Pre-populate TLB1[0] to cover the boot flash region.
     *
     * QEMU's BOOKE206 debug accessor (ppc_cpu_get_phys_page_debug) walks
     * the software TLB — it has no real-mode bypass.  Without at least one
     * valid TLB entry GDB cannot read memory before the firmware sets up
     * its own TLB entries.
     *
     * Entry: 0xfc0000-0xffffff, 256 KiB, supervisor R+X, not cached.
     */
    {
        ppcmas_tlb_t *tlb = booke206_get_tlbm(env, 1, SPC58_BOOT_FLASH_BASE, 0);
        if (tlb) {
            /* MAS1: valid, TID=0, TS=0, TSIZE=256KiB (tsize encoding = 9) */
            tlb->mas1 = MAS1_VALID | (9u << MAS1_TSIZE_SHIFT);
            /* MAS2: EPN = flash base, cache-inhibited + guarded */
            tlb->mas2 = (SPC58_BOOT_FLASH_BASE & MAS2_EPN_MASK) | 0x18u;
            /* MAS7:MAS3: RPN = flash base, supervisor R+X */
            tlb->mas7_3 = (SPC58_BOOT_FLASH_BASE & MAS3_RPN_MASK)
                          | MAS3_SR | MAS3_SX;
        }
    }

    error_report("spc58e: flash @0x%08"PRIx64" size=0x%"PRIx64, (uint64_t)SPC58_BOOT_FLASH_BASE, (uint64_t)SPC58_BOOT_FLASH_SIZE);
    error_report("spc58e: sys   @0x%08"PRIx64" size=0x%"PRIx64, (uint64_t)SPC58_SYS_SRAM_BASE, (uint64_t)SPC58_SYS_SRAM_SIZE);
    error_report("spc58e: c2ram @0x%08"PRIx64" size=0x%"PRIx64, (uint64_t)SPC58_CORE2_SRAM_BASE, (uint64_t)SPC58_CORE2_SRAM_SIZE);
    error_report("spc58e: intc  @0x%08"PRIx64" size=0x%"PRIx64, (uint64_t)SPC58_INTC_BASE, (uint64_t)SPC58_INTC_SIZE);
    error_report("spc58e: cgm   @0x%08"PRIx64" size=0x%"PRIx64, (uint64_t)SPC58_MC_CGM_BASE, (uint64_t)SPC58_MC_CGM_SIZE);
    error_report("spc58e: flash @0x%08"PRIx64" size=0x%"PRIx64, (uint64_t)SPC58_FLASHC_BASE, (uint64_t)SPC58_FLASHC_SIZE);
    error_report("spc58e: pit   @0x%08"PRIx64" size=0x%"PRIx64, (uint64_t)SPC58_PIT_BASE, (uint64_t)SPC58_PIT_SIZE);
    error_report("spc58e: stm   @0x%08"PRIx64" size=0x%"PRIx64, (uint64_t)SPC58_STM_BASE, (uint64_t)SPC58_STM_SIZE);
    error_report("spc58e: valid @0x%08"PRIx64" size=0x%"PRIx64, (uint64_t)SPC58_VALID_BASE, (uint64_t)SPC58_VALID_SIZE);
    error_report("spc58e: swt   @0x%08"PRIx64" size=0x%"PRIx64, (uint64_t)SPC58_SWT_BASE, (uint64_t)SPC58_SWT_SIZE);
    error_report("spc58e: clock core=%uHz periph=%uHz", spc58e.cgm_core_hz, spc58e.cgm_periph_hz);
}

static void spc58e_machine_class_init(MachineClass *mc)
{
    mc->desc = "SPC58E bring-up machine";
    mc->init = spc58e_machine_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = POWERPC_CPU_TYPE_NAME("e200z6");
    mc->default_ram_id = "spc58e.ram";
}

DEFINE_MACHINE("spc58e", spc58e_machine_class_init)
