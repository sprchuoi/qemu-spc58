/*
 * SPC58E machine skeleton for incremental emulation bring-up.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "qemu/log.h"
#include "qemu/units.h"
#include "hw/core/boards.h"
#include "hw/core/loader.h"
#include "hw/ppc/ppc.h"
#include "exec/address-spaces.h"
#include "exec/memory.h"
#include "elf.h"

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

#define SPC58_SWT_BASE         0xfff38000u
#define SPC58_SWT_SIZE         0x4000u

typedef struct SPC58EState {
    PowerPCCPU *cpu;
    MemoryRegion boot_flash;
    MemoryRegion sys_sram;
    MemoryRegion core2_sram;
    MemoryRegion intc_mmio;
    MemoryRegion mc_cgm_mmio;
    MemoryRegion swt_mmio;
    uint32_t intc_regs[SPC58_INTC_SIZE / sizeof(uint32_t)];
    uint32_t intc_pending[SPC58_INTC_NUM_IRQS / 32u];
    uint32_t mc_cgm_regs[SPC58_MC_CGM_SIZE / sizeof(uint32_t)];
    uint32_t swt_regs[SPC58_SWT_SIZE / sizeof(uint32_t)];
    uint8_t intc_enabled;
    int32_t intc_current_irq;
    uint8_t cgm_enabled;
    uint32_t cgm_core_hz;
    uint32_t cgm_periph_hz;
} SPC58EState;

static SPC58EState spc58e;

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

static uint64_t spc58e_swt_read(void *opaque, hwaddr addr, unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;
    uint32_t value = (idx < ARRAY_SIZE(s->swt_regs)) ? s->swt_regs[idx] : 0;

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

    if (idx < ARRAY_SIZE(s->swt_regs)) {
        s->swt_regs[idx] = (uint32_t)data;
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

static uint64_t spc58e_translate_elf(void *opaque, uint64_t addr)
{
    return addr;
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

    memory_region_init_io(&spc58e.swt_mmio, NULL, &spc58e_swt_ops, &spc58e,
                          "spc58e.swt", SPC58_SWT_SIZE);
    memory_region_add_subregion(sysmem, SPC58_SWT_BASE, &spc58e.swt_mmio);
}

static void spc58e_load_firmware(MachineState *machine, CPUPPCState *env)
{
    uint64_t entry = 0;
    uint64_t low = 0;
    uint64_t high = 0;

    if (!machine->kernel_filename) {
        error_report("spc58e: no firmware provided, use -kernel <MasterMcu_Fbl.elf>");
        return;
    }

    if (load_elf(machine->kernel_filename,
                 NULL,
                 spc58e_translate_elf,
                 NULL,
                 &entry,
                 &low,
                 &high,
                 NULL,
                 ELFDATA2MSB,
                 PPC_ELF_MACHINE,
                 0,
                 0) < 0) {
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
    } else {
        error_report("spc58e: ELF loaded low=0x%" PRIx64 " high=0x%" PRIx64,
                     low, high);
    }

    env->nip = entry;
    error_report("spc58e: entry=0x%" PRIx64, entry);
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

    cpu_ppc_tb_init(env, 160000000UL);

    spc58e_map_memories();
    spc58e_load_firmware(machine, env);

    error_report("spc58e: flash @0x%08x size=0x%x", SPC58_BOOT_FLASH_BASE, SPC58_BOOT_FLASH_SIZE);
    error_report("spc58e: sys   @0x%08x size=0x%x", SPC58_SYS_SRAM_BASE, SPC58_SYS_SRAM_SIZE);
    error_report("spc58e: c2ram @0x%08x size=0x%x", SPC58_CORE2_SRAM_BASE, SPC58_CORE2_SRAM_SIZE);
    error_report("spc58e: intc  @0x%08x size=0x%x", SPC58_INTC_BASE, SPC58_INTC_SIZE);
    error_report("spc58e: cgm   @0x%08x size=0x%x", SPC58_MC_CGM_BASE, SPC58_MC_CGM_SIZE);
    error_report("spc58e: swt   @0x%08x size=0x%x", SPC58_SWT_BASE, SPC58_SWT_SIZE);
    error_report("spc58e: clock core=%uHz periph=%uHz", spc58e.cgm_core_hz, spc58e.cgm_periph_hz);
}

static void spc58e_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "SPC58E bring-up machine";
    mc->init = spc58e_machine_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = POWERPC_CPU_TYPE_NAME("e200z6");
    mc->default_ram_id = "spc58e.ram";
}

DEFINE_MACHINE("spc58e", spc58e_machine_class_init)
