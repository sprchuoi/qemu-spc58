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

typedef struct SPC58EState {
    MemoryRegion boot_flash;
    MemoryRegion sys_sram;
    MemoryRegion core2_sram;
    MemoryRegion intc_mmio;
    uint32_t intc_regs[SPC58_INTC_SIZE / sizeof(uint32_t)];
} SPC58EState;

static SPC58EState spc58e;

static uint64_t spc58e_intc_read(void *opaque, hwaddr addr, unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;
    uint32_t value = (idx < ARRAY_SIZE(s->intc_regs)) ? s->intc_regs[idx] : 0;

    qemu_log_mask(LOG_UNIMP,
                  "spc58e:intc rd addr=0x%08" HWADDR_PRIx " size=%u -> 0x%08x\n",
                  addr, size, value);
    return value;
}

static void spc58e_intc_write(void *opaque, hwaddr addr, uint64_t data,
                              unsigned size)
{
    SPC58EState *s = opaque;
    uint32_t idx = addr >> 2;

    if (idx < ARRAY_SIZE(s->intc_regs)) {
        s->intc_regs[idx] = (uint32_t)data;
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

    cpu_ppc_tb_init(env, 160000000UL);

    spc58e_map_memories();
    spc58e_load_firmware(machine, env);

    error_report("spc58e: flash @0x%08x size=0x%x", SPC58_BOOT_FLASH_BASE, SPC58_BOOT_FLASH_SIZE);
    error_report("spc58e: sys   @0x%08x size=0x%x", SPC58_SYS_SRAM_BASE, SPC58_SYS_SRAM_SIZE);
    error_report("spc58e: c2ram @0x%08x size=0x%x", SPC58_CORE2_SRAM_BASE, SPC58_CORE2_SRAM_SIZE);
    error_report("spc58e: intc  @0x%08x size=0x%x", SPC58_INTC_BASE, SPC58_INTC_SIZE);
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
