/*
 * SPC58E machine skeleton for incremental emulation bring-up.
 */

#include "qemu/osdep.h"
#include "qemu/error-report.h"
#include "hw/core/boards.h"

static void spc58e_machine_init(MachineState *machine)
{
    error_report("spc58e: machine skeleton loaded (no memory/peripherals yet)");
}

static void spc58e_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "SPC58E skeleton machine";
    mc->init = spc58e_machine_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = POWERPC_CPU_TYPE_NAME("e200z6");
    mc->default_ram_id = "spc58e.ram";
}

DEFINE_MACHINE("spc58e", spc58e_machine_class_init)
