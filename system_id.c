/*
 * system_id.c
 *
 *  Created on: 4 авг. 2017 г.
 *      Author: frost
 */

#include <stdint.h>

#include "system_id.h"

#include "libopencm3_headers.h"

#include "FreeRTOS.h"
#include "task.h"

void system_id_get_cpuid(sysinfo_cpuinfo_t* sysinfo_cpuid_ptr)
{
    sysinfo_cpuid_ptr->uid0 = DESIG_UNIQUE_ID0;
    sysinfo_cpuid_ptr->uid1 = DESIG_UNIQUE_ID1;
    sysinfo_cpuid_ptr->uid2 = DESIG_UNIQUE_ID2;

    sysinfo_cpuid_ptr->cpuid_reg = SCB_CPUID;
}

sysinfo_firmware_version_t* system_id_get_fw_version(const sysinfo_firmware_version_t *sysinfo_struct)
{
    return  &sysinfo_struct;
}

