/*
 * system_id.h
 *
 *  Created on: 4 авг. 2017 г.
 *      Author: frost
 */

#ifndef SYSTEM_ID_H_
#define SYSTEM_ID_H_

typedef struct
{
    int32_t revision : 4;
    int32_t partno  : 12;
    int32_t constant : 4;
    int32_t vaitant : 4;
    int32_t implementer : 8;
} cpuid_t;

typedef struct
{
    uint32_t uid0;
    uint32_t uid1;
    uint32_t uid2;

    union
    {
        cpuid_t cpuid;
        uint32_t cpuid_reg;
    };

} sysinfo_cpuinfo_t;


typedef struct
{
    const uint8_t firmware_version_major;
    const uint8_t firmware_version_minor;
    const uint16_t firmware_build_number;
    const uint32_t firmware_build_date;
    const char* firmware_name_string;

    const uint8_t freertos_version_major;
    const uint8_t freertos_version_minor;
    const uint8_t freertos_version_build;
    char* freertos_name_string;

} sysinfo_firmware_version_t;



void system_id_get_cpuid(sysinfo_cpuinfo_t* sysinfo_cpuid_ptr);

sysinfo_firmware_version_t* system_id_get_fw_version(const sysinfo_firmware_version_t *sysinfo_struct);

#endif /* SYSTEM_ID_H_ */
