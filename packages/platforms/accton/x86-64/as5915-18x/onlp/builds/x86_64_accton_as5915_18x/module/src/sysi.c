/************************************************************
 * <bsn.cl fy=2014 v=onl>
 *
 *           Copyright 2014 Big Switch Networks, Inc.
 *           Copyright 2014 Accton Technology Corporation.
 *
 * Licensed under the Eclipse Public License, Version 1.0 (the
 * "License"); you may not use this file except in compliance
 * with the License. You may obtain a copy of the License at
 *
 *        http://www.eclipse.org/legal/epl-v10.html
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the
 * License.
 *
 * </bsn.cl>
 ************************************************************
 *
 *
 *
 ***********************************************************/
#include <onlplib/file.h>
#include <onlp/platformi/sysi.h>
#include <onlp/platformi/ledi.h>
#include <onlp/platformi/thermali.h>
#include <onlp/platformi/fani.h>
#include <onlp/platformi/psui.h>
#include "platform_lib.h"

#include "x86_64_accton_as5915_18x_int.h"
#include "x86_64_accton_as5915_18x_log.h"

#define NUM_OF_CPLD_VER   6

static char* cpld_ver_path[] =
{
 "/sys/bus/i2c/devices/31-0064/pcb_version", /* Main FPGA */
 "/sys/bus/i2c/devices/31-0064/version",     /* Main FPGA */
 "/sys/bus/i2c/devices/32-0063/version",     /* Main CPLD */
 "/sys/bus/i2c/devices/32-0063/sub_version", /* Main CPLD */
 "/sys/bus/i2c/devices/32-0066/version",     /* Fan  CPLD */
 "/sys/bus/i2c/devices/32-0066/sub_version", /* Fan  CPLD */
};

const char*
onlp_sysi_platform_get(void)
{
    return "x86-64-accton-as5915-18x-r0";
}

int
onlp_sysi_onie_data_get(uint8_t** data, int* size)
{
    uint8_t* rdata = aim_zmalloc(256);
    if(onlp_file_read(rdata, 256, size, IDPROM_PATH) == ONLP_STATUS_OK) {
        if(*size == 256) {
            *data = rdata;
            return ONLP_STATUS_OK;
        }
    }

    aim_free(rdata);
    *size = 0;
    return ONLP_STATUS_E_INTERNAL;
}

int
onlp_sysi_oids_get(onlp_oid_t* table, int max)
{
    int i;
    onlp_oid_t* e = table;
    memset(table, 0, max*sizeof(onlp_oid_t));

    /* 8 Thermal sensors on the chassis */
    for (i = 1; i <= CHASSIS_THERMAL_COUNT; i++) {
        *e++ = ONLP_THERMAL_ID_CREATE(i);
    }

    /* 5 LEDs on the chassis */
    for (i = 1; i <= CHASSIS_LED_COUNT; i++) {
        *e++ = ONLP_LED_ID_CREATE(i);
    }

    /* 2 PSUs on the chassis */
    for (i = 1; i <= CHASSIS_PSU_COUNT; i++) {
        *e++ = ONLP_PSU_ID_CREATE(i);
    }

    /* 6 Fans on the chassis */
    for (i = 1; i <= CHASSIS_FAN_COUNT; i++) {
        *e++ = ONLP_FAN_ID_CREATE(i);
    }

    return 0;
}

int
onlp_sysi_platform_info_get(onlp_platform_info_t* pi)
{
    int   i, v[NUM_OF_CPLD_VER] = {0};

    for (i = 0; i < AIM_ARRAYSIZE(cpld_ver_path); i++) {
        v[i] = 0;

        if(onlp_file_read_int(v+i, cpld_ver_path[i]) < 0) {
            return ONLP_STATUS_E_INTERNAL;
        }
    }

    pi->cpld_versions = aim_fstrdup("\r\nPCB:%d\r\nMain FPGA:%d\r\nMain CPLD:%d.%d\r\nFan CPLD:%d.%d",
                                     v[0], v[1], v[2], v[3], v[4], v[5]);

    return ONLP_STATUS_OK;
}

void
onlp_sysi_platform_info_free(onlp_platform_info_t* pi)
{
    aim_free(pi->cpld_versions);
}
