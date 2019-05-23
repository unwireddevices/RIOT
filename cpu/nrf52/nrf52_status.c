/*
 * Copyright (C) 2019 Unwired Devices LLC <info@unwds.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_nrf52
 * @{
 *
 * @file
 * @brief       Implementation of nRF52 CPU identification and status data
 *
 * @author      Oleg Artamonov <oleg@unwds.com>
 *
 * @}
 */

#if defined(MODULE_PERIPH_STATUS)
 
#include "cpu.h"
#include "periph_conf.h"
#include <string.h>

#if defined(MODULE_PERIPH_STATUS_EXTENDED)
    #include "periph/adc.h"
#endif

cpu_status_t cpu_status;

static size_t get_cpu_ram_size(void) {
    return 256*1024;
}

static size_t get_cpu_flash_size(void) {
    return 512*1024;
}

static size_t get_cpu_eeprom_size(void) {
    /* emulated */
    return EEPROM_SIZE;
}

#if defined(MODULE_PERIPH_STATUS_EXTENDED)
static void get_cpu_name(char *name) {
    snprintf(name, CPU_NAME_MAX_SIZE, "NRF52832");    
    return;
}
#endif /* MODULE_PERIPH_STATUS_EXTENDED */

static void get_cpu_flash(cpu_status_t* status) {
    status->flash.size = get_cpu_flash_size();
    status->flash.pagesize = 4096;
    status->flash.pages = status->flash.size / status->flash.pagesize;
    status->flash.alignment = 4;
    return;
}

static void get_cpu_ram(cpu_status_t* status) {
    status->ram.size = get_cpu_ram_size();
    status->ram.alignment = 4;
}

static void get_cpu_eeprom(cpu_status_t* status) {
    status->eeprom.size = get_cpu_eeprom_size();
    status->eeprom.alignment = 4;
}

#if defined(MODULE_PERIPH_STATUS_EXTENDED)
static void get_cpu_voltage(cpu_status_t* status) {
    status->voltage.vdd  = INT16_MIN;
    status->voltage.vdda = INT16_MIN;
    status->voltage.vbat = INT16_MIN;
}

static void get_cpu_temp(cpu_status_t* status) {
    /* Start temperature measurement task */
    NRF_TEMP->TASKS_START = 1;

    /* Wait for temperature measurement to be ready */
    while (!NRF_TEMP->EVENTS_DATARDY); /* takes 36us according to manual */

    /* temperature is in 0.25°C step, so just divide by 4 */
    status->temp.core_temp = (int16_t)NRF_TEMP->TEMP >> 2;

    /* Clear data ready bit and stop temperature measurement task */
    NRF_TEMP->EVENTS_DATARDY = 0;
    NRF_TEMP->TASKS_STOP = 1;
}
#endif

void cpu_init_status(void) {
    get_cpu_ram(&cpu_status);
    get_cpu_flash(&cpu_status);
    get_cpu_eeprom(&cpu_status);

#if defined(MODULE_PERIPH_STATUS_EXTENDED)
    cpu_status.voltage.vdd      = INT16_MIN;
    cpu_status.voltage.vdda     = INT16_MIN;
    cpu_status.voltage.vbat     = INT16_MIN;
    cpu_status.temp.core_temp   = INT16_MIN;
    get_cpu_name(cpu_status.model);
#endif
    
    cpu_status.category = 0;
}

#if defined(MODULE_PERIPH_STATUS_EXTENDED)
void cpu_update_status(void) {
    get_cpu_voltage(&cpu_status);
    get_cpu_temp(&cpu_status);
}
#endif

#endif /* MODULE_PERIPH_STATUS */