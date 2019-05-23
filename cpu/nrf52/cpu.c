/*
 * Copyright (C) 2015 Jan Wagner <mail@jwagner.eu>
 *               2016 Freie Universität Berlin
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
 * @brief       Implementation of the CPU initialization
 *
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Jan Wagner <mail@jwagner.eu>
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 *
 * @}
 */

#define DONT_OVERRIDE_NVIC

#include "cpu.h"
#include "nrf_clock.h"
#include "periph_conf.h"
#include "periph/init.h"

static bool errata_12(void);
static bool errata_16(void);
static bool errata_31(void);
static bool errata_32(void);
static bool errata_36(void);
static bool errata_37(void);
static bool errata_57(void);
static bool errata_66(void);
static bool errata_108(void);
static bool errata_136(void);
static bool errata_182(void);

#ifdef SOFTDEVICE_PRESENT
#include "softdevice_handler.h"
uint8_t _ble_evt_buffer[BLE_STACK_EVT_MSG_BUF_SIZE];
#endif

/**
 * @brief   Initialize the CPU, set IRQ priorities
 */
void cpu_init(void)
{
    /* Workaround for Errata 12 "COMP: Reference ladder not correctly calibrated" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/ */
    if (errata_12()){
        *(volatile uint32_t *)0x40013540 = (*(uint32_t *)0x10000324 & 0x00001F00) >> 8;
    }
    
    /* Workaround for Errata 16 "System: RAM may be corrupt on wakeup from CPU IDLE" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/ */
    if (errata_16()){
        *(volatile uint32_t *)0x4007C074 = 3131961357ul;
    }

    /* Workaround for Errata 31 "CLOCK: Calibration values are not correctly loaded from FICR at reset" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/ */
    if (errata_31()){
        *(volatile uint32_t *)0x4000053C = ((*(volatile uint32_t *)0x10000244) & 0x0000E000) >> 13;
    }

    /* Workaround for Errata 32 "DIF: Debug session automatically enables TracePort pins" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/ */
    if (errata_32()){
        CoreDebug->DEMCR &= ~CoreDebug_DEMCR_TRCENA_Msk;
    }

    /* Workaround for Errata 36 "CLOCK: Some registers are not reset when expected" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/  */
    if (errata_36()){
        NRF_CLOCK->EVENTS_DONE = 0;
        NRF_CLOCK->EVENTS_CTTO = 0;
        NRF_CLOCK->CTIV = 0;
    }

    /* Workaround for Errata 37 "RADIO: Encryption engine is slow by default" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/  */
    if (errata_37()){
        *(volatile uint32_t *)0x400005A0 = 0x3;
    }

    /* Workaround for Errata 57 "NFCT: NFC Modulation amplitude" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/  */
    if (errata_57()){
        *(volatile uint32_t *)0x40005610 = 0x00000005;
        *(volatile uint32_t *)0x40005688 = 0x00000001;
        *(volatile uint32_t *)0x40005618 = 0x00000000;
        *(volatile uint32_t *)0x40005614 = 0x0000003F;
    }

    /* Workaround for Errata 66 "TEMP: Linearity specification not met with default settings" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/  */
    if (errata_66()){
        NRF_TEMP->A0 = NRF_FICR->TEMP.A0;
        NRF_TEMP->A1 = NRF_FICR->TEMP.A1;
        NRF_TEMP->A2 = NRF_FICR->TEMP.A2;
        NRF_TEMP->A3 = NRF_FICR->TEMP.A3;
        NRF_TEMP->A4 = NRF_FICR->TEMP.A4;
        NRF_TEMP->A5 = NRF_FICR->TEMP.A5;
        NRF_TEMP->B0 = NRF_FICR->TEMP.B0;
        NRF_TEMP->B1 = NRF_FICR->TEMP.B1;
        NRF_TEMP->B2 = NRF_FICR->TEMP.B2;
        NRF_TEMP->B3 = NRF_FICR->TEMP.B3;
        NRF_TEMP->B4 = NRF_FICR->TEMP.B4;
        NRF_TEMP->B5 = NRF_FICR->TEMP.B5;
        NRF_TEMP->T0 = NRF_FICR->TEMP.T0;
        NRF_TEMP->T1 = NRF_FICR->TEMP.T1;
        NRF_TEMP->T2 = NRF_FICR->TEMP.T2;
        NRF_TEMP->T3 = NRF_FICR->TEMP.T3;
        NRF_TEMP->T4 = NRF_FICR->TEMP.T4;
    }

    /* Workaround for Errata 108 "RAM: RAM content cannot be trusted upon waking up from System ON Idle or System OFF mode" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/  */
    if (errata_108()){
        *(volatile uint32_t *)0x40000EE4 = *(volatile uint32_t *)0x10000258 & 0x0000004F;
    }
    
    /* Workaround for Errata 136 "System: Bits in RESETREAS are set when they should not be" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/  */
    if (errata_136()){
        if (NRF_POWER->RESETREAS & POWER_RESETREAS_RESETPIN_Msk){
            NRF_POWER->RESETREAS =  ~POWER_RESETREAS_RESETPIN_Msk;
        }
    }
    
    /* Workaround for Errata 182 "RADIO: Fixes for anomalies #102, #106, and #107 do not take effect" found at the Errata document
       for your device located at https://infocenter.nordicsemi.com/  */
    if (errata_182()){
        *(volatile uint32_t *) 0x4000173C |= (0x1 << 10);
    }
	
	/* Configure GPIO pads as pPin Reset pin if Pin Reset capabilities desired. If CONFIG_GPIO_AS_PINRESET is not
      defined, pin reset will not be available. One GPIO (see Product Specification to see which one) will then be
      reserved for PinReset and not available as normal GPIO. */
    #if defined (CONFIG_GPIO_AS_PINRESET)
        if (((NRF_UICR->PSELRESET[0] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos)) ||
            ((NRF_UICR->PSELRESET[1] & UICR_PSELRESET_CONNECT_Msk) != (UICR_PSELRESET_CONNECT_Connected << UICR_PSELRESET_CONNECT_Pos))){
            NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
            NRF_UICR->PSELRESET[0] = 21;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
            NRF_UICR->PSELRESET[1] = 21;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
            NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos;
            while (NRF_NVMC->READY == NVMC_READY_READY_Busy){}
            NVIC_SystemReset();
        }
    #endif
	
    /* initialize hf clock */
    clock_init_hf();

    /* enable instruction cache */
    NRF_NVMC->ICACHECNF = (NVMC_ICACHECNF_CACHEEN_Msk);

    /* softdevice needs to be enabled from ISR context */
#ifdef SOFTDEVICE_PRESENT
    softdevice_handler_init(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM, &_ble_evt_buffer,
            BLE_STACK_EVT_MSG_BUF_SIZE, NULL);

    /* fixup swi0 (used as softdevice PendSV trampoline) */
    NVIC_EnableIRQ(SWI0_EGU0_IRQn);
    NVIC_SetPriority(SWI0_EGU0_IRQn, 6);
#else
    /* call cortexm default initialization */
    cortexm_init();
#endif

    /* enable wake up on events for __WFE CPU sleep */
    SCB->SCR |= SCB_SCR_SEVONPEND_Msk;

    /* trigger static peripheral initialization */
    periph_init();
    
    /* fill CPU status data */
    cpu_init_status();
}

static bool errata_12(void)
{
    if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0)){
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30){
            return true;
        }
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x40){
            return true;
        }
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x50){
            return true;
        }
    }

    return false;
}

static bool errata_16(void)
{
    if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0)){
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30){
            return true;
        }
    }

    return false;
}

static bool errata_31(void)
{
    if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0)){
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30){
            return true;
        }
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x40){
            return true;
        }
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x50){
            return true;
        }
    }

    return false;
}

static bool errata_32(void)
{
    if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0)){
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30){
            return true;
        }
    }

    return false;
}

static bool errata_36(void)
{
    if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0)){
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30){
            return true;
        }
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x40){
            return true;
        }
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x50){
            return true;
        }
    }

    return false;
}

static bool errata_37(void)
{
    if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0)){
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30){
            return true;
        }
    }

    return false;
}

static bool errata_57(void)
{
    if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0)){
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30){
            return true;
        }
    }

    return false;
}

static bool errata_66(void)
{
    if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0)){
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x50){
            return true;
        }
    }

    return false;
}


static bool errata_108(void)
{
    if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0)){
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30){
            return true;
        }
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x40){
            return true;
        }
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x50){
            return true;
        }
    }

    return false;
}


static bool errata_136(void)
{
    if ((((*(uint32_t *)0xF0000FE0) & 0x000000FF) == 0x6) && (((*(uint32_t *)0xF0000FE4) & 0x0000000F) == 0x0)){
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x30){
            return true;
        }
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x40){
            return true;
        }
        if (((*(uint32_t *)0xF0000FE8) & 0x000000F0) == 0x50){
            return true;
        }
    }

    return false;
}


static bool errata_182(void)
{
    if (*(uint32_t *)0x10000130ul == 0x6ul){
        if (*(uint32_t *)0x10000134ul == 0x6ul){
            return true;
        }
    }

    return false;
}