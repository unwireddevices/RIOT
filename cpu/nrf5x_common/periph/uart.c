/*
 * Copyright (C) 2014-2017 Freie Universität Berlin
 *               2015 Jan Wagner <mail@jwagner.eu>
 *               2018 Inria
 *               2019 Unwired Devices LLC
 *
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     cpu_nrf5x_common
 * @ingroup     drivers_periph_uart
 * @{
 *
 * @file
 * @brief       Implementation of the peripheral UART interface
 *
 * @author      Christian Kühling <kuehling@zedat.fu-berlin.de>
 * @author      Timo Ziegler <timo.ziegler@fu-berlin.de>
 * @author      Hauke Petersen <hauke.petersen@fu-berlin.de>
 * @author      Jan Wagner <mail@jwagner.eu>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @author      Oleg Artamonov <oleg@unwds.com>
 * @author      Oleg Manchenko <manchenkoos@gmail.com>
 *
 * @}
 */

#include <stdint.h>

#include "cpu.h"
#include "periph/uart.h"
#include "periph/gpio.h"

#include "board.h"

#if defined(CPU_FAM_NRF52)
#define UART_INVALID    (uart >= UART_NUMOF)
#define REG_BAUDRATE    dev(uart)->BAUDRATE
#define REG_CONFIG      dev(uart)->CONFIG
#define PSEL_RXD        dev(uart)->PSEL.RXD
#define PSEL_TXD        dev(uart)->PSEL.TXD
#define UART_IRQN       uart_config[uart].irqn
#define UART_PIN_RX     uart_config[uart].rx_pin
#define UART_PIN_TX     uart_config[uart].tx_pin
#define UART_PIN_RXMODE uart_config[uart].rx_mode
#define UART_PIN_TXMODE uart_config[uart].tx_mode

#define UART_PIN_RTS    uart_config[uart].rts_pin
#define UART_PIN_CTS    uart_config[uart].cts_pin
#define UART_HWFLOWCTRL (uart_config[uart].rts_pin != (uint8_t)GPIO_UNDEF && \
                         uart_config[uart].cts_pin != (uint8_t)GPIO_UNDEF)
#define ISR_CTX         isr_ctx[uart]
#define MAX_LEN_FOR_SEND_VIA_EASY_DMA   (0xFF)

#if defined(MODULE_PERIPH_UART_DMA_TX)
#include <string.h>
#include "mutex.h"
#include "periph/pm.h"

static uint8_t data_tmp[PERIPH_UART_TX_BUFFER_SIZE];
static mutex_t uart_mtx = MUTEX_INIT;
#endif

/**
 * @brief Allocate memory for the interrupt context
 */
static uart_isr_ctx_t isr_ctx[UART_NUMOF];
static uint8_t rx_buf[UART_NUMOF];
static uint8_t uart_current;
static volatile bool blocking;

static inline NRF_UARTE_Type *dev(uart_t uart)
{
    return uart_config[uart].dev;
}

#else /* nrf51 */

#if defined(UART_NUMOF)
#define UART_INVALID    (uart >= UART_NUMOF)
#else
#define UART_INVALID    (uart != 0)
#endif

#define REG_BAUDRATE    NRF_UART0->BAUDRATE
#define REG_CONFIG      NRF_UART0->CONFIG
#define PSEL_RXD        NRF_UART0->PSELRXD
#define PSEL_TXD        NRF_UART0->PSELTXD

#if defined(UART_NUMOF)
#define UART_PIN_RX     uart_config[uart].rx_pin
#define UART_PIN_RXMODE uart_config[uart].rx_mode
#define UART_PIN_TX     uart_config[uart].tx_pin
#define UART_PIN_TXMODE uart_config[uart].tx_mode
#endif

#define UART_0_ISR      isr_uart0
#define ISR_CTX         isr_ctx

/**
 * @brief Allocate memory for the interrupt context
 */
static uart_isr_ctx_t isr_ctx;

#endif  /* CPU_FAM_NRF52 */

static inline void _uart_lock(void) {
#if defined(MODULE_PERIPH_UART_DMA_TX) && defined(CPU_FAM_NRF52)
    mutex_lock(&uart_mtx);
#endif
}

static inline void _uart_unlock(void) {
#if defined(MODULE_PERIPH_UART_DMA_TX) && defined(CPU_FAM_NRF52)
    mutex_unlock(&uart_mtx);
#endif
}

static int _uart_set_baudrate(uart_t uart, uint32_t baudrate) {
#if !defined(CPU_FAM_NRF52)
    (void) uart;
#endif
    
    /* select baudrate */
    switch (baudrate) {
        case 1200:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud1200;
            break;
        case 2400:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud2400;
            break;
        case 4800:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud4800;
            break;
        case 9600:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud9600;
            break;
        case 14400:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud14400;
            break;
        case 19200:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud19200;
            break;
        case 28800:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud28800;
            break;
        case 38400:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud38400;
            break;
        case 57600:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud57600;
            break;
        case 76800:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud76800;
            break;
        case 115200:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud115200;
            break;
        case 230400:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud230400;
            break;
        case 250000:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud250000;
            break;
        case 460800:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud460800;
            break;
        case 921600:
            REG_BAUDRATE = UART_BAUDRATE_BAUDRATE_Baud921600;
            break;
        default:
            return UART_NOBAUD;
    }
    
    return UART_OK;
}

int uart_set_baudrate(uart_t uart, uint32_t baudrate) {
    _uart_lock();
    int res = _uart_set_baudrate(uart, baudrate);
    _uart_unlock();
    
    return res;
}

int uart_init(uart_t uart, uint32_t baudrate, uart_rx_cb_t rx_cb, void *arg)
{
    if (UART_INVALID) {
        return UART_NODEV;
    }
    
    _uart_lock();
    
    /* remember callback addresses and argument */
    ISR_CTX.rx_cb = rx_cb;
    ISR_CTX.arg = arg;

#ifdef CPU_FAM_NRF51
   /* power on the UART device */
    NRF_UART0->POWER = 1;
#endif

    /* reset configuration registers */
    REG_CONFIG = 0;

    /* configure RX pin */
    if (rx_cb) {
        gpio_init(UART_PIN_RX, UART_PIN_RXMODE);
        PSEL_RXD = UART_PIN_RX;
    }

    /* configure TX pin */
    gpio_init(UART_PIN_TX, UART_PIN_TXMODE);
    PSEL_TXD = UART_PIN_TX;

#if defined(CPU_FAM_NRF52)
    /* enable HW-flow control if defined */
    if (UART_HWFLOWCTRL) {
        /* set pin mode for RTS and CTS pins */
        gpio_init(UART_PIN_RTS, GPIO_OUT);
        gpio_init(UART_PIN_CTS, GPIO_IN);
        /* configure RTS and CTS pins to use */
        dev(uart)->PSEL.RTS = UART_PIN_RTS;
        dev(uart)->PSEL.CTS = UART_PIN_CTS;
        REG_CONFIG |= UART_CONFIG_HWFC_Msk; /* enable HW flow control */
    } else {
        dev(uart)->PSEL.RTS = 0xffffffff;   /* pin disconnected */
        dev(uart)->PSEL.CTS = 0xffffffff;   /* pin disconnected */
    }
#else
#if UART_HWFLOWCTRL
    /* set pin mode for RTS and CTS pins */
    gpio_init(UART_PIN_RTS, GPIO_OUT);
    gpio_init(UART_PIN_CTS, GPIO_IN);
    /* configure RTS and CTS pins to use */
    NRF_UART0->PSELRTS = UART_PIN_RTS;
    NRF_UART0->PSELCTS = UART_PIN_CTS;
    REG_CONFIG |= UART_CONFIG_HWFC_Msk;     /* enable HW flow control */
#else
    NRF_UART0->PSELRTS = 0xffffffff;        /* pin disconnected */
    NRF_UART0->PSELCTS = 0xffffffff;        /* pin disconnected */
#endif
#endif

    /* select baudrate */
    if (_uart_set_baudrate(uart, baudrate) == UART_NOBAUD) {
        _uart_unlock();
        return UART_NOBAUD;
    }

    /* enable the UART device */
#if defined(CPU_FAM_NRF52)
    dev(uart)->ENABLE = UARTE_ENABLE_ENABLE_Enabled;
#else
    NRF_UART0->ENABLE = UART_ENABLE_ENABLE_Enabled;
    NRF_UART0->TASKS_STARTTX = 1;
#endif

    if (rx_cb) {
#if defined(CPU_FAM_NRF52)
        uart_current = uart;
        dev(uart)->RXD.MAXCNT = 1;
        dev(uart)->RXD.PTR = (uint32_t)&rx_buf[uart];
        dev(uart)->INTENSET = UARTE_INTENSET_ENDRX_Msk;
        dev(uart)->SHORTS |= UARTE_SHORTS_ENDRX_STARTRX_Msk;
        dev(uart)->TASKS_STARTRX = 1;
#else
        NRF_UART0->INTENSET = UART_INTENSET_RXDRDY_Msk;
        NRF_UART0->TASKS_STARTRX = 1;
#endif

        /* enable global and receiving interrupt */
        NVIC_EnableIRQ(UART_IRQN);
    }
    
    _uart_unlock();

    return UART_OK;
}

#if defined(CPU_FAM_NRF52) /* nRF52 (using EasyDMA) */

void uart_write(uart_t uart, const uint8_t *data, size_t len)
{
    assert(uart < UART_NUMOF);

    uint32_t data_ptr = (uint32_t)data;

#if defined(MODULE_PERIPH_UART_DMA_TX)
    _uart_lock();
    if ((len <= PERIPH_UART_TX_BUFFER_SIZE) && !irq_is_in()) {
        blocking = false;
        memcpy(data_tmp, data, len);
        data_ptr = (uint32_t)data_tmp;
        pm_block(PM_SLEEP);
        dev(uart)->INTENSET = UARTE_INTENSET_ENDTX_Msk;
    } else {
        blocking = true;
        dev(uart)->INTENCLR = UARTE_INTENCLR_ENDTX_Msk;
    }
#endif

    if (uart_config[uart].tx_pin != PSEL_TXD) {
        /* Pin changes must be done with UART disabled */
        dev(uart)->ENABLE = UARTE_ENABLE_ENABLE_Disabled;
        
        gpio_t uart_pin_prev = PSEL_TXD;
        PSEL_TXD = 0xffffffff;
        gpio_set(uart_pin_prev);
        gpio_init(UART_PIN_TX, UART_PIN_TXMODE);
        PSEL_TXD = UART_PIN_TX;
        
        /* Restart UART */
        dev(uart)->ENABLE        = UARTE_ENABLE_ENABLE_Enabled;
        dev(uart)->TASKS_STARTRX = 1;
    }

    /* Send data via UART */
    if (len <= MAX_LEN_FOR_SEND_VIA_EASY_DMA) {
        /* Set data to transfer to DMA TX pointer */
        dev(uart)->TXD.PTR    = data_ptr;
        dev(uart)->TXD.MAXCNT = len;

        /* Start transmission */
        dev(uart)->TASKS_STARTTX = 1;
    } else {
        size_t sent_byte = 0;
        size_t send_byte;

        /* Sending a command in multiple packages */
        do {
            /* The remaining number of bytes to send */
            send_byte = len - sent_byte;

            /* Trimming to MAX_LEN_FOR_SEND_VIA_EASY_DMA */
            if (send_byte > MAX_LEN_FOR_SEND_VIA_EASY_DMA) {
                send_byte = MAX_LEN_FOR_SEND_VIA_EASY_DMA;
            } 

            /* Set data to transfer to DMA TX pointer */
            dev(uart)->TXD.PTR    = data_ptr + sent_byte;
            dev(uart)->TXD.MAXCNT = send_byte;

            /* Start transmission */
            dev(uart)->TASKS_STARTTX = 1;

            /* Wait for the end of transmission */
            while (dev(uart)->EVENTS_ENDTX == 0) {}
            
            /* Reset endtx flag */
            dev(uart)->EVENTS_ENDTX = 0;

            /* Update the number of bytes sent */
            sent_byte += send_byte;
        } while (sent_byte < len);
    }

#if defined(MODULE_PERIPH_UART_DMA_TX)
    if (blocking) {
        while (dev(uart)->EVENTS_ENDTX == 0) {}
        
        /* Reset endtx flag */
        dev(uart)->EVENTS_ENDTX = 0;
        _uart_unlock();
    }
#else
    /* Wait for the end of transmission */
    while (dev(uart)->EVENTS_ENDTX == 0) {}

    /* Reset endtx flag */
    dev(uart)->EVENTS_ENDTX = 0;
#endif
}

#if defined(MODULE_PERIPH_UART_DMA_TX)
void uart_wait(uart_t uart) {
    assert(uart < UART_NUMOF);
    
    _uart_lock();
    _uart_unlock();
}
#endif

void uart_poweron(uart_t uart)
{
    assert(uart < UART_NUMOF);
    
    dev(uart)->ENABLE = UARTE_ENABLE_ENABLE_Enabled;

    if (isr_ctx[uart].rx_cb) {
        NRF_UART0->TASKS_STARTRX = 1;
    }
}

void uart_poweroff(uart_t uart)
{
    assert(uart < UART_NUMOF);
    
    _uart_lock();
    _uart_unlock();

    dev(uart)->TASKS_STOPRX = 1;
    
    dev(uart)->ENABLE = UARTE_ENABLE_ENABLE_Disabled;
}

static inline void irq_handler(uart_t uart)
{
#if defined(MODULE_PERIPH_UART_DMA_TX)
    if (dev(uart)->EVENTS_ENDTX == 1) {
        dev(uart)->EVENTS_ENDTX = 0;
        _uart_unlock();
        if (!blocking) {
            pm_unblock(PM_SLEEP);
        }
    }
#endif
    
    if (dev(uart)->EVENTS_ENDRX == 1) {
        dev(uart)->EVENTS_ENDRX = 0;

        /* make sure we actually received new data */
        if (dev(uart)->RXD.AMOUNT == 0) {
            return;
        }
        /* Process received byte */
        isr_ctx[uart_current].rx_cb(isr_ctx[uart_current].arg, rx_buf[uart_current]);
    }

    cortexm_isr_end();
}

#else /* nrf51 */

void uart_write(uart_t uart, const uint8_t *data, size_t len)
{
#if defined(UART_NUMOF)
    assert(uart < UART_NUMOF);
   
    gpio_init(UART_PIN_TX, UART_PIN_TXMODE);
    PSEL_TXD = UART_PIN_TX;
#else
    (void)uart;
#endif

    for (size_t i = 0; i < len; i++) {
        /* This section of the function is not thread safe:
            - another thread may mess up with the uart at the same time.
           In order to avoid an infinite loop in the interrupted thread,
           the TXRDY flag must be cleared before writing the data to be
           sent and not after. This way, the higher priority thread will
           exit this function with the TXRDY flag set, then the interrupted
           thread may have not transmitted his data but will still exit the
           while loop.
        */
        /* reset ready flag */
        NRF_UART0->EVENTS_TXDRDY = 0;
        /* write data into transmit register */
        NRF_UART0->TXD = data[i];
        /* wait for any transmission to be done */
        while (NRF_UART0->EVENTS_TXDRDY == 0) {}
    }
}

void uart_poweron(uart_t uart)
{
    (void)uart;

    NRF_UART0->TASKS_STARTTX = 1;
    if (isr_ctx.rx_cb) {
        NRF_UART0->TASKS_STARTRX = 1;
    }
}

void uart_poweroff(uart_t uart)
{
    (void)uart;

    NRF_UART0->TASKS_SUSPEND;
}

static inline void irq_handler(uart_t uart)
{
    (void)uart;

    if (NRF_UART0->EVENTS_RXDRDY == 1) {
        NRF_UART0->EVENTS_RXDRDY = 0;
        uint8_t byte = (uint8_t)(NRF_UART0->RXD & 0xff);
        isr_ctx.rx_cb(isr_ctx.arg, byte);
    }

    cortexm_isr_end();
}

#endif /* CPU_FAM_NRF52 */

#ifdef UART_0_ISR
void UART_0_ISR(void)
{
    irq_handler(UART_DEV(0));
}
#endif

#ifdef UART_1_ISR
void UART_1_ISR(void)
{
    irq_handler(UART_DEV(1));
}
#endif

#ifdef UART_2_ISR
void UART_2_ISR(void)
{
    irq_handler(UART_DEV(2));
}
#endif

#ifdef UART_3_ISR
void UART_3_ISR(void)
{
    irq_handler(UART_DEV(3));
}
#endif

#ifdef UART_4_ISR
void UART_4_ISR(void)
{
    irq_handler(UART_DEV(4));
}
#endif

#ifdef UART_5_ISR
void UART_5_ISR(void)
{
    irq_handler(UART_DEV(5));
}
#endif
