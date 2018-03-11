/*
 * Copyright (C) 2017 HAW Hamburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_hd44780
 *
 * @{
 * @file
 * @brief       Driver for the HD44780 LCD
 *
 * @note        The display is also known as LCM1602C from Arduino kits
 *
 * @author      Sebastian Meiling <s@mlng.net>
 * @author      Oleg Artamonov <info@unwds.com>
 *
 * @}
 */

#include <assert.h>
#include <string.h>

#include "log.h"
#include "periph/gpio.h"
#include "xtimer.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#include "hd44780.h"
#include "hd44780_internal.h"

static inline void _command(const hd44780_t *dev, uint8_t value);
static void _pulse(const hd44780_t *dev);
static void _send(const hd44780_t *dev, uint8_t value, uint8_t mode);
static void _write_bits(const hd44780_t *dev, uint8_t bits, uint8_t value);

#ifdef HD44780_CYRILLIC
static const char hd44780_cyrillic_codes[64] =
{
	0x41,0xA0,0x42,0xA1,0xE0,0x45,0xA3,0xA4,
	0xA5,0xA6,0x4B,0xA7,0x4D,0x48,0x4F,0xA8,
	0x50,0x43,0x54,0xA9,0xAA,0x58,0xE1,0xAB,
	0xAC,0xE2,0xAD,0xAE,0xAD,0xAF,0xB0,0xB1,
	0x61,0xB2,0xB3,0xB4,0xE3,0x65,0xB6,0xB7,
	0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0x6F,0xBE,
	0x70,0x63,0xBF,0x79,0xE4,0x78,0xE5,0xC0,
	0xC1,0xE6,0xC2,0xC3,0xC4,0xC5,0xC6,0xC7
};
#endif

#ifdef HD44780_PCF8574
    static uint8_t pcf8574_bitmask = 0;   
#endif

#ifdef HD44780_PCF8574
static void _i2c_init(const hd44780_t *dev) {
    i2c_acquire(dev->p.i2c_dev);

    /* Initialize I2C interface */
    if (i2c_init_master(dev->p.i2c_dev, I2C_SPEED_NORMAL)) {
        DEBUG("[Error] I2C device not enabled\n");
        i2c_release(dev->p.i2c_dev);
        return;
    }
    
    i2c_release(dev->p.i2c_dev);
}


static inline void _i2c_write(const hd44780_t *dev, uint8_t data) {
    i2c_acquire(dev->p.i2c_dev);
    i2c_write_byte(dev->p.i2c_dev, dev->p.i2c_address, data);    
    i2c_release(dev->p.i2c_dev);
}
#endif

static inline void _gpio_clear(const hd44780_t *dev, gpio_t pin) {
#ifdef HD44780_PCF8574
    pcf8574_bitmask &= ~(1<<pin);
    _i2c_write(dev, pcf8574_bitmask);
#else
    gpio_clear(pin);
#endif
}

static inline void _gpio_set(const hd44780_t *dev, gpio_t pin) {
#ifdef HD44780_PCF8574
    pcf8574_bitmask |= (1<<pin);
    _i2c_write(dev, pcf8574_bitmask);
#else
    gpio_set(pin);
#endif
}

static inline void _gpio_init(const hd44780_t *dev, gpio_t pin, gpio_mode_t mode) {
#ifdef HD44780_PCF8574
    (void)mode;
    _gpio_clear(dev, pin);
#else
    gpio_init(pin, mode);
#endif
}

/**
 * @brief   Send a command to the display
 *
 * @param[in]  dev          device descriptor of display to initialize
 * @param[in]  value        the command
 */
static inline void _command(const hd44780_t *dev, uint8_t value)
{
    _send(dev, value, HD44780_OFF);
}

/**
 * @brief   Send a pulse to the display to enable new state
 *
 * @param[in]  dev          device descriptor of display to initialize
 */
static void _pulse(const hd44780_t *dev)
{
    _gpio_clear(dev, dev->p.enable);
    xtimer_usleep(HD44780_PULSE_WAIT_SHORT);
    _gpio_set(dev, dev->p.enable);
    xtimer_usleep(HD44780_PULSE_WAIT_SHORT);
    _gpio_clear(dev, dev->p.enable);
    xtimer_usleep(HD44780_PULSE_WAIT_LONG);
}

/**
 * @brief Enable LCD backlight
 *
 * @param[in]  dev          LCD device descriptor
 * @param[in]  enable       backlight state
 */
void hd44780_backlight(const hd44780_t *dev, const bool enable) {
    if (enable) {
        _gpio_set(dev, dev->p.backlight);
    } else {
        _gpio_clear(dev, dev->p.backlight);
    }
    return;
}

/**
 * @brief   Send a data value to the display
 *
 * @param[in]  dev          device descriptor of display to initialize
 * @param[in]  value        the data value, either char or command
 * @param[in]  state        send state, to distinguish chars and commands
 */
static void _send(const hd44780_t *dev, uint8_t value, hd44780_state_t state)
{
    (state == HD44780_ON) ? _gpio_set(dev, dev->p.rs) : _gpio_clear(dev, dev->p.rs);
    /* if RW pin is available, set it to LOW */
    if (dev->p.rw != HD44780_RW_OFF) {
        _gpio_clear(dev, dev->p.rw);
    }
    /* write data in 8Bit or 4Bit mode */
    if (dev->flag & HD44780_8BITMODE) {
        _write_bits(dev, 8, value);
    }
    else {
        _write_bits(dev, 4, value>>4);
        _write_bits(dev, 4, value);
    }
}

static void _write_bits(const hd44780_t *dev, uint8_t bits, uint8_t value)
{
    DEBUG("[hd44780] write %d-bits 0x%x\n", bits, value);
#ifdef HD44780_PCF8574
    for (unsigned i = 0; i < bits; ++i) {
        if ((value >> i) & 0x01) {
            pcf8574_bitmask |= (1 << dev->p.data[i]);
        }
        else {
            pcf8574_bitmask &= ~(1 << dev->p.data[i]);
        }
    }
    _i2c_write(dev, pcf8574_bitmask);
#else
    for (unsigned i = 0; i < bits; ++i) {
        if ((value >> i) & 0x01) {
            _gpio_set(dev, dev->p.data[i]);
        }
        else {
            _gpio_clear(dev, dev->p.data[i]);
        }
    }
#endif
    _pulse(dev);
}

int hd44780_init(hd44780_t *dev, const hd44780_params_t *params)
{   
    /* write config params to device descriptor */
    memcpy(&dev->p, params, sizeof(hd44780_params_t));
    
#ifdef HD44780_PCF8574
    DEBUG("HD44780: I2C mode\n");
    _i2c_init(dev);
#else
    DEBUG("HD44780: GPIO mode\n");
#endif
    
    /* verify cols and rows */
    if ((dev->p.cols > HD44780_MAX_COLS) || (dev->p.rows > HD44780_MAX_ROWS)
                                         || (dev->p.rows * dev->p.cols > 80)) {
        DEBUG("hd44780_init: invalid LCD size!\n");
        return -1;
    }
    uint8_t count_pins = 0;
    /* check which pins are used */
    for (unsigned i = 0; i < HD44780_MAX_PINS; ++i) {
        if (dev->p.data[i] != HD44780_RW_OFF) {
            ++count_pins;
        }
    }
      
    /* set mode depending on configured pins */
    if (count_pins < HD44780_MAX_PINS) {
        DEBUG("[hd44780]: 4-bit mode\n");
        dev->flag |= HD44780_4BITMODE;
    }
    else {
        DEBUG("[hd44780]: 8-bit mode\n");
        dev->flag |= HD44780_8BITMODE;
    }
    /* set flag for 1 or 2 row mode, 4 rows are 2 rows split half */
    if (dev->p.rows > 1) {
        DEBUG("[hd44780]: 2 lines\n");
        dev->flag |= HD44780_2LINE;
    }
    else {
        DEBUG("[hd44780]: 1 line\n");
        dev->flag |= HD44780_1LINE;
    }
    /* set char size to 5x8 as default, 5x10 is hardly used by LCDs anyway */
    dev->flag |= HD44780_5x8DOTS;
    /* calc and set row offsets, depending on number of columns */
    dev->roff[0] = 0x00;
    dev->roff[1] = 0x40;
    dev->roff[2] = 0x00 + dev->p.cols;
    dev->roff[3] = 0x40 + dev->p.cols;

    _gpio_init(dev, dev->p.rs, GPIO_OUT);
    DEBUG("RS GPIO initialized\n"); 
    
    /* RW (read/write) of LCD not required, set it to HD44780_RW_OFF (255) */
    if (dev->p.rw != HD44780_RW_OFF) {
        _gpio_init(dev, dev->p.rw, GPIO_OUT);
    }
    DEBUG("RW GPIO initialized\n");
    
    _gpio_init(dev, dev->p.enable, GPIO_OUT);
    DEBUG("Enable GPIO initialized\n");
    
    
    if (dev->p.backlight != HD44780_RW_OFF) {
        _gpio_init(dev, dev->p.backlight, GPIO_OUT);
        _gpio_clear(dev, dev->p.backlight);
    }
    DEBUG("Backlight GPIO initialized\n");
    
    /* configure all data pins as output */
    for (int i = 0; i < ((dev->flag & HD44780_8BITMODE) ? 8 : 4); ++i) {
        _gpio_init(dev, dev->p.data[i], GPIO_OUT);
    }    
    DEBUG("Data GPIOs initialized\n");
    
    /* see hitachi HD44780 datasheet pages 45/46 for init specs */
    xtimer_usleep(HD44780_INIT_WAIT_XXL);
    _gpio_clear(dev, dev->p.rs);
    _gpio_clear(dev, dev->p.enable);
    if (dev->p.rw != HD44780_RW_OFF) {
        _gpio_clear(dev, dev->p.rw);
    }
    
    DEBUG("Setting LCD bus width\n");
    /* put the LCD into 4 bit or 8 bit mode */
    if (!(dev->flag & HD44780_8BITMODE)) {
        /* see hitachi HD44780 datasheet figure 24, pg 46 */
        _write_bits(dev, 4, 0x03);
        xtimer_usleep(HD44780_INIT_WAIT_LONG);

        _write_bits(dev, 4, 0x03);
        xtimer_usleep(HD44780_INIT_WAIT_LONG);

        _write_bits(dev, 4, 0x03);
        xtimer_usleep(HD44780_INIT_WAIT_SHORT);

        _write_bits(dev, 4, 0x02);
    } else {
        /* see hitachi HD44780 datasheet page 45 figure 23 */
        _command(dev, HD44780_FUNCTIONSET | dev->flag);
        xtimer_usleep(HD44780_INIT_WAIT_LONG);  // wait more than 4.1ms

        _command(dev, HD44780_FUNCTIONSET | dev->flag);
        xtimer_usleep(HD44780_INIT_WAIT_SHORT);

        _command(dev, HD44780_FUNCTIONSET | dev->flag);
    }
    _command(dev, HD44780_FUNCTIONSET | dev->flag);

    /* turn the display on with no cursor or blinking default, and clear */
    dev->ctrl = HD44780_DISPLAYON | HD44780_CURSOROFF | HD44780_BLINKOFF;
    hd44780_display(dev, HD44780_ON);
    hd44780_clear(dev);
    /* Initialize to default text direction for western languages */
    dev->mode = HD44780_ENTRYLEFT | HD44780_ENTRYSHIFTDECREMENT;
    _command(dev, HD44780_ENTRYMODESET | dev->mode);

    return 0;
}

void hd44780_clear(const hd44780_t *dev)
{
    _command(dev, HD44780_CLEARDISPLAY);
    xtimer_usleep(HD44780_CMD_WAIT);
}

void hd44780_home(const hd44780_t *dev)
{
    _command(dev, HD44780_RETURNHOME);
    xtimer_usleep(HD44780_CMD_WAIT);
}

void hd44780_set_cursor(const hd44780_t *dev, uint8_t col, uint8_t row)
{
    if (row >= dev->p.rows) {
        row = dev->p.rows - 1;
    }
    _command(dev, HD44780_SETDDRAMADDR | (col + dev->roff[row]));
}

void hd44780_display(hd44780_t *dev, hd44780_state_t state)
{
    if (state == HD44780_ON) {
        dev->ctrl |= HD44780_DISPLAYON;
    }
    else {
        dev->ctrl &= ~HD44780_DISPLAYON;
    }
    _command(dev, HD44780_DISPLAYCONTROL | dev->ctrl);
}

void hd44780_cursor(hd44780_t *dev, hd44780_state_t state)
{
    if (state == HD44780_ON) {
        dev->ctrl |= HD44780_CURSORON;
    }
    else {
        dev->ctrl &= ~HD44780_CURSORON;
    }
    _command(dev, HD44780_DISPLAYCONTROL | dev->ctrl);
}

void hd44780_blink(hd44780_t *dev, hd44780_state_t state)
{
    if (state == HD44780_ON) {
        dev->ctrl |= HD44780_BLINKON;
    }
    else {
        dev->ctrl &= ~HD44780_BLINKON;
    }
    _command(dev, HD44780_DISPLAYCONTROL | dev->ctrl);
}

void hd44780_scroll_left(const hd44780_t *dev)
{
    _command(dev, HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVELEFT);
}

void hd44780_scroll_right(const hd44780_t *dev) {
    _command(dev, HD44780_CURSORSHIFT | HD44780_DISPLAYMOVE | HD44780_MOVERIGHT);
}

void hd44780_left2right(hd44780_t *dev)
{
    dev->mode |= HD44780_ENTRYLEFT;
    _command(dev, HD44780_ENTRYMODESET | dev->mode);
}

void hd44780_right2left(hd44780_t *dev)
{
    dev->mode &= ~HD44780_ENTRYLEFT;
    _command(dev, HD44780_ENTRYMODESET | dev->mode);
}

void hd44780_autoscroll(hd44780_t *dev, hd44780_state_t state)
{
    if (state == HD44780_ON) {
        dev->mode |= HD44780_ENTRYSHIFTINCREMENT;
    }
    else {
        dev->mode &= ~HD44780_ENTRYSHIFTINCREMENT;
    }
    _command(dev, HD44780_ENTRYMODESET | dev->mode);
}

void hd44780_create_char(const hd44780_t *dev, uint8_t location, uint8_t charmap[])
{
    location &= 0x7; /* 8 locations (0-7) possible */
    _command(dev, HD44780_SETCGRAMADDR | (location << 3));
    for (unsigned i = 0; i < HD44780_CGRAM_SIZE; ++i) {
        hd44780_write(dev, charmap[i]);
    }
}

void hd44780_write(const hd44780_t *dev, uint8_t value)
{
    _send(dev, value, HD44780_ON);
}

void hd44780_print(const hd44780_t *dev, const char *data )
{
    while (*data != '\0') {
        char c = *data++;
#ifdef HD44780_CYRILLIC
        if (c > 191) {
            c = hd44780_cyrillic_codes[c-192];
        }
#endif
        hd44780_write(dev, c);
    }
}
