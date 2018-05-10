/*
 * Copyright (c) 2013,2014 Fingerprint Cards AB <tech@fingerprints.com>
 * Copyright (c) 2018 Unwired Devices LLC <info@unwds.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_fpc1020
 * @{
 *
 * @file
 * @brief       Device driver implementation for FPC1020 fingerprint sensor.
 *
 * @author      Oleg Artamonov <info@unwds.com>
 *
 * @}
 */

#include <string.h>

#include "fpc1020.h"
#include "fpc1020_internal.h"
#include "periph/spi.h"
#include "xtimer.h"

#define ENABLE_DEBUG        (1)
#include "debug.h"

static int fpc1020_reset(fpc1020_t *dev);
static int fpc1020_wait_for_irq(fpc1020_t *dev);
static int fpc1020_check_hwid(fpc1020_t *dev);
static int fpc1020_get_revision(fpc1020_t *dev);
static int fpc1020_get_revision_setup(fpc1020_t *dev);
static int fpc1020_image_crop(fpc1020_t *dev, int first_column, int columns, int first_row, int rows);
static int fpc1020_get_image(fpc1020_t *dev, uint8_t *data, int image_size);
static int fpc1020_reg_write(fpc1020_t *dev, uint8_t cmd, uint8_t *data_write, uint8_t *data_read, uint8_t bytes);
static bool fpc1020_check_range(int val, int min, int max);

static int fpc1020_reg_write(fpc1020_t *dev, uint8_t cmd, uint8_t *data_write, uint8_t *data_read, uint8_t bytes) {
    /* acquire SPI */
    spi_acquire(dev->spi, SPI_CS_UNDEF, SPI_MODE_0, FPC1020_SPI_SPEED);
    gpio_clear(dev->cs);
   
    /* send command */
    if (cmd) {
        spi_transfer_bytes(dev->spi, SPI_CS_UNDEF, false, (char *)&cmd, NULL, 1);
    }
   
    /* send data and/or receive reply */
    if (bytes) {
        spi_transfer_bytes(dev->spi, SPI_CS_UNDEF, false, (char *)data_write, (char *)data_read, bytes);
    }
    
    /* release SPI */
    gpio_set(dev->cs);
    spi_release(dev->spi);
    
    return 0;
}

static int fpc1020_reset(fpc1020_t *dev) {
    DEBUG("Resetting FPC1020\n");
    gpio_clear(dev->reset);
    xtimer_spin(xtimer_ticks_from_usec(10000));
    gpio_set(dev->reset);
    
    int res = fpc1020_wait_for_irq(dev);
    if (res != 0xff) {
        return res;
    }
    
    return 0;
}

static int fpc1020_wait_for_irq(fpc1020_t *dev) {
    /* wait for interrupt from FPC1020 */
    
    /* TBD: rewrite using interrupts */
    int i = 0;
    while (gpio_read(dev->irq) == 0) {
        xtimer_spin(xtimer_ticks_from_usec(1000));
        if (++i > 100) {
            DEBUG("Timeout waiting for IRQ\n");
            return -FPC102X_ERROR_IRQ_TIMEOUT;
        }
    }
    
    /* clear interrupt */
    uint8_t irq_source;
    fpc1020_reg_write(dev, FPC102X_REG_READ_IRQ_WITH_CLEAR, NULL, &irq_source, 1);
    
    if (!gpio_read(dev->irq)) {
#if ENABLE_DEBUG
        DEBUG("IRQ cleared: 0x%02x\n", irq_source);
        if (irq_source == 0xff) {
            DEBUG("IRQ: power-on interrupt\n");
        } else {
            if (irq_source & 1) {
                DEBUG("IRQ: finger down\n");
            }
            if (irq_source & (1<<2)) {
                DEBUG("IRQ: error\n");
            }
            if (irq_source & (1<<5)) {
                DEBUG("IRQ: new image data available\n");
            }
            if (irq_source & (1<<7)) {
                DEBUG("IRQ: command done\n");
            }
            if ((irq_source & (1<<1)) || (irq_source & (1<<3)) || (irq_source & (1<<4)) || (irq_source & (1<<6))) {
                DEBUG("IRQ: unknown interrupt\n");
            }
        }
#endif
        return irq_source;
    } else {
        DEBUG("IRQ not cleared");
    }
    
    return -FPC102X_ERROR_IRQ_NOT_CLEARED;
}

static int fpc1020_check_hwid(fpc1020_t *dev) {
    uint16_t hwid;
    fpc1020_reg_write(dev, FPC102X_REG_HWID, NULL, (uint8_t *)&hwid, 2);
    
    hwid = (hwid >> 8) | (hwid << 8);
    DEBUG("HWID: 0x%04X\n", hwid);
    
    if (hwid != 0x020a) {
        DEBUG("HWID mismatch");
        return -FPC102X_ERROR_HWID_MISMATCH;
    }
    
    return 0;
}

static int fpc1020_get_revision_setup(fpc1020_t *dev) {
	uint8_t temp_u8;
	uint16_t temp_u16;

	temp_u16 = 15 << 8; /* ADC Shift = 1111, Gain = 0000 */
    fpc1020_reg_write(dev, FPC102X_REG_ADC_SHIFT_GAIN, (uint8_t *)&temp_u16, NULL, 2);
    
	temp_u16 = 0xffff;
    fpc1020_reg_write(dev, FPC102X_REG_TST_COL_PATTERN_EN, (uint8_t *)&temp_u16, NULL, 2);

	temp_u8 = 0x04; /* FngrDrvTst = 1 */
    fpc1020_reg_write(dev, FPC102X_REG_FINGER_DRIVE_CONF, &temp_u8, NULL, 1);
    
    return 0;
}

static int fpc1020_image_crop(fpc1020_t *dev, int first_column, int columns, int first_row, int rows) {
	uint32_t temp_u32;

	temp_u32 = first_row;
	temp_u32 <<= 8;
	temp_u32 |= rows;
	temp_u32 <<= 8;
	temp_u32 |= (first_column * FPC1020_ADC_GROUP_SIZE);
	temp_u32 <<= 8;
	temp_u32 |= (columns * FPC1020_ADC_GROUP_SIZE);
    
    fpc1020_reg_write(dev, FPC102X_REG_IMG_CAPT_SIZE, (uint8_t *)&temp_u32, NULL, 4);

	return 0;
}

static int fpc1020_get_image(fpc1020_t *dev, uint8_t *data, int image_size) {
    fpc1020_reg_write(dev, FPC102X_REG_CAPTURE_IMAGE, NULL, NULL, 0);
    
    /* now wait for image to be ready */
    if (fpc1020_wait_for_irq(dev) != FPC_1020_IRQ_REG_BIT_FIFO_NEW_DATA) {
        DEBUG("Error getting an image\n");
        return -FPC102X_ERROR_IMAGE_CAPTURE;
    }
    
    /* send command to fetch image, plus dummy byte */
    uint8_t temp_u8 = 0;
    fpc1020_reg_write(dev, FPC102X_REG_READ_IMAGE, &temp_u8, NULL, 1);
    
    /* get image data, no command needed */
    fpc1020_reg_write(dev, 0, NULL, data, image_size);
    
#if ENABLE_DEBUG
    DEBUG("Image data (%d bytes):", image_size);
    for (int i = 0; i < image_size; i++) {
        DEBUG(" %02x", data[i]);
    }
    DEBUG("\n");
#endif
    
    return 0;
}

static bool fpc1020_check_range(int val, int min, int max)
{
	return (val >= min) && (val <= max);
}

static int fpc1020_get_revision(fpc1020_t *dev) {
    int error = 0;
	int xpos, ypos, m1, m2, count;

	const int num_rows = FPC1020_EXT_HWID_CHECK_ID1020A_ROWS;
	const int num_pixels = 32;
	const uint32_t image_size = num_rows * FPC1020_PIXEL_COLUMNS;

	error = fpc1020_get_revision_setup(dev);
	if (error) {
		return error;
    }

	error = fpc1020_image_crop(dev, 0,
                               FPC1020_PIXEL_COLUMNS / FPC1020_ADC_GROUP_SIZE,
                               0, num_rows);
	if (error) {
		return error;
    }

	error = fpc1020_get_image(dev, dev->image, image_size);
	if (error) {
		return error;
    }

	m1 = m2 = count = 0;

	for (ypos = 1; ypos < num_rows; ypos++) {
		for (xpos = 0; xpos < num_pixels; xpos++) {
			m1 += dev->image[(ypos * FPC1020_PIXEL_COLUMNS) + xpos];

			m2 += dev->image[(ypos * FPC1020_PIXEL_COLUMNS) + (FPC1020_PIXEL_COLUMNS - 1 - xpos)];
			count++;
		}
	}

	m1 /= count;
	m2 /= count;

	if (fpc1020_check_range(m1, 181, 219) && fpc1020_check_range(m2, 101, 179)) {
		dev->revision = 1;
	} else if (fpc1020_check_range(m1, 181, 219) && fpc1020_check_range(m2, 181, 219)) {
		dev->revision = 2;
	} else if (fpc1020_check_range(m1, 101, 179) && fpc1020_check_range(m2, 151, 179)) {
		dev->revision = 3;
	} else if (fpc1020_check_range(m1, 0, 99) && fpc1020_check_range(m2, 0, 99)) {
		dev->revision = 4;
	} else {
		dev->revision = 0;
	}
    
    DEBUG("FPC1020 HW revision %d\n", dev->revision);
    
    return 0;
}

int fpc1020_get_fingerprint(fpc1020_t *dev) {
    (void)dev;
    
    return 0;
}

int fpc1020_init(fpc1020_t *dev, spi_t spi, gpio_t cs, gpio_t reset, gpio_t irq) {
    dev->spi = spi;
    dev->cs = cs;
    dev->reset = reset;
    dev->irq = irq;
    
    gpio_init(dev->reset, GPIO_OUT);
    gpio_set(reset);
    
    gpio_init(dev->cs, GPIO_OUT);
    gpio_set(cs);
    
    gpio_init(dev->irq, GPIO_IN);
    
    spi_init(dev->spi);
    
    int res = fpc1020_reset(dev);
    if (res < 0) {
        DEBUG("Error resetting FPC1020\n");
        return res;
    }
    
    res = fpc1020_check_hwid(dev);
    if (res < 0) {
        DEBUG("HwID mismatch\n");
        return res;
    }
    
    xtimer_spin(xtimer_ticks_from_usec(1000000));
    
    res = fpc1020_get_revision(dev);
    if (res < 0) {
        DEBUG("Error getting chip revision\n");
        return res;
    }
    
    return 0;
}