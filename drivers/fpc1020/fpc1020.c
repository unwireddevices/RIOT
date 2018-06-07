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
#include "byteorder.h"

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
static int fpc1020_setup_defaults(fpc1020_t *dev);
static int fpc1020_write_sensor_setup(fpc1020_t *dev);
static int fpc1020_write_sensor_a1a2_setup(fpc1020_t *dev);
static int fpc1020_write_sensor_a3a4_setup(fpc1020_t *dev);
static int fpc1020_capture_settings(fpc1020_t *dev, int select);

const fpc1020_setup_t fpc1020_setup_default_CT_a1a2 = {
    .adc_gain = {1, 0, 1},    /*Dry, Wet, Normal */
    .adc_shift = {1, 5, 0},
    .pxl_ctrl = {0x0f0b, 0x0f1b, 0x0f1b},
    .capture_settings_mux = 0,
    .capture_count = 3,
    .capture_mode = FPC1020_MODE_WAIT_AND_CAPTURE,
    .capture_row_start = 18,
    .capture_row_count = 173,
    .capture_col_start = 1,
    .capture_col_groups = 20,
    .capture_finger_up_threshold = 0,
    .capture_finger_down_threshold = 6,
    .finger_detect_threshold = 0x50,
    .wakeup_detect_rows = {92, 92},
    .wakeup_detect_cols = {8, 15},
    .finger_auto_threshold = true,
};

const fpc1020_setup_t fpc1020_setup_default_CT_a3a4 = {
    .adc_gain = {10, 2, 1},    /* Dry , Wet, Normal */
    .adc_shift = {3, 8, 13},
    .pxl_ctrl = {0x0f0a, 0x0f1b, 0x0f1b},
    .capture_settings_mux = 0,
    .capture_count = 3,
    .capture_mode = FPC1020_MODE_WAIT_AND_CAPTURE,
    .capture_row_start = 18,
    .capture_row_count = 173,
    .capture_col_start = 1,
    .capture_col_groups = 20,
    .capture_finger_up_threshold = 0,
    .capture_finger_down_threshold = 6,
    .finger_detect_threshold = 0x50,
    .wakeup_detect_rows = {92, 92},
    .wakeup_detect_cols = {8, 15},
    .finger_auto_threshold = true,
};

const fpc1020_setup_t fpc1020_setup_default_LO_a1a2 = {
    .adc_gain = {0, 1, 2},    /*Dry, Wet, Normal */
    .adc_shift = {5, 1, 4},
    .pxl_ctrl = {0x0f0b, 0x0f1f, 0x0f0b},
    .capture_settings_mux = 0,
    .capture_count = 3,
    .capture_mode = FPC1020_MODE_WAIT_AND_CAPTURE,
    .capture_row_start = 19,
    .capture_row_count = 171,
    .capture_col_start = 2,
    .capture_col_groups = 21,
    .capture_finger_up_threshold = 0,
    .capture_finger_down_threshold = 6,
    .finger_detect_threshold = 0x50,
    .wakeup_detect_rows = {92, 92},
    .wakeup_detect_cols = {8, 15},
    .finger_auto_threshold = true,
};

const fpc1020_setup_t fpc1020_setup_default_LO_a3a4 = {
    .adc_gain = {8, 0, 1},    /* Dry, Wet, Normal */
    .adc_shift = {4, 6, 11},
    .pxl_ctrl = {0x0f0a, 0x0f1f, 0x0f0B},
    .capture_settings_mux = 0,
    .capture_count = 3,
    .capture_mode = FPC1020_MODE_WAIT_AND_CAPTURE,
    .capture_row_start = 19,
    .capture_row_count = 171,
    .capture_col_start = 2,
    .capture_col_groups = 21,
    .capture_finger_up_threshold = 0,
    .capture_finger_down_threshold = 6,
    .finger_detect_threshold = 0x50,
    .wakeup_detect_rows = {92, 92},
    .wakeup_detect_cols = {8, 15},
    .finger_auto_threshold = true,
};

static int fpc1020_reg_write(fpc1020_t *dev, uint8_t cmd, uint8_t *data_write, uint8_t *data_read, uint8_t bytes) {
    /* acquire SPI */
    spi_acquire(dev->spi, SPI_CS_UNDEF, SPI_MODE_0, FPC1020_SPI_SPEED);
    gpio_clear(dev->cs);
   
    /* send command */
    if (cmd != 0) {
        spi_transfer_bytes(dev->spi, SPI_CS_UNDEF, false, (char *)&cmd, NULL, 1);
    }
   
    /* send data and/or receive reply */
    if (bytes) {
        if (byteorder_is_little_endian()) {
            byteorder_swap(data_write, bytes);
        }
        
        spi_transfer_bytes(dev->spi, SPI_CS_UNDEF, false, (char *)data_write, (char *)data_read, bytes);
        
        if (byteorder_is_little_endian()) {
            byteorder_swap(data_read, bytes);
        }
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
        DEBUG("IRQ cleared: 0x%02x\n", irq_source);
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
    
    DEBUG("HWID: 0x%04X\n", hwid);
    
    if (hwid != 0x020a) {
        DEBUG("HWID mismatch");
        return -FPC102X_ERROR_HWID_MISMATCH;
    } else {
        DEBUG("HWID OK\n");
    }
    
    return 0;
}

static int fpc1020_get_revision_setup(fpc1020_t *dev) {
    uint8_t temp_u8;
    uint16_t temp_u16;
    
    DEBUG("Setup chip revision check\n");

    temp_u16 = 15 << 8; /* ADC Shift = 1111, Gain = 0000 */
    fpc1020_reg_write(dev, FPC102X_REG_ADC_SHIFT_GAIN, (uint8_t *)&temp_u16, NULL, 2);
    
    temp_u16 = 0xffff;
    fpc1020_reg_write(dev, FPC102X_REG_TST_COL_PATTERN_EN, (uint8_t *)&temp_u16, NULL, 2);

    temp_u8 = 0x04; /* FngrDrvTst = 1 */
    fpc1020_reg_write(dev, FPC102X_REG_FINGER_DRIVE_CONF, &temp_u8, NULL, 1);
    
    return 0;
}

static int fpc1020_setup_defaults(fpc1020_t *dev)
{
    const fpc1020_setup_t *ptr;

    ptr =
        (dev->revision ==
         1) ? &fpc1020_setup_default_LO_a1a2
        : (dev->revision ==
           2) ? &fpc1020_setup_default_LO_a1a2 : (dev->
                              revision
                              ==
                              3) ?
        &fpc1020_setup_default_LO_a3a4 : (dev->
                          revision ==
                          4) ?
        &fpc1020_setup_default_LO_a3a4 : NULL;
    
    memcpy((void *)&dev->setup, ptr, sizeof(fpc1020_setup_t));
    dev->fp_threshold = dev->setup.finger_detect_threshold;
    
    printf("FPC1020 setup_defaults fp_threshold=%d \n", dev->fp_threshold);

    return 0;
}

static int fpc1020_image_crop(fpc1020_t *dev, int first_column, int columns, int first_row, int rows) {
    uint32_t temp_u32;

    /* reverse bytes order */
    /*
    temp_u32 = (columns * FPC1020_ADC_GROUP_SIZE);
    temp_u32 <<= 8;
    temp_u32 |= (first_column * FPC1020_ADC_GROUP_SIZE);
    temp_u32 <<= 8;
    temp_u32 |= rows;
    temp_u32 <<= 8;
    temp_u32 |= first_row;
    */
    
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
    
    spi_acquire(dev->spi, SPI_CS_UNDEF, SPI_MODE_0, FPC1020_SPI_SPEED);
    gpio_clear(dev->cs);
   
    /* send command */
    uint16_t cmd = FPC102X_REG_READ_IMAGE;
    spi_transfer_bytes(dev->spi, SPI_CS_UNDEF, false, (char *)&cmd, NULL, 2);
    spi_transfer_bytes(dev->spi, SPI_CS_UNDEF, false, NULL, (char *)data, image_size);
   
    gpio_set(dev->cs);
    spi_release(dev->spi);
    
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
    
    DEBUG("Getting actual chip revision\n");

    const int num_rows = FPC1020_EXT_HWID_CHECK_ID1020A_ROWS;
    const int num_pixels = 32;
    const uint32_t image_size = num_rows * FPC1020_PIXEL_COLUMNS;

    error = fpc1020_get_revision_setup(dev);
    if (error) {
        return error;
    }

    DEBUG("Setting image crop: ");
    error = fpc1020_image_crop(dev, 0,
                               FPC1020_PIXEL_COLUMNS / FPC1020_ADC_GROUP_SIZE,
                               0, num_rows);
    if (error) {
        DEBUG("failed\n");
        return error;
    } else {
        DEBUG("ok\n");
    }

    DEBUG("Getting test image\n");
    error = fpc1020_get_image(dev, dev->image, image_size);
    if (error < 0) {
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
    
    DEBUG("m1: %d, m2: %d\n", m1, m2);

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

static int fpc1020_write_sensor_a1a2_setup(fpc1020_t *dev)
{
    uint8_t temp_u8;
    uint16_t temp_u16;
    uint32_t temp_u32;
    uint64_t temp_u64;
    const int mux = dev->fp_conditions;
    const int rev = dev->revision;

    if (rev == 0)
        return -1;

    temp_u64 = (rev == 1) ? 0x363636363f3f3f3f : 0x141414141e1e1e1e;
    fpc1020_reg_write(dev, FPC102X_REG_SAMPLE_PX_DLY, (void *)&temp_u64, NULL, sizeof(temp_u64));

    temp_u8 = (rev == 1) ? 0x33 : 0x0f;
    fpc1020_reg_write(dev, FPC102X_REG_PXL_RST_DLY, (void *)&temp_u8, NULL, sizeof(temp_u8));

    temp_u8 = (rev == 1) ? 0x37 : 0x15;
    fpc1020_reg_write(dev, FPC102X_REG_FINGER_DRIVE_DLY, (void *)&temp_u8, NULL, sizeof(temp_u8));

    if (rev < 3) {
        temp_u64 = 0x5540003f24;
        fpc1020_reg_write(dev, FPC102X_REG_ADC_SETUP, (void *)&temp_u64, NULL, sizeof(temp_u64));

        temp_u32 = 0x00080000;
        fpc1020_reg_write(dev, FPC102X_REG_ANA_TEST_MUX, (void *)&temp_u32, NULL, sizeof(temp_u32));

        temp_u64 = 0x5540003f34;
        fpc1020_reg_write(dev, FPC102X_REG_ADC_SETUP, (void *)&temp_u64, NULL, sizeof(temp_u64));
    }

    temp_u8 = 0x02;        //32内部3.3V  02是外部供电VDDTX
    fpc1020_reg_write(dev, FPC102X_REG_FINGER_DRIVE_CONF, (void *)&temp_u8, NULL, sizeof(temp_u8));

    temp_u16 = dev->setup.adc_shift[mux];
    temp_u16 <<= 8;
    temp_u16 |= dev->setup.adc_gain[mux];
    fpc1020_reg_write(dev, FPC102X_REG_ADC_SHIFT_GAIN, (void *)&temp_u16, NULL, sizeof(temp_u16));

    temp_u16 = dev->setup.pxl_ctrl[mux];
    fpc1020_reg_write(dev, FPC102X_REG_PXL_CTRL, (void *)&temp_u16, NULL, sizeof(temp_u16));

    temp_u8 = 0x03 | 0x08;
    fpc1020_reg_write(dev, FPC102X_REG_IMAGE_SETUP, (void *)&temp_u8, NULL, sizeof(temp_u8));
    
    temp_u8 = dev->fp_threshold;
    fpc1020_reg_write(dev, FPC1020_REG_FNGR_DET_THRES, (void *)&temp_u8, NULL, sizeof(temp_u8));
    
    temp_u16 = 0x0005;
    fpc1020_reg_write(dev, FPC1020_REG_FNGR_DET_CNTR, (void *)&temp_u16, NULL, sizeof(temp_u16));
    
    return 0;
}

/* -------------------------------------------------------------------- */
static int fpc1020_write_sensor_a3a4_setup(fpc1020_t *dev)
{
    uint8_t temp_u8;
    uint16_t temp_u16;
    uint64_t temp_u64;
    const int mux = dev->fp_conditions;
    const int rev = dev->revision;

    if (rev == 4) {
        temp_u8 = 0x09;
        fpc1020_reg_write(dev, FPC102X_REG_FINGER_DRIVE_DLY, (void *)&temp_u8, NULL, sizeof(temp_u8));

        temp_u64 = 0x0808080814141414;
        fpc1020_reg_write(dev, FPC102X_REG_SAMPLE_PX_DLY, (void *)&temp_u64, NULL, sizeof(temp_u64));
    } else if (rev == 3) {
        temp_u64 = 0x1717171723232323;
        fpc1020_reg_write(dev, FPC102X_REG_SAMPLE_PX_DLY, (void *)&temp_u64, NULL, sizeof(temp_u64));

        temp_u8 = 0x0f;
        fpc1020_reg_write(dev, FPC102X_REG_PXL_RST_DLY, (void *)&temp_u8, NULL, sizeof(temp_u8));

        temp_u8 = 0x18;
        fpc1020_reg_write(dev, FPC102X_REG_FINGER_DRIVE_DLY, (void *)&temp_u8, NULL, sizeof(temp_u8));
    } else {
        return -1;
    }

    temp_u8 = 0x02;        /* internal supply */
    fpc1020_reg_write(dev, FPC102X_REG_FINGER_DRIVE_CONF, (void *)&temp_u8, NULL, sizeof(temp_u8));

    temp_u16 = dev->setup.adc_shift[mux];
    temp_u16 <<= 8;
    temp_u16 |= dev->setup.adc_gain[mux];
    fpc1020_reg_write(dev, FPC102X_REG_ADC_SHIFT_GAIN, (void *)&temp_u16, NULL, sizeof(temp_u16));

    temp_u16 = dev->setup.pxl_ctrl[mux];
    fpc1020_reg_write(dev, FPC102X_REG_PXL_CTRL, (void *)&temp_u16, NULL, sizeof(temp_u16));

    temp_u8 = 0x03 | 0x08;
    fpc1020_reg_write(dev, FPC102X_REG_IMAGE_SETUP, (void *)&temp_u8, NULL, sizeof(temp_u8));

    temp_u8 = dev->fp_threshold;
    fpc1020_reg_write(dev, FPC1020_REG_FNGR_DET_THRES, (void *)&temp_u8, NULL, sizeof(temp_u8));

    temp_u16 = 0x0005;
    fpc1020_reg_write(dev, FPC1020_REG_FNGR_DET_CNTR, (void *)&temp_u16, NULL, sizeof(temp_u16));
    
    return 0;
}

static int fpc1020_write_sensor_setup(fpc1020_t *dev)
{
    switch (dev->revision) {
    case 1:
    case 2:
        return fpc1020_write_sensor_a1a2_setup(dev);

    case 3:
    case 4:
        return fpc1020_write_sensor_a3a4_setup(dev);

    default:
        return fpc1020_write_sensor_a3a4_setup(dev);
        break;
    }

    return 0;
}

int fpc1020_capture_settings(fpc1020_t *dev, int select)
{
    uint16_t pxlCtrl;
    uint16_t adc_shift_gain;

    if (select >= FPC1020_BUFFER_MAX_IMAGES) {
        return -FPC102X_ERROR_OUT_OF_BUFFER;
    }

    pxlCtrl = dev->setup.pxl_ctrl[select];

    adc_shift_gain = dev->setup.adc_shift[select];
    adc_shift_gain <<= 8;
    adc_shift_gain |= dev->setup.adc_gain[select];

    fpc1020_reg_write(dev, FPC102X_REG_PXL_CTRL, (void *)&pxlCtrl, NULL, sizeof(pxlCtrl));

    fpc1020_reg_write(dev, FPC102X_REG_ADC_SHIFT_GAIN, (void *)&adc_shift_gain, NULL, sizeof(adc_shift_gain));

    return FPC102X_ERROR_NO_ERROR;
}

int fpc1020_get_fingerprint(fpc1020_t *dev) {
    int error = 0;
    
    error = fpc1020_write_sensor_setup(dev);
    if (error < 0) {
        return error;
    }
    
    error = fpc1020_image_crop(dev,
                               dev->setup.capture_col_start,
                               dev->setup.capture_col_groups,
                               dev->setup.capture_row_start,
                               dev->setup.capture_row_count);
    
    int image_size = dev->setup.capture_row_count * dev->setup.capture_col_groups * FPC1020_ADC_GROUP_SIZE;
    
    for (int i = 0; i < FPC1020_BUFFER_MAX_IMAGES; i++) {
        DEBUG("Getting fingerprint image %d of %d\n", i, FPC1020_BUFFER_MAX_IMAGES);
        error = fpc1020_capture_settings(dev, i);
        if (error) {
            return error;
        }
        
        error = fpc1020_get_image(dev, dev->image + (i*image_size), image_size);
        if (error < 0) {
            return error;
        }
    }
    
    return image_size;
}

int fpc1020_init(fpc1020_t *dev, spi_t spi, gpio_t cs, gpio_t reset, gpio_t irq) {
    dev->spi = spi;
    dev->cs = cs;
    dev->reset = reset;
    dev->irq = irq;
    
    dev->fp_conditions = FPC1020_CONDITIONS_DRY;
    
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
    
    res = fpc1020_setup_defaults(dev);
    if (res < 0) {
        DEBUG("Error setting defaults\n");
        return res;
    }
    
    /*
    error = fpc1020_test_deadpixels(fpc1020);
    if (res < 0) {
        DEBUG("Error testing for dead pixels\n");
        return res;
    }
    */
    
    return 0;
}