/*
 * Copyright (c) 2018 Unwired Devices LLC <info@unwds.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_lis3dh
 * @{
 *
 * @files       lis3dh.c
 * @brief       Implementation of LIS3DH SPI/I2C driver
 *
 * @author      Alexander Ugorelov <alex_u@unwds.com>
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "xtimer.h"

#include "periph/gpio.h"

#include "lis3dh.h"
#include "include/lis3dh_internal.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#include "log.h"

#if ENABLE_DEBUG
    #define PRINTBUFF _printbuff
    static void _printbuff(uint8_t *buff, unsigned len)
    {
        while (len) {
            len--;
            printf("%02Xh ", *buff++);
        }
        printf("\n");
    }
#else
    #define PRINTBUFF(...)
#endif


#if defined (MODULE_LIS3DH_SPI)
#include "periph/spi.h"
    #define SPI_MODE        SPI_MODE_3
    #define DEV_SPI         (dev->params.spi)
    #define DEV_CS          (dev->params.cs)
    #define DEV_CLK         (dev->params.clk)
    #define DEV_SCALE       (dev->params.scale)
#elif defined (MODULE_LIS3DH_I2C)
    #include "periph/i2c.h"
    #define DEV_I2C         (dev->params.i2c)
    #define DEV_ADDR        (dev->params.addr)
    #define DEV_SCALE       (dev->params.scale)
#endif

/**
 * @brief Write a data to registers in the LIS3DH.
 *
 * @param[in] dev     Device descriptor
 * @param[in] reg     The source register starting address
 * @param[in] data    The values of the source registers will be written here
 * @param[in] length  Number of bytes to write
 * 
 * @return            Error status
 */
static int _write(const lis3dh_t *dev, uint8_t reg, uint8_t *data, uint16_t length);

/**
 * @brief Read sequential registers from the LIS3DH.
 *
 * @param[in]  dev     Device descriptor
 * @param[in]  reg     The source register starting address
 * @param[out] data    The values of the source registers will be written here
 * @param[in]  length  Number of bytes to read
 *
 * @return             Error status
 */
static int _read(const lis3dh_t *dev, uint8_t reg, uint8_t *data, uint16_t length);

/**
 * @brief   Initialize a LIS3DH sensor intereface read/write
 *
 * @param[out] dev         Device descriptor of sensor to initialize
 * @param[in]  params      Configuration parameters
 *
 * @return                 Error status
 */
static void _platform_init(lis3dh_t *dev, const lis3dh_params_t *params);

/**
 * @brief  Read generic device register
 *
 * @param  dev    Device descriptor
 * @param  reg    Address of the register to read
 * @param  data   Pointer to buffer that store the data read
 * @param  len    Number of consecutive register to read
 * 
 * @return        Error status
 */
int32_t lis3dh_read_reg(lis3dh_t *dev, uint8_t reg, uint8_t* data, uint16_t len);

/**
 * @brief  Write generic device register
 *
 * @param  dev    Device descriptor
 * @param  reg    Address of the register to write
 * @param  data   Pointer to data to write in register reg
 * @param  len    Number of consecutive register to write
 * 
 * @return        Error status
 */
int32_t lis3dh_write_reg(lis3dh_t *dev, uint8_t reg, uint8_t* data, uint16_t len);


/**
 * @defgroup  LIS3DH_Data_generation
 * @brief     This section group all the functions concerning data generation.
 * @{
 *
 */

/**
 * @brief  Temperature status register.[get]
 *
 * @param  dev    Device descriptor
 * @param  buff   Buffer that stores data read
 * 
 * @return        Error status
 */
int32_t lis3dh_temp_status_reg_get(lis3dh_t *dev, uint8_t *buff);

/**
 * @brief  Temperature data available.[get]
 *
 * @param  dev    Device descriptor
 * @param  val    Get the values of tda in reg STATUS_REG_AUX
 * 
 * @return        Error status
 */
int32_t lis3dh_temp_data_ready_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  Temperature data overrun.[get]
 *
 * @param  dev    Device descriptor
 * @param  val    Get the values of tor in reg STATUS_REG_AUX
 * 
 * @return        Error status
 */
int32_t lis3dh_temp_data_ovr_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  Temperature output value.[get]
 *
 * @param  dev    Device descriptor
 * @param  buff   Buffer that stores data read
 * 
 * @return        Error status
 */
int32_t lis3dh_temperature_raw_get(lis3dh_t *dev, uint8_t *buff);

/**
 * @brief  ADC output value.[get]
 *         Sample frequency: the same as the ODR CTRL_REG1
 *         The resolution:
 *                    10bit if LPen bit in CTRL_REG1 (20h) is clear
 *                     8bit if LPen bit in CTRL_REG1 (20h) is set
 *         Data Format:
 *                     Outputs are Left Justified in 2’ complements
 *                     range 800mV
 *                     code zero means an analogue value of about 1.2V
 *                     Voltage values smaller than centre values are positive
 *                           (Example:  800mV = 7Fh / 127 dec)
 *                     Voltage values bigger than centre values are negative
 *                           (Example: 1600mV = 80h / -128 dec)
 *
 * @param  dev    Device descriptor
 * @param  buff   Buffer that stores data read
 * 
 * @return        Error status
 */
int32_t lis3dh_adc_raw_get(lis3dh_t *dev, uint8_t *buff);

/**
 * @brief  Auxiliary ADC.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Configure the auxiliary ADC
 * 
 * @return       Error status
 */
int32_t lis3dh_aux_adc_set(lis3dh_t *dev, lis3dh_temp_en_t val);

/**
 * @brief  Auxiliary ADC.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the auxiliary ADC
 * 
 * @return       Error status
 */
int32_t lis3dh_aux_adc_get(lis3dh_t *dev, lis3dh_temp_en_t *val);

/**
 * @brief  Operating mode selection.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of lpen in reg CTRL_REG1
 *               and HR in reg CTRL_REG4
 *                  
 * @return       Error status
 */
int32_t lis3dh_operating_mode_set(lis3dh_t *dev, lis3dh_res_t val);

/**
 * @brief  Operating mode selection.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of lpen in reg CTRL_REG1
 *               and HR in reg CTRL_REG4
 *
 * @return       Error status
 */
int32_t lis3dh_operating_mode_get(lis3dh_t *dev, lis3dh_res_t *val);

/**
 * @brief  Output data rate selection.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of odr in reg CTRL_REG1
 *
 * @return       Error status
 */
int32_t lis3dh_data_rate_set(lis3dh_t *dev, lis3dh_odr_t val);

/**
 * @brief  Output data rate selection.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of odr in reg CTRL_REG1
 *
 * @return       Error status
 */
int32_t lis3dh_data_rate_get(lis3dh_t *dev, lis3dh_odr_t *val);

/**
 * @brief   High pass data from internal filter sent to output register
 *          and FIFO.
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of fds in reg CTRL_REG2
 *
 * @return       Error status
 */
int32_t lis3dh_high_pass_on_outputs_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief   High pass data from internal filter sent to output register
 *          and FIFO.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of fds in reg CTRL_REG2
 *
 * @return       Error status
 */
int32_t lis3dh_high_pass_on_outputs_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief   High-pass filter cutoff frequency selection.[set]
 *
 * HPCF[2:1]\ft @1Hz    @10Hz  @25Hz  @50Hz @100Hz @200Hz @400Hz @1kHz6 ft@5kHz
 * AGGRESSIVE   0.02Hz  0.2Hz  0.5Hz  1Hz   2Hz    4Hz    8Hz    32Hz   100Hz
 * STRONG       0.008Hz 0.08Hz 0.2Hz  0.5Hz 1Hz    2Hz    4Hz    16Hz   50Hz
 * MEDIUM       0.004Hz 0.04Hz 0.1Hz  0.2Hz 0.5Hz  1Hz    2Hz    8Hz    25Hz
 * LIGHT        0.002Hz 0.02Hz 0.05Hz 0.1Hz 0.2Hz  0.5Hz  1Hz    4Hz    12Hz
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of hpcf in reg CTRL_REG2
 *
 * @return       Error status
 */
int32_t lis3dh_high_pass_bandwidth_set(lis3dh_t *dev, lis3dh_hpcf_t val);

/**
 * @brief   High-pass filter cutoff frequency selection.[get]
 *
 * HPCF[2:1]\ft @1Hz    @10Hz  @25Hz  @50Hz @100Hz @200Hz @400Hz @1kHz6 ft@5kHz
 * AGGRESSIVE   0.02Hz  0.2Hz  0.5Hz  1Hz   2Hz    4Hz    8Hz    32Hz   100Hz
 * STRONG       0.008Hz 0.08Hz 0.2Hz  0.5Hz 1Hz    2Hz    4Hz    16Hz   50Hz
 * MEDIUM       0.004Hz 0.04Hz 0.1Hz  0.2Hz 0.5Hz  1Hz    2Hz    8Hz    25Hz
 * LIGHT        0.002Hz 0.02Hz 0.05Hz 0.1Hz 0.2Hz  0.5Hz  1Hz    4Hz    12Hz
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of hpcf in reg CTRL_REG2
 *
 * @return       Error status
 */
int32_t lis3dh_high_pass_bandwidth_get(lis3dh_t *dev, lis3dh_hpcf_t *val);

/**
 * @brief  High-pass filter mode selection.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of hpm in reg CTRL_REG2
 *
 * @return       Error status
 */
int32_t lis3dh_high_pass_mode_set(lis3dh_t *dev, lis3dh_hpm_t val);

/**
 * @brief  High-pass filter mode selection.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of hpm in reg CTRL_REG2
 *
 * @return       Error status
 */
int32_t lis3dh_high_pass_mode_get(lis3dh_t *dev, lis3dh_hpm_t *val);

/**
 * @brief  Full-scale configuration.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of fs in reg CTRL_REG4
 *
 * @return       Error status
 */
int32_t lis3dh_full_scale_set(lis3dh_t *dev, lis3dh_scale_t val);

/**
 * @brief  Full-scale configuration.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of fs in reg CTRL_REG4
 *
 * @return       Error status
 */
int32_t lis3dh_full_scale_get(lis3dh_t *dev, lis3dh_scale_t *val);

/**
 * @brief  Block Data Update.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Сhange the values of bdu in reg CTRL_REG4
 *
 * @return       Error status
 */
int32_t lis3dh_block_data_update_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  Block Data Update.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of bdu in reg CTRL_REG4
 *
 * @return       Error status
 */
int32_t lis3dh_block_data_update_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  Reference value for interrupt generation.[set]
 *         LSB = ~16@2g / ~31@4g / ~63@8g / ~127@16g
 *
 * @param  dev   Device descriptor
 * @param  buff  Buffer that contains data to write
 *
 * @return       Error status
 */
int32_t lis3dh_filter_reference_set(lis3dh_t *dev, uint8_t *buff);

/**
 * @brief  Reference value for interrupt generation.[get]
 *         LSB = ~16@2g / ~31@4g / ~63@8g / ~127@16g
 *
 * @param  dev   Device descriptor
 * @param  buff  Buffer that stores data read
 *
 * @return       Error status
 */
int32_t lis3dh_filter_reference_get(lis3dh_t *dev, uint8_t *buff);

/**
 * @brief  Acceleration set of data available.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of zyxda in reg STATUS_REG
 *
 * @return       Error status
 */
int32_t lis3dh_xl_data_ready_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  Acceleration set of data overrun.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of zyxor in reg STATUS_REG
 *
 * @return       Error status
 */
int32_t lis3dh_xl_data_ovr_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  Acceleration output value.[get]
 *
 * @param  dev   Device descriptor
 * @param  buff  Buffer that stores data read
 *
 * @return       Error status
 */
int32_t lis3dh_acceleration_raw_get(lis3dh_t *dev, uint8_t *buff);

/**
 * @brief All axis enable.[set]
 * 
 * @param  dev   Device descriptor
 * @param  val   Сhange the values of zen, yen, xen in reg CTRL_REG1
 *
 * @return       Error status
 */
int32_t lis3dh_axis_set(lis3dh_t *dev, lis3dh_axis_t val);

/**
 * @brief All axis enable.[get]
 * 
 * @param  dev   Device descriptor
 * @param  val   Get the values of zen, yen, xen in reg CTRL_REG1
 *
 * @return       Error status
 */
int32_t lis3dh_axis_get(lis3dh_t *dev, lis3dh_axis_t *val);
/** @} */

/**
 * @defgroup  LIS3DH_Common
 * @brief     This section group common usefull functions
 * @{
 *
 */
/**
 * @brief  DeviceWhoamI .[get]
 *
 * @param  dev   Device descriptor
 * @param  buff  Buffer that stores data read
 *
 * @return       Error status
 */
int32_t lis3dh_device_id_get(lis3dh_t *dev, uint8_t *buff);

/**
 * @brief  Self Test.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of st in reg CTRL_REG4
 *
 * @return       Error status
 */
int32_t lis3dh_self_test_set(lis3dh_t *dev, lis3dh_st_t val);

/**
 * @brief  Self Test.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of st in reg CTRL_REG4
 *
 * @return       Error status
 */
int32_t lis3dh_self_test_get(lis3dh_t *dev, lis3dh_st_t *val);

/**
 * @brief  Big/Little Endian data selection.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of ble in reg CTRL_REG4
 *
 * @return       Error status
 */
int32_t lis3dh_data_format_set(lis3dh_t *dev, lis3dh_ble_t val);

/**
 * @brief  Big/Little Endian data selection.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of ble in reg CTRL_REG4
 *
 * @return       Error status
 */
int32_t lis3dh_data_format_get(lis3dh_t *dev, lis3dh_ble_t *val);

/**
 * @brief  Reboot memory content. Reload the calibration parameters.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of boot in reg CTRL_REG5
 *
 * @return       Error status
 */
int32_t lis3dh_boot_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  Reboot memory content. Reload the calibration parameters.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of boot in reg CTRL_REG5
 *
 * @return       Error status
 */
int32_t lis3dh_boot_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  Info about device status.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of register STATUS_REG
 *
 * @return       Error status
 */
int32_t lis3dh_status_get(lis3dh_t *dev, lis3dh_status_reg_t *val);
/** @} */

/**
 * @defgroup   LIS3DH_Interrupts_generator_1
 * @brief      This section group all the functions that manage the first
 *             interrupts generator
 * @{
 *
 */
/**
 * @brief  Interrupt generator 1 configuration register.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of register INT1_CFG
 *
 * @return       Error status
 */
int32_t lis3dh_int1_gen_conf_set(lis3dh_t *dev, lis3dh_int1_cfg_t *val);

/**
 * @brief  Interrupt generator 1 configuration register.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of register INT1_CFG
 *
 * @return       Error status
 */
int32_t lis3dh_int1_gen_conf_get(lis3dh_t *dev, lis3dh_int1_cfg_t *val);

/**
 * @brief  Interrupt generator 1 source register.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of register INT1_SRC
 *
 * @return       Error status
 */
int32_t lis3dh_int1_gen_source_get(lis3dh_t *dev, lis3dh_int1_src_t *val);

/**
 * @brief  User-defined threshold value for xl interrupt event on
 *         generator 1.[set]
 *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of ths in reg INT1_THS
 *
 * @return       Error status
 */
int32_t lis3dh_int1_gen_threshold_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  User-defined threshold value for xl interrupt event on
 *         generator 1.[get]
 *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of ths in reg INT1_THS
 *
 * @return       Error status
 */
int32_t lis3dh_int1_gen_threshold_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
 *         recognized.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of d in reg INT1_DURATION
 *
 * @return       Error status
 */
int32_t lis3dh_int1_gen_duration_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 1 event to be
 *         recognized.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of d in reg INT1_DURATION
 *
 * @return       Error status
 */
int32_t lis3dh_int1_gen_duration_get(lis3dh_t *dev, uint8_t *val);
/** @} */

/**
 * @defgroup   LIS3DH_Interrupts_generator_2
 * @brief      This section group all the functions that manage the second
 *             interrupts generator
 * @{
 *
 */
/**
 * @brief  Interrupt generator 2 configuration register.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of register INT2_CFG
 *
 * @return       Error status
 */
int32_t lis3dh_int2_gen_conf_set(lis3dh_t *dev, lis3dh_int2_cfg_t *val);

/**
 * @brief  Interrupt generator 2 configuration register.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of register INT2_CFG
 *
 * @return       Error status
 */
int32_t lis3dh_int2_gen_conf_get(lis3dh_t *dev, lis3dh_int2_cfg_t *val);

/**
 * @brief  Interrupt generator 2 source register.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of register INT2_SRC
 *
 * @return       Error status
 */
int32_t lis3dh_int2_gen_source_get(lis3dh_t *dev, lis3dh_int2_src_t *val);

/**
 * @brief  User-defined threshold value for xl interrupt event on
 *         generator 2.[set]
 *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of ths in reg INT2_THS
 *
 * @return       Error status
 */
int32_t lis3dh_int2_gen_threshold_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  User-defined threshold value for xl interrupt event on
 *         generator 2.[get]
 *         LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of ths in reg INT2_THS
 *
 * @return       Error status
 */
int32_t lis3dh_int2_gen_threshold_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 2 event to be
 *         recognized.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of d in reg INT2_DURATION
 *
 * @return       Error status
 */
int32_t lis3dh_int2_gen_duration_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  The minimum duration (LSb = 1/ODR) of the Interrupt 2 event to be
 *         recognized.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of d in reg INT2_DURATION
 *
 * @return       Error status
 */
int32_t lis3dh_int2_gen_duration_get(lis3dh_t *dev, uint8_t *val);
/** @} */

/**
 * @defgroup  LIS3DH_Interrupt_pins
 * @brief     This section group all the functions that manage interrup pins
 * @{
 *
 */
/**
 * @brief  High-pass filter on interrupts/tap generator.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of hp in reg CTRL_REG2
 *
 * @return       Error status
 */
int32_t lis3dh_high_pass_int_conf_set(lis3dh_t *dev, lis3dh_hp_t val);

/**
 * @brief  High-pass filter on interrupts/tap generator.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of hp in reg CTRL_REG2
 *
 * @return       Error status
 */
int32_t lis3dh_high_pass_int_conf_get(lis3dh_t *dev, lis3dh_hp_t *val);

/**
 * @brief  Int1 pin routing configuration register.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of registers CTRL_REG3
 *
 * @return       Error status
 */
int32_t lis3dh_pin_int1_config_set(lis3dh_t *dev, lis3dh_ctrl_reg3_t *val);

/**
 * @brief  Int1 pin routing configuration register.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of registers CTRL_REG3
 *
 * @return       Error status
 */
int32_t lis3dh_pin_int1_config_get(lis3dh_t *dev, lis3dh_ctrl_reg3_t *val);

/**
 * @brief  int2_pin_detect_4d: [set]  4D enable: 4D detection is enabled
 *                                    on INT2 pin when 6D bit on
 *                                    INT2_CFG (34h) is set to 1.
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of d4d_int2 in reg CTRL_REG5
 *
 * @return       Error status
 */
int32_t lis3dh_int2_pin_detect_4d_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  4D enable: 4D detection is enabled on INT2 pin when 6D bit on
 *         INT2_CFG (34h) is set to 1.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of d4d_int2 in reg CTRL_REG5
 *
 * @return       Error status
 */
int32_t lis3dh_int2_pin_detect_4d_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief   Latch interrupt request on INT2_SRC (35h) register, with
 *          INT2_SRC (35h) register cleared by reading INT2_SRC(35h)
 *          itself.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of lir_int2 in reg CTRL_REG5
 *
 * @return       Error status
 */
int32_t lis3dh_int2_pin_notification_mode_set(lis3dh_t *dev, lis3dh_lir_int2_t val);

/**
 * @brief   Latch interrupt request on INT2_SRC (35h) register, with
 *          INT2_SRC (35h) register cleared by reading INT2_SRC(35h)
 *          itself.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of lir_int2 in reg CTRL_REG5
 *
 * @return       Error status
 */
int32_t lis3dh_int2_pin_notification_mode_get(lis3dh_t *dev, lis3dh_lir_int2_t *val);

/**
 * @brief  4D enable: 4D detection is enabled on INT1 pin when 6D bit
 *                    on INT1_CFG(30h) is set to 1.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of d4d_int1 in reg CTRL_REG5
 *
 * @return       Error status
 */
int32_t lis3dh_int1_pin_detect_4d_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  4D enable: 4D detection is enabled on INT1 pin when 6D bit on
 *         INT1_CFG(30h) is set to 1.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of d4d_int1 in reg CTRL_REG5
 *
 * @return       Error status
 */
int32_t lis3dh_int1_pin_detect_4d_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief   Latch interrupt request on INT1_SRC (31h), with INT1_SRC(31h)
 *          register cleared by reading INT1_SRC (31h) itself.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of lir_int1 in reg CTRL_REG5
 *
 * @return       Error status
 */
int32_t lis3dh_int1_pin_notification_mode_set(lis3dh_t *dev, lis3dh_lir_int1_t val);

/**
 * @brief   Latch interrupt request on INT1_SRC (31h), with INT1_SRC(31h)
 *          register cleared by reading INT1_SRC (31h) itself.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of lir_int1 in reg CTRL_REG5
 *
 * @return       Error status
 */
int32_t lis3dh_int1_pin_notification_mode_get(lis3dh_t *dev, lis3dh_lir_int1_t *val);

/**
 * @brief  Int2 pin routing configuration register.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of registers CTRL_REG6
 *
 * @return       Error status
 */
int32_t lis3dh_pin_int2_config_set(lis3dh_t *dev, lis3dh_ctrl_reg6_t *val);

/**
 * @brief  Int2 pin routing configuration register.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of registers CTRL_REG6
 *
 * @return       Error status
 */
int32_t lis3dh_pin_int2_config_get(lis3dh_t *dev, lis3dh_ctrl_reg6_t *val);
/** @} */

/**
 * @defgroup  LIS3DH_Fifo
 * @brief     This section group all the functions concerning the fifo usage
 * @{
 *
 */
/**
 * @brief  FIFO enable.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of fifo_en in reg CTRL_REG5
 *
 * @return       Error status
 */
int32_t lis3dh_fifo_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  FIFO enable.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of fifo_en in reg CTRL_REG5
 *
 * @return       Error status
 */
int32_t lis3dh_fifo_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  FIFO watermark level selection.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of fth in reg FIFO_CTRL_REG
 *
 * @return       Error status
 */
int32_t lis3dh_fifo_watermark_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  FIFO watermark level selection.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of fth in reg FIFO_CTRL_REG
 *
 * @return       Error status
 */
int32_t lis3dh_fifo_watermark_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief Trigger FIFO selection.[set]
 * 
 * @param  dev   Device descriptor
 * @param  val   Change the values of tr in reg FIFO_CTRL_REG
 *
 * @return       Error status
 */
int32_t lis3dh_fifo_trigger_event_set(lis3dh_t *dev, lis3dh_tr_t val);

/**
 * @brief Trigger FIFO selection.[get]
 * 
 * @param  dev   Device descriptor
 * @param  val   Get the values of tr in reg FIFO_CTRL_REG
 *
 * @return       Error status
 */
int32_t lis3dh_fifo_trigger_event_get(lis3dh_t *dev, lis3dh_tr_t *val);

/**
 * @brief  FIFO mode selection.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of fm in reg FIFO_CTRL_REG
 *
 * @return       Error status
 */
int32_t lis3dh_fifo_mode_set(lis3dh_t *dev, lis3dh_fm_t val);

/**
 * @brief  FIFO mode selection.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of fm in reg FIFO_CTRL_REG
 *
 * @return       Error status
 */
int32_t lis3dh_fifo_mode_get(lis3dh_t *dev, lis3dh_fm_t *val);

/**
 * @brief  FIFO status register.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of registers FIFO_SRC_REG
 *
 * @return       Error status
 */
int32_t lis3dh_fifo_status_get(lis3dh_t *dev, lis3dh_fifo_src_reg_t *val);

/**
 * @brief  FIFO stored data level.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of fss in reg FIFO_SRC_REG
 *
 * @return       Error status
 */
int32_t lis3dh_fifo_data_level_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  Empty FIFO status flag.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of empty in reg FIFO_SRC_REG
 *
 * @return       Error status
 */
int32_t lis3dh_fifo_empty_flag_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  FIFO overrun status flag.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of ovrn_fifo in reg FIFO_SRC_REG
 *
 * @return       Error status
 */
int32_t lis3dh_fifo_ovr_flag_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  FIFO watermark status.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of wtm in reg FIFO_SRC_REG
 *
 * @return       Error status
 */
int32_t lis3dh_fifo_fth_flag_get(lis3dh_t *dev, uint8_t *val);
/** @} */

/**
 * @defgroup  LIS3DH_Tap_generator
 * @brief     This section group all the functions that manage the tap and
 *            double tap event generation
 * @{
 *
 */
/**
 * @brief  Tap/Double Tap generator configuration register.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of registers CLICK_CFG
 *
 * @return       Error status
 */
int32_t lis3dh_tap_conf_set(lis3dh_t *dev, lis3dh_click_cfg_t *val);

/**
 * @brief  Tap/Double Tap generator configuration register.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of registers CLICK_CFG
 *
 * @return       Error status
 */
int32_t lis3dh_tap_conf_get(lis3dh_t *dev, lis3dh_click_cfg_t *val);

/**
 * @brief  Tap/Double Tap generator source register.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of registers CLICK_SRC
 *
 * @return       Error status
 */
int32_t lis3dh_tap_source_get(lis3dh_t *dev, lis3dh_click_src_t *val);

/**
 * @brief  User-defined threshold value for Tap/Double Tap event.[set]
 *         1 LSB = full scale/128
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of ths in reg CLICK_THS
 *
 * @return       Error status
 */
int32_t lis3dh_tap_threshold_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  User-defined threshold value for Tap/Double Tap event.[get]
 *         1 LSB = full scale/128
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of ths in reg CLICK_THS
 *
 * @return       Error status
 */
int32_t lis3dh_tap_threshold_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief   If the LIR_Click bit is not set, the interrupt is kept high
 *          for the duration of the latency window.
 *          If the LIR_Click bit is set, the interrupt is kept high until the
 *          CLICK_SRC(39h) register is read.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of lir_click in reg CLICK_THS
 *
 * @return       Error status
 */
int32_t lis3dh_tap_notification_mode_set(lis3dh_t *dev, lis3dh_lir_click_t val);

/**
 * @brief   If the LIR_Click bit is not set, the interrupt is kept high
 *          for the duration of the latency window.
 *          If the LIR_Click bit is set, the interrupt is kept high until the
 *          CLICK_SRC(39h) register is read.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of lir_click in reg CLICK_THS
 *
 * @return       Error status
 */
int32_t lis3dh_tap_notification_mode_get(lis3dh_t *dev, lis3dh_lir_click_t *val);

/**
 * @brief  The maximum time (1 LSB = 1/ODR) interval that can elapse
 *         between the start of the click-detection procedure and when the
 *         acceleration falls back below the threshold.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of tli in reg TIME_LIMIT
 *
 * @return       Error status
 */
int32_t lis3dh_shock_dur_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  The maximum time (1 LSB = 1/ODR) interval that can elapse
 *         between the start of the click-detection procedure and when the
 *         acceleration falls back below the threshold.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of tli in reg TIME_LIMIT
 *
 * @return       Error status
 */
int32_t lis3dh_shock_dur_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  The time (1 LSB = 1/ODR) interval that starts after the first
 *         click detection where the click-detection procedure is
 *         disabled, in cases where the device is configured for
 *         double-click detection.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of tla in reg TIME_LATENCY
 *
 * @return       Error status
 */
int32_t lis3dh_quiet_dur_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  The time (1 LSB = 1/ODR) interval that starts after the first
 *         click detection where the click-detection procedure is
 *         disabled, in cases where the device is configured for
 *         double-click detection.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of tla in reg TIME_LATENCY
 *
 * @return       Error status
 */
int32_t lis3dh_quiet_dur_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  The maximum interval of time (1 LSB = 1/ODR) that can elapse
 *         after the end of the latency interval in which the click-detection
 *         procedure can start, in cases where the device is configured
 *         for double-click detection.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of tw in reg TIME_WINDOW
 *
 * @return       Error status
 */
int32_t lis3dh_double_tap_timeout_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  The maximum interval of time (1 LSB = 1/ODR) that can elapse
 *         after the end of the latency interval in which the click-detection
 *         procedure can start, in cases where the device is configured
 *         for double-click detection.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of tw in reg TIME_WINDOW
 *
 * @return       Error status
 */
int32_t lis3dh_double_tap_timeout_get(lis3dh_t *dev, uint8_t *val);
/** @} */

/**
 * @defgroup  LIS3DH_Activity_inactivity
 * @brief     This section group all the functions concerning activity
 *            inactivity functionality
 * @{
 *
 */
/**
 * @brief    Sleep-to-wake, return-to-sleep activation threshold in
 *           low-power mode.[set]
 *           1 LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of acth in reg ACT_THS
 *
 * @return       Error status
 */
int32_t lis3dh_act_threshold_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief    Sleep-to-wake, return-to-sleep activation threshold in
 *           low-power mode.[get]
 *           1 LSb = 16mg@2g / 32mg@4g / 62mg@8g / 186mg@16g
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of acth in reg ACT_THS
 *
 * @return       Error status
 */
int32_t lis3dh_act_threshold_get(lis3dh_t *dev, uint8_t *val);

/**
 * @brief  Sleep-to-wake, return-to-sleep.[set]
 *         duration = (8*1[LSb]+1)/ODR
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of actd in reg ACT_DUR
 *
 * @return       Error status
 */
int32_t lis3dh_act_timeout_set(lis3dh_t *dev, uint8_t val);

/**
 * @brief  Sleep-to-wake, return-to-sleep.[get]
 *         duration = (8*1[LSb]+1)/ODR
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of actd in reg ACT_DUR
 *
 * @return       Error status
 */
int32_t lis3dh_act_timeout_get(lis3dh_t *dev, uint8_t *val);
/** @} */

/**
 * @defgroup  LIS3DH_Serial_interface
 * @brief     This section group all the functions concerning serial
 *            interface management
 * @{
 *
 */
/**
 * @brief  Connect/Disconnect SDO/SA0 internal pull-up.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of sdo_pu_disc in reg CTRL_REG0
 *
 * @return       Error status
 */
int32_t lis3dh_pin_sdo_sa0_mode_set(lis3dh_t *dev, lis3dh_sdo_pu_disc_t val);

/**
 * @brief  Connect/Disconnect SDO/SA0 internal pull-up.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of sdo_pu_disc in reg CTRL_REG0
 *
 * @return       Error status
 */
int32_t lis3dh_pin_sdo_sa0_mode_get(lis3dh_t *dev, lis3dh_sdo_pu_disc_t *val);

/**
 * @brief  SPI Serial Interface Mode selection.[set]
 *
 * @param  dev   Device descriptor
 * @param  val   Change the values of sim in reg CTRL_REG4
 *
 * @return       Error status
 */
int32_t lis3dh_spi_mode_set(lis3dh_t *dev, lis3dh_sim_t val);

/**
 * @brief  SPI Serial Interface Mode selection.[get]
 *
 * @param  dev   Device descriptor
 * @param  val   Get the values of sim in reg CTRL_REG4
 *
 * @return       Error status
 */
int32_t lis3dh_spi_mode_get(lis3dh_t *dev, lis3dh_sim_t *val);
/** @} */

/**
 * @defgroup  LIS3DH_Convert_raw_data
 * @brief     This section group all the functions converting raw data
 * 
 * @{
 *
 */
/**
 * @brief  Measurement accuracy.[get]
 * 
 * @param  dev   Device descriptor
 *
 * @return       Error status
 */
int32_t lis3dh_accuracy_get(lis3dh_t *dev);

/**
 * @brief  Accleration convert raw-data into engineering units.
 * 
 * @param  dev      Device descriptor
 * @param  acc_raw  Raw data accleration
 * 
 * @return          Error status
 */
int32_t lis3dh_calculation_acceleration(lis3dh_t *dev, int16_t acc_raw);

/**
 * @brief  Temperature convert raw-data into engineering units.
 * 
 * @param  dev      Device descriptor
 * @param temp_raw  Raw data temperature
 * 
 * @return          Error status
 */
int32_t lis3dh_calculation_temperature(lis3dh_t *dev, int16_t temp_raw);
/** @} */

/**
 * @defgroup  LIS3DH_Internal_Functions
 * @brief     This section group all the internal functions
 *
 * @{
 *
 */
#if defined (MODULE_LIS3DH_SPI)
static int _read(const lis3dh_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    /* Set READ MULTIPLE mode */
    uint8_t addr = (reg & LIS3DH_SPI_ADDRESS_MASK) | LIS3DH_SPI_READ_MASK |
                   LIS3DH_SPI_MULTI_MASK;

    /* Acquire exclusive access to the bus. */
    spi_acquire(DEV_SPI, DEV_CS, SPI_MODE, DEV_CLK);
    /* Perform the transaction */
    spi_transfer_regs(DEV_SPI, DEV_CS, addr, NULL, data, (size_t)length);
    /* Release the bus for other threads. */
    spi_release(DEV_SPI);

    return 0;
}

static int _write(const lis3dh_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    /* Set WRITE SINGLE mode */
    uint8_t addr = ((reg & LIS3DH_SPI_ADDRESS_MASK) | LIS3DH_SPI_WRITE_MASK |
                    LIS3DH_SPI_SINGLE_MASK);

    /* Acquire exclusive access to the bus. */
    spi_acquire(DEV_SPI, DEV_CS, SPI_MODE, DEV_CLK);
    /* Perform the transaction */
    spi_transfer_regs(DEV_SPI, DEV_CS, addr, data, NULL, (size_t)length);
    /* Release the bus for other threads. */
    spi_release(DEV_SPI);

    return 0;
}

static void _platform_init(lis3dh_t *dev, const lis3dh_params_t *params)
{
    dev->params = *params;

    uint8_t test;

    /* initialize the chip select line */
    spi_init(DEV_SPI);
    
    if (spi_init_cs(DEV_SPI, DEV_CS) != SPI_OK) {
        DEBUG("[lis3dh] error while initializing CS pin\n");
        return -1;
    }

    return 0;
}

#elif defined (MODULE_LIS3DH_I2C)

static int _read(const lis3dh_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    int status = 0x00;

    /* Read multiple command */
    reg &= ~(0x80);
    if (length > 1) {
        reg |= 0x80;
    }

    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->params.i2c_dev);
    /* Perform the transaction */
    status = i2c_read_regs(dev->params.i2c_dev, dev->params.i2c_addr, (uint16_t)reg, data, (size_t)length, 0);
    /* Release the bus for other threads. */
    i2c_release(dev->params.i2c_dev);

    DEBUG("LIS3DH [REG %02X]: <- ", (reg & 0x7F));
    PRINTBUFF(data, length);

    return status;
}

static int _write(const lis3dh_t *dev, uint8_t reg, uint8_t *data, uint16_t length)
{
    int status = 0x00;

    /* Write multiple command */
    reg &= ~(0x80);
    if (length > 1) {
        reg |= 0x80;
    }

    DEBUG("LIS3DH [REG %02X]: -> %02Xh ", (reg & 0x7F), reg);
    PRINTBUFF(data, length);

    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->params.i2c_dev);
    /* Perform the transaction */
    // int i2c_write_regs(i2c_t dev, uint16_t address, uint16_t reg, const void *data, size_t length, uint8_t flags)
    status = i2c_write_regs(dev->params.i2c_dev, dev->params.i2c_addr, (uint16_t)reg, data, (size_t)length, 0);
    /* Release the bus for other threads. */
    i2c_release(dev->params.i2c_dev);
    return status;
}

static void _platform_init(lis3dh_t *dev, const lis3dh_params_t *params) {
    dev->params = *params;

    /* Acquire exclusive access to the bus. */
    i2c_acquire(dev->params.i2c_dev);

    /* initialize the chip select line */
    i2c_init(dev->params.i2c_dev);

    /* Release the bus for other threads. */
    i2c_release(dev->params.i2c_dev);

}
#endif

int32_t lis3dh_read_reg(lis3dh_t *dev, uint8_t reg, uint8_t* data, uint16_t len)
{
    int32_t ret;
    ret = _read(dev, reg, data, len);
    return ret;
}

int32_t lis3dh_write_reg(lis3dh_t *dev, uint8_t reg, uint8_t* data, uint16_t len)
{
    int32_t ret;
    ret = _write(dev, reg, data, len);
    return ret;
}

int32_t lis3dh_temp_status_reg_get(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_STATUS_AUX, buff, 1);
    return ret;
}

int32_t lis3dh_temp_data_ready_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_status_reg_aux_t status_reg_aux;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_STATUS_AUX, (uint8_t*)&status_reg_aux, 1);
    *val = status_reg_aux._3da;

    return ret;
}

int32_t lis3dh_temp_data_ovr_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_status_reg_aux_t status_reg_aux;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_STATUS_AUX, (uint8_t*)&status_reg_aux, 1);
    *val = status_reg_aux._3or;

    return ret;
}

int32_t lis3dh_temperature_raw_get(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_OUT_AUX_ADC3_L, buff, 2);
    return ret;
}

int32_t lis3dh_adc_raw_get(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_OUT_AUX_ADC1_L, buff, 6);
    return ret;
}

int32_t lis3dh_aux_adc_set(lis3dh_t *dev, lis3dh_temp_en_t val)
{
    lis3dh_temp_cfg_reg_t temp_cfg_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TEMP_CFG_REG, (uint8_t*)&temp_cfg_reg, 1);
    if (ret == 0) {
        if (val != LIS3DH_AUX_DISABLE) {
            /* Required in order to use auxiliary adc */
            ret = lis3dh_block_data_update_set(dev, PROPERTY_ENABLE);
        }
    }
    if (ret == 0) {
        temp_cfg_reg.temp_en = ( (uint8_t) val & 0x02U) >> 1;
        temp_cfg_reg.adc_pd  = (uint8_t) val &  0x01U;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_TEMP_CFG_REG, (uint8_t*)&temp_cfg_reg, 1);
    }
    return ret;
}

int32_t lis3dh_aux_adc_get(lis3dh_t *dev, lis3dh_temp_en_t *val)
{
    lis3dh_temp_cfg_reg_t temp_cfg_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TEMP_CFG_REG, (uint8_t*)&temp_cfg_reg, 1);
    if ( ( temp_cfg_reg.temp_en & temp_cfg_reg.adc_pd ) == PROPERTY_ENABLE ) {
        *val = LIS3DH_AUX_ON_TEMPERATURE;
    }
    if ( ( temp_cfg_reg.temp_en  == PROPERTY_DISABLE ) &&
            ( temp_cfg_reg.adc_pd == PROPERTY_ENABLE ) ) {
        *val = LIS3DH_AUX_ON_PADS;
    } else {
        *val = LIS3DH_AUX_DISABLE;
    }
    return ret;
}

int32_t lis3dh_operating_mode_set(lis3dh_t *dev, lis3dh_res_t val)
{
    lis3dh_ctrl_reg1_t ctrl_reg1;
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    if (ret == 0) {
        ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    }
    if (ret == 0) {
        if ( val == LIS3DH_HR_12BIT ) {
            ctrl_reg1.lpen = 0;
            ctrl_reg4.hr   = 1;
        }
        if (val == LIS3DH_NM_10BIT) {
            ctrl_reg1.lpen = 0;
            ctrl_reg4.hr   = 0;
        }
        if (val == LIS3DH_LP_8BIT) {
            ctrl_reg1.lpen = 1;
            ctrl_reg4.hr   = 0;
        }
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    }
    if (ret == 0) {
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    }
    return ret;
}

int32_t lis3dh_operating_mode_get(lis3dh_t *dev, lis3dh_res_t *val)
{
    lis3dh_ctrl_reg1_t ctrl_reg1;
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    if (ret == 0) {
        ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
        if ( ctrl_reg1.lpen == PROPERTY_ENABLE ) {
            *val = LIS3DH_LP_8BIT;
            DEBUG("LIS3DH_LP_8BIT\n");
        }
        if (ctrl_reg4.hr == PROPERTY_ENABLE ) {
            *val = LIS3DH_HR_12BIT;
            DEBUG("LIS3DH_HR_12BIT\n");
        } else {
            *val = LIS3DH_NM_10BIT;
            DEBUG("LIS3DH_NM_10BIT\n");
        }
    }
    return ret;
}

int32_t lis3dh_data_rate_set(lis3dh_t *dev, lis3dh_odr_t val)
{
    lis3dh_ctrl_reg1_t ctrl_reg1;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    if (ret == 0) {
        ctrl_reg1.odr = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    }
    return ret;
}

int32_t lis3dh_data_rate_get(lis3dh_t *dev, lis3dh_odr_t *val)
{
    lis3dh_ctrl_reg1_t ctrl_reg1;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    switch (ctrl_reg1.odr) {
    case LIS3DH_POWER_DOWN:
        *val = LIS3DH_POWER_DOWN;
        break;
    case LIS3DH_ODR_1HZ:
        *val = LIS3DH_ODR_1HZ;
        break;
    case LIS3DH_ODR_10HZ:
        *val = LIS3DH_ODR_10HZ;
        break;
    case LIS3DH_ODR_25HZ:
        *val = LIS3DH_ODR_25HZ;
        break;
    case LIS3DH_ODR_50HZ:
        *val = LIS3DH_ODR_50HZ;
        break;
    case LIS3DH_ODR_100HZ:
        *val = LIS3DH_ODR_100HZ;
        break;
    case LIS3DH_ODR_200HZ:
        *val = LIS3DH_ODR_200HZ;
        break;
    case LIS3DH_ODR_400HZ:
        *val = LIS3DH_ODR_400HZ;
        break;
    case LIS3DH_ODR_1KHZ620_LP:
        *val = LIS3DH_ODR_1KHZ620_LP;
        break;
    case LIS3DH_ODR_5KHZ376_LP_1KHZ344_NM_HP:
        *val = LIS3DH_ODR_5KHZ376_LP_1KHZ344_NM_HP;
        break;
    default:
        *val = LIS3DH_POWER_DOWN;
        break;
    }
    return ret;
}

int32_t lis3dh_high_pass_on_outputs_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    if (ret == 0) {
        ctrl_reg2.fds = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    }
    return ret;
}

int32_t lis3dh_high_pass_on_outputs_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    *val = (uint8_t)ctrl_reg2.fds;

    return ret;
}

int32_t lis3dh_high_pass_bandwidth_set(lis3dh_t *dev, lis3dh_hpcf_t val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    if (ret == 0) {
        ctrl_reg2.hpcf = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    }
    return ret;
}

int32_t lis3dh_high_pass_bandwidth_get(lis3dh_t *dev, lis3dh_hpcf_t *val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    switch (ctrl_reg2.hpcf) {
    case LIS3DH_AGGRESSIVE:
        *val = LIS3DH_AGGRESSIVE;
        break;
    case LIS3DH_STRONG:
        *val = LIS3DH_STRONG;
        break;
    case LIS3DH_MEDIUM:
        *val = LIS3DH_MEDIUM;
        break;
    case LIS3DH_LIGHT:
        *val = LIS3DH_LIGHT;
        break;
    default:
        *val = LIS3DH_LIGHT;
        break;
    }
    return ret;
}

int32_t lis3dh_high_pass_mode_set(lis3dh_t *dev, lis3dh_hpm_t val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    if (ret == 0) {
        ctrl_reg2.hpm = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    }
    return ret;
}

int32_t lis3dh_high_pass_mode_get(lis3dh_t *dev, lis3dh_hpm_t *val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    switch (ctrl_reg2.hpm) {
    case LIS3DH_NORMAL_WITH_RST:
        *val = LIS3DH_NORMAL_WITH_RST;
        break;
    case LIS3DH_REFERENCE_MODE:
        *val = LIS3DH_REFERENCE_MODE;
        break;
    case LIS3DH_NORMAL:
        *val = LIS3DH_NORMAL;
        break;
    case LIS3DH_AUTORST_ON_INT:
        *val = LIS3DH_AUTORST_ON_INT;
        break;
    default:
        *val = LIS3DH_NORMAL_WITH_RST;
        break;
    }
    return ret;
}

int32_t lis3dh_full_scale_set(lis3dh_t *dev, lis3dh_scale_t val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (ret == 0) {
        ctrl_reg4.fs = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    }
    return ret;
}

int32_t lis3dh_full_scale_get(lis3dh_t *dev, lis3dh_scale_t *val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    switch (ctrl_reg4.fs) {
    case LIS3DH_SCALE_2G:
        *val = LIS3DH_SCALE_2G;
        DEBUG("LIS3DH_SCALE_2G\n");
        break;
    case LIS3DH_SCALE_4G:
        *val = LIS3DH_SCALE_4G;
        DEBUG("LIS3DH_4g\n");
        break;
    case LIS3DH_SCALE_8G:
        *val = LIS3DH_SCALE_8G;
        DEBUG("LIS3DH_SCALE_8G\n");
        break;
    case LIS3DH_SCALE_16G:
        *val = LIS3DH_SCALE_16G;
        DEBUG("LIS3DH_SCALE_16G\n");
        break;
    default:
        *val = LIS3DH_SCALE_2G;
        DEBUG("Default LIS3DH_SCALE_2G\n");
        break;
    }
    return ret;
}

int32_t lis3dh_block_data_update_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (ret == 0) {
        ctrl_reg4.bdu = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    }
    return ret;
}

int32_t lis3dh_block_data_update_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    *val = (uint8_t)ctrl_reg4.bdu;

    return ret;
}

int32_t lis3dh_filter_reference_set(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_write_reg(dev, LIS3DH_REG_REFERENCE, buff, 1);
    return ret;
}

int32_t lis3dh_filter_reference_get(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_REFERENCE, buff, 1);
    return ret;
}

int32_t lis3dh_xl_data_ready_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_status_reg_t status_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_STATUS_REG, (uint8_t *)&status_reg, 1);
    *val = status_reg._zyxda;
    DEBUG("Bit ZYXDA STATUS_REG is %d\n", status_reg._zyxda);
    return ret;
}

int32_t lis3dh_xl_data_ovr_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_status_reg_t status_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_STATUS_REG, (uint8_t*)&status_reg, 1);
    *val = status_reg._zyxor;

    return ret;
}

int32_t lis3dh_acceleration_raw_get(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_OUT_X_L, buff, 6);
    DEBUG("Acceleration Raw Data: ");
    PRINTBUFF(buff, 6);
    return ret;
}

int32_t lis3dh_acceleration_raw_axis_get(lis3dh_t *dev, uint8_t reg_axis, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, reg_axis, buff, 2);
    DEBUG("Acceleration Raw Data: ");
    PRINTBUFF(buff, 2);
    return ret;
}

int32_t lis3dh_device_id_get(lis3dh_t *dev, uint8_t *buff)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_WHO_AM_I, buff, 1);
    return ret;
}

int32_t lis3dh_self_test_set(lis3dh_t *dev, lis3dh_st_t val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (ret == 0) {
        ctrl_reg4.st = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    }
    return ret;
}

int32_t lis3dh_self_test_get(lis3dh_t *dev, lis3dh_st_t *val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    switch (ctrl_reg4.st) {
    case LIS3DH_ST_DISABLE:
        *val = LIS3DH_ST_DISABLE;
        break;
    case LIS3DH_ST_POSITIVE:
        *val = LIS3DH_ST_POSITIVE;
        break;
    case LIS3DH_ST_NEGATIVE:
        *val = LIS3DH_ST_NEGATIVE;
        break;
    default:
        *val = LIS3DH_ST_DISABLE;
        break;
    }
    return ret;
}

int32_t lis3dh_data_format_set(lis3dh_t *dev, lis3dh_ble_t val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (ret == 0) {
        ctrl_reg4.ble = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    }
    return ret;
}

int32_t lis3dh_data_format_get(lis3dh_t *dev, lis3dh_ble_t *val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    switch (ctrl_reg4.ble) {
    case LIS3DH_LSB_AT_LOW_ADD:
        *val = LIS3DH_LSB_AT_LOW_ADD;
        break;
    case LIS3DH_MSB_AT_LOW_ADD:
        *val = LIS3DH_MSB_AT_LOW_ADD;
        break;
    default:
        *val = LIS3DH_LSB_AT_LOW_ADD;
        break;
    }
    return ret;
}

int32_t lis3dh_boot_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    if (ret == 0) {
        ctrl_reg5.boot = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    }
    return ret;
}

int32_t lis3dh_boot_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    *val = (uint8_t)ctrl_reg5.boot;

    return ret;
}

int32_t lis3dh_status_get(lis3dh_t *dev, lis3dh_status_reg_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_STATUS_REG, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_int1_gen_conf_set(lis3dh_t *dev, lis3dh_int1_cfg_t *val)
{
    int32_t ret;
    ret = lis3dh_write_reg(dev, LIS3DH_REG_INT1_CFG, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_int1_gen_conf_get(lis3dh_t *dev, lis3dh_int1_cfg_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT1_CFG, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_int1_gen_source_get(lis3dh_t *dev, lis3dh_int1_src_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT1_SOURCE, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_int1_gen_threshold_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_int1_ths_t int1_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT1_THS, (uint8_t*)&int1_ths, 1);
    if (ret == 0) {
        int1_ths.ths = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_INT1_THS, (uint8_t*)&int1_ths, 1);
    }
    return ret;
}

int32_t lis3dh_int1_gen_threshold_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_int1_ths_t int1_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT1_THS, (uint8_t*)&int1_ths, 1);
    *val = (uint8_t)int1_ths.ths;

    return ret;
}

int32_t lis3dh_int1_gen_duration_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_int1_duration_t int1_duration;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT1_DURATION, (uint8_t*)&int1_duration, 1);
    if (ret == 0) {
        int1_duration.d = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_INT1_DURATION, (uint8_t*)&int1_duration, 1);
    }
    return ret;
}

int32_t lis3dh_int1_gen_duration_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_int1_duration_t int1_duration;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT1_DURATION, (uint8_t*)&int1_duration, 1);
    *val = (uint8_t)int1_duration.d;

    return ret;
}

int32_t lis3dh_int2_gen_conf_set(lis3dh_t *dev, lis3dh_int2_cfg_t *val)
{
    int32_t ret;
    ret = lis3dh_write_reg(dev, LIS3DH_REG_INT2_CFG, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_int2_gen_conf_get(lis3dh_t *dev, lis3dh_int2_cfg_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT2_CFG, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_int2_gen_source_get(lis3dh_t *dev, lis3dh_int2_src_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT2_SOURCE, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_int2_gen_threshold_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_int2_ths_t int2_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT2_THS, (uint8_t*)&int2_ths, 1);
    if (ret == 0) {
        int2_ths.ths = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_INT2_THS, (uint8_t*)&int2_ths, 1);
    }
    return ret;
}

int32_t lis3dh_int2_gen_threshold_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_int2_ths_t int2_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT2_THS, (uint8_t*)&int2_ths, 1);
    *val = (uint8_t)int2_ths.ths;

    return ret;
}

int32_t lis3dh_int2_gen_duration_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_int2_duration_t int2_duration;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT2_DURATION, (uint8_t*)&int2_duration, 1);
    if (ret == 0) {
        int2_duration.d = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_INT2_DURATION, (uint8_t*)&int2_duration, 1);
    }
    return ret;
}

int32_t lis3dh_int2_gen_duration_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_int2_duration_t int2_duration;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_INT2_DURATION, (uint8_t*)&int2_duration, 1);
    *val = (uint8_t)int2_duration.d;

    return ret;
}

int32_t lis3dh_high_pass_int_conf_set(lis3dh_t *dev, lis3dh_hp_t val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    if (ret == 0) {
        ctrl_reg2.hp = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    }
    return ret;
}

int32_t lis3dh_high_pass_int_conf_get(lis3dh_t *dev, lis3dh_hp_t *val)
{
    lis3dh_ctrl_reg2_t ctrl_reg2;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG2, (uint8_t*)&ctrl_reg2, 1);
    switch (ctrl_reg2.hp) {
    case LIS3DH_DISC_FROM_INT_GENERATOR:
        *val = LIS3DH_DISC_FROM_INT_GENERATOR;
        break;
    case LIS3DH_ON_INT1_GEN:
        *val = LIS3DH_ON_INT1_GEN;
        break;
    case LIS3DH_ON_INT2_GEN:
        *val = LIS3DH_ON_INT2_GEN;
        break;
    case LIS3DH_ON_TAP_GEN:
        *val = LIS3DH_ON_TAP_GEN;
        break;
    case LIS3DH_ON_INT1_INT2_GEN:
        *val = LIS3DH_ON_INT1_INT2_GEN;
        break;
    case LIS3DH_ON_INT1_TAP_GEN:
        *val = LIS3DH_ON_INT1_TAP_GEN;
        break;
    case LIS3DH_ON_INT2_TAP_GEN:
        *val = LIS3DH_ON_INT2_TAP_GEN;
        break;
    case LIS3DH_ON_INT1_INT2_TAP_GEN:
        *val = LIS3DH_ON_INT1_INT2_TAP_GEN;
        break;
    default:
        *val = LIS3DH_DISC_FROM_INT_GENERATOR;
        break;
    }
    return ret;
}

int32_t lis3dh_pin_int1_config_set(lis3dh_t *dev, lis3dh_ctrl_reg3_t *val)
{
    int32_t ret;
    ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG3, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_pin_int1_config_get(lis3dh_t *dev, lis3dh_ctrl_reg3_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG3, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_int2_pin_detect_4d_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    if (ret == 0) {
        ctrl_reg5.d4d_int2 = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    }
    return ret;
}

int32_t lis3dh_int2_pin_detect_4d_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    *val = (uint8_t)ctrl_reg5.d4d_int2;

    return ret;
}

int32_t lis3dh_int2_pin_notification_mode_set(lis3dh_t *dev,
        lis3dh_lir_int2_t val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    if (ret == 0) {
        ctrl_reg5.lir_int2 = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    }
    return ret;
}

int32_t lis3dh_int2_pin_notification_mode_get(lis3dh_t *dev,
        lis3dh_lir_int2_t *val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    switch (ctrl_reg5.lir_int2) {
    case LIS3DH_INT2_PULSED:
        *val = LIS3DH_INT2_PULSED;
        break;
    case LIS3DH_INT2_LATCHED:
        *val = LIS3DH_INT2_LATCHED;
        break;
    default:
        *val = LIS3DH_INT2_PULSED;
        break;
    }
    return ret;
}

int32_t lis3dh_int1_pin_detect_4d_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    if (ret == 0) {
        ctrl_reg5.d4d_int1 = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    }
    return ret;
}

int32_t lis3dh_int1_pin_detect_4d_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    *val = (uint8_t)ctrl_reg5.d4d_int1;

    return ret;
}

int32_t lis3dh_int1_pin_notification_mode_set(lis3dh_t *dev,
        lis3dh_lir_int1_t val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    if (ret == 0) {
        ctrl_reg5.lir_int1 = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    }
    return ret;
}

int32_t lis3dh_int1_pin_notification_mode_get(lis3dh_t *dev,
        lis3dh_lir_int1_t *val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    switch (ctrl_reg5.lir_int1) {
    case LIS3DH_INT1_PULSED:
        *val = LIS3DH_INT1_PULSED;
        break;
    case LIS3DH_INT1_LATCHED:
        *val = LIS3DH_INT1_LATCHED;
        break;
    default:
        *val = LIS3DH_INT1_PULSED;
        break;
    }
    return ret;
}

int32_t lis3dh_pin_int2_config_set(lis3dh_t *dev, lis3dh_ctrl_reg6_t *val)
{
    int32_t ret;
    ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG6, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_pin_int2_config_get(lis3dh_t *dev, lis3dh_ctrl_reg6_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG6, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_fifo_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    if (ret == 0) {
        ctrl_reg5.fifo_en = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    }
    return ret;
}

int32_t lis3dh_fifo_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_ctrl_reg5_t ctrl_reg5;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG5, (uint8_t*)&ctrl_reg5, 1);
    *val = (uint8_t)ctrl_reg5.fifo_en;

    return ret;
}

int32_t lis3dh_fifo_watermark_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    if (ret == 0) {
        fifo_ctrl_reg.fth = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    }
    return ret;
}

int32_t lis3dh_fifo_watermark_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    *val = (uint8_t)fifo_ctrl_reg.fth;

    return ret;
}

int32_t lis3dh_fifo_trigger_event_set(lis3dh_t *dev, lis3dh_tr_t val)
{
    lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    if (ret == 0) {
        fifo_ctrl_reg.tr = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    }
    return ret;
}

int32_t lis3dh_fifo_trigger_event_get(lis3dh_t *dev, lis3dh_tr_t *val)
{
    lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    switch (fifo_ctrl_reg.tr) {
    case LIS3DH_INT1_GEN:
        *val = LIS3DH_INT1_GEN;
        break;
    case LIS3DH_INT2_GEN:
        *val = LIS3DH_INT2_GEN;
        break;
    default:
        *val = LIS3DH_INT1_GEN;
        break;
    }
    return ret;
}

int32_t lis3dh_fifo_mode_set(lis3dh_t *dev, lis3dh_fm_t val)
{
    lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    if (ret == 0) {
        fifo_ctrl_reg.fm = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    }
    return ret;
}

int32_t lis3dh_fifo_mode_get(lis3dh_t *dev, lis3dh_fm_t *val)
{
    lis3dh_fifo_ctrl_reg_t fifo_ctrl_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_CTRL_REG, (uint8_t*)&fifo_ctrl_reg, 1);
    switch (fifo_ctrl_reg.fm) {
    case LIS3DH_BYPASS_MODE:
        *val = LIS3DH_BYPASS_MODE;
        break;
    case LIS3DH_FIFO_MODE:
        *val = LIS3DH_FIFO_MODE;
        break;
    case LIS3DH_DYNAMIC_STREAM_MODE:
        *val = LIS3DH_DYNAMIC_STREAM_MODE;
        break;
    case LIS3DH_STREAM_TO_FIFO_MODE:
        *val = LIS3DH_STREAM_TO_FIFO_MODE;
        break;
    default:
        *val = LIS3DH_BYPASS_MODE;
        break;
    }
    return ret;
}

int32_t lis3dh_fifo_status_get(lis3dh_t *dev, lis3dh_fifo_src_reg_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_SRC_REG, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_fifo_data_level_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_fifo_src_reg_t fifo_src_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_SRC_REG, (uint8_t*)&fifo_src_reg, 1);
    *val = (uint8_t)fifo_src_reg.fss;

    return ret;
}

int32_t lis3dh_fifo_empty_flag_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_fifo_src_reg_t fifo_src_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_SRC_REG, (uint8_t*)&fifo_src_reg, 1);
    *val = (uint8_t)fifo_src_reg.empty;

    return ret;
}

int32_t lis3dh_fifo_ovr_flag_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_fifo_src_reg_t fifo_src_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_SRC_REG, (uint8_t*)&fifo_src_reg, 1);
    *val = (uint8_t)fifo_src_reg.ovrn_fifo;

    return ret;
}

int32_t lis3dh_fifo_fth_flag_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_fifo_src_reg_t fifo_src_reg;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_FIFO_SRC_REG, (uint8_t*)&fifo_src_reg, 1);
    *val = (uint8_t)fifo_src_reg.wtm;

    return ret;
}

int32_t lis3dh_tap_conf_set(lis3dh_t *dev, lis3dh_click_cfg_t *val)
{
    int32_t ret;
    ret = lis3dh_write_reg(dev, LIS3DH_REG_CLICK_CFG, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_tap_conf_get(lis3dh_t *dev, lis3dh_click_cfg_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_CLICK_CFG, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_tap_source_get(lis3dh_t *dev, lis3dh_click_src_t *val)
{
    int32_t ret;
    ret = lis3dh_read_reg(dev, LIS3DH_REG_CLICK_SRC, (uint8_t*) val, 1);
    return ret;
}

int32_t lis3dh_tap_threshold_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_click_ths_t click_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CLICK_THS, (uint8_t*)&click_ths, 1);
    if (ret == 0) {
        click_ths.ths = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CLICK_THS, (uint8_t*)&click_ths, 1);
    }
    return ret;
}

int32_t lis3dh_tap_threshold_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_click_ths_t click_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CLICK_THS, (uint8_t*)&click_ths, 1);
    *val = (uint8_t)click_ths.ths;

    return ret;
}

int32_t lis3dh_tap_notification_mode_set(lis3dh_t *dev, lis3dh_lir_click_t val)
{
    lis3dh_click_ths_t click_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CLICK_THS, (uint8_t*)&click_ths, 1);
    if (ret == 0) {
        click_ths.lir_click = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CLICK_THS, (uint8_t*)&click_ths, 1);
    }
    return ret;
}

int32_t lis3dh_tap_notification_mode_get(lis3dh_t *dev,
        lis3dh_lir_click_t *val)
{
    lis3dh_click_ths_t click_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CLICK_THS, (uint8_t*)&click_ths, 1);
    switch (click_ths.lir_click) {
    case LIS3DH_TAP_PULSED:
        *val = LIS3DH_TAP_PULSED;
        break;
    case LIS3DH_TAP_LATCHED:
        *val = LIS3DH_TAP_LATCHED;
        break;
    default:
        *val = LIS3DH_TAP_PULSED;
        break;
    }
    return ret;
}

int32_t lis3dh_shock_dur_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_time_limit_t time_limit;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TIME_LIMIT, (uint8_t*)&time_limit, 1);
    if (ret == 0) {
        time_limit.tli = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_TIME_LIMIT, (uint8_t*)&time_limit, 1);
    }
    return ret;
}

int32_t lis3dh_shock_dur_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_time_limit_t time_limit;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TIME_LIMIT, (uint8_t*)&time_limit, 1);
    *val = (uint8_t)time_limit.tli;

    return ret;
}

int32_t lis3dh_quiet_dur_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_time_latency_t time_latency;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TIME_LATENCY, (uint8_t*)&time_latency, 1);
    if (ret == 0) {
        time_latency.tla = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_TIME_LATENCY, (uint8_t*)&time_latency, 1);
    }
    return ret;
}

int32_t lis3dh_quiet_dur_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_time_latency_t time_latency;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TIME_LATENCY, (uint8_t*)&time_latency, 1);
    *val = (uint8_t)time_latency.tla;

    return ret;
}

int32_t lis3dh_double_tap_timeout_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_time_window_t time_window;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TIME_WINDOW, (uint8_t*)&time_window, 1);
    if (ret == 0) {
        time_window.tw = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_TIME_WINDOW, (uint8_t*)&time_window, 1);
    }
    return ret;
}

int32_t lis3dh_double_tap_timeout_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_time_window_t time_window;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_TIME_WINDOW, (uint8_t*)&time_window, 1);
    *val = (uint8_t)time_window.tw;

    return ret;
}

int32_t lis3dh_act_threshold_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_act_ths_t act_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_ACT_THS, (uint8_t*)&act_ths, 1);
    if (ret == 0) {
        act_ths.acth = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_ACT_THS, (uint8_t*)&act_ths, 1);
    }
    return ret;
}

int32_t lis3dh_act_threshold_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_act_ths_t act_ths;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_ACT_THS, (uint8_t*)&act_ths, 1);
    *val = (uint8_t)act_ths.acth;

    return ret;
}

int32_t lis3dh_act_timeout_set(lis3dh_t *dev, uint8_t val)
{
    lis3dh_act_dur_t act_dur;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_IACT_DUR, (uint8_t*)&act_dur, 1);
    if (ret == 0) {
        act_dur.actd = val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_IACT_DUR, (uint8_t*)&act_dur, 1);
    }
    return ret;
}

int32_t lis3dh_act_timeout_get(lis3dh_t *dev, uint8_t *val)
{
    lis3dh_act_dur_t act_dur;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_IACT_DUR, (uint8_t*)&act_dur, 1);
    *val = (uint8_t)act_dur.actd;

    return ret;
}

int32_t lis3dh_pin_sdo_sa0_mode_set(lis3dh_t *dev, lis3dh_sdo_pu_disc_t val)
{
    lis3dh_ctrl_reg0_t ctrl_reg0;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG0, (uint8_t*)&ctrl_reg0, 1);
    if (ret == 0) {
        ctrl_reg0.sdo_pu_disc = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG0, (uint8_t*)&ctrl_reg0, 1);
    }
    return ret;
}

int32_t lis3dh_pin_sdo_sa0_mode_get(lis3dh_t *dev, lis3dh_sdo_pu_disc_t *val)
{
    lis3dh_ctrl_reg0_t ctrl_reg0;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG0, (uint8_t*)&ctrl_reg0, 1);
    switch (ctrl_reg0.sdo_pu_disc) {
    case LIS3DH_PULL_UP_DISCONNECT:
        *val = LIS3DH_PULL_UP_DISCONNECT;
        break;
    case LIS3DH_PULL_UP_CONNECT:
        *val = LIS3DH_PULL_UP_CONNECT;
        break;
    default:
        *val = LIS3DH_PULL_UP_DISCONNECT;
        break;
    }
    return ret;
}

int32_t lis3dh_spi_mode_set(lis3dh_t *dev, lis3dh_sim_t val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    if (ret == 0) {
        ctrl_reg4.sim = (uint8_t)val;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    }
    return ret;
}

int32_t lis3dh_spi_mode_get(lis3dh_t *dev, lis3dh_sim_t *val)
{
    lis3dh_ctrl_reg4_t ctrl_reg4;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG4, (uint8_t*)&ctrl_reg4, 1);
    switch (ctrl_reg4.sim) {
    case LIS3DH_SPI_4_WIRE:
        *val = LIS3DH_SPI_4_WIRE;
        break;
    case LIS3DH_SPI_3_WIRE:
        *val = LIS3DH_SPI_3_WIRE;
        break;
    default:
        *val = LIS3DH_SPI_4_WIRE;
        break;
    }
    return ret;
}

int32_t lis3dh_axis_set(lis3dh_t *dev, lis3dh_axis_t val)
{
    lis3dh_ctrl_reg1_t ctrl_reg1;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    if (ret == 0) {
        if (val == LIS3DH_AXES_ALL_DISABLE) {
            ctrl_reg1.xen = 0;
            ctrl_reg1.yen = 0;
            ctrl_reg1.zen = 0;
        }
        if (val == LIS3DH_AXES_ALL_ENABLE) {
            ctrl_reg1.xen = 1;
            ctrl_reg1.yen = 1;
            ctrl_reg1.zen = 1;
        }
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    }
    return ret;
}

int32_t lis3dh_axis_get(lis3dh_t *dev, lis3dh_axis_t *val) 
{
    lis3dh_ctrl_reg1_t ctrl_reg1;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    if (ret == 0) {
        if ( (ctrl_reg1.xen == 1) && (ctrl_reg1.yen == 1) && (ctrl_reg1.zen == 1)) {
            *val = LIS3DH_AXES_ALL_ENABLE;
            DEBUG("LIS3DH_AXES_ALL_ENABLE\n");
        }
        if ((ctrl_reg1.xen == 0) && (ctrl_reg1.yen == 0) && (ctrl_reg1.zen == 0)) {
            *val = LIS3DH_AXES_ALL_DISABLE;
            DEBUG("LIS3DH_AXES_ALL_DISABLE\n");
        }
    }
    return ret;
}

int32_t lis3dh_accuracy_get(lis3dh_t *dev) {
    lis3dh_res_t res;
    lis3dh_scale_t scale;

    DEBUG("Get device operation mode\n");
    if (lis3dh_operating_mode_get(dev, &res) < 0) {
        return LIS3DH_NOCOM;
    }
    DEBUG("Get full scale\n");
    if (lis3dh_full_scale_get(dev, &scale) < 0) {
        return LIS3DH_NOCOM;
    }
    dev->scale = scale;
    dev->res   = res;

    return LIS3DH_OK;
}

int32_t lis3dh_calculation_acceleration(lis3dh_t *dev, int16_t acc_raw) {

    DEBUG("acc raw data: %d[%04X]\n", acc_raw, acc_raw);

    switch (dev->res) {
        case LIS3DH_HR_12BIT:
            switch (dev->scale) {
                case LIS3DH_SCALE_2G:
                    return ((acc_raw / 16) * 1);
                    break;
                case LIS3DH_SCALE_4G:
                    return ((acc_raw / 16) * 2);
                    break;
                case LIS3DH_SCALE_8G:
                    return ((acc_raw / 16) * 4);
                    break;
                case LIS3DH_SCALE_16G:
                    return ((acc_raw / 16) * 12);
                    break;
            } 
            break;
        case LIS3DH_NM_10BIT:
            switch (dev->scale) {
                case LIS3DH_SCALE_2G:
                    return ((acc_raw / 64) * 4);
                    break;
                case LIS3DH_SCALE_4G:
                    return ((acc_raw / 64) * 8);
                    break;
                case LIS3DH_SCALE_8G:
                    return ((acc_raw / 64) * 16);
                    break;
                case LIS3DH_SCALE_16G:
                    return ((acc_raw / 64) * 48);
                    break;
            }
            break;
        case LIS3DH_LP_8BIT:
            switch (dev->scale) {
                case LIS3DH_SCALE_2G:
                    return ((acc_raw / 256) * 16);
                    break;
                case LIS3DH_SCALE_4G:
                    return ((acc_raw / 256) * 32);
                    break;
                case LIS3DH_SCALE_8G:
                    return ((acc_raw / 256) * 64);
                    break;
                case LIS3DH_SCALE_16G:
                    return ((acc_raw / 256) * 192);
                    break;
            }
            break;
    }

    return 0xF0000000;
}

int32_t lis3dh_calculation_temperature(lis3dh_t *dev, int16_t temp_raw) {
    lis3dh_res_t res;

    lis3dh_operating_mode_get(dev, &res);

    switch (res) {
        case LIS3DH_HR_12BIT:
            return (((temp_raw / 64) / 4) + 25);
            break;
        case LIS3DH_NM_10BIT:
            return (((temp_raw / 64) / 4) + 25);
            break;
        case LIS3DH_LP_8BIT:
            return (((temp_raw / 256 ) * 1) + 25);
            break;
    }

    return 0xF0000000;
}
/** @} */

/**
 * @defgroup  LIS3DH_Global_Functions
 * @brief     This section group all the global functions
 *
 * @{
 *
 */
int lis3dh_init(lis3dh_t *dev, const lis3dh_params_t *params, lis3dh_int1_cb_t cb, void *arg)
{
    uint8_t dev_id;

    lis3dh_ctrl_reg3_t ctrl_reg3;

    _platform_init(dev, params);

    /* test connection to the device */
    if (lis3dh_device_id_get(dev, &dev_id) < 0) {
        return LIS3DH_NOCOM;
    }

    if (dev_id != LIS3DH_WHO_AM_I_RESPONSE) {
        /* chip is not responding correctly */
        DEBUG("[lis3dh] error reading the who am i reg [0x%02x]\n", (int)dev_id);
        return LIS3DH_NODEV;
    }

    /* Disable Pull-Up */
    if (lis3dh_pin_sdo_sa0_mode_set(dev, LIS3DH_PULL_UP_DISCONNECT) < 0) {
        return LIS3DH_NOCOM;
    }

    /* Enable all axis */
    lis3dh_axis_set(dev, LIS3DH_AXES_ALL_ENABLE);

    /* Enable Block Data Update */
    DEBUG("Enable Block Data Update\n");
    if (lis3dh_block_data_update_set(dev, PROPERTY_ENABLE) < 0) {
        return LIS3DH_NOCOM;
    }

    /* Set Big endian output value */
    DEBUG("Set Big endian output value\n");
    if (lis3dh_data_format_set(dev, LIS3DH_MSB_AT_LOW_ADD) < 0) {
        return LIS3DH_NOCOM;
    }

    /* Set full scale */ 
    DEBUG("Set full scale [%d]\n", dev->params.scale); 
    if (lis3dh_full_scale_set(dev, dev->params.scale) < 0) {
        return LIS3DH_NOCOM;
    }

    /* Enable temperature sensor */
    DEBUG("Enable temperature sensor\n");
    if (lis3dh_aux_adc_set(dev, LIS3DH_AUX_ON_TEMPERATURE) < 0) {
        return LIS3DH_NOCOM;
    }

    /* Set device operation mode */
    DEBUG("Set device operation mode [%d]\n", dev->params.res);
    if (lis3dh_operating_mode_set(dev, dev->params.res) < 0) {
        return LIS3DH_NOCOM;
    }

    if (dev->params.int1 != GPIO_UNDEF) {
        
        /* Enable interrupt pin - DRDY*/
        DEBUG("Enable interrupt pin - DRDY\n");
        if (lis3dh_pin_int1_config_get(dev, &ctrl_reg3) < 0) {
            return LIS3DH_NOCOM;
        }
        ctrl_reg3.i1_zyxda = PROPERTY_ENABLE;
        if (lis3dh_pin_int1_config_set(dev, &ctrl_reg3) < 0) {
            return LIS3DH_NOCOM;
        } 
        /* Cleaning all interrupt flags */
        lis3dh_int1_src_t int1_src;
        if (lis3dh_int1_gen_source_get(dev, &int1_src) < 0) {
            return LIS3DH_NOCOM;
        }
        /* Enable interrupt handler */
        DEBUG("Enable interrupt handler\n");
        dev->arg = arg;
        dev->cb = cb;
        if (gpio_init_int(dev->params.int1, GPIO_IN, GPIO_RISING, cb, arg)) {
            return LIS3DH_ERROR;
        }
    } else {
        (void)cb;
        (void)arg;
    }

    /* Set Output Data Rate */
    DEBUG("Set Output Data Rate [%d]\n", dev->params.odr);
    if (lis3dh_data_rate_set(dev, dev->params.odr) < 0) {
        return LIS3DH_NOCOM;
    }

    /* Delay to start */
    xtimer_usleep(100 * 1000);

    /* discard first measurement after power-on*/
    if (dev->params.odr != LIS3DH_POWER_DOWN) {
        lis3dh_data_t data;
        if (lis3dh_read_xyz(dev, &data) < 0) {
            DEBUG("Doesn't read ACC raw data (First Start)\n");
            return LIS3DH_NOCOM;
        }
    }

    return LIS3DH_OK;
}

int lis3dh_read_xyz(lis3dh_t *dev, lis3dh_data_t *acceleration) {
    lis3dh_reg_t reg_da;
    lis3dh_reg_t reg_ovr;
    uint8_t acc_raw[6] = {0x00};
    uint16_t tick = 0xFFFF;

    reg_da.byte = 0x00;
    reg_ovr.byte = 0x00;
    /* Read output only if new value available */
    do {
        if (lis3dh_xl_data_ready_get(dev, &reg_da.byte) < 0) {
            return LIS3DH_NODATA;
        }
        DEBUG("Bit ZYXDA STATUS_REG is %d\n", reg_da.byte);
        if (reg_da.byte) {
            /* Read accelerometer data */
            if (lis3dh_acceleration_raw_get(dev, acc_raw) < 0) {
                return LIS3DH_NOCOM;
            }
            if (lis3dh_accuracy_get(dev) < 0) {
                return  LIS3DH_NOCOM;
            }
            acceleration->axis_x = lis3dh_calculation_acceleration(dev, ((acc_raw[0] << 8) | acc_raw[1]));
            acceleration->axis_y = lis3dh_calculation_acceleration(dev, ((acc_raw[2] << 8) | acc_raw[3]));
            acceleration->axis_z = lis3dh_calculation_acceleration(dev, ((acc_raw[4] << 8) | acc_raw[5]));
            DEBUG("Acceleration [mg]:%d\t%d\t%d\n", acceleration->axis_x, acceleration->axis_y, acceleration->axis_z);
        }
        if (lis3dh_xl_data_ovr_get(dev, &reg_ovr.byte) < 0) {
            return LIS3DH_NOCOM;
        }
        DEBUG("Bit ZYXOR STATUS_REG is %d\n", reg_ovr.byte);
        if (reg_ovr.byte) {
            if (lis3dh_acceleration_raw_get(dev, acc_raw) < 0) {
                return LIS3DH_NOCOM;
            }
        }

    } while(!reg_da.byte && --tick);

    if (!tick) {
        DEBUG("Timeout\n");
        return LIS3DH_ERROR;
    }

    return LIS3DH_OK;
}

int lis3dh_read_temp(lis3dh_t *dev, int16_t *temperature_degC) {

    lis3dh_reg_t reg;
    uint8_t raw_temp[2] = {0x00};
    uint8_t tmp[6] = {0x00};
    uint16_t tick = 0xFFFF;

    /* Read output only if new value available */
    do {
        if (lis3dh_temp_data_ready_get(dev, &reg.byte) < 0) {
            return LIS3DH_NOCOM;
        }      
        if (reg.byte) {
            /* Read temperature data */
            if (lis3dh_temperature_raw_get(dev, raw_temp) < 0) {
                return LIS3DH_NOCOM;
            }
            if (lis3dh_adc_raw_get(dev, tmp) < 0) {
                return LIS3DH_NOCOM;
            } 
            *temperature_degC = ((((uint16_t)raw_temp[0]) << 8) | raw_temp[1]);
            *temperature_degC = lis3dh_calculation_temperature(dev, *temperature_degC);
            DEBUG("Temperature [degC]: %d\n", *temperature_degC);
        }
    } while(!reg.byte && --tick);

    if (!tick) {
        return LIS3DH_ERROR;
    }

    return LIS3DH_OK;
}

int lis3dh_poweron(lis3dh_t *dev) 
{
    lis3dh_ctrl_reg1_t ctrl_reg1;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    if (ret == 0) {
        ctrl_reg1.xen = 1;
        ctrl_reg1.yen = 1;
        ctrl_reg1.zen = 1;
        ctrl_reg1.odr = dev->params.odr;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);

        /* discard first measurement after power-on*/
        if (ret == 0) {
            if (dev->params.odr != LIS3DH_POWER_DOWN) {
                lis3dh_data_t data;
                lis3dh_read_xyz(dev, &data);
            }
        }
    }
    return ret;
}

int lis3dh_poweroff(lis3dh_t *dev)
{
    lis3dh_ctrl_reg1_t ctrl_reg1;
    int32_t ret;

    ret = lis3dh_read_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    if (ret == 0) {
        ctrl_reg1.xen = 0;
        ctrl_reg1.yen = 0;
        ctrl_reg1.zen = 0;
        ctrl_reg1.odr = LIS3DH_POWER_DOWN;
        ret = lis3dh_write_reg(dev, LIS3DH_REG_CTRL_REG1, (uint8_t*)&ctrl_reg1, 1);
    }
    return ret;
}
/** @} */
/** @} */
