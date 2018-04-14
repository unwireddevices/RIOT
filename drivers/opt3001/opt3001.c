/* Copyright (C) 2017 Unwired Devices [info@unwds.com]
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup
 * @ingroup
 * @brief
 * @{
 * @file        opt3001.c
 * @brief       basic driver for OPT3001 sensor
 * @authoh      Oleg Artamonov [info@unwds.com], Indrishenok Alexandr [https://github.com/sashaindrish]
 */


#include "opt3001.h"
#include "periph/i2c.h"

#include "rtctimers-millis.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief OPT3001 driver initialization routine
 * @note Corresponding I2C peripheral MUST be initialized before
 *
 * @param[out] dev device structure pointer
 * @param[in] param OPT3001 driver parameters, data will be copied into device parameters
 *
 * @return 0 if initialization succeeded
 * @return <0 in case of an error
 */
int opt3001_init(opt3001_t *dev)
{
    uint16_t chipid;

    assert(dev != NULL);

    /* Acquire I2C bus */
    i2c_acquire(dev->i2c);

    if (i2c_init_master(dev->i2c, I2C_SPEED_NORMAL) < 0) {
        i2c_release(dev->i2c);
        puts("[opt3001 driver] Error initializing I2C bus");
        
        return -1;
    }
    
    if (i2c_read_regs(dev->i2c, OPT3001_ADDRESS, OPT3001_REG_ID, (char *)&chipid, 2) != 2) {
        puts("[opt3001 driver] Error: sensor not found");
        i2c_release(dev->i2c);
        return -1;
    }
    
    if (chipid != OPT3001_CHIP_ID) {
        puts("[opt3001 driver] Error: ID mismatch");
        i2c_release(dev->i2c);
        return -1;
    }
	

    /*
    i2c_read_regs(dev->i2c, OPT3001_ADDRESS, OPT3001_REG_CONFIG, (char *)&chipid, 2);
    
    if (chipid != OPT3001_CFG_DEFAULT) {
        puts("[opt3001 driver] Error: default config mismatch");
        i2c_release(dev->i2c);
        return -1;
    }
    */
    i2c_release(dev->i2c);
    
    return 0;
}

static uint32_t read_sensor(opt3001_t *dev) {
	
    uint16_t data;
    data = OPT3001_CFG;
	/* write in reg dev , addres reg, data reg */
	
	write_registr(dev, OPT3001_REG_CONFIG, data, 2);

    /* read result */
    i2c_read_regs(dev->i2c, OPT3001_ADDRESS, OPT3001_REG_RESULT, (char *)&data, 2);
    
    /* swap bytes, as OPT3001 sends MSB first and I2C driver expects LSB first */
    data = ((data >> 8) | (data << 8));

    /* calculate lux per LSB */
    uint8_t exp = (data >> 12) & 0x0F;
    uint32_t lsb_size_x100 = (1 << exp);
    
    /* remove 4-bit exponent and leave 12-bit mantissa only */
    data &= 0x0FFF;
    
    return (((uint32_t)data * lsb_size_x100) / 100);
}

/**
 * @brief Gets OPT3001 measure.
 *
 * @param[in] dev pointer to the initialized OPT3001 device
 * @param[in] measure pointer to the allocated memory
 * for retval
 * @retval 0 for success
 *
 * Example:
 *
 * ...
 * opt3001_measure_t measure;
 *
 * opt3001_init(opt3001);
 * opt3001_measure(opt3001, &measure)
 */
uint32_t opt3001_measure(opt3001_t *dev, opt3001_measure_t *measure)
{
    assert(dev != NULL);

    i2c_acquire(dev->i2c);
    uint32_t lum = read_sensor(dev);
    i2c_release(dev->i2c);

    measure->luminocity = lum;

    return 0;
}


 /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void write_sensor_lim(opt3001_t *dev, float lim_lum_high, float lim_lum_low) {
    uint16_t data;
    data = OPT3001_CFG;
	write_registr(dev, OPT3001_REG_CONFIG, data, 2);
	
	
	
	float lum = lim_lum_high;
	
	lum = get_data_reg(lum);
	
	
	write_registr(dev, OPT3001_REG_HIGH_LIM, lum, 2);
	
	lum = lim_lum_low;
	
	lum = get_data_reg(lum);
		
	write_registr(dev, OPT3001_REG_LOW_LIM, lum, 2);
	
    
   
    
}

 /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
int write_registr(opt3001_t *dev, uint8_t addres_reg,uint16_t reg, uint16_t size){
	
	uint16_t data_reg = reg;
	
	i2c_write_regs(dev->i2c, OPT3001_ADDRESS, addres_reg, (char *)&data_reg, size);
    
    /* wait till measurement is finished */
    int i = 100;
    do {
        /* 10 ms delay */
        rtctimers_millis_sleep(10);
        
        i2c_read_regs(dev->i2c, OPT3001_ADDRESS, addres_reg, (char *)&data_reg, size);
        if (data_reg & reg) {
            break;
        }
        
        i--;
    } while (i);
    
    if (i == 0) {
        return 0;
    }else return 1;

	
}

uint16_t get_data_reg(float data){
	uint16_t res=0;
	
	if(data<=81.90)
		res=0;
	else if(data<=163.80)
		res=1;
	else if(data<=327.60)
		res=2;
	else if(data<=655.2)
		res=3;
	else if(data<=1310.40)
		res=4;
	else if(data<=2620.80)
		res=5;
	else if(data<=5241.60)
		res=6;
	else if(data<=10483.20)
		res=7;
	else if(data<=20966.40)
		res=8;
	else if(data<=41932.80)
		res=9;
	else if(data<=83865.60)
		res=10;
	
	
	uint32_t lsb_size_x100 = (1 << res);
	uint16_t reg_data = (uint16_t)(data/lsb_size_x100)*100;
	reg_data = ((reg_data << 8) | (reg_data >> 8));
	
	res = (res<<12);
	return reg_data|res;
}


#ifdef __cplusplus
}
#endif
