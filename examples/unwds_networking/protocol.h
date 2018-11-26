/*
 * Copyright (c) 2016, Unwired Devices LLC - http://www.unwireddevices.com/
 * All rights reserved.
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Unwired Devices nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/*
* \file
*         Protocol
* \author
*         Manchenko Oleg man4enkoos@gmail.com
*/
/*---------------------------------------------------------------------------*/
#include "net/gnrc/netif.h"
#include "unwds-ids.h"
/*---------------------------------------------------------------------------*/
#ifndef PROTOCOL_H
#define PROTOCOL_H

/*Упаковка структуры*/
#define pack 		__attribute__((packed))

/*---------------------------------------------------------------------------*/
/*Типы объединений*/
typedef union u8_u16_t {
	uint16_t u16;
	uint8_t u8[2];
} u8_u16_t;

typedef union u8_i16_t {
	int16_t i16;
	uint8_t u8[2];
} u8_i16_t;

typedef union u8_u32_t {
	uint32_t u32;
	uint8_t u8[4];
} u8_u32_t;

/*---------------------------------------------------------------------------*/
/*ID устройства*/
// typedef enum 
// {
    // UNWDS_GPIO_MODULE_ID = 1,
    // UNWDS_4BTN_MODULE_ID = 2, 
    // UNWDS_GPS_MODULE_ID = 3,
    // UNWDS_LSM6DS3_MODULE_ID = 4,
    // UNWDS_LM75_MODULE_ID = 5,
    // UNWDS_LMT01_MODULE_ID = 6,
    // UNWDS_UART_MODULE_ID = 7,
    // UNWDS_SHT21_MODULE_ID = 8,
    // UNWDS_PIR_MODULE_ID = 9,
    // UNWDS_ADC_MODULE_ID = 10,
    // UNWDS_LPS331_MODULE_ID = 11,
    // UNWDS_COUNTER_MODULE_ID = 12,
    // UNWDS_RSSIECHO_MODULE_ID = 13,
    // UNWDS_PWM_MODULE_ID = 14,
    // UNWDS_OPT3001_MODULE_ID = 15,
	// UNWDS_RESERVED1_MODULE_ID = 16,
    // UNWDS_BME280_MODULE_ID = 17,
    // UNWDS_MHZ19_MODULE_ID = 18,
    // UNWDS_USOUND_MODULE_ID = 19,
    // UNWDS_ADXL345_MODULE_ID = 20,
    // UNWDS_IBUTTON_MODULE_ID = 21,
    // UNWDS_HD44780_MODULE_ID = 22,
    // UNWDS_R300_MODULE_ID = 23,
    // UNWDS_IRBLASTER_MODULE_ID = 24,
    // UNWDS_HX711_MODULE_ID = 25,
    // UNWDS_FDC1004_MODULE_ID = 26,
	// UNWDS_CL420_MODULE_ID = 27,
	// UNWDS_MODBUS_MODULE_ID = 28,
	// UNWDS_RADIORELAY_MODULE_ID = 29,
	// UNWDS_CR95_MODULE_ID = 30,
    // /* Proprietary 50 to 99 */
    // UNWDS_M200_MODULE_ID = 50,
    // UNWDS_PULSE_MODULE_ID = 51,
    // UNWDS_PACS_MODULE_ID = 52,
    // UNWDS_SWITCH_MODULE_ID = 53,
    // UNWDS_M230_MODULE_ID = 54,	
	// UNWDS_IEC61107_MODULE_ID = 55,
    // UNWDS_IDCARD_MODULE_ID = 56,
	// UNWDS_DALI_MODULE_ID = 57,
	// UNWDS_WIEGAND_MODULE_ID = 58,
    // UNWDS_INCLINOMETER_MODULE_ID = 59,
    // UNWDS_PARKING_MODULE_ID = 60,
    // UNWDS_GARBAGE_MODULE_ID = 61,
    // /* Customer 100 to 125*/
    // UNWDS_CUSTOMER_MODULE_ID = 100,
    // /* System module 126 */
    // UNWDS_CONFIG_MODULE_ID = 126,
	// UNWDS_6LOWPAN_SYSTEM_MODULE_ID = 127,
// } UNWDS_MODULE_IDS_t;

// typedef enum {
	// UNWDS_GPIO_MODULE_ID = 1,
	// UNWDS_4BTN_MODULE_ID = 2,
	// UNWDS_GPS_MODULE_ID = 3,
	// UNWDS_LSM6DS3_MODULE_ID = 4,
	// UNWDS_LM75_MODULE_ID = 5,
	// UNWDS_LMT01_MODULE_ID = 6,
	// UNWDS_UART_MODULE_ID = 7,
	// UNWDS_SHT21_MODULE_ID = 8,
	// UNWDS_PIR_MODULE_ID = 9,
	// UNWDS_ADC_MODULE_ID = 10,
	// UNWDS_LPS331_MODULE_ID = 11,
	// UNWDS_COUNTER_MODULE_ID = 12,
	// UNWDS_RSSIECHO_MODULE_ID = 13,
	// UNWDS_6FET_MODULE_ID = 14,
	// UNWDS_LIT_MODULE_ID = 15,
	// UNWDS_DALI_MODULE_ID = 16,
	// UNWDS_BME280_MODULE_ID = 17,
	// UNWDS_MHZ19_MODULE_ID = 18,
	// UNWDS_RANGE_MODULE_ID = 19,
	// UNWDS_ADXL345_MODULE_ID = 20,
	// /* Proprietary 50 to 99 */
	// UNWDS_M200_MODULE_ID = 50,
	// UNWDS_PULSE_MODULE_ID = 51,
	// UNWDS_IBUTTON_MODULE_ID = 52,
	// UNWDS_SWITCH_MODULE_ID = 53,
	// UNWDS_M230_MODULE_ID = 54,
	// UNWDS_IEC61107_MODULE_ID = 55,
	// /* Customer 100 to 125*/
	// UNWDS_CUSTOMER_MODULE_ID = 100,
	// UNWDS_FIREBUTTON_MODULE_ID = 101,
	// /* System module 126 */
	// UNWDS_CONFIG_MODULE_ID = 126,
	// UNWDS_6LOWPAN_SYSTEM_MODULE_ID = 127,
// } UNWDS_MODULE_IDS_t;

/*---------------------------------------------------------------------------*/
/*CR->ROOT*/
typedef struct {		
	ipv6_addr_t dest_addr; 		/* Адрес модуля */
	uint8_t device_id; 			/* Индентификатор модуля */
	uint8_t data_type; 			/* Команда */
	uint8_t payload_len;		/* Размер payload'а */
} pack uart_header_t;

/*---------------------------------------------------------------------------*/
/*HEADER*/
typedef struct {
	uint8_t protocol_version;	/* Текущая версия протокола */ 
    uint8_t device_id;			/* ID устройства */
	uint8_t data_type;			/* Тип пакета */ 
	// uint8_t rssi;			/* RSSI */ 
	uint8_t temperature;		/* Температура */ 
	uint8_t voltage;			/* Напряжение */ 
} pack header_up_t;

typedef struct {		
    u8_u16_t counter;			/* Счетчик пакетов */ 
	u8_u16_t crc;				/* CRC16 */ 
	uint8_t length;				/* Размер пакета */
} pack header_down_t;

typedef struct {		
	uint8_t protocol_version;	/* Текущая версия протокола */ 
    uint8_t device_id;			/* ID устройства */
	uint8_t data_type;			/* Тип пакета */ 
	// uint8_t rssi;			/* RSSI */ 
	uint8_t temperature;		/* Температура */ 
	uint8_t voltage;			/* Напряжение */ 
	u8_u16_t counter;			/* Счетчик пакетов */ 
	u8_u16_t crc;				/* CRC16 */ 
	uint8_t length;				/* Размер пакета */
} pack header_t; 

/*---------------------------------------------------------------------------*/

typedef struct {		
	uint8_t ciphertext[16];
} pack crypto_1_block_t;

typedef struct {		
	uint8_t ciphertext[32];
} pack crypto_2_block_t;

typedef struct {		
	uint8_t ciphertext[48];
} pack crypto_3_block_t;

typedef struct {		
	uint8_t ciphertext[64];
} pack crypto_4_block_t;

typedef struct {		
	uint8_t ciphertext[80];
} pack crypto_5_block_t;

typedef struct {		
	uint8_t ciphertext[96];
} pack crypto_6_block_t;

typedef struct {		
	uint8_t ciphertext[112];
} pack crypto_7_block_t;

/*---------------------------------------------------------------------------*/
/*UNWDS-4BTN*/

/*Struct for BUTTON_STATUS*/
typedef struct {		
	uint8_t button_status;	/* Номер нажатой кнопки. Если долгое нажатие: button_status |= (1<<7) */
} pack button_status_t;

/*---------------------------------------------------------------------------*/
/*UNWDS-6FET*/

/*Struct for PWM_SETTINGS*/
typedef struct {
	uint32_t frequency;		/* Частота ШИМ'а от 100 Hz до 100 kHz */
	uint8_t channel;		/* Номер канала от 0 до 5 */
	uint8_t duty;			/* Коэффицент заполнения от 0% до 100% */
} pack pwm_settings_t;

/*Struct for PWM_POWER*/
typedef struct {		
	uint8_t pwm_power;		/* Номер канала от 0 до 5. Для включения: номер канала |= (1<<7) */
} pack pwm_power_t;

/*---------------------------------------------------------------------------*/
/*UNWDS-LIT*/

/*Struct for LIT_MEASURE_STATUS*/
typedef struct {		
	uint32_t lit_measure_status;	/* Значение освещенности в люксах */
} pack lit_measure_status_t;

/*---------------------------------------------------------------------------*/
/*UNWDS-GPIO*/

/*Struct for GPIO_CMD*/
typedef struct {		
	uint8_t pin;				/* Номер ножки порта которую будем переводить в нужное состояние */
	uint8_t command;			/* Команда для ножки порта */
} pack gpio_command_t;

/*---------------------------------------------------------------------------*/
/*UNWDS-6LOWPAN_SYSTEM*/

/*Struct for JOIN_STAGE_1*/
typedef struct {		
	uint8_t module_id;		/*ID устройства: module_id = UNWDS_MODULE_ID */	
} pack join_stage_1_t;

/*Struct for JOIN_STAGE_2*/
typedef struct {		
	u8_u16_t nonce;			/*Генерируем сессионный ключ nonce = random_rand() */			
} pack join_stage_2_t;

/*Struct for JOIN_STAGE_3*/
typedef struct {		
	u8_u16_t nonce;			/* Увеличиваем nonce на единицу: nonce += 1 */	
} pack join_stage_3_t;

/*Struct for JOIN_STAGE_4*/
typedef struct {		
	uint8_t status_code;	/* status_code = true если авторизация прошла успешно, иначе status_code = false */	
} pack join_stage_4_t;

/*Struct for PING*/
typedef struct {		
	u8_u16_t nonce;			/* Отправляем ROOT'у nonce. Если 3 раза не ответит, то перезагружаемся и переавторизируемся */
} pack ping_t;

/*Struct for PONG*/
typedef struct {		
	uint8_t status_code;	/* status_code = true если настройки шифрования совпадают */
} pack pong_t;

/*Struct for ACK*/
typedef struct {		
	u8_u16_t counter;		/* Номер подтвержденного пакета */
} pack ack_t;

/*Struct for NACK*/
typedef struct {		
	u8_u16_t counter;		/* Номер неподтвержденного пакета */
} pack nack_t;


/*---------------------------------------------------------------------------*/
/*Основные*/
#define UDP_DATA_PORT					(0xF0B0) //(0xF0B0) //‭61616‬
#define UDBP_PROTOCOL_VERSION			(1)

/*---------------------------------------------------------------------------*/
/*OFFSET*/
#define HEADER_OFFSET 					(0)
#define HEADER_DOWN_OFFSET 				(HEADER_OFFSET + HEADER_UP_LENGTH)
#define PAYLOAD_OFFSET 					(HEADER_OFFSET + HEADER_UP_LENGTH + HEADER_DOWN_LENGTH)

/*---------------------------------------------------------------------------*/
/*LENGTH*/
#define HEADER_UP_LENGTH				(sizeof(header_up_t))
#define HEADER_DOWN_LENGTH				(sizeof(header_down_t))
#define HEADER_LENGTH 					(HEADER_UP_LENGTH + HEADER_DOWN_LENGTH)

#define CRYPTO_1_BLOCK_LENGTH 			(sizeof(crypto_1_block_t))
#define CRYPTO_2_BLOCK_LENGTH 			(sizeof(crypto_2_block_t))
#define CRYPTO_3_BLOCK_LENGTH 			(sizeof(crypto_3_block_t))
#define CRYPTO_4_BLOCK_LENGTH 			(sizeof(crypto_4_block_t))
#define CRYPTO_5_BLOCK_LENGTH 			(sizeof(crypto_5_block_t))
#define CRYPTO_6_BLOCK_LENGTH 			(sizeof(crypto_6_block_t))
#define CRYPTO_7_BLOCK_LENGTH 			(sizeof(crypto_7_block_t))

#define JOIN_STAGE_1_PAYLOAD_LENGTH 	(JOIN_STAGE_1_LENGTH)
#define JOIN_STAGE_2_PAYLOAD_LENGTH 	(CRYPTO_1_BLOCK_LENGTH)
#define JOIN_STAGE_3_PAYLOAD_LENGTH 	(CRYPTO_1_BLOCK_LENGTH)

#define JOIN_STAGE_1_LENGTH 			(sizeof(join_stage_1_t))
#define JOIN_STAGE_2_LENGTH 			(sizeof(join_stage_2_t))
#define JOIN_STAGE_3_LENGTH 			(sizeof(join_stage_3_t))
#define JOIN_STAGE_4_LENGTH 			(sizeof(join_stage_4_t))
#define PING_LENGTH 					(sizeof(ping_t))
#define PONG_LENGTH 					(sizeof(pong_t))
#define ACK_LENGTH 						(sizeof(ack_t))
#define NACK_LENGTH 					(sizeof(nack_t))
#define BUTTON_STATUS_LENGTH 			(sizeof(button_status_t))
#define PWM_SETTINGS_LENGTH 			(sizeof(pwm_settings_t))
#define PWM_POWER_LENGTH 				(sizeof(pwm_power_t))
#define LIT_MEASURE_LENGTH				(0)
#define LIT_MEASURE_STATUS_LENGTH		(sizeof(lit_measure_status_t))
#define GPIO_CMD_LENGTH					(sizeof(gpio_command_t))

/*---------------------------------------------------------------------------*/
/*COMMAND*/

/*UNWDS-6LOWPAN_SYSTEM*/
#define JOIN_STAGE_1				(0x00) /* Нода посылает запрос координатору */
#define JOIN_STAGE_2				(0x01) /* Координатор отправляет ecb_encrypt(nonce=rand()) */
#define JOIN_STAGE_3				(0x02) /* Нода удостоверяет, что она знает ключ отправляя cbc_encrypt(nonce+1) */
#define JOIN_STAGE_4				(0x03) /* Координатор отвечает ноде что она имеет право быть в сети */
#define PING						(0x04) /* Ping */
#define PONG						(0x05) /* Pong */
#define ACK							(0x07) /* ACK */
#define NACK						(0x08) /* NACK */

/*UNWDS-4BTN*/
#define BUTTON_STATUS				(0x00) /* Пакет статусом нажатой кнопки */

/*UNWDS-6FET*/
#define PWM_SETTINGS				(0x00) /* Пакет с настройкоами ШИМ канала */
#define PWM_POWER					(0x01) /* Команда включения/выключения канала ШИМ'а */

/*UNWDS-LIT*/
#define LIT_MEASURE					(0x00) /* Команда запроса замера освещенности */
#define LIT_MEASURE_STATUS			(0x01) /* Результаты замера освещенности */

/*UNWDS-GPIO*/
#define GPIO_CMD					(0x00) /* Отправка команды для ножки порта */

/*UNWDS-UART*/
#define SEND_BY_UART 				(0x00) /* Команда отправки данных по UART'у */

/*---------------------------------------------------------------------------*/
#endif
/*---------------------------------------------------------------------------*/
