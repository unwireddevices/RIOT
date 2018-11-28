/*
 * Copyright (C) 2015 Freie Universität Berlin
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     unwds_networking
 * @{
 *
 * @file
 * @brief       DAG node 
 *
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 *
 * @}
 */

#include <stdio.h>

#include "unwds-udp.h"
#include "dag_node.h"
#include "system-common.h"

#include "net/gnrc/ipv6.h"
#include "net/gnrc/netif.h"
#include "checksum/ucrc16.h"
#include "crypto/ciphers.h"
#include "crypto/modes/cbc.h"

#define ENABLE_DEBUG            (1)
#include "debug.h"
#include "od.h"

#define MODE_NORMAL				(0x01)
#define MODE_NOTROOT			(0x02)
#define MODE_JOIN_PROGRESS		(0x03)
#define MODE_NEED_REBOOT		(0x04)

#define AES_KEY_LEN 	(16)

ipv6_addr_t root_addr;

uint8_t aes_key[AES_KEY_LEN] = {
    0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88,
    0x99, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
};	/* Ключ шифрования */

uint8_t aes_iv[AES_KEY_LEN] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};	/* Вектор инициализации */

uint8_t nonce_xor_aes_key[AES_KEY_LEN];

cipher_t cipher_aes_128;
cipher_t cipher_nonce_xor_aes_128;

volatile u8_u16_t packet_counter_node;			/* Счетчик пакетов */
volatile u8_u16_t packet_counter_root;			/* Счетчик пакетов */
volatile uint8_t node_mode = MODE_NOTROOT;		/* Режим работы ноды */

/* Первая стадия авторизации */
static void join_stage_1_sender(void);

/* Третья стадия авторизации */
static void join_stage_3_sender(uint8_t *data);

void unwds_dag_server(gnrc_pktsnip_t *pkt)
{
#if ENABLE_DEBUG
	DEBUG("UNWDS_UDP: data received:\n");
	od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
#endif /* ENABLE_DEBUG */

	/* LED ON */ 
	
	printf("[DAG Node] ");

	/* Отражаем структуру на массив */ 
	header_t *header_pack = pkt->data; 
	
	switch(header_pack->protocol_version) 
	{
        case UDBP_PROTOCOL_VERSION: 
			/* Проверяем ID модуля и тип пакета */ 
			if((header_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID) && (header_pack->data_type == JOIN_STAGE_2))
			{
				/* Третья стадия авторизации */
				join_stage_3_sender(pkt->data);
				break;
			}
			
			if(node_mode != MODE_NORMAL)
				return;

			/* Расшифровываем данные */
			cipher_encrypt_cbc (&cipher_nonce_xor_aes_128, aes_iv, 
								&((uint8_t*)(pkt->data))[HEADER_DOWN_OFFSET], 
								iterator_to_byte(pkt->size - HEADER_UP_LENGTH), 
								&((uint8_t*)(pkt->data))[HEADER_DOWN_OFFSET]);

			/* Вывод информационного сообщения в консоль */
			// od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
			// printf("DAG Node: UDP no crypto packet received(%"PRIu8"): ", datalen);
			// for (uint16_t i = 0; i < datalen; i++)	/*Выводим принятый пакет*/ 
				// printf("%"PRIXX8, data[i]);
			// printf("\n");
			
			/* CRC16 check */ 
			uint16_t crc16 = ucrc16_calc_be (&((uint8_t*)(pkt->data))[PAYLOAD_OFFSET], 
											header_pack->length, 
											UCRC16_CCITT_POLY_BE, 
											0xFFFF);
			if(crc16 != header_pack->crc.u16)
			{
				/* Вывод сообщения об ошибке счетчика пакетов */
				printf("CRC16 Error!\n");
				break;
			}
			
			/* Защита от атаки повтором */
			/* Проверяем счетчик пакетов на валидность данного пакета */
			if(packet_counter_root.u16 >= header_pack->counter.u16)
			{	
				/* Вывод сообщения об ошибке счетчика пакетов */
				printf("Counter error!\n");
				break;
			}
			
			/* Обновляем значение счетчика ROOT'а */
			packet_counter_root.u16 = header_pack->counter.u16;	

			switch(header_pack->device_id)
			{
				case UNWDS_GPIO_MODULE_ID: /* ID: 1 */
					switch(header_pack->data_type)
					{
						case GPIO_CMD:
							break;
						default:
							print_unknown_command_for_umdk("GPIO");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_GPIO_MODULE_ID */

				case UNWDS_4BTN_MODULE_ID: /* ID: 2 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("4BTN");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_4BTN_MODULE_ID */

				case UNWDS_GPS_MODULE_ID: /* ID: 3 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("GPS");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_GPS_MODULE_ID */

				case UNWDS_LSM6DS3_MODULE_ID: /* ID: 4 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("LSM6DS3");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_LSM6DS3_MODULE_ID */

				case UNWDS_LM75_MODULE_ID: /* ID: 5 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("LM75");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_LM75_MODULE_ID */

				case UNWDS_LMT01_MODULE_ID: /* ID: 6 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("LMT01");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_LMT01_MODULE_ID */

				case UNWDS_UART_MODULE_ID: /* ID: 7 */
					switch(header_pack->data_type)
					{
						case SEND_BY_UART:
							break;
						default:
							print_unknown_command_for_umdk("UART");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_UART_MODULE_ID */

				case UNWDS_SHT21_MODULE_ID: /* ID: 8 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("SHT21");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_SHT21_MODULE_ID */

				case UNWDS_PIR_MODULE_ID: /* ID: 9 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("PIR");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_PIR_MODULE_ID */

				case UNWDS_ADC_MODULE_ID: /* ID: 10 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("ADC");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_ADC_MODULE_ID */

				case UNWDS_LPS331_MODULE_ID: /* ID: 11 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("LPS331");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_LPS331_MODULE_ID */

				case UNWDS_COUNTER_MODULE_ID: /* ID: 12 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("COUNTER");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_COUNTER_MODULE_ID */

				case UNWDS_RSSIECHO_MODULE_ID: /* ID: 13 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("RSSIECHO");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_RSSIECHO_MODULE_ID */

				case UNWDS_PWM_MODULE_ID: /* ID: 14 */
					switch(header_pack->data_type)
					{
						case PWM_SETTINGS:
							break;
						case PWM_POWER:
							break;
						default:
							print_unknown_command_for_umdk("PWM");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_PWM_MODULE_ID */

				case UNWDS_OPT3001_MODULE_ID: /* ID: 15 */
					switch(header_pack->data_type)
					{
						case LIT_MEASURE:
							lit_measure_dag_sender();
							break;
						default:
							print_unknown_command_for_umdk("OPT3001");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_OPT3001_MODULE_ID */

				case UNWDS_RESERVED1_MODULE_ID: /* ID: 16 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("RESERVED1");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_RESERVED1_MODULE_ID */

				case UNWDS_BME280_MODULE_ID: /* ID: 17 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("BME280");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_BME280_MODULE_ID */

				case UNWDS_MHZ19_MODULE_ID: /* ID: 18 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("MHZ19");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_MHZ19_MODULE_ID */

				case UNWDS_USOUND_MODULE_ID: /* ID: 19 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("USOUND");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_USOUND_MODULE_ID */

				case UNWDS_ADXL345_MODULE_ID: /* ID: 20 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("ADXL345");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_ADXL345_MODULE_ID */

				case UNWDS_IBUTTON_MODULE_ID: /* ID: 21 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("IBUTTON");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_IBUTTON_MODULE_ID */

				case UNWDS_HD44780_MODULE_ID: /* ID: 22 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("UNWDS_HD44780");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_HD44780_MODULE_ID */

				case UNWDS_R300_MODULE_ID: /* ID: 23 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("R300");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_R300_MODULE_ID */

				case UNWDS_IRBLASTER_MODULE_ID: /* ID: 24 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("IRBLASTER");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_IRBLASTER_MODULE_ID */

				case UNWDS_HX711_MODULE_ID: /* ID: 25 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("HX711");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_HX711_MODULE_ID */

				case UNWDS_FDC1004_MODULE_ID: /* ID: 26 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("FDC1004");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_FDC1004_MODULE_ID */

				case UNWDS_CL420_MODULE_ID: /* ID: 27 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("CL420");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_CL420_MODULE_ID */

				case UNWDS_MODBUS_MODULE_ID: /* ID: 28 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("MODBUS");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_MODBUS_MODULE_ID */

				case UNWDS_RADIORELAY_MODULE_ID: /* ID: 29 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("RADIORELAY");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_RADIORELAY_MODULE_ID */

				case UNWDS_CR95_MODULE_ID: /* ID: 30 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("CR95");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_CR95_MODULE_ID */

				case UNWDS_M200_MODULE_ID: /* ID: 50 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("M200");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_M200_MODULE_ID */

				case UNWDS_PULSE_MODULE_ID: /* ID: 51 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("PULSE");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_PULSE_MODULE_ID */

				case UNWDS_PACS_MODULE_ID: /* ID: 52 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("PACS");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_PACS_MODULE_ID */

				case UNWDS_SWITCH_MODULE_ID: /* ID: 53 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("SWITCH");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_SWITCH_MODULE_ID */

				case UNWDS_M230_MODULE_ID: /* ID: 54 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("M230");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_M230_MODULE_ID */

				case UNWDS_IEC61107_MODULE_ID: /* ID: 55 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("IEC61107");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_IEC61107_MODULE_ID */

				case UNWDS_IDCARD_MODULE_ID: /* ID: 56 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("IDCARD");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_IDCARD_MODULE_ID */

				case UNWDS_DALI_MODULE_ID: /* ID: 57 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("DALI");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_DALI_MODULE_ID */

				case UNWDS_WIEGAND_MODULE_ID: /* ID: 58 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("WIEGAND");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_WIEGAND_MODULE_ID */

				case UNWDS_INCLINOMETER_MODULE_ID: /* ID: 59 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("INCLINOMETER");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_INCLINOMETER_MODULE_ID */

				case UNWDS_PARKING_MODULE_ID: /* ID: 60 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("PARKING");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_PARKING_MODULE_ID */

				case UNWDS_GARBAGE_MODULE_ID: /* ID: 61 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("GARBAGE");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_GARBAGE_MODULE_ID */

				case UNWDS_CUSTOMER_MODULE_ID: /* ID: 100 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("CUSTOMER");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_CUSTOMER_MODULE_ID */

				case UNWDS_CONFIG_MODULE_ID: /* ID: 126 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("CONFIG");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_CONFIG_MODULE_ID */

				case UNWDS_6LOWPAN_SYSTEM_MODULE_ID: /* ID: 127 */
					switch(header_pack->data_type)
					{
						case PONG:
							break;
						default:
							printf("Unknown command for system!\n");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_6LOWPAN_SYSTEM_MODULE_ID */
				default:
					printf("Unknown module!\n");
					break;
			} /* header_pack->device_id */
            break;
        default:
            printf("Unknown protocol version!\n");
			break;
    } /* header_pack->protocol_version */
	
	/*LED OFF*/ 

    gnrc_pktbuf_release(pkt);
    return;
}

/* Конструктор пакета */
void unwds_pack_sender( uint8_t device_id, 
						uint8_t data_type, 
						uint8_t payload_len, 
						uint8_t *payload)
{	
	if(node_mode != MODE_NORMAL)
		return;

	/* Проверка на то что передан не нулевой адрес буфера */
	if ((payload == NULL) && (payload_len != 0))
		return;
	
	/* Выделяем память под пакет. Общий размер пакета (header + payload) */
	uint8_t crypto_length = iterator_to_byte(HEADER_DOWN_LENGTH + payload_len);
	uint8_t udp_buffer[HEADER_UP_LENGTH + crypto_length];
	
	/* Отражаем структуры на массивы */ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	
	/* Заполняем пакет */  
	/* Header */ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/* Текущая версия протокола */ 
	header_pack->device_id = device_id;							/* ID устройства */
	header_pack->data_type = data_type;							/* Тип пакета */  
	header_pack->temperature = (int8_t)(nrf_temp_read() >> 2);	/* Температура */ 
	header_pack->voltage = 0x00;								/* Напряжение */ 
	header_pack->counter.u16 = packet_counter_node.u16;			/* Счетчик пакетов */ 
	header_pack->length = payload_len;							/* Размер пакета (незашифрованного) */
	
	/* Payload */ 	
	/* Заполняем пакет, зашифровываем и отправляем его DAG'у. */ 
	for(uint8_t i = 0; i < (crypto_length - HEADER_DOWN_LENGTH); i++)
	{
		if(i < payload_len)
			udp_buffer[PAYLOAD_OFFSET + i] = payload[i];
		else
			udp_buffer[PAYLOAD_OFFSET + i] = 0x00;
	}
	// memcpy(&udp_buffer[PAYLOAD_OFFSET], payload, payload_len);
	// memset(&udp_buffer[PAYLOAD_OFFSET+payload_len], 0x00, crypto_length);

	/* CRC16 */ 
	header_pack->crc.u16 = ucrc16_calc_be ((uint8_t*)&udp_buffer[PAYLOAD_OFFSET], 
											header_pack->length, 
											UCRC16_CCITT_POLY_BE, 
											0xFFFF);
	
#if ENABLE_DEBUG
	char root_addr_str[IPV6_ADDR_MAX_STR_LEN];
	ipv6_addr_to_str(root_addr_str, &root_addr, sizeof(root_addr_str));
	
	DEBUG("[UNWDS_UDP] Success: sent %u byte(s) to [%s]:%u\n", (HEADER_DOWN_LENGTH + crypto_length), root_addr_str, UNWDS_UDP_SERVER_PORT);
	od_hex_dump(udp_buffer, (HEADER_DOWN_LENGTH + crypto_length), OD_WIDTH_DEFAULT);
#endif /* ENABLE_DEBUG */
	
	/* Зашифровываем данные */
	cipher_encrypt_cbc (&cipher_nonce_xor_aes_128, 
						aes_iv,
						&udp_buffer[HEADER_DOWN_OFFSET], 
						crypto_length, 
						&udp_buffer[HEADER_DOWN_OFFSET]);

	/*Отправляем пакет*/ 
	udp_send(&root_addr, UNWDS_UDP_SERVER_PORT, udp_buffer, (HEADER_UP_LENGTH + crypto_length));
	/* Инкрементируем счетчик пакетов */
}

/* Первая стадия авторизации */
/* Передаём свой серийный номер */
static void join_stage_1_sender(void)
{
	node_mode = MODE_JOIN_PROGRESS;			/* Режим работы ноды */

	/* Вывод информационного сообщения в консоль */
	printf("[DAG Node] Send join packet to DAG-root node: ");
	char root_addr_str[IPV6_ADDR_MAX_STR_LEN];
	printf("%s \n", ipv6_addr_to_str(root_addr_str, &root_addr, sizeof(root_addr_str)));

	/* Выделяем память под пакет. Общий размер пакета (header + payload) */	
	uint8_t udp_buffer[HEADER_LENGTH + JOIN_STAGE_1_PAYLOAD_LENGTH];
	
	/* Отражаем структуры на массив */ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	join_stage_1_t *join_stage_1_pack = (join_stage_1_t*)&udp_buffer[PAYLOAD_OFFSET];
	
	/* Заполняем пакет */  
	/* Header */ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/* Текущая версия протокола */ 
	header_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/* ID устройства */
	header_pack->data_type = JOIN_STAGE_1;						/* Тип пакета */  
	header_pack->temperature = (int8_t)(nrf_temp_read() >> 2);	/* Температура */ 
	header_pack->voltage = 0x00;								/* Напряжение */ 
	header_pack->counter.u16 = 0x0000;							/* Счетчик пакетов */ 
	header_pack->length = JOIN_STAGE_1_LENGTH;					/* Размер пакета */

	/* Payload */
	/* Потом здесь будет список установленных модулей */
	join_stage_1_pack->module_id = 0;	//UNWDS_MODULE_ID;
	
	/* CRC16 */ 
	header_pack->crc.u16 = ucrc16_calc_be ((uint8_t*)&udp_buffer[PAYLOAD_OFFSET], 
											header_pack->length, 
											UCRC16_CCITT_POLY_BE, 
											0xFFFF);
	
	/* Отправляем пакет */ 
	udp_send(&root_addr, UNWDS_UDP_SERVER_PORT, udp_buffer, (HEADER_LENGTH + JOIN_STAGE_1_PAYLOAD_LENGTH));
	packet_counter_node.u16++;	/* Инкрементируем счетчик пакетов */ 
}

/* Третья стадия авторизации */
static void join_stage_3_sender(uint8_t *data) 
{	
	/* Вывод информационного сообщения в консоль */
	printf("Send join packet stage 3 to DAG-root node: ");
	char root_addr_str[IPV6_ADDR_MAX_STR_LEN];
	printf("%s \n", ipv6_addr_to_str(root_addr_str, &root_addr, sizeof(root_addr_str)));
	
	/* Выделяем память под пакет. Общий размер пакета (header + payload) */
	uint8_t udp_buffer[HEADER_UP_LENGTH + JOIN_STAGE_3_PAYLOAD_LENGTH];	
	
	/* Отражаем структуры на массивы */ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	header_t *header_root_pack = (header_t*)&data[HEADER_OFFSET];
	join_stage_2_t *join_stage_2_pack = (join_stage_2_t*)&data[PAYLOAD_OFFSET];
	join_stage_3_t *join_stage_3_pack = (join_stage_3_t*)&udp_buffer[PAYLOAD_OFFSET];
	
	/* Заполняем пакет */  
	/* Header */ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/* Текущая версия протокола */ 
	header_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/* ID устройства */
	header_pack->data_type = JOIN_STAGE_3;						/* Тип пакета */  
	header_pack->temperature = (int8_t)(nrf_temp_read() >> 2);	/* Температура */ 
	header_pack->voltage = 0x00;								/* Напряжение */ 
	header_pack->counter.u16 = 0x0000;							/* Счетчик пакетов */ 
	header_pack->length = JOIN_STAGE_3_LENGTH;					/* Размер пакета */
	
	/* Payload*/ 
	/* Расшифровываем данные */ 
	cipher_encrypt_cbc (&cipher_aes_128,
						aes_iv, 
						&data[HEADER_DOWN_OFFSET], 
						CRYPTO_1_BLOCK_LENGTH, 
						&data[HEADER_DOWN_OFFSET]);
	
	/* CRC16 check */ 
	uint16_t crc16 = ucrc16_calc_be(&data[PAYLOAD_OFFSET], 
									header_root_pack->length, 
									UCRC16_CCITT_POLY_BE, 
									0xFFFF);

	if(crc16 != header_root_pack->crc.u16)
	{
		/* Вывод сообщения об ошибке счетчика пакетов */
		printf("CRC16 Error!\n");
		return;
	}
	
	packet_counter_root.u16 = header_root_pack->counter.u16;
	
	/* Копируем полученный nonce и используем его в качестве сессионного ключа */
	memcpy(nonce_xor_aes_key, aes_key, AES_KEY_LEN);
	nonce_xor_aes_key[0] ^= join_stage_2_pack->nonce.u8[0];
	nonce_xor_aes_key[1] ^= join_stage_2_pack->nonce.u8[1];

	int err = cipher_init(&cipher_nonce_xor_aes_128, CIPHER_AES_128, nonce_xor_aes_key, AES_KEY_LEN);
	if(!(err))
	{
		printf("Error cipher_init(): %i\n", err);
		return;
	}

	/* Режим работы ноды */
	node_mode = MODE_NORMAL;			
	
	/* Отправляем ROOT'у nonce на еденицу больше для того что бы он был уверен что у нас одинаковое шифрование */ 
	join_stage_3_pack->nonce.u16 = join_stage_2_pack->nonce.u16 + 1; /* Увеличиваем nonce на единицу: nonce += 1 */	
	
	/* Дозаполняем блок для шифрования нулями */ 
	for(uint8_t i = JOIN_STAGE_3_LENGTH; i < (JOIN_STAGE_3_PAYLOAD_LENGTH - HEADER_DOWN_LENGTH); i++)
		udp_buffer[PAYLOAD_OFFSET + i] = 0x00;
	
	/* CRC16 */ 
	header_pack->crc.u16 = ucrc16_calc_be ((uint8_t*)&udp_buffer[PAYLOAD_OFFSET], 
											header_pack->length, 
											UCRC16_CCITT_POLY_BE, 
											0xFFFF);
	
	/* Зашифровываем данные */
	cipher_encrypt_cbc (&cipher_aes_128, 
						aes_iv, 
						&udp_buffer[HEADER_DOWN_OFFSET], 
						CRYPTO_1_BLOCK_LENGTH, 
						&udp_buffer[HEADER_DOWN_OFFSET]);

	/* Отправляем пакет */ 
	udp_send(&root_addr, UNWDS_UDP_SERVER_PORT, udp_buffer, (HEADER_UP_LENGTH + JOIN_STAGE_3_PAYLOAD_LENGTH));
	packet_counter_node.u16++;	/* Инкрементируем счетчик пакетов */
}

int dag_node_init(void)
{
	int err;

	nrf_temp_init();

	err = cipher_init(&cipher_aes_128, CIPHER_AES_128, aes_key, AES_KEY_LEN);
	if(!(err))
	{
		printf("Error cipher_init(): %i\n", err);
		return -1;
	}

	packet_counter_node.u16 = 0x0000;		/* Счетчик пакетов */
	packet_counter_root.u16 = 0x0000;		/* Счетчик пакетов */
	
	// fe80::2f3a:f0dc:a085:a2ab
	root_addr.u8[0] = 0xFE;
	root_addr.u8[1] = 0x80;
	root_addr.u8[2] = 0x00;
	root_addr.u8[3] = 0x00;
	root_addr.u8[4] = 0x00;
	root_addr.u8[5] = 0x00;
	root_addr.u8[6] = 0x00;
	root_addr.u8[7] = 0x00;
	root_addr.u8[8] = 0x2F;
	root_addr.u8[9] = 0x3A;
	root_addr.u8[10] = 0xF0;
	root_addr.u8[11] = 0xDC;
	root_addr.u8[12] = 0xA0;
	root_addr.u8[13] = 0x85;
	root_addr.u8[14] = 0xA2;
	root_addr.u8[15] = 0xAB;

	err = unwds_udp_server_init();
	if(err < 0)
	{
		puts("Error init unwds udp server.");	
		return -1;
	}
	
	puts("Init unwds udp server.");	
	start_unwds_udp_server();

	join_stage_1_sender();

	return 0;
}

#ifdef UMDK_4BTN
/* Функция отправки состояния кнопок */
void button_status_dag_sender ( uint8_t button_number,
								uint8_t click_type)
{
	/* Заполняем payload */
	button_status_t button_status_pack;				/* Создаем структуру */
	
	button_status_pack.button_status = button_number;

	if(click_type == LONG_CLICK)
		button_status_pack.button_status |= LONG_CLICK;
	
	/* Вывод информационного сообщения в консоль */
	print_send_packet("button status");
	
	/* Отправляем пакет */
	unwds_pack_sender ( UNWDS_4BTN_MODULE_ID,				/* ID модуля */
						BUTTON_STATUS, 						/* Команда включения канала ШИМ'а */
						BUTTON_STATUS_LENGTH, 				/* Размер payload'а */
						(uint8_t*)&button_status_pack );	/* Payload */		
}
#endif

#ifdef UMDK_LIT
/* Совершить замер освещенности */			
bool lit_measure_dag_sender(void)
{
	/* Заполняем payload */
	lit_measure_status_t lit_measure_status_pack;						/* Создаем структуру */
	
	lit_measure_status_pack.lit_measure_status = 123;//lit_measure_status;	/* Измеряем освещенность */
	
	/* Вывод информационного сообщения в консоль */
	print_send_packet("OPT3001 measure status");
	printf("[UMDK-OPT3001] Luminocity: %lu lux\n", lit_measure_status_pack.lit_measure_status);
	
	/* Отправляем пакет */	
	unwds_pack_sender ( UNWDS_OPT3001_MODULE_ID,				/* ID модуля */
						LIT_MEASURE_STATUS, 					/* Команда включения канала ШИМ'а */
						LIT_MEASURE_STATUS_LENGTH, 				/* Размер payload'а */
						(uint8_t*)&lit_measure_status_pack);	/* Payload */		
				
	return lit_measure_status_pack.lit_measure_status;
}
#endif
