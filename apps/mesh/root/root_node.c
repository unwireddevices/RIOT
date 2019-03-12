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
 * @brief       ROOT node 
 *
 * @author      Manchenko Oleg <man4enkoos@gmail.com>
 *
 * @}
 */

#include <stdio.h>

#include "unwds-udp.h"
#include "root_node.h"
#include "system-common.h"

#include "net/gnrc/ipv6.h"
#include "net/gnrc/netif.h"
#include "net/gnrc/rpl.h"
#include "checksum/ucrc16.h"
#include "crypto/ciphers.h"
#include "crypto/modes/cbc.h"
#include "periph/hwrng.h"

#define ENABLE_DEBUG		(0)
#include "debug.h"
#include "od.h"

#define AES_KEY_LEN 		(16)

#define MAX_NUM_DEVICES 	(16)

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

volatile u8_u16_t packet_counter_root;			/* Счетчик пакетов */

/* Конструктор пакета */
void unwds_pack_sender (ipv6_addr_t *dest_addr, 
						uint8_t device_id, 
						uint8_t data_type, 
						uint8_t payload_len, 
						uint8_t *payload);

/* Struct for security table */
typedef struct {
	ipv6_addr_t addr;		/* addr */
	u8_u16_t counter;		/* counter */
	u8_u16_t nonce;			/* nonce */
} pack security_table_t;

security_table_t security_table[MAX_NUM_DEVICES];

static int16_t add_device_security_table(ipv6_addr_t *addr, uint16_t nonce);
static int16_t unlock_device_security_table(ipv6_addr_t *addr);
static int16_t find_empty_elements(void);
static int16_t find_device_security_table(ipv6_addr_t *addr);



/* Вторая стадия авторизации */
static void join_stage_2_sender(ipv6_addr_t *dest_addr);

/* Обработчик третий стадии авторизации */
static void join_stage_3_handler(ipv6_addr_t *dest_addr, 
								uint8_t *data);

/* Обработчик нажатой кнопки */
static void button_status_root_handler (ipv6_addr_t *src_addr, 
										button_status_t *button_status_pack);

/* Обработчик пакета с измерением освещенности */
static void lit_measure_status_root_handler( ipv6_addr_t *src_addr, 
											 lit_measure_status_t *lit_measure_status_pack);
										

void unwds_root_server(gnrc_pktsnip_t *pkt)
{
#if ENABLE_DEBUG
	DEBUG("UNWDS_UDP: data received:\n");
	od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
#endif /* ENABLE_DEBUG */

	/* LED ON */ 
	
	/* Copy IPv6 address */ 
	gnrc_pktsnip_t *snip = pkt;
	ipv6_addr_t src_addr;

    while (snip != NULL) {
		if(snip->type == GNRC_NETTYPE_IPV6)
		{
			memcpy(&src_addr, &(((ipv6_hdr_t *)(snip->data))->src), sizeof(ipv6_addr_t));
			break;
		}
		else
			snip = snip->next;
    }
	
	/* Print IPv6 address */
	print_ipv6_addr(&src_addr);
	
	/* Отражаем структуру на массив */ 
	header_t *header_pack = pkt->data; 
	
	switch(header_pack->protocol_version) 
	{
        case UDBP_PROTOCOL_VERSION: 
			/* Проверяем ID модуля и тип пакета */ 
			if((header_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID) && (header_pack->data_type == JOIN_STAGE_1))
			{
				/* Вторая стадия авторизации */			
				join_stage_2_sender(&src_addr);
				break;
			}
			
			else if((header_pack->device_id == UNWDS_6LOWPAN_SYSTEM_MODULE_ID) && (header_pack->data_type == JOIN_STAGE_3))
			{
				/* Обработчик третий стадии авторизации */
				join_stage_3_handler(&src_addr, pkt->data);
				break;
			}

			/* Получаем nonce */
			int16_t device = find_device_security_table(&src_addr);
			if(device < 0)
				return;

			u8_u16_t nonce;
			nonce.u16 = security_table[device].nonce.u16;
			
			/* Копируем полученный nonce и используем его в качестве сессионного ключа */
			memcpy(nonce_xor_aes_key, aes_key, AES_KEY_LEN);
			nonce_xor_aes_key[0] ^= nonce.u8[0];
			nonce_xor_aes_key[1] ^= nonce.u8[1];

			int err = cipher_init(&cipher_nonce_xor_aes_128, CIPHER_AES_128, nonce_xor_aes_key, AES_KEY_LEN);
			if(!(err))
			{
				printf("Error cipher_init(): %i\n", err);
				return;
			}

			/* Расшифровываем данные */
			cipher_decrypt_cbc (&cipher_nonce_xor_aes_128, aes_iv, 
								&((uint8_t*)(pkt->data))[HEADER_DOWN_OFFSET], 
								iterator_to_byte(pkt->size - HEADER_UP_LENGTH), 
								&((uint8_t*)(pkt->data))[HEADER_DOWN_OFFSET]);

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
			if(security_table[device].counter.u16 >= header_pack->counter.u16)
			{	
				/* Вывод сообщения об ошибке счетчика пакетов */
				printf("Counter error!\n");
				break;
			}
			/* Обновляем значение счетчика в таблице */
			security_table[device].counter.u16 = header_pack->counter.u16;
			
			/* Вывод принятого пакета микрокомпьютеру */ 
			// print_cr(&src_addr, pkt->data, (HEADER_LENGTH + header_pack->length));

			switch(header_pack->device_id)
			{
				case UNWDS_GPIO_MODULE_ID: /* ID: 1 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("GPIO");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_GPIO_MODULE_ID */

				case UNWDS_4BTN_MODULE_ID: /* ID: 2 */
					switch(header_pack->data_type)
					{
						case BUTTON_STATUS:
							button_status_root_handler(&src_addr, (button_status_t*)&((uint8_t*)(pkt->data))[PAYLOAD_OFFSET]);
							break;
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

				case UNWDS_GASSENSOR_MODULE_ID: /* ID: 4 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("GASSENSOR");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_GASSENSOR_MODULE_ID */

				// case UNWDS_LM75_MODULE_ID: /* ID: 5 */
				// 	switch(header_pack->data_type)
				// 	{
				// 		default:
				// 			print_unknown_command_for_umdk("LM75");
				// 			break;
				// 	} /* header_pack->data_type */
				// 	break; /* UNWDS_LM75_MODULE_ID */

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

				// case UNWDS_SHT21_MODULE_ID: /* ID: 8 */
				// 	switch(header_pack->data_type)
				// 	{
				// 		default:
				// 			print_unknown_command_for_umdk("SHT21");
				// 			break;
				// 	} /* header_pack->data_type */
				// 	break; /* UNWDS_SHT21_MODULE_ID */

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

				// case UNWDS_LPS331_MODULE_ID: /* ID: 11 */
				// 	switch(header_pack->data_type)
				// 	{
				// 		default:
				// 			print_unknown_command_for_umdk("LPS331");
				// 			break;
				// 	} /* header_pack->data_type */
				// 	break; /* UNWDS_LPS331_MODULE_ID */

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
						default:
							print_unknown_command_for_umdk("PWM");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_PWM_MODULE_ID */

				case UNWDS_LIGHT_MODULE_ID: /* ID: 15 */
					switch(header_pack->data_type)
					{
						case LIT_MEASURE_STATUS:
							lit_measure_status_root_handler(&src_addr, (lit_measure_status_t*)&((uint8_t*)(pkt->data))[PAYLOAD_OFFSET]);
							break;
						default:
							print_unknown_command_for_umdk("LIGHT");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_LIGHT_MODULE_ID */

				// case UNWDS_RESERVED1_MODULE_ID: /* ID: 16 */
				// 	switch(header_pack->data_type)
				// 	{
				// 		default:
				// 			print_unknown_command_for_umdk("RESERVED1");
				// 			break;
				// 	} /* header_pack->data_type */
				// 	break; /* UNWDS_RESERVED1_MODULE_ID */

				case UNWDS_METEO_MODULE_ID: /* ID: 17 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("METEO");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_METEO_MODULE_ID */

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

				case UNWDS_INCLINOMETER_MODULE_ID: /* ID: 20 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("INCLINOMETER");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_INCLINOMETER_MODULE_ID */

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

				case UNWDS_ST95_MODULE_ID: /* ID: 30 */
					switch(header_pack->data_type)
					{
						default:
							print_unknown_command_for_umdk("ST95");
							break;
					} /* header_pack->data_type */
					break; /* UNWDS_ST95_MODULE_ID */

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

				// case UNWDS_INCLINOMETER_MODULE_ID: /* ID: 59 */
				// 	switch(header_pack->data_type)
				// 	{
				// 		default:
				// 			print_unknown_command_for_umdk("INCLINOMETER");
				// 			break;
				// 	} /* header_pack->data_type */
				// 	break; /* UNWDS_INCLINOMETER_MODULE_ID */

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
						case PING:
							break;
						case ACK:
							break;
						case NACK:
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
            break; /* protocol version: l */
        default:
            printf("Unknown protocol version!\n");
			break;
    } /* header_pack->protocol_version */
	
	/* LED OFF */ 
	
    gnrc_pktbuf_release(pkt);
    return;
}

/* Конструктор пакета */
void unwds_pack_sender (ipv6_addr_t *dest_addr, 
						uint8_t device_id, 
						uint8_t data_type, 
						uint8_t payload_len, 
						uint8_t *payload)
{
	/* Проверка на то что передан существующий адрес */
	if (dest_addr == NULL)
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
	header_pack->voltage = 0x00; 								/* Напряжение */ 
	header_pack->counter.u16 = packet_counter_root.u16;			/* Счетчик пакетов */ 
	header_pack->length = payload_len;							/* Размер пакета (незашифрованного) */
	
	/* Payload */ 
	/* Заполняем пакет, зашифровываем и отправляем его DAG'у */ 
	// memcpy(&udp_buffer[PAYLOAD_OFFSET], payload, payload_len);
	for(uint8_t i = 0; i < (crypto_length - HEADER_DOWN_LENGTH); i++)
	{
		if(i < payload_len)
			udp_buffer[PAYLOAD_OFFSET + i] = payload[i];
		else
			udp_buffer[PAYLOAD_OFFSET + i] = 0x00;
	}
	
	/* CRC16 */ 
	header_pack->crc.u16 = ucrc16_calc_be ((uint8_t*)&udp_buffer[PAYLOAD_OFFSET], 
											header_pack->length, 
											UCRC16_CCITT_POLY_BE, 
											0xFFFF);

#if ENABLE_DEBUG
	char dest_addr_str[IPV6_ADDR_MAX_STR_LEN];
	ipv6_addr_to_str(dest_addr_str, dest_addr, sizeof(dest_addr_str));
	
	DEBUG("UNWDS_UDP Success: sent %u byte(s) to [%s]:%u\n", (HEADER_UP_LENGTH + crypto_length), dest_addr_str, UNWDS_UDP_SERVER_PORT);
	od_hex_dump(udp_buffer, (HEADER_UP_LENGTH + crypto_length), OD_WIDTH_DEFAULT);
#endif /* ENABLE_DEBUG */

	/* Получаем nonce */
	int16_t device = find_device_security_table(dest_addr);
	if(device < 0)
		return;

	u8_u16_t nonce;
	nonce.u16 = security_table[device].nonce.u16;
	
	/* Копируем полученный nonce и используем его в качестве сессионного ключа */
	memcpy(nonce_xor_aes_key, aes_key, AES_KEY_LEN);
	nonce_xor_aes_key[0] ^= nonce.u8[0];
	nonce_xor_aes_key[1] ^= nonce.u8[1];

	int err = cipher_init(&cipher_nonce_xor_aes_128, CIPHER_AES_128, nonce_xor_aes_key, AES_KEY_LEN);
	if(!(err))
	{
		printf("Error cipher_init(): %i\n", err);
		return;
	}

	/* Зашифровываем блок */ 
	cipher_decrypt_cbc (&cipher_nonce_xor_aes_128, 
						aes_iv, 
						&udp_buffer[HEADER_DOWN_OFFSET], 
						crypto_length, 
						&udp_buffer[HEADER_DOWN_OFFSET]);

	/* Отправляем пакет */ 
	udp_send(dest_addr, UNWDS_UDP_SERVER_PORT, udp_buffer, (HEADER_UP_LENGTH + crypto_length));
	packet_counter_root.u16++;	/* Инкрементируем счетчик пакетов */
}

static void join_stage_2_sender(ipv6_addr_t *dest_addr)
{	
	/* Вывод принятого пакета микрокомпьютеру */ 
	// print_cr((uip_ip6addr_t*)dest_addr, (uint8_t*)data, (HEADER_LENGTH + JOIN_STAGE_1_LENGTH));

	puts("Join stage 2");
	
	/* Выделяем память под пакет. Общий размер пакета (header + payload) */
	uint8_t udp_buffer[HEADER_UP_LENGTH + JOIN_STAGE_2_PAYLOAD_LENGTH];
	
	/* Отражаем структуры на массивы*/ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	join_stage_2_t *join_stage_2_pack = (join_stage_2_t*)&udp_buffer[PAYLOAD_OFFSET];
	
	/* CRC16 check */ 
	// uint16_t crc16 = ucrc16_calc_be (&((uint8_t*)&data[PAYLOAD_OFFSET], 
	// 								header_root_pack->length, 
	// 								UCRC16_CCITT_POLY_BE, 
	// 								0xFFFF);
	// if(crc16 != header_root_pack->crc)
	// {
	// 	/* Вывод сообщения об ошибке счетчика пакетов */
	// 	printf("CRC16 Error!\n");
	// 	return;
	// }

	/* Заполняем пакет */  
	/* Header */ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/* Текущая версия протокола */ 
	header_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/* ID устройства */
	header_pack->data_type = JOIN_STAGE_2;						/* Тип пакета */  
	header_pack->temperature = (int8_t)(nrf_temp_read() >> 2);	/* Температура */ 
	header_pack->voltage = 0x00;								/* Напряжение */ 
	header_pack->counter.u16 = packet_counter_root.u16;			/* Счетчик пакетов */ 
	header_pack->length = JOIN_STAGE_2_LENGTH;					/* Размер пакета (незашифрованного) */
	
	/* Payload */ 
	hwrng_read(&(join_stage_2_pack->nonce.u16), 2);
	
	/* Добавляем устройство */ 
	add_device_security_table ( dest_addr,						/* Address */ 
	 							join_stage_2_pack->nonce.u16);	/* Nonce */ 
	
	/* Дозаполняем блок для шифрования нулями */ 
	for(uint8_t i = JOIN_STAGE_2_LENGTH; i < (JOIN_STAGE_2_PAYLOAD_LENGTH - HEADER_DOWN_LENGTH); i++)
		udp_buffer[PAYLOAD_OFFSET + i] = 0x00;
	
	/* CRC16 */ 
	header_pack->crc.u16 = ucrc16_calc_be ((uint8_t*)&udp_buffer[PAYLOAD_OFFSET], 
											header_pack->length, 
											UCRC16_CCITT_POLY_BE, 
											0xFFFF);
	
	/* Зашифровываем блок */ 
	cipher_decrypt_cbc (&cipher_aes_128, 
						aes_iv, 
						&udp_buffer[HEADER_DOWN_OFFSET], 
						CRYPTO_1_BLOCK_LENGTH, 
						&udp_buffer[HEADER_DOWN_OFFSET]);

	/* Отправляем пакет */ 
	udp_send(dest_addr, UNWDS_UDP_SERVER_PORT, udp_buffer, (HEADER_UP_LENGTH + JOIN_STAGE_2_PAYLOAD_LENGTH));
	packet_counter_root.u16++;	/* Инкрементируем счетчик пакетов */ 
}

/* Обработчик третий стадии авторизации */
static void join_stage_3_handler(ipv6_addr_t *dest_addr, 
								uint8_t *data)
{	
	int16_t device = find_device_security_table(dest_addr);
	if(device < 0)
		return;

	/* Получаем nonce */
	u8_u16_t nonce;
	nonce.u16 = security_table[device].nonce.u16; 

	/* Расшифровываем данные */
	cipher_decrypt_cbc(&cipher_aes_128, aes_iv, &data[HEADER_DOWN_OFFSET], 16, &data[HEADER_DOWN_OFFSET]);

	/* Вывод принятого пакета микрокомпьютеру */ 
	// print_cr((uip_ip6addr_t*)dest_addr, (uint8_t*)data, (HEADER_LENGTH + JOIN_STAGE_3_LENGTH));
	
	/* Отражаем структуры на массивы */ 
	join_stage_3_t *join_stage_3_pack = (join_stage_3_t*)&data[PAYLOAD_OFFSET];
	
	/* Проверяем одинаковые ли у нас настройки шифрования */ 
	if((nonce.u16 + 1) != join_stage_3_pack->nonce.u16)
	{
	 	/* Вывод сообщения об успешной авторизации */
	 	printf("Authorization error node\n");
	}
	
	/* Если nonce'ы совпадают, то авторизация прошла успешно, шифрование настроенно правильно */ 
	else
	{
		/* Разрешаем обрабатывать пакеты принятые с авторизированного устройства */
		unlock_device_security_table(dest_addr);

		/* TEST */
		ipv6_addr_t addr_dag;
	
		addr_dag.u8[0] = 0xFE;
		addr_dag.u8[1] = 0x80;
		addr_dag.u8[2] = 0x00;
		addr_dag.u8[3] = 0x00;
		addr_dag.u8[4] = 0x00;
		addr_dag.u8[5] = 0x00;
		addr_dag.u8[6] = 0x00;
		addr_dag.u8[7] = 0x00;
		addr_dag.u8[8] = 0x1D;
		addr_dag.u8[9] = 0x9A;
		addr_dag.u8[10] = 0x5E;
		addr_dag.u8[11] = 0x49;
		addr_dag.u8[12] = 0x41;
		addr_dag.u8[13] = 0x5B;
		addr_dag.u8[14] = 0x29;
		addr_dag.u8[15] = 0xA4;
		
		unwds_pack_sender ( &addr_dag, 
							UNWDS_LIGHT_MODULE_ID, 
							LIT_MEASURE, 
							LIT_MEASURE_LENGTH, 
							NULL);
		/* END TEST */

		/* Вывод сообщения об успешной авторизации */
		printf("Authorized node\n");		
	}
}

int root_node_init(void)
{
	int err;

	nrf_temp_init();
	hwrng_init();

	err = cipher_init(&cipher_aes_128, CIPHER_AES_128, aes_key, AES_KEY_LEN);
	if(!(err))
	{
		printf("Error cipher_init(): %i\n", err);
		return -1;
	}

	packet_counter_root.u16 = 0x0000;

	memset(security_table, 0xFF, sizeof(security_table));

	err = unwds_udp_server_init();
	if(err < 0)
	{
		puts("Error init unwds udp server.");	
		return -1;
	}
	
	puts("Init unwds udp server.");	
	start_unwds_udp_server();

	uint16_t flags = GNRC_NETIF_IPV6_ADDRS_FLAGS_STATE_VALID | (64 << 8);
	ipv6_addr_t addr;
	uint8_t instance_id = 1;
	
	addr.u8[0] = 0x20;
	addr.u8[1] = 0x01;
	addr.u8[2] = 0x0D;
	addr.u8[3] = 0xB8;
	addr.u8[4] = 0x00;
	addr.u8[5] = 0x00;
	addr.u8[6] = 0x00;
	addr.u8[7] = 0x00;
	addr.u8[8] = 0x00;
	addr.u8[9] = 0x00;
	addr.u8[10] = 0x00;
	addr.u8[11] = 0x00;
	addr.u8[12] = 0x00;
	addr.u8[13] = 0x00;
	addr.u8[14] = 0x00;
	addr.u8[15] = 0x01;

	// TEST
	// uint16_t crc16;
	// crc16 = ucrc16_calc_be((uint8_t*)&addr, 16, UCRC16_CCITT_POLY_BE, 0xFFFF);
	// printf("CRC16: 0x%08x\n", crc16);
	// END TEST

	// ifconfig 7 add 2001:db8::1  
	printf("Number of network interfaces: %i\n", gnrc_netif_numof()); 
	gnrc_netif_t *netif = gnrc_netif_iter(NULL);
	if(netif == NULL)
	{
		puts("Error. No interface");
		return -1;
	}

	if(gnrc_netapi_set(netif->pid, NETOPT_IPV6_ADDR, flags, &addr, sizeof(addr)) < 0) 
	{
		puts("Error: unable to add IPv6 address");
		return -1;
	}
	printf("Success: added root IPv6 address to interface %i\n", netif->pid);
	
	// rpl root 1 2001:db8::1
	gnrc_rpl_instance_t *inst = gnrc_rpl_root_init(instance_id, &addr, false, false);
    if (inst == NULL) {
        printf("Error: could not add DODAG to instance %i\n", instance_id);
        return -1;
    }

    puts("Successfully added a new RPL DODAG");

	return 0;
}

static int16_t add_device_security_table(ipv6_addr_t *addr, uint16_t nonce)
{
	int16_t device;

	for(uint8_t i = 0; i < MAX_NUM_DEVICES; i++)
	{
		device = find_device_security_table(addr);

		if(device >= 0)
		{
			security_table[device].counter.u16 = 0xFFFF;
			security_table[device].nonce.u16 = nonce;
			return device;
		}
	}

	device = find_empty_elements();
	if(device >= 0)
	{
		memcpy(&(security_table[device].addr), addr, sizeof(ipv6_addr_t));
		security_table[device].counter.u16 = 0xFFFF;
		security_table[device].nonce.u16 = nonce;
		return device;
	}

	return -1;
}

static int16_t unlock_device_security_table(ipv6_addr_t *addr)
{
	int16_t device = find_device_security_table(addr);

	security_table[device].counter.u16 = 0x0000;
	return device;
}

static int16_t find_empty_elements(void)
{
	security_table_t empty_security_table;
	memset(&empty_security_table, 0xFF, sizeof(security_table_t));

	for(uint8_t device = 0; device < MAX_NUM_DEVICES; device++)
	{
		if(memcmp(&(security_table[device]), &empty_security_table, sizeof(ipv6_addr_t)) == 0)
			return device;
	}

	return -1;
}

static int16_t find_device_security_table(ipv6_addr_t *addr)
{
	for(uint8_t device = 0; device < MAX_NUM_DEVICES; device++)
	{
		if(memcmp(&(security_table[device].addr), addr, sizeof(ipv6_addr_t)) == 0)
			return device;
	}

	return -1;
}

/* Обработчик нажатой кнопки */
static void button_status_root_handler (ipv6_addr_t *src_addr, 
										button_status_t *button_status_pack)
{
	(void)src_addr;
	
	bool long_click = ((button_status_pack->button_status & LONG_CLICK) >> 7);
	uint8_t dio = button_status_pack->button_status & DIO_MASK;

	/* Вывод сообщения */	
	printf("Button %i %s\n", dio, long_click ? "long_click" : "click");
}

/* Обработчик пакета с измерением освещенности */
static void lit_measure_status_root_handler(ipv6_addr_t *src_addr, 
											lit_measure_status_t *lit_measure_status_pack)
{
	(void)src_addr;
	
	/* Вывод сообщения */
	printf("Luminocity: %lu lux\n", lit_measure_status_pack->lit_measure_status);
}

