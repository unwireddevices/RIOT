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
#include "crypto/ciphers.h"
#include "crypto/modes/cbc.h"

#define ENABLE_DEBUG            (1)
#include "debug.h"
#include "od.h"

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

cipher_t cipher_aes_128;

/*Первая стадия авторизации*/
static void join_stage_1_sender(void);

// ipv6_addr_t root_addr.u8[] = {0x20, 0x01, 0x0D, 0xB8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

// root_addr.u8[0] = 0x20;
// root_addr.u8[1] = 0x01;
// root_addr.u8[2] = 0x0D;
// root_addr.u8[3] = 0xB8;
// root_addr.u8[4] = 0x00;
// root_addr.u8[5] = 0x00;
// root_addr.u8[6] = 0x00;
// root_addr.u8[7] = 0x00;
// root_addr.u8[8] = 0x00;
// root_addr.u8[9] = 0x00;
// root_addr.u8[10] = 0x00;
// root_addr.u8[11] = 0x00;
// root_addr.u8[12] = 0x00;
// root_addr.u8[13] = 0x00;
// root_addr.u8[14] = 0x00;
// root_addr.u8[15] = 0x01;

void unwds_dag_server(gnrc_pktsnip_t *pkt)
{
#if ENABLE_DEBUG
	DEBUG("UNWDS_UDP: data received:\n");
	od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
#endif /* ENABLE_DEBUG */

	/* LED ON */ 
	
	printf("[DAG Node] ");

	/* Отражаем структуру на массив */ 
	header_t *header_pack = pkt->data; //(header_t*)&data[HEADER_OFFSET];
	
	switch(header_pack->protocol_version) 
	{
        case UDBP_PROTOCOL_VERSION: 
			//
			/* Получаем nonce */
			
			/* Защита от атаки повтором */
			/* Проверяем счетчик пакетов на валидность данного пакета */
			
			/* Вывод принятого пакета микрокомпьютеру */ 
			//
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
						case JOIN_STAGE_2:
							break;
						case JOIN_STAGE_4:
							break;
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

/*Конструктор пакета*/
void unwds_pack_sender( uint8_t device_id, 
						uint8_t data_type, 
						uint8_t payload_len, 
						uint8_t *payload)
{	
	/*Проверка на то что передан не нулевой адрес буфера*/
	if ((payload == NULL) && (payload_len != 0))
		return;
	
	/*Выделяем память под пакет. Общий размер пакета (header + payload)*/
	uint8_t udp_buffer[HEADER_LENGTH + payload_len];
	
	/*Отражаем структуры на массивы*/ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	
	/*Заполняем пакет*/  
	/*Header*/ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/*Текущая версия протокола*/ 
	header_pack->device_id = device_id;							/*ID устройства*/
	header_pack->data_type = data_type;							/*Тип пакета*/  
	// header_pack->rssi = get_parent_rssi();						/*RSSI*/ 
	header_pack->temperature = (int8_t)(nrf_temp_read() >> 2);	/*Температура*/ 
	// header_pack->voltage = get_voltage();						/*Напряжение*/ 
	// header_pack->counter.u16 = packet_counter_node.u16;			/*Счетчик пакетов*/ 
	header_pack->length = payload_len;							/*Размер пакета (незашифрованного)*/
	
	/*Payload*/ 	
	/*Заполняем пакет, зашифровываем и отправляем его DAG'у. */ 
	memcpy(&udp_buffer[PAYLOAD_OFFSET], payload, payload_len);
	
#if ENABLE_DEBUG
	char root_addr_str[IPV6_ADDR_MAX_STR_LEN];
	ipv6_addr_to_str(root_addr_str, &root_addr, sizeof(root_addr_str));
	
	DEBUG("[UNWDS_UDP] Success: sent %u byte(s) to [%s]:%u\n", (HEADER_LENGTH + payload_len), root_addr_str, UNWDS_UDP_SERVER_PORT);
	od_hex_dump(udp_buffer, (HEADER_LENGTH + payload_len), OD_WIDTH_DEFAULT);
#endif /* ENABLE_DEBUG */
	
	/*Отправляем пакет*/ 
	udp_send(&root_addr, UNWDS_UDP_SERVER_PORT, udp_buffer, (HEADER_LENGTH + payload_len));
	/*Инкрементируем счетчик пакетов*/
}

/*Первая стадия авторизации*/
/*Передаём свой серийный номер*/
static void join_stage_1_sender(void)
{
	/*Проверка на то что передан существующий адрес*/
	// if (addr == NULL)
	// 	return;

	// uip_ipaddr_t addr;						/*Выделяем память для адреса на который отправится пакет*/
	// uip_ip6addr_copy(&addr, root_addr);		/*Копируем адрес*/
	
	/*Выделяем память под пакет. Общий размер пакета (header + payload)*/	
	uint8_t udp_buffer[HEADER_LENGTH + JOIN_STAGE_1_PAYLOAD_LENGTH];
	
	/*Отражаем структуры на массив*/ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	join_stage_1_t *join_stage_1_pack = (join_stage_1_t*)&udp_buffer[PAYLOAD_OFFSET];
	
	/*Заполняем пакет*/  
	/*Header*/ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/*Текущая версия протокола*/ 
	header_pack->device_id = UNWDS_6LOWPAN_SYSTEM_MODULE_ID;	/*ID устройства*/
	header_pack->data_type = JOIN_STAGE_1;						/*Тип пакета*/  
	// header_pack->rssi = get_parent_rssi();					/*RSSI*/ 
	header_pack->temperature = (int8_t)(nrf_temp_read() >> 2);	/*Температура*/ 
	// header_pack->voltage = get_voltage();					/*Напряжение*/ 
	// header_pack->counter.u16 = packet_counter_node.u16;			/*Счетчик пакетов*/ 
	header_pack->length = JOIN_STAGE_1_LENGTH;					/*Размер пакета*/

	/*Payload*/
	join_stage_1_pack->module_id = 0;//UNWDS_MODULE_ID;
	
	/*CRC16*/ 
	header_pack->crc.u16 = 0;
	

	/*Вывод информационного сообщения в консоль*/
	printf("[DAG Node] Send join packet to DAG-root node: ");

	char root_addr_str[IPV6_ADDR_MAX_STR_LEN];
	printf("%s \n", ipv6_addr_to_str(root_addr_str, &root_addr, sizeof(root_addr_str)));

#if ENABLE_DEBUG
	DEBUG("[UNWDS_UDP] Success: sent %u byte(s) to [%s]:%u\n", (HEADER_LENGTH + JOIN_STAGE_1_PAYLOAD_LENGTH), root_addr_str, UNWDS_UDP_SERVER_PORT);
	od_hex_dump(udp_buffer, (HEADER_LENGTH + JOIN_STAGE_1_PAYLOAD_LENGTH), OD_WIDTH_DEFAULT);
#endif /* ENABLE_DEBUG */
	
	/*Отправляем пакет*/ 
	udp_send(&root_addr, UNWDS_UDP_SERVER_PORT, udp_buffer, (HEADER_LENGTH + JOIN_STAGE_1_PAYLOAD_LENGTH));
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
/*Функция отправки состояния кнопок*/
void button_status_dag_sender ( uint8_t button_number,
								uint8_t click_type)
{
	/*Заполняем payload*/
	button_status_t button_status_pack;				/*Создаем структуру*/
	
	button_status_pack.button_status = button_number;

	if(click_type == LONG_CLICK)
		button_status_pack.button_status |= LONG_CLICK;
	
	/*Вывод информационного сообщения в консоль*/
	print_send_packet("button status");
	
	/*Отправляем пакет*/
	unwds_pack_sender ( UNWDS_4BTN_MODULE_ID,				/*ID модуля*/
						BUTTON_STATUS, 						/*Команда включения канала ШИМ'а*/
						BUTTON_STATUS_LENGTH, 				/*Размер payload'а*/
						(uint8_t*)&button_status_pack );	/*Payload*/		
}
#endif

#ifdef UMDK_LIT
/*Совершить замер освещенности*/			
bool lit_measure_dag_sender(void)
{
	/*Заполняем payload*/
	lit_measure_status_t lit_measure_status_pack;						/*Создаем структуру*/
	
	lit_measure_status_pack.lit_measure_status = 123;//lit_measure_status;	/*Измеряем освещенность*/
	
	/*Вывод информационного сообщения в консоль*/
	print_send_packet("OPT3001 measure status");
	printf("[UMDK-OPT3001] Luminocity: %lu lux\n", lit_measure_status_pack.lit_measure_status);
	
	/*Отправляем пакет*/	
	unwds_pack_sender ( UNWDS_OPT3001_MODULE_ID,				/*ID модуля*/
						LIT_MEASURE_STATUS, 					/*Команда включения канала ШИМ'а*/
						LIT_MEASURE_STATUS_LENGTH, 				/*Размер payload'а*/
						(uint8_t*)&lit_measure_status_pack);	/*Payload*/		
				
	return lit_measure_status_pack.lit_measure_status;
}
#endif
