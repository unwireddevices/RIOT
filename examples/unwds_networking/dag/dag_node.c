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

#define ENABLE_DEBUG            (1)
#include "debug.h"
#include "od.h"

ipv6_addr_t root_addr;

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

	/*LED ON*/ 
	
	/*Отражаем структуру на массив*/ 
	header_t *header_pack = pkt->data; //(header_t*)&data[HEADER_OFFSET];
	
	switch(header_pack->protocol_version) 
	{
        case UDBP_PROTOCOL_VERSION: 
			//
			/*Получаем nonce*/
			
			/*Защита от атаки повтором*/
			/*Проверяем счетчик пакетов на валидность данного пакета*/
			
			/*Вывод принятого пакета микрокомпьютеру*/ 
			//
			switch(header_pack->device_id)
			{
				case UNWDS_6LOWPAN_SYSTEM_MODULE_ID:
					switch(header_pack->data_type)
					{
						case JOIN_STAGE_2:
							break;
						case JOIN_STAGE_4:
							break;
						case PONG:
							break;
						default:
							printf("[DAG Node] Unknown command for system!\n");
							break;
					} /* header_pack->data_type */
					break;
#ifdef UMDK_6FET 
				case UNWDS_PWM_MODULE_ID:
					switch(header_pack->data_type)
					{
						case PWM_SETTINGS:
							break;
						case PWM_POWER:
							break;
						default:
							printf("[DAG Node] Unknown command for UMDK-6FET!\n");
							break;
					} /* header_pack->data_type */
					break;
#endif /* UMDK_6FET */

#ifdef UMDK_LIT
				case UNWDS_OPT3001_MODULE_ID:
					switch(header_pack->data_type)
					{
						case LIT_MEASURE:
							lit_measure_dag_sender();
							break;
						default:
							printf("[DAG Node] Unknown command for UMDK-LIT!\n");
							break;
					} /* header_pack->data_type */
					break;
#endif /* UMDK_LIT */

#ifdef UMDK_GPIO
				case UNWDS_GPIO_MODULE_ID:
					switch(header_pack->data_type)
					{
						case GPIO_CMD:
							break;
						default:
							printf("[DAG Node] Unknown command for UMDK-GPIO!\n");
							break;
					} /* header_pack->data_type */
					break;
#endif /* UMDK_GPIO */

				default:
					printf("[DAG Node] Unknown module!\n");
					break;
			} /* header_pack->device_id */
            break;
        default:
            printf("[DAG Node] Unknown protocol version!\n");
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
	
#if ENABLE_DEBUG
	char root_addr_str[IPV6_ADDR_MAX_STR_LEN];
	ipv6_addr_to_str(root_addr_str, &root_addr, sizeof(root_addr_str));
	
	DEBUG("UNWDS_UDP Success: sent %u byte(s) to [%s]:%u\n", (HEADER_LENGTH + payload_len), root_addr_str, UNWDS_UDP_SERVER_PORT);
	od_hex_dump(udp_buffer, (HEADER_LENGTH + payload_len), OD_WIDTH_DEFAULT);
#endif /* ENABLE_DEBUG */
	
	/*Отправляем пакет*/ 
	udp_send(&root_addr, UNWDS_UDP_SERVER_PORT, udp_buffer, (HEADER_LENGTH + payload_len));
	/*Инкрементируем счетчик пакетов*/
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
	printf("[DAG Node] Send button status packet\n");
	
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
	printf("[DAG Node] Send LIT measure status packet\n");
	printf("[UMDK-LIT] Luminocity: %lu lux\n", lit_measure_status_pack.lit_measure_status);
	
	/*Отправляем пакет*/	
	unwds_pack_sender ( UNWDS_OPT3001_MODULE_ID,				/*ID модуля*/
						LIT_MEASURE_STATUS, 					/*Команда включения канала ШИМ'а*/
						LIT_MEASURE_STATUS_LENGTH, 				/*Размер payload'а*/
						(uint8_t*)&lit_measure_status_pack);	/*Payload*/		
				
	return lit_measure_status_pack.lit_measure_status;
}
#endif
