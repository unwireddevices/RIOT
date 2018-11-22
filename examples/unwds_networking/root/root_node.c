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

#include "unwds_udp.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/netif.h"

#define ENABLE_DEBUG            (1)
#include "debug.h"
#include "od.h"

/*Конструктор пакета*/
void unwds_pack_sender (ipv6_addr_t *dest_addr, 
						uint8_t device_id, 
						uint8_t data_type, 
						uint8_t payload_len, 
						uint8_t *payload);

/*Print IPv6 address*/
static void print_ipv6_addr(ipv6_addr_t *src_addr);

/*Обработчик нажатой кнопки*/
static void button_status_root_handler (ipv6_addr_t *src_addr, 
										button_status_t *button_status_pack);

/*Обработчик пакета с измерением освещенности*/
static void lit_measure_status_root_handler( ipv6_addr_t *src_addr, 
											 lit_measure_status_t *lit_measure_status_pack);
										

void unwds_root_server(gnrc_pktsnip_t *pkt)
{
#if ENABLE_DEBUG
	DEBUG("UNWDS_UDP: data received:\n");
	od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
#endif /* ENABLE_DEBUG */

	/*LED ON*/ 
	
	/*Copy IPv6 address*/ 
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
	
	/*Print IPv6 address*/
	print_ipv6_addr(&src_addr);
	
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
						case JOIN_STAGE_1:
							break;
						case JOIN_STAGE_3:
							break;
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
					break;
				case UNWDS_4BTN_MODULE_ID:
					switch(header_pack->data_type)
					{
						case BUTTON_STATUS:
							button_status_root_handler(&src_addr, (button_status_t*)&((uint8_t*)(pkt->data))[PAYLOAD_OFFSET]);
							break;
						default:
							printf("Unknown command for UMDK-4BTN!\n");
							break;
					} /* header_pack->data_type */
					break;
				case UNWDS_LIT_MODULE_ID:
					switch(header_pack->data_type)
					{
						case LIT_MEASURE_STATUS:
							lit_measure_status_root_handler(&src_addr, (lit_measure_status_t*)&((uint8_t*)(pkt->data))[PAYLOAD_OFFSET]);
							break;
						default:
							printf("Unknown command for UMDK-LIT!\n");
							break;
					} /* header_pack->data_type */
					break;
				case UNWDS_UART_MODULE_ID:
					switch(header_pack->data_type)
					{
						case SEND_BY_UART:
							break;
						default:
							printf("Unknown command for UMDK-UART!\n");
							break;
					} /* header_pack->data_type */
					break;
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
void unwds_pack_sender (ipv6_addr_t *dest_addr, 
						uint8_t device_id, 
						uint8_t data_type, 
						uint8_t payload_len, 
						uint8_t *payload)
{
	/*Проверка на то что передан существующий адрес*/
	if (dest_addr == NULL)
		return;
	
	/*Проверка на то что передан не нулевой адрес буфера*/
	if ((payload == NULL) && (payload_len != 0))
		return;
	
	/*Выделяем память под пакет. Общий размер пакета (header + payload)*/
	uint8_t udp_buffer[HEADER_LENGTH + payload_len];
	
	/*Отражаем структуры на массивы*/ 
	header_t *header_pack = (header_t*)&udp_buffer[HEADER_OFFSET];
	
	/*Получаем nonce*/
	/*Копируем полученный nonce и используем его в качестве сессионного ключа (AES128-CBC)*/
	
	/*Заполняем пакет*/  
	/*Header*/ 
	header_pack->protocol_version = UDBP_PROTOCOL_VERSION; 		/*Текущая версия протокола*/ 
	header_pack->device_id = device_id;							/*ID устройства*/
	header_pack->data_type = data_type;							/*Тип пакета*/  
	// header_pack->rssi = get_parent_rssi();						/*RSSI*/ 
	// header_pack->temperature = get_temperature();				/*Температура*/ 
	// header_pack->voltage = get_voltage();						/*Напряжение*/ 
	// header_pack->counter.u16 = packet_counter_root.u16;			/*Счетчик пакетов*/ 
	header_pack->length = payload_len;							/*Размер пакета (незашифрованного)*/
	
	/*Payload*/ 	
	/*Заполняем пакет, зашифровываем и отправляем его DAG'у. */ 
	memcpy(&udp_buffer[PAYLOAD_OFFSET], payload, payload_len);
	

#if ENABLE_DEBUG
	DEBUG("UNWDS_UDP: data send:\n");
	od_hex_dump(udp_buffer, (HEADER_LENGTH + payload_len), OD_WIDTH_DEFAULT);
#endif /* ENABLE_DEBUG */

	/*Отправляем пакет*/ 
	udp_send(dest_addr, UNWDS_UDP_SERVER_PORT, udp_buffer, (HEADER_LENGTH + payload_len));
	/*Инкрементируем счетчик пакетов*/
}

static void print_ipv6_addr(ipv6_addr_t *src_addr)
{
	char src_addr_str[IPV6_ADDR_MAX_STR_LEN];
	printf("[%s] ", ipv6_addr_to_str(src_addr_str, src_addr, sizeof(src_addr_str)));
}

/*Обработчик нажатой кнопки*/
static void button_status_root_handler (ipv6_addr_t *src_addr, 
										button_status_t *button_status_pack)
{
	(void)src_addr;
	
	bool long_click = ((button_status_pack->button_status & LONG_CLICK) >> 7);
	uint8_t dio = button_status_pack->button_status & DIO_MASK;

	/*Вывод сообщения*/	
	printf("Button %i %s\n", dio, long_click ? "long_click" : "click");
}

/*Обработчик пакета с измерением освещенности*/
static void lit_measure_status_root_handler(ipv6_addr_t *src_addr, 
											lit_measure_status_t *lit_measure_status_pack)
{
	(void)src_addr;
	
	/*Вывод сообщения*/
	printf("Luminocity: %lu lux\n", lit_measure_status_pack->lit_measure_status);
}

