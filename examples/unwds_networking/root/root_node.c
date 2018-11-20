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

#include "udp.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/netif.h"

#define ENABLE_DEBUG            (1)
#include "debug.h"
#include "od.h"

void unwds_root_server(gnrc_pktsnip_t *pkt)
{
#if ENABLE_DEBUG
	DEBUG("UNWDS_UDP: data received:\n");
	od_hex_dump(pkt->data, pkt->size, OD_WIDTH_DEFAULT);
#endif /* ENABLE_DEBUG */

	/*Отражаем структуру на массив*/ 
	header_t *header_pack = pkt->data; //(header_t*)&data[HEADER_OFFSET];
	
	switch(header_pack->protocol_version) 
	{
        case UDBP_PROTOCOL_VERSION: 
			//
			/*Получаем nonce*/
					
			/*CRC16 проверка*/ 
			
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
							break;
						default:
							printf("Unknown command for UMDK-4BTN!\n");
							break;
					} /* header_pack->data_type */
					break;
				case LIT_MEASURE_STATUS:
					switch(header_pack->data_type)
					{
						case LIT_MEASURE_STATUS:
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

    gnrc_pktbuf_release(pkt);
    return;
}
