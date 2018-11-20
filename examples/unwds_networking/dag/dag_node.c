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

#include "udp.h"
#include "net/gnrc/ipv6.h"
#include "net/gnrc/netif.h"

#define ENABLE_DEBUG            (1)
#include "debug.h"
#include "od.h"

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
				case UNWDS_6FET_MODULE_ID:
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
				case UNWDS_LIT_MODULE_ID:
					switch(header_pack->data_type)
					{
						case LIT_MEASURE:
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
