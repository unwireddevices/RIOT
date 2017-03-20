/*
 * Copyright (C) 2017 Unwired Devices [info@unwds.com]
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
 * @file	umdk-electro.c
 * @brief       umdk-electro module implementation
 * @author      Mikhail Perkov
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <limits.h>

#include "periph/gpio.h"
#include "periph/uart.h"
#include "lpm.h"

#include "board.h"

#include "unwds-common.h"
#include "include/umdk-electro.h"

#include "thread.h"
#include "xtimer.h"
#include "rtctimers.h"

static const uint16_t Crc16Table_mercury[256] = {
     0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
     0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
     0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
     0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
     0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
     0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
     0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
     0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
     0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
     0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
     0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
     0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
     0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
     0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
     0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
     0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
     0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
     0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
     0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
     0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
     0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
     0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
     0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
     0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
     0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
     0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
     0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
     0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
     0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
     0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
     0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
     0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040
};

static kernel_pid_t radio_pid;
static uwnds_cb_t *callback;

static msg_t radio_msg;

static umdk_electro_pack_t umdk_electro_pack;

static volatile uint8_t num_bytes_rx = 0;
static volatile uint8_t length_rx;
static volatile uint8_t length_rx_data;
static volatile uint8_t len_tx = 0;

static flag_decode_t flag_decode = MERCURY_DECODE_DEFAULT;

static uint8_t flag_reply = FLAG_ALLOW_REPLY;
static bool pack_is_true = false;
static bool need_check_addr = true;

static const umdk_electro_cmd_mercury_t cmd_mercury[UMDK_ELECTRO_NUM_CMD_MERCURY] = {
    {
      .cmd = 0x53, .length_tx_data = 0, .length_rx_data = 4,
    },
    {
      .cmd = 0x2F, .length_tx_data = 0, .length_rx_data = 4,
    },
    {
      .cmd = 0x00, .length_tx_data = 4, .length_rx_data = 0,
    },
    {
      .cmd = 0x60, .length_tx_data = 0, .length_rx_data = 1,
    },
    {
      .cmd = 0x61, .length_tx_data = 0, .length_rx_data = 7,
    },
    {
      .cmd = 0x62, .length_tx_data = 0, .length_rx_data = 7,
    },
    {
      .cmd = 0x63, .length_tx_data = 0, .length_rx_data = 7,
    },
    {
      .cmd = 0x21, .length_tx_data = 0, .length_rx_data = 7,
    },
    {
      .cmd = 0x22, .length_tx_data = 0, .length_rx_data = 2,
    },
    {
      .cmd = 0x26, .length_tx_data = 0, .length_rx_data = 2,
    },
    {
      .cmd = 0x27, .length_tx_data = 0, .length_rx_data = 16,
    },
    {
      .cmd = 0x2B, .length_tx_data = 0, .length_rx_data = 7,
    },
    {
      .cmd = 0x2C, .length_tx_data = 0, .length_rx_data = 7,
    },
    {
      .cmd = 0x30, .length_tx_data = 1, .length_rx_data = 16,
    },
    {
      .cmd = 0x31, .length_tx_data = 1, .length_rx_data = 16,
    },
    {
      .cmd = 0x32, .length_tx_data = 1, .length_rx_data = 16,
    },
    {
      .cmd = 0x2E, .length_tx_data = 0, .length_rx_data = 1,
    },
    {
      .cmd = 0x08, .length_tx_data = 1, .length_rx_data = 0,
    },
    {
      .cmd = 0x0A, .length_tx_data = 1, .length_rx_data = 0,
    },
    {
      .cmd = 0x0B, .length_tx_data = 1, .length_rx_data = 0,
    },
    {
      .cmd = 0x10, .length_tx_data = 17, .length_rx_data = 0,
    },
    {
      .cmd = 0x11, .length_tx_data = 17, .length_rx_data = 0,
    },
    {
      .cmd = 0x69, .length_tx_data = 0, .length_rx_data = 6,
    },
};

static uint8_t rxbuf[UMDK_ELECTRO_BUF_SIZE] = {};
static uint8_t txbuf[UMDK_ELECTRO_BUF_SIZE] = {};

static uint32_t rx_addr;

static int baudrates[UMDK_ELECTRO_NUM_BAUDRATES] = {
	      1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800
};

static umdk_electro_config_t umdk_electro_config = { 0, UMDK_ELECTRO_DEV, UMDK_ELECTRO_BAUDRATE_NO, UMDK_MERCURY_ADDR_DEF };

static uint8_t len_addr_cmd = sizeof(umdk_electro_pack.addr) + sizeof(umdk_electro_pack.cmd);

static uint8_t data_in_radio[MAX_BYTES_IM_RADIO] = {0};


static uint16_t crc16(uint8_t * ptrBlock, uint8_t length, const uint16_t * Crc16Table, uint16_t crc_init )
{
    uint16_t crc = crc_init;
    uint8_t i = 0;

    while (i < length) {
      crc = (crc >> 8) ^ (*(Crc16Table + ((crc & 0xFF) ^ (*ptrBlock))));
      ptrBlock++;
      i++;
    }

    return crc;
}

static void send_pack(uint8_t length)
{
  /* Send data */
   gpio_set(RS_485_RE_PIN);
   gpio_set(RS_485_DE_PIN);

   printf("Sending pack...  ");

   uart_write(UMDK_ELECTRO_DEV, txbuf, length);

   uint32_t delay1 = xtimer_now_usec();
   uint32_t delay2 = xtimer_now_usec();

   uint16_t delta = delay2 - delay1;

   while(delta < PACK_DELAY_USEC) {
       delay2 = xtimer_now_usec();
       delta = delay2 - delay1;
   }

   gpio_clear(RS_485_RE_PIN);
   gpio_clear(RS_485_DE_PIN);

   printf("End of sending\n");
}

static bool check_pack(void)
{
  rx_addr = (rxbuf[0] << 0) + (rxbuf[1] << 8) + (rxbuf[2] << 16) + (rxbuf[3] << 24);

  uint8_t cmd  = rxbuf[4];

  printf("RXbuf before checking:\n");
  for(int i = 0; i < length_rx; i++) {
      printf(" %X ", rxbuf[i]);
  }
printf("\n");


  if(need_check_addr) {
    if(rx_addr != umdk_electro_pack.addr) {
	printf("[umdk-electro]: Error -> Wrong received current address  %lX\n", rx_addr);
	return false;
    }
  }

  if(cmd != umdk_electro_pack.cmd) {
      printf("[umdk-electro]: Error -> Wrong received current command   %X\n", cmd);
      return false;
  }

  uint16_t crc = (rxbuf[length_rx - 2] << 0) +  (rxbuf[length_rx - 1] << 8);		// TODO
  uint16_t crc_rx = crc16(rxbuf, length_rx - 2, Crc16Table_mercury, MERCURY_CRC16_INIT);

  if(crc != crc_rx) {
      printf("[umdk-electro]: Error -> Wrong received CRC\n");
      return false;
  }

  need_check_addr = true;

  return true;

}

static uint8_t umdk_mercury_decoding(uint8_t * buf_ptr, flag_decode_t flag)
{
  uint8_t num_bytes = 0;

  switch(flag) {
      case MERCURY_DECODE_DEFAULT: {

	memcpy(data_in_radio, buf_ptr,  length_rx_data);
	num_bytes = length_rx_data;

	break;
      }
      case MERCURY_DECODE_VALUE: {

	uint32_t data_tmp;
	uint8_t *data_ptr;
	uint32_t value_count[5] = {0};

	for(int i = 0; i < 4; i++) {

	    value_count[i] = ((*(buf_ptr + 4*i + 0)) >> 4) * 10000000;
	    value_count[i] += ((*(buf_ptr + 4*i + 0)) & 0x0F) * 1000000;

	    value_count[i] += ((*(buf_ptr + 4*i + 1)) >> 4) * 100000;
	    value_count[i] += ((*(buf_ptr + 4*i + 1)) & 0x0F) * 10000;

	    value_count[i] += ((*(buf_ptr + 4*i + 2)) >> 4) * 1000;
	    value_count[i] += ((*(buf_ptr + 4*i + 2)) & 0x0F) * 100;

	    value_count[i] += ((*(buf_ptr + 4*i + 3)) >> 4) * 10;
	    value_count[i] += ((*(buf_ptr + 4*i + 3)) & 0x0F) * 1;

	    value_count[4] += value_count[i];
	}

	for(int i = 0; i < 5; i++) {
	    data_tmp = value_count[i];
	    data_ptr = (uint8_t *)(&data_tmp);
	    value_count[i] = (uint32_t)( (*(data_ptr + 0) << 24) + (*(data_ptr + 1) << 16) + (*(data_ptr + 2) << 8) + (*(data_ptr + 3) << 0) );
	}

	memcpy(data_in_radio, (uint8_t*)value_count,  sizeof(value_count));
	num_bytes = sizeof(value_count);

	break;
      }
      case MERCURY_DECODE_TIMEDATE: {

	uint8_t timedate[7] = {0};

	printf("Time date:  ");
	for(int i = 0; i < length_rx_data; i++) {
	  timedate[i] = ((*(buf_ptr + i)) >> 4) * 10;
	  timedate[i] += ((*(buf_ptr + i)) & 0x0F) * 1;
	  printf("%d ", timedate[i]);
	}
	printf("\n");
	memcpy(data_in_radio, (uint8_t *)(timedate),  sizeof(timedate));
	num_bytes = sizeof(timedate);

	break;
      }
      case MERCURY_DECODE_SCHEDULE: {

	uint8_t schedule[24] = {0};
	uint8_t cnt = length_rx_data / 2;

	for(int i = 0; i < cnt; i++) {
	    if(*(buf_ptr + 2*i) != 0xFF ) {
		num_bytes += 3;
	      schedule[3*i] = (*(buf_ptr + 2*i) >> 6);
	      printf("Tariff: %d  ", schedule[3*i]);
	      schedule[3*i + 1] = ((*((buf_ptr + 2*i)) >> 4) & 0x3) * 10;
	      schedule[3*i + 1] += ((*(buf_ptr + 2*i)) & 0x0F) * 1;
	      printf("Time: %d: ", schedule[3*i + 1]);
	      schedule[3*i + 2] = ((*(buf_ptr + 2*i + 1)) >> 4) * 10;
	      schedule[3*i + 2] += ((*(buf_ptr + 2*i + 1)) & 0x0F) * 1;
	      printf("%d\n", schedule[3*i + 2]);
	    }
	}

	  memcpy(data_in_radio, (uint8_t *)(schedule),  num_bytes);

	break;
      }
      case MERCURY_DECODE_UIP: {

	uint8_t uip[7] = {0};
	uint8_t* tmp;
	uint16_t voltage = 0;
	uint16_t current = 0;
	uint32_t power = 0;

	voltage = ((*(buf_ptr + 0)) >> 4) * 1000;
	voltage += ((*(buf_ptr + 0)) & 0x0F) * 100;
	voltage += ((*(buf_ptr + 1)) >> 4) * 10;
	voltage += ((*(buf_ptr + 1)) & 0x0F) * 1;
	tmp = (uint8_t*)(&voltage);
	uip[0] = *(tmp + 1);
	uip[1] = *(tmp + 0);

	current = ((*(buf_ptr + 2)) >> 4) * 1000;
	current += ((*(buf_ptr + 2)) & 0x0F) * 100;
	current += ((*(buf_ptr + 3)) >> 4) * 10;
	current += ((*(buf_ptr + 3)) & 0x0F) * 1;
	tmp = (uint8_t*)(&current);
	uip[2] = *(tmp + 1);
	uip[3] = *(tmp + 0);

	power = ((*(buf_ptr + 4)) >> 4) * 100000;
	power += ((*(buf_ptr + 4)) & 0x0F) * 10000;
	power += ((*(buf_ptr + 5)) >> 4) * 1000;
	power += ((*(buf_ptr + 5)) & 0x0F) * 100;
	power += ((*(buf_ptr + 6)) >> 4) * 10;
	power += ((*(buf_ptr + 6)) & 0x0F) * 1;
	power = power & 0x00FFFFFF;
	tmp = (uint8_t*)(&power);
	uip[4] = *(tmp + 2);
	uip[5] = *(tmp + 1);
	uip[6] = *(tmp + 0);

	  memcpy(data_in_radio, uip,  sizeof(uip));
	  num_bytes = sizeof(uip);

	printf("Voltage ->  %d,%d V\n",voltage/10, voltage%10 );
	printf("Current ->  %d,%d A\n",current/10, current%10 );
	printf("Power ->  %ld,%ld W\n",power/10, power%10 );

	break;
      }
      case MERCURY_DECODE_WORKING_TIME: {

	uint8_t time_load[6] = {0};
	uint32_t tl, tlb;
	uint8_t* tmp;

	tl = ((*(buf_ptr + 0)) >> 4) * 100000;
	tl += ((*(buf_ptr + 0)) & 0x0F) * 10000;
	tl += ((*(buf_ptr + 1)) >> 4) * 1000;
	tl += ((*(buf_ptr + 1)) & 0x0F) * 100;
	tl += ((*(buf_ptr + 2)) >> 4) * 10;
	tl += ((*(buf_ptr + 2)) & 0x0F) * 1;
	tl = tl & 0x00FFFFFF;
	tmp = (uint8_t*)(&tl);
	time_load[0] = *(tmp + 2);
	time_load[1] = *(tmp + 1);
	time_load[2] = *(tmp + 0);

	tlb = ((*(buf_ptr + 3)) >> 4) * 100000;
	tlb += ((*(buf_ptr + 3)) & 0x0F) * 10000;
	tlb += ((*(buf_ptr + 4)) >> 4) * 1000;
	tlb += ((*(buf_ptr + 4)) & 0x0F) * 100;
	tlb += ((*(buf_ptr + 5)) >> 4) * 10;
	tlb += ((*(buf_ptr + 5)) & 0x0F) * 1;
	tlb = tlb & 0x00FFFFFF;
	tmp = (uint8_t*)(&tlb);
	time_load[3] = *(tmp + 2);
	time_load[4] = *(tmp + 1);
	time_load[5] = *(tmp + 0);

	  memcpy(data_in_radio, time_load,  sizeof(time_load));
	  num_bytes = sizeof(time_load);

	  printf("TL ->  %ld	TLB ->  %ld\n", tl, tlb);

	break;
      }

      default:
	break;
  }

  flag_decode = MERCURY_DECODE_DEFAULT;

  return num_bytes;
}

static void *radio_send(void *arg)
{
    msg_t msg;
    msg_t msg_queue[4];

    msg_init_queue(msg_queue, 4);

    while (1) {
	msg_receive(&msg);

	module_data_t data;

	pack_is_true = check_pack();

	if(pack_is_true) {
	    flag_reply = FLAG_ALLOW_REPLY;
	    if(length_rx_data != 0) {

		data.data[0] = UNWDS_ELECTRO_MODULE_ID;

		uint8_t length_in_radio = umdk_mercury_decoding(rxbuf + len_addr_cmd, flag_decode);

		memcpy(data.data + 1, data_in_radio,  length_in_radio);
		data.length = length_in_radio + 1;

		printf("INTO RADIO(hex):\n");
		for(int i = 0; i < data.length; i++) {
		    printf("%X ", data.data[i]);
		}
		 printf("\n");
	    }
	    else {
	      printf("[umdk-electro]: Ok\n");

	      data.length = 4;
		data.data[0] = UNWDS_ELECTRO_MODULE_ID;
		data.data[1] = 'o';
		data.data[2] = 'k';
		data.data[3] = '\0';
	     }

	    callback(&data);
	}
	else {
	    printf("[umdk-electro]: Error -> Wrong received packet\n");
	   if(flag_reply == FLAG_ALLOW_REPLY) {
	       send_pack(len_tx);
	       flag_reply = FLAG_NOT_ALLOW_REPLY;
	   }
	}
    }
    return NULL;
}


void rx_cb(void *arg, uint8_t data)
{
  (void) arg;

  rxbuf[num_bytes_rx] = data;
  num_bytes_rx++;

  if (num_bytes_rx == length_rx) {

      uint32_t delay1 = xtimer_now_usec();
      uint32_t delay2 = xtimer_now_usec();
      uint16_t delta = delay2 - delay1;

      while(delta < PACK_DELAY_USEC) {
	  delay2 = xtimer_now_usec();
	  delta = delay2 - delay1;
      }

      num_bytes_rx = 0;

    msg_send(&radio_msg, radio_pid);
    return;
  }
}

static inline void reset_config(void) {
    umdk_electro_config.is_valid = 0;
    umdk_electro_config.current_baudrate_idx = UMDK_ELECTRO_BAUDRATE_NO;
    umdk_electro_config.uart_dev = UMDK_ELECTRO_DEV;
    umdk_electro_config.addr = UMDK_MERCURY_ADDR_DEF;
}

static void init_config(void) {
    reset_config();

    if (!unwds_read_nvram_config(UNWDS_ELECTRO_MODULE_ID, (uint8_t *) &umdk_electro_config, sizeof(umdk_electro_config)))
      return;

    if ((umdk_electro_config.is_valid == 0xFF) || (umdk_electro_config.is_valid == 0))  {
      reset_config();
      return;
    }

    if (umdk_electro_config.current_baudrate_idx >= UMDK_ELECTRO_NUM_BAUDRATES) {
      reset_config();
      return;
    }

    if (umdk_electro_config.uart_dev >= UART_NUMOF) {
      reset_config();
      return;
    }
}

static inline void save_config(void) {
      umdk_electro_config.is_valid = 1;
      unwds_write_nvram_config(UNWDS_ELECTRO_MODULE_ID, (uint8_t *) &umdk_electro_config, sizeof(umdk_electro_config));
}

void umdk_electro_init(uint32_t *non_gpio_pin_map, uwnds_cb_t *event_callback)
{
    (void) non_gpio_pin_map;

    callback = event_callback;

    init_config();

    uint8_t* ptr_addr_cfg = (uint8_t *)(&umdk_electro_config.addr);
    uint32_t addr_cfg = ( ((*(ptr_addr_cfg + 0) << 24) ) + ((*(ptr_addr_cfg + 1) << 16)) + ((*(ptr_addr_cfg + 2) << 8)) + ((*(ptr_addr_cfg + 3) << 0)) );

    printf("[umdk-electro]: Baudrate: %d\n", baudrates[umdk_electro_config.current_baudrate_idx]);
    printf("[umdk-electro]: Address(hex) ->  %lX   Address(dec) ->  %ld \n", addr_cfg,  addr_cfg);

    /* Initialize the UART */
    uart_init(UART_DEV(umdk_electro_config.uart_dev), baudrates[umdk_electro_config.current_baudrate_idx], rx_cb, NULL);

    /* Initialize DE/RE pins */
    gpio_init(RS_485_DE_PIN, GPIO_OUT);
    gpio_init(RS_485_RE_PIN, GPIO_OUT);

    gpio_clear(RS_485_DE_PIN);
    gpio_clear(RS_485_RE_PIN);

    /* Create handler thread */
    char *stack = (char *) allocate_stack();
    if (!stack) {
	puts("umdk-electro: unable to allocate memory. Is too many modules enabled?");
	return;
    }

    radio_pid = thread_create(stack, UNWDS_STACK_SIZE_BYTES, THREAD_PRIORITY_MAIN - 1, THREAD_CREATE_STACKTEST, radio_send, NULL, "electro thread");

}


static void transmit_packet(mercury_cmd_t c, module_data_t *cmd)
{
  len_tx = cmd_mercury[c].length_tx_data;

  if(cmd->length != (len_tx + 1)) {
      printf("[umdk-electro]: Invalid request -> Wrong length of command\n");
      return;
  }

  length_rx_data = cmd_mercury[c].length_rx_data;
  length_rx = cmd_mercury[c].length_rx_data + len_addr_cmd;
  length_rx += sizeof(umdk_electro_pack.crc);

  if(c == MERCURY_CMD_GET_ADDR) {
      umdk_electro_pack.addr = UMDK_MERCURY_ADDR_DEF;
  }
  else {
      umdk_electro_pack.addr = umdk_electro_config.addr;
  }

  memcpy(txbuf, (uint8_t *)(&umdk_electro_pack.addr), sizeof(umdk_electro_pack.addr));

  umdk_electro_pack.cmd = cmd_mercury[c].cmd;

  memcpy(txbuf + sizeof(umdk_electro_pack.addr), (uint8_t *)(&umdk_electro_pack.cmd), sizeof(umdk_electro_pack.cmd));

  if(cmd_mercury[c].length_tx_data > 0) {
      for(int i = 0; i < len_tx; i++) {
	  umdk_electro_pack.data[i] = cmd->data[i + 1];
      }
      memcpy(txbuf + len_addr_cmd, umdk_electro_pack.data, len_tx);
  }

  len_tx += len_addr_cmd;

  umdk_electro_pack.crc = crc16(txbuf, len_tx, Crc16Table_mercury, MERCURY_CRC16_INIT);

  memcpy(txbuf + len_tx, (uint8_t *)(&umdk_electro_pack.crc),  sizeof(umdk_electro_pack.crc));

  len_tx += sizeof(umdk_electro_pack.crc);

  printf("PACK: ");
  for(int i = 0; i < len_tx; i++) {
      printf(" %X ", txbuf[i]);
  }
  printf("\n");

  send_pack(len_tx);

  return;
}

static void reply_ok(module_data_t *reply)
{
  reply->length = 4;
  reply->data[0] = UNWDS_ELECTRO_MODULE_ID;
  reply->data[1] = 'o';
  reply->data[2] = 'k';
  reply->data[3] = '\0';
}

bool umdk_electro_cmd(module_data_t *cmd, module_data_t *reply)
{
    if (cmd->length < 1) {
	return false;
    }

    mercury_cmd_t c = cmd->data[0];

    switch (c) {
      case MERCURY_CMD_INIT_ADDR: {
	memcpy((uint8_t *)(&umdk_electro_config.addr), (uint8_t *)(&cmd->data[1]), sizeof(umdk_electro_config.addr));
	save_config();

	printf("New config ADDR ->  %lX\n",  umdk_electro_config.addr);

	reply_ok(reply);
	  return true; /* Allow reply */
	break;
      }

      case MERCURY_CMD_GET_ADDR: {

	need_check_addr = false;
	transmit_packet(c, cmd);
	      need_check_addr = true;
	  return false; /* Don't reply */
	  break;
      }

	case MERCURY_CMD_GET_SERIAL: {
	  transmit_packet(c, cmd);

	    return false; /* Don't reply */
	}

	case MERCURY_CMD_SET_NEW_ADDR: {
	  need_check_addr = false;
	  transmit_packet(c, cmd);
	  transmit_packet(c, cmd);

	  if(pack_is_true) {
	    memcpy((uint8_t *)(&umdk_electro_config.addr), (uint8_t *)(txbuf + len_addr_cmd), sizeof(umdk_electro_config.addr));
	    printf("New ADDR of device ->  %lX\n",  umdk_electro_config.addr);
	    save_config();
	    pack_is_true = false;
	  }

	  need_check_addr = true;
	  reply_ok(reply);
	    return true; /* Allow reply */

	    break;
	}
	case MERCURY_CMD_GET_CURR_TARIFF: {

	  transmit_packet(c, cmd);

	    return false; /* Don't reply */

	    break;
	}
	case MERCURY_CMD_GET_LAST_OPEN: {

	  flag_decode = MERCURY_DECODE_TIMEDATE;
	  transmit_packet(c, cmd);

	    return false; /* Don't reply */

	    break;
	}
	case MERCURY_CMD_GET_LAST_CLOSE: {

	  flag_decode = MERCURY_DECODE_TIMEDATE;
	  transmit_packet(c, cmd);

	    return false; /* Don't reply */

	    break;
	 }
	case MERCURY_CMD_GET_U_I_P: {

	  flag_decode = MERCURY_DECODE_UIP;
	  transmit_packet(c, cmd);

	     return false; /* Don't reply */

	     break;
	  }
	case MERCURY_CMD_GET_TIMEDATE: {

	  flag_decode = MERCURY_DECODE_TIMEDATE;
	  transmit_packet(c, cmd);

	     return false; /* Don't reply */

	     break;
	  }
	case MERCURY_CMD_GET_LIMIT_POWER: {

	  transmit_packet(c, cmd);

	     return false; /* Don't reply */

	     break;
	  }
	case MERCURY_CMD_GET_CURR_POWER_LOAD: {

	  transmit_packet(c, cmd);

	     return false; /* Don't reply */

	     break;
	  }
	case MERCURY_CMD_GET_TOTAL_VALUE: {

	  flag_decode = MERCURY_DECODE_VALUE;
	  transmit_packet(c, cmd);

	     return false; /* Don't reply */

	     break;
	  }
	case MERCURY_CMD_GET_LAST_POWER_OFF: {

	  flag_decode = MERCURY_DECODE_TIMEDATE;
	  transmit_packet(c, cmd);

	     return false; /* Don't reply */

	     break;
	  }
	case MERCURY_CMD_GET_LAST_POWER_ON: {

	  flag_decode = MERCURY_DECODE_TIMEDATE;
	  transmit_packet(c, cmd);

	     return false; /* Don't reply */

	     break;
	  }
	case MERCURY_CMD_GET_HOLIDAYS: {

	  transmit_packet(c, cmd);

	     return false; /* Don't reply */

	     break;
	  }
	case MERCURY_CMD_GET_SCHEDULE: {

	  flag_decode = MERCURY_DECODE_SCHEDULE;
	  transmit_packet(c, cmd);

	     return false; /* Don't reply */

	     break;
	  }
	case MERCURY_CMD_GET_VALUE: {

	  flag_decode = MERCURY_DECODE_VALUE;
	  transmit_packet(c, cmd);

	     return false; /* Don't reply */

	     break;
	  }
	case MERCURY_CMD_GET_NUM_TARIFFS: {

	  transmit_packet(c, cmd);

	     return false; /* Don't reply */

	     break;
	  }
	case MERCURY_CMD_SET_SPEED: {

	  transmit_packet(c, cmd);

	     return false; /* Don't reply */

	     break;
	  }
	case MERCURY_CMD_SET_NUM_TARIFFS: {

	  transmit_packet(c, cmd);
	  reply_ok(reply);
	    return true; /* Allow reply */

	     break;
	  }
	case MERCURY_CMD_SET_TARIFF: {

	  transmit_packet(c, cmd);
	  reply_ok(reply);
	    return true; /* Allow reply */

	     break;
	  }
	case MERCURY_CMD_SET_HOLIDAYS: {

	  transmit_packet(c, cmd);
	  reply_ok(reply);
	    return true; /* Allow reply */

	      break;
	   }
	case MERCURY_CMD_SET_SCHEDULE: {

	  transmit_packet(c, cmd);
	  reply_ok(reply);
	    return true; /* Allow reply */

	      break;
	   }
	case MERCURY_CMD_GET_WORKING_TIME: {

	  flag_decode = MERCURY_DECODE_WORKING_TIME;
	  transmit_packet(c, cmd);

	      return false; /* Don't reply */

	      break;
	   }

	default:
	    break;
    }

    /* Don't reply by default */
    return false;
}

#ifdef __cplusplus
}
#endif
