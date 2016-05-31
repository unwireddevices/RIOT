/*
 * Copyright (C) 2016 Cr0s
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     tests
 * @{
 *
 * @file
 * @brief       Test application for SX1276 modem driver
 *
 * @author      Cr0s
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "shell.h"
#include "shell_commands.h"
#include "thread.h"
#include "xtimer.h"

#include "common.h"

#include "board.h"

#include "sx1276_regs_lora.h"
#include "sx1276_regs_fsk.h"


#define _STACKSIZE      (THREAD_STACKSIZE_DEFAULT + THREAD_EXTRA_STACKSIZE_PRINTF)
#define MSG_TYPE_ISR    (0x3456)

static char stack[_STACKSIZE];
static kernel_pid_t _recv_pid;

void print_logo(void) {
	puts("                                                .@                           @  ");
	puts("                                                                             @  ");
	puts("  @@@           %@@,     &@**%@. .#    ./   .#  .@   #@*.   *@@@@@,    @#%.%%@  ");
	puts("  @@@           %@@,    @#    .&  .# ..@.  .#*  .@  /#    .@.    ,%   @.    .@  ");
	puts("  @@@           %@@,    @*    .@  .&,,&  @.#%   .@  %*    .@&&&&&&&*  @      @  ");
	puts("  @@@           %@@,    @*    .@   .@@   .@%    .@  %*    ,@          *@,   ,@, ");
	puts("  @@@           %@@,    *.     *                .#  ,.      %@&&@#     **,*.@   ");
	puts("  @@@           %@@,															  ");
	puts("  @@@   .,,,,,,,...            %@@@%   %     .# *%   .#@@@@,    *@@@@,    @@@*  ");
	puts("  @@@   @@@@@@@@@@@@@@&.     %&     &%  @    @  .@  &*        .@.    ,@  @      ");
	puts("  @@@   @@@     /.. *@@@@.   @&&&&&&&&  ,&  @.  .@  @         %@&&&&&&&*  #@(   ");
	puts("  &@@*  @@@     @@@   (@@@   @#          (@@/   .@  @.        ,@             @  ");
	puts("   @@@. @@@    @@@#    #@@%   .@@&@@.     /*    .@   /@@&@@(    %@&&@#   &@&@*  ");
	puts("    @@% @@@ @@@@@.     .@@@                                                     ");
	puts("        @@@ ####/#####/ @@@ ##################################################  ");
	puts("        @@@            *@@&                                                     ");
	puts("        @@@            @@@,                                                     ");
	puts("        @@@          *@@@#                                                      ");
	puts("        @@@,...,,#&@@@@@                                                        ");
	puts("        @@@@@@@@@@@%,                                                           ");
	puts("                                                                                ");
	puts("                                                                                ");
	puts("                                                                                ");
	puts("");
}


void tx_done(void) {
	//puts("sx1276: tx done");
}

void tx_timeout (void) {
	//puts("sx1276: tx timeout");
}

/**
 * @brief Rx Done callback
 *
 * @param [IN] payload Received buffer pointer
 * @param [IN] size    Received buffer size
 * @param [IN] rssi    RSSI value computed while receiving the frame [dBm]
 * @param [IN] snr     Raw SNR value given by the radio hardware [dB]
 */
void rx_done(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
	if (size > 0) {
		//printf("sx1276: received payload with RSSI %d and size %d (snr: %d)", rssi, size, snr);

		///uint16_t i;
		//for (i = 0; i < size; i++) {
		//	printf("0x%2x ", payload[i]);
		//}
	} else {
		//puts("sx1276: rx done with zero size?");
	}
}

/**
 * @brief  Rx Timeout callback prototype.
 */
void rx_timeout(void) {
	//puts("sx1276: rx timeout");
}

/**
 * @brief Rx Error callback prototype.
 */
void rx_error(void) {
	//puts("sx1276: rx error");
}

void fhss_change_channel(uint8_t current_channel) {
	//printf("sx1276: changed channel %d", current_channel);
}

void cad_done(bool activity_detected) {
	//puts("sx1276: CAD done");
}

int spi_init(void) {
	int res;

	/* Setup SPI config for SX1276 */
    spi_acquire(SX1276_SPI);
    res = spi_init_master(SX1276_SPI, SX1276_SPI_MODE, SX1276_SPI_SPEED);
    spi_release(SX1276_SPI);

    if (res < 0) {
        printf("spi_init_master: error initializing SPI_%i device (code %i)\n",
        		SX1276_SPI, res);
        return 0;
    }

    res = gpio_init(SX1276_SPI_NSS, GPIO_OUT);
    if (res < 0){
        printf("gpio_init: error initializing GPIO_%ld as CS line (code %i)\n",
                (long)SX1276_SPI_NSS, res);
        return 0;
    }

    gpio_set(SX1276_SPI_NSS);

    printf("spi_init: SPI_%i successfully initialized as master, cs: GPIO_%ld, mode: %i, speed: %i\n",
    		SX1276_SPI, (long)SX1276_SPI_NSS, SX1276_SPI_MODE, SX1276_SPI_SPEED);

    return 1;
}

void sx1276_board_set_ant_sw_low_power(uint8_t lp) {
	if (lp) {
		gpio_init(SX1276_ANTSW, GPIO_OD);		/* open-drain output for low power mode */
	} else {
		gpio_init(SX1276_ANTSW, GPIO_OUT);
	}
}

void sx1276_board_set_ant_sw(uint8_t tx) {
	if (tx) {
		gpio_set(SX1276_ANTSW);
	} else {
		gpio_clear(SX1276_ANTSW);
	}
}

void init_configs(void) {
	sx1276_set_rx_config(&sx1276, MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
            LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
            LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
            6, true, 0, 0, LORA_IQ_INVERSION, true);

	sx1276_set_tx_config(&sx1276, MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
            LORA_SPREADING_FACTOR, LORA_CODINGRATE,
            LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
            true, 0, 0, LORA_IQ_INVERSION, 3000000);
}

void init_radio(void) {
	sx1276.nss_pin = GPIO_PIN(PORT_B, SX1276_SPI_NSS);
	sx1276.spi = SX1276_SPI;

	sx1276.dio0_pin = SX1276_DIO0;
	sx1276.dio1_pin = SX1276_DIO1;
	sx1276.dio2_pin = SX1276_DIO2;
	sx1276.dio3_pin = SX1276_DIO3;

	/* Unused pins */
	sx1276.dio4_pin = (gpio_t) SX1276_DIO4;
	sx1276.dio5_pin = (gpio_t) SX1276_DIO5;
	sx1276.reset_pin = (gpio_t) SX1276_RESET;

	sx1276_settings_t settings;
	settings.channel = CHANNEL_HF;
	settings.modem = MODEM_LORA;
	settings.state = RF_IDLE;

	sx1276.settings = settings;


	sx1276_events_t handlers;
	handlers.tx_timeout = tx_timeout;
	handlers.tx_done = tx_done;
	handlers.rx_timeout = rx_timeout;
	handlers.rx_error = rx_error;
	handlers.rx_done = rx_done;
	handlers.fhss_change_channel = fhss_change_channel;
	handlers.cad_done = cad_done;

	if (!spi_init()) {
		puts("init_radio: failed to initialize SPI for sx1276");
		return;
	}

	if (!sx1276_test(&sx1276)) {
		puts("init_radio: test failed");
		return;
	} else {
		puts("init_radio: radio test passed");
	}

	/* Finally init the radio */
	puts("init_radio: initializing SX1276...");

	sx1276_init(&sx1276, &handlers);

	gpio_init_int(SX1276_DIO0, GPIO_IN, GPIO_RISING, sx1276_on_dio0_isr, &sx1276);
	gpio_init_int(SX1276_DIO1, GPIO_IN, GPIO_RISING, sx1276_on_dio1_isr, &sx1276);
	gpio_init_int(SX1276_DIO2, GPIO_IN, GPIO_RISING, sx1276_on_dio2_isr, &sx1276);
	gpio_init_int(SX1276_DIO3, GPIO_IN, GPIO_RISING, sx1276_on_dio3_isr, &sx1276);

	sx1276_set_channel(&sx1276, RF_FREQUENCY);
    init_configs();

	puts("init_radio: sx1276 initialization done");
}

int random(int argc, char **argv) {
	printf("random: number from sx1276: %u\n", (unsigned int) sx1276_random(&sx1276));
	init_configs();

	return 0;
}


int read_temp(int argc, char **argv) {
	printf("read_temp: temperature of the chip %d *C\n", sx1276_read_temp(&sx1276));

	return 0;
}

int regs(int argc, char **argv) {
	if (argc <= 1) {
		puts("usage: get <all | regnum>");
		return -1;
	}

	if (strcmp(argv[1], "all") == 0) {
		puts("- listing all registers -");
		uint16_t i = 0;
		uint8_t reg, data = 0;
		uint8_t j = 0;

		/* Listing registers map*/
		puts("Reg   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F");
		for (i = 0; i <= 7; i++) {
			printf("0x%02X ", i << 4);

			for (j = 0; j <= 15; j++, reg++) {
				data = sx1276_reg_read(&sx1276, reg);

				printf("%02X ", data);
		    }

			puts("");
		}

		puts("-done-");

		return 0;
	} else {
		long int num = 0;

		/* Register number in hex */
		if (strstr(argv[1], "0x") != NULL) {
			num = strtol(argv[1], NULL, 16);
		} else {
			num = atoi(argv[1]);
		}

		if (num >= 0 && num <= 255) {
			printf("[regs] 0x%02X = 0x%02X\n", (uint8_t) num, sx1276_reg_read(&sx1276, (uint8_t) num));
		} else {
			puts("regs: invalid register number specified");
			return -1;
		}
	}

	return 0;
}

int tx_test(int argc, char **argv) {
	if (argc <= 1) {
		puts("tx_test: payload is not specified");
		return -1;
	}

	printf("tx_test: sending \"%s\" payload (%d bytes)\n", argv[1], strlen(argv[1]));


	sx1276_send(&sx1276, (uint8_t*) argv[1], strlen(argv[1]));

	xtimer_usleep(10000); /* wait for the chip */

	puts("tx_test: sended");

	return 0;
}

int regs_set(int argc, char **argv) {
	if (argc <= 2) {
		puts("usage: set <num> <value>");
		return -1;
	}

	long num, val;

	// Register number in hex
	if (strstr(argv[1], "0x") != NULL) {
		num = strtol(argv[1], NULL, 16);
	} else {
		num = atoi(argv[1]);
	}

	// Register value in hex
	if (strstr(argv[2], "0x") != NULL) {
		val = strtol(argv[2], NULL, 16);
	} else {
		val = atoi(argv[2]);
	}

	sx1276_reg_write(&sx1276, (uint8_t) num, (uint8_t) val);

	return 0;

}

int fsk(int argc, char **argv) {
	sx1276_set_modem(&sx1276, MODEM_FSK);

	return 0;
}

int lora(int argc, char **argv) {
	sx1276_set_modem(&sx1276, MODEM_LORA);

	return 0;
}

static const shell_command_t shell_commands[] = {
	{ "random", "Get random number from sx1276", random },
	{ "temp", "Get temperature of sx1276", read_temp },
	{ "get", "<all | num> - gets value of registers of sx1276, all or by specified number from 0 to 255", regs },
	{ "set", "<num> <value> - sets value of register with specified number", regs_set },
	{ "tx_test", "<payload> Send test payload string", tx_test },

	{ "fsk", "Set modem to FSK mode", fsk },
	{ "lora", "Set modem to LoRa mode", lora },

    { NULL, NULL, NULL }
};

void *_recv_thread(void *arg)
{
	return NULL;
}

void sx1276_set_op_mode(sx1276_t *dev, uint8_t op_mode);

int main(void)
{
	print_logo();

    xtimer_init();

    init_radio();

    _recv_pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 1,
                              THREAD_CREATE_STACKTEST, _recv_thread, NULL,
                              "recv_thread");

    if (_recv_pid <= KERNEL_PID_UNDEF) {
        puts("Creation of receiver thread failed");
        return 1;
    }

    sx1276_set_op_mode(&sx1276, RF_IDLE);

    /* start the shell */
    puts("Initialization successful - starting the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
