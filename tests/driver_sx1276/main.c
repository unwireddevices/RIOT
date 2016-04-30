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

#include "shell.h"
#include "shell_commands.h"
#include "thread.h"
#include "xtimer.h"

#include "common.h"

#include "board.h"

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
	puts("  @@@           %@@,");
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
	puts("sx1276: tx done");
}

void tx_timeout (void) {
	puts("sx1276: tx timeout");
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
		printf("sx1276: received payload with RSSI %d and size %d (snr: %d)", rssi, size, snr);

		uint16_t i;
		for (i = 0; i < size; i++) {
			printf("0x%2x ", payload[i]);
		}
	} else {
		puts("sx1276: rx done with zero size?");
	}
}

/**
 * @brief  Rx Timeout callback prototype.
 */
void rx_timeout(void) {
	puts("sx1276: rx timeout");
}

/**
 * @brief Rx Error callback prototype.
 */
void rx_error(void) {
	puts("sx1276: rx error");
}

void fhss_change_channel(uint8_t current_channel) {
	printf("sx1276: changed channel %d", current_channel);
}

void cad_done(bool activity_detected) {
	puts("sx1276: CAD done");
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



void init_radio(void) {
	sx1276.nss_pin = GPIO_PIN(PORT_B, SPI_1_PIN_NSS);
	sx1276.spi = SPI_1;

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

	puts("init_radio: sx1276 initialization done");
}

int test1(int argc, char **argv) {
	puts("test 1 called");

	return 0;
}

int test2(int argc, char **argv) {
	puts("test 2 called");

	return 0;
}

static const shell_command_t shell_commands[] = {
    { "test1", "Test 1", test1 },
    { "test2", "Test 2", test2 },
    { NULL, NULL, NULL }
};

void *_recv_thread(void *arg)
{
	return NULL;
}

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

    /* start the shell */
    puts("Initialization successful - starting the shell now");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
