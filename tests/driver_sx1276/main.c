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


#define _STACKSIZE      (THREAD_STACKSIZE_DEFAULT + 2 * THREAD_EXTRA_STACKSIZE_PRINTF)
#define MSG_TYPE_ISR    (0x3456)

static char stack[_STACKSIZE];
static kernel_pid_t event_handler_thread_pid;
static msg_t event_queue[10];

void print_logo(void)
{
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

void blink_led(void)
{
    volatile int i;

    LED0_OFF;

    for (i = 0; i < 5; i++) {
        LED0_TOGGLE;
        xtimer_usleep(50000);

        LED0_TOGGLE;
        xtimer_usleep(50000);
    }

    LED0_OFF;
}

int spi_init(void)
{
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
    if (res < 0) {
        printf("gpio_init: error initializing GPIO_%ld as CS line (code %i)\n",
               (long)SX1276_SPI_NSS, res);
        return 0;
    }

    gpio_set(SX1276_SPI_NSS);

    printf("spi_init: SPI_%i successfully initialized as master, cs: GPIO_%ld, mode: %i, speed: %i\n",
           SX1276_SPI, (long)SX1276_SPI_NSS, SX1276_SPI_MODE, SX1276_SPI_SPEED);

    return 1;
}

void sx1276_board_set_ant_sw_low_power(uint8_t lp)
{
    if (lp) {
        gpio_init(SX1276_ANTSW, GPIO_OD);       /* open-drain output for low power mode */
    }
    else {
        gpio_init(SX1276_ANTSW, GPIO_OUT);
    }
}

void sx1276_board_set_ant_sw(uint8_t tx)
{
    if (tx) {
        gpio_set(SX1276_ANTSW);
    }
    else {
        gpio_clear(SX1276_ANTSW);
    }
}

void init_configs(void)
{
    sx1276_set_rx_config(&sx1276, MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                         LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                         LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                         6, true, 0, 0, LORA_IQ_INVERSION, true);

    sx1276_set_tx_config(&sx1276, MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                         LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                         LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                         true, 0, 0, LORA_IQ_INVERSION, 3000000);
}

void *event_handler_thread(void *arg)
{
    puts("sx1276: event handler thread started");

    //sx1276_t *dev = (sx1276_t*) arg;
    msg_init_queue(event_queue, sizeof(event_queue));
    msg_t msg;

    while (1) {
        msg_receive(&msg);

        sx1276_event_t *event = (sx1276_event_t *) msg.content.ptr;
        sx1276_rx_packet_t *packet = (sx1276_rx_packet_t *) event->event_data;

        switch (event->type) {
            case RX_DONE:

                printf("RX: %u bytes: '%s' | RSSI: %d\n",
                       packet->size,
                       packet->content,
                       packet->rssi_value);

                free(packet->content);

                if (packet->rssi_value > -100) {
                    blink_led();
                }
                else {
                    blink_led();
                    blink_led();
                }

                break;

            case RX_ERROR_CRC:
                puts("sx1276: RX CRC failed");
                break;

            case TX_DONE:
                puts("sx1276: transmission done.");
                break;

            case RX_TIMEOUT:
                puts("sx1276: RX timeout");
                break;

            case TX_TIMEOUT:
                puts("sx1276: TX timeout");
                break;

            default:
                printf("sx1276: received event #%d\n", (int) event->type);
                break;
        }

    }

    return NULL;
}

void init_radio(void)
{
    sx1276.nss_pin = SX1276_SPI_NSS;
    sx1276.spi = SX1276_SPI;

    sx1276.dio0_pin = SX1276_DIO0;
    sx1276.dio1_pin = SX1276_DIO1;
    sx1276.dio2_pin = SX1276_DIO2;
    sx1276.dio3_pin = SX1276_DIO3;

    sx1276.dio4_pin = (gpio_t) NULL;
    sx1276.dio5_pin = (gpio_t) NULL;
    sx1276.reset_pin = (gpio_t) SX1276_RESET;

    sx1276_settings_t settings;
    settings.channel = RF_FREQUENCY;
    settings.modem = MODEM_LORA;
    settings.state = RF_IDLE;

    sx1276.settings = settings;

    if (!spi_init()) {
        puts("init_radio: failed to initialize SPI for sx1276");
        return;
    }

    /* Check presence of SX1276 */
    if (!sx1276_test(&sx1276)) {
        puts("init_radio: test failed");
        return;
    }
    else {
        puts("init_radio: radio test passed");
    }


    /* Create event listener thread */

    puts("init_radio: creating event listener...");

    event_handler_thread_pid = thread_create(stack, sizeof(stack), THREAD_PRIORITY_MAIN - 1,
                                             THREAD_CREATE_STACKTEST, event_handler_thread, NULL,
                                             "sx1276 event handler thread");

    if (event_handler_thread_pid <= KERNEL_PID_UNDEF) {
        puts("Creation of receiver thread failed");
        return;
    }

    sx1276.event_handler_thread_pid = event_handler_thread_pid;

    /* Launch initialization of driver and device */
    puts("init_radio: initializing driver...");
    sx1276_init(&sx1276);

    init_configs();

    puts("init_radio: sx1276 initialization done");
}

int random(int argc, char **argv)
{
    printf("random: number from sx1276: %u\n", (unsigned int) sx1276_random(&sx1276));
    init_configs();

    return 0;
}

int regs(int argc, char **argv)
{
    if (argc <= 1) {
        puts("usage: get <all | allinline | regnum>");
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
    }
    if (strcmp(argv[1], "allinline") == 0) {
        puts("- listing all registers in one line -");
        uint16_t reg;
        uint8_t data = 0;

        /* Listing registers map*/
        for (reg = 0; reg < 256; reg++) {
            data = sx1276_reg_read(&sx1276, (uint8_t) reg);

            printf("%02X ", data);
        }

        puts("-done-");

        return 0;
    }
    else {
        long int num = 0;

        /* Register number in hex */
        if (strstr(argv[1], "0x") != NULL) {
            num = strtol(argv[1], NULL, 16);
        }
        else {
            num = atoi(argv[1]);
        }

        if (num >= 0 && num <= 255) {
            printf("[regs] 0x%02X = 0x%02X\n", (uint8_t) num, sx1276_reg_read(&sx1276, (uint8_t) num));
        }
        else {
            puts("regs: invalid register number specified");
            return -1;
        }
    }

    return 0;
}

int tx_test(int argc, char **argv)
{
    if (argc <= 1) {
        puts("tx_test: payload is not specified");
        return -1;
    }

    printf("tx_test: sending \"%s\" payload (%d bytes)\n", argv[1], strlen(argv[1]) + 1);

    sx1276_send(&sx1276, (uint8_t *) argv[1], strlen(argv[1]) + 1);

    xtimer_usleep(10000); /* wait for the chip */

    puts("tx_test: sended");

    return 0;
}

int regs_set(int argc, char **argv)
{
    if (argc <= 2) {
        puts("usage: set <num> <value>");
        return -1;
    }

    long num, val;

    // Register number in hex
    if (strstr(argv[1], "0x") != NULL) {
        num = strtol(argv[1], NULL, 16);
    }
    else {
        num = atoi(argv[1]);
    }

    // Register value in hex
    if (strstr(argv[2], "0x") != NULL) {
        val = strtol(argv[2], NULL, 16);
    }
    else {
        val = atoi(argv[2]);
    }

    sx1276_reg_write(&sx1276, (uint8_t) num, (uint8_t) val);

    return 0;

}

int rx_test(int argc, char **argv)
{
    sx1276_set_rx(&sx1276, 1000 * 1000 * 5); // 5 sec timeout

    return 0;
}

static const shell_command_t shell_commands[] = {
    { "random", "Get random number from sx1276", random },
    { "get", "<all | num> - gets value of registers of sx1276, all or by specified number from 0 to 255", regs },
    { "set", "<num> <value> - sets value of register with specified number", regs_set },
    { "tx_test", "<payload> Send test payload string", tx_test },
    { "rx_test", "Start rx test", rx_test },

    { NULL, NULL, NULL }
};

int main(void)
{
    print_logo();

    xtimer_init();

    init_radio();

    blink_led();

#define RX_TEST
//#define TX_BEACON
#ifdef TX_BEACON
    char *args[] = {
        "tx_test", "Hello world!! This is a test..."
    };

    for (;; ) {
        tx_test(2, args);
        blink_led();

        xtimer_usleep(1000 * 1000 * 3);
    }
#endif

#ifdef RX_TEST
    for (;; ) {
        rx_test(0, NULL);
        xtimer_usleep(1000 * 1000);
    }
#endif

    /* start the shell */
    puts("Initialization successful - starting the shell now");
    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);

    return 0;
}
