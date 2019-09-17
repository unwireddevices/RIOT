 
#include <string.h>

#include "sx128x.h"


/**
 * @brief Defines the nominal frequency
 */
#ifndef RF_FREQUENCY
#define RF_FREQUENCY                                    (2404000000UL) // Hz
#endif
/**
 * @brief Defines the output power in dBm
 *
 * @remark The range of the output power is [-18..+13] dBm
 */
#ifndef TX_OUTPUT_POWER
#define TX_OUTPUT_POWER                                 (13)
#endif
/**
 * @brief Defines the buffer size, i.e. the payload size
 */
#define BUFFER_SIZE                                     (20)

/**
 * @brief Number of tick size steps for tx timeout
 */
#define TX_TIMEOUT_VALUE                                (10000) // ms

/**
 * @brief Number of tick size steps for rx timeout
 */
#define RX_TIMEOUT_VALUE                                (1000) // ms

/**
 * @brief Size of ticks (used for Tx and Rx timeout)
 */
#define RX_TIMEOUT_TICK_SIZE                            (SX128X_RADIO_TICK_SIZE_1000_US)

/**
 * @brief Defines the size of the token defining message type in the payload
 */
#define PING_PONG_SIZE                                  (4)

/**
 * @brief Defines GPIO pin for LED
 * 
 */
#define LED_PIN                                         (GPIO_PIN(PORT_A, 15))

/**
 * @brief Defines the states of the application
 */
typedef enum
{
    APP_LOWPOWER,
    APP_RX,
    APP_RX_TIMEOUT,
    APP_RX_ERROR,
    APP_TX,
    APP_TX_TIMEOUT,
} app_states_t;

/**
 * @brief Function to be executed on Radio Tx Done event
 */
void on_tx_done(void);

/**
 * @brief Function to be executed on Radio Rx Done event
 */
void on_rx_done(void);

/**
 * @brief Function executed on Radio Tx Timeout event
 */
void on_tx_timeout( void );

/**
 * @brief Function executed on Radio Rx Timeout event
 */
void on_rx_timeout( void );

/**
 * @brief Function executed on Radio Rx Error event
 */
void on_rx_error( sx128x_irq_error_code_t );

/**
 * @brief Define the possible message type for this application
 */
const uint8_t ping_msg[] = "PING";
const uint8_t pong_msg[] = "PONG";

/**
 * @brief All the callbacks are stored in a structure
 */
sx128x_radio_callbacks_t callbacks =
{
    &on_tx_done,                // txDone
    &on_rx_done,                // rxDone
    NULL,                       // syncWordDone
    NULL,                       // headerDone
    &on_tx_timeout,             // txTimeout
    &on_rx_timeout,             // rxTimeout
    &on_rx_error,               // rxError
    NULL,                       // rangingDone
    NULL,                       // cadDone
};

/**
 * @brief The size of the buffer
 */
uint8_t buffer_size = BUFFER_SIZE;

/**
 * @brief The buffer
 */
uint8_t buffer[BUFFER_SIZE];

/**
 * @brief Mask of IRQs to listen to in rx mode
 */
uint16_t rx_irq_mask = (SX128X_IRQ_RX_DONE | SX128X_IRQ_RX_TX_TIMEOUT);

/**
 * @brief Mask of IRQs to listen to in tx mode
 */
uint16_t tx_irq_mask = (SX128X_IRQ_TX_DONE | SX128X_IRQ_RX_TX_TIMEOUT);

/**
 * @brief The State of the application
 */
app_states_t app_state = APP_LOWPOWER;

sx128x_packet_params_t packet_params;

sx128x_packet_status_t packet_status;


void _hw_init( void )
{
    lptimer_init();
    gpio_init(LED_PIN, GPIO_OUT);

}

static sx128x_t        sx1280_dev;
static sx128x_params_t sx1280_params;

/** SX1280 GPIO */
#define SX128X_BUSY                     GPIO_PIN(PORT_B, 8)
#define SX128X_DIO1                     GPIO_PIN(PORT_B, 0)
#define SX128X_DIO2                     GPIO_PIN(PORT_B, 1)
#define SX128X_DIO3                     GPIO_PIN(PORT_B, 2)
#define SX128X_RESET                    GPIO_PIN(PORT_B, 7)

/** SX1280 SPI */
#define SX128X_SPI                      SPI_DEV(0)
#define SX128X_SPI_NSS                  GPIO_PIN(PORT_B, 6)
#define SX128X_SPI_SPEED                SPI_CLK_1MHZ;
#define SX128X_SPI_MODE                 SPI_MODE_0;

int main( void )
{
    bool is_master = true;
    sx128x_modulation_params_t modulation_params;

    _hw_init( );

    /* let DC/DC power ramp up */
    lptimer_sleep(500);

    sx1280_params.spi_dev   = SX128X_SPI;
    sx1280_params.spi_speed = SX128X_SPI_SPEED;
    sx1280_params.spi_mode  = SX128X_SPI_MODE;
    sx1280_params.nss_pin   = SX128X_SPI_NSS;
    sx1280_params.reset_pin = SX128X_RESET;
    sx1280_params.busy_pin  = SX128X_BUSY;
    sx1280_params.dio1_pin  = SX128X_DIO1;
    sx1280_params.dio2_pin  = GPIO_UNDEF;
    sx1280_params.dio3_pin  = GPIO_UNDEF;

    sx1280_init(&sx1280_dev, &sx1280_params, &callbacks);

    /* Can also be set in LDO mode but consume more power */
    sx1280_set_regulator_mode(&sx1280_dev, SX128X_USE_DCDC); 
    memset(&buffer, 0x00, buffer_size);

    printf("SX1280 Ping Pong Demo Application.\n");
    printf("Radio firmware version 0x%x\n", sx1280_get_firmware_version(&sx1280_dev));

    printf("Ping Pong running in LORA mode\n");


    modulation_params.packet_type                  = SX128X_PACKET_TYPE_LORA;
    modulation_params.params.lora.spreading_factor = SX128X_LORA_SF12;
    modulation_params.params.lora.bandwidth        = SX128X_LORA_BW_1600;
    modulation_params.params.lora.coding_rate      = SX128X_LORA_CR_LI_4_7;

    packet_params.packet_type                 = SX128X_PACKET_TYPE_LORA;
    packet_params.params.lora.preamble_length = 12;
    packet_params.params.lora.header_type     = SX128X_LORA_PACKET_VARIABLE_LENGTH;
    packet_params.params.lora.payload_length  = BUFFER_SIZE;
    packet_params.params.lora.crc_mode        = SX128X_LORA_CRC_ON;
    packet_params.params.lora.invert_iq       = SX128X_LORA_IQ_NORMAL;

    sx1280_set_standby(&sx1280_dev, SX128X_STDBY_RC);
    sx1280_set_packet_type(&sx1280_dev, modulation_params.packet_type);
    sx1280_set_modulation_params(&sx1280_dev, &modulation_params);
    sx1280_set_packet_params(&sx1280_dev, &packet_params);
    sx1280_set_rf_frequency(&sx1280_dev, RF_FREQUENCY);
    sx1280_set_buffer_base_addresses(&sx1280_dev, 0x00, 0x00);
    sx1280_set_tx_params(&sx1280_dev, TX_OUTPUT_POWER, SX128X_RADIO_RAMP_02_US);
    
    sx1280_set_polling_mode();

    gpio_set(LED_PIN);

    sx1280_set_dio_irq_params(&sx1280_dev, rx_irq_mask, rx_irq_mask, SX128X_IRQ_RADIO_NONE, SX128X_IRQ_RADIO_NONE);

    sx1280_set_rx(&sx1280_dev, (sx128x_tick_time_t){RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE});
    app_state = APP_LOWPOWER;

    while(true) {
        sx1280_process_irqs(&sx1280_dev);
        
        switch(app_state)
        {
            case APP_RX:
                app_state = APP_LOWPOWER;
                gpio_toggle(LED_PIN);
                sx1280_get_payload(&sx1280_dev, buffer, &buffer_size, BUFFER_SIZE);
                sx1280_get_packet_status(&sx1280_dev, &packet_status);
                if(is_master == true) {
                    if(buffer_size > 0) {
                        if(strncmp((const char*)buffer, (const char *)pong_msg, PING_PONG_SIZE) == 0) {
                            printf("...Pong\n");
                            memcpy(buffer, ping_msg, PING_PONG_SIZE);
                            sx1280_set_dio_irq_params(&sx1280_dev, tx_irq_mask, tx_irq_mask, SX128X_IRQ_RADIO_NONE, SX128X_IRQ_RADIO_NONE);
                            sx1280_send_payload(&sx1280_dev, buffer, buffer_size, (sx128x_tick_time_t){RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE});
                        } else if(strncmp((const char *)buffer, (const char *)ping_msg, PING_PONG_SIZE) == 0) {
                            // A master already exists then become a slave
                            printf( "...Ping  -  switch to Slave\n" );
                            is_master = false;
                            memcpy(buffer, pong_msg, PING_PONG_SIZE);
                            sx1280_set_dio_irq_params(&sx1280_dev, tx_irq_mask, tx_irq_mask, SX128X_IRQ_RADIO_NONE, SX128X_IRQ_RADIO_NONE);
                            sx1280_send_payload(&sx1280_dev, buffer, buffer_size, (sx128x_tick_time_t){RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE});
                        } else {
                            // valid reception but neither a PING or a PONG message
                            // Set device as master ans start again
                            is_master = true;
                            sx1280_set_dio_irq_params(&sx1280_dev, rx_irq_mask, rx_irq_mask, SX128X_IRQ_RADIO_NONE, SX128X_IRQ_RADIO_NONE);
                            sx1280_set_rx(&sx1280_dev, (sx128x_tick_time_t){RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE});
                        }
                    }
                } else {
                    if(buffer_size > 0) {
                        if(strncmp((const char *)buffer, (const char *)ping_msg, PING_PONG_SIZE) == 0) {
                            printf( "...Ping\n" );
                            memcpy(buffer, pong_msg, PING_PONG_SIZE);
                            sx1280_set_dio_irq_params(&sx1280_dev, tx_irq_mask, tx_irq_mask, SX128X_IRQ_RADIO_NONE, SX128X_IRQ_RADIO_NONE);
                            sx1280_send_payload(&sx1280_dev, buffer, buffer_size, (sx128x_tick_time_t){RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE});
                        } else {
                            // valid reception but not a PING as expected
                            printf( "...Unexpected packet  -  switch to master\r\n");
                            is_master = true;
                            sx1280_set_dio_irq_params(&sx1280_dev, rx_irq_mask, rx_irq_mask, SX128X_IRQ_RADIO_NONE, SX128X_IRQ_RADIO_NONE);
                            sx1280_set_rx(&sx1280_dev, (sx128x_tick_time_t){RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE}); 
                        }
                    }
                }
                break;

            case APP_TX:
                app_state = APP_LOWPOWER;
                gpio_toggle(LED_PIN);
                if(is_master == true) {
                    printf( "Ping...\r\n" );
                } else {
                    printf( "Pong...\r\n" );
                }
                sx1280_set_dio_irq_params(&sx1280_dev, rx_irq_mask, rx_irq_mask, SX128X_IRQ_RADIO_NONE, SX128X_IRQ_RADIO_NONE);
                sx1280_set_rx(&sx1280_dev, (sx128x_tick_time_t){RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE});
                break;

            case APP_RX_TIMEOUT:
                app_state = APP_LOWPOWER;
                if(is_master == true) {
                    // Send the next PING frame
                    memcpy(buffer, ping_msg, PING_PONG_SIZE);
                    sx1280_set_dio_irq_params(&sx1280_dev, tx_irq_mask, tx_irq_mask, SX128X_IRQ_RADIO_NONE, SX128X_IRQ_RADIO_NONE);
                    sx1280_send_payload(&sx1280_dev, buffer, buffer_size, (sx128x_tick_time_t){RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE});
                } else {
                    sx1280_set_dio_irq_params(&sx1280_dev, rx_irq_mask, rx_irq_mask, SX128X_IRQ_RADIO_NONE, SX128X_IRQ_RADIO_NONE);
                    sx1280_set_rx(&sx1280_dev, (sx128x_tick_time_t){RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE});
                }
                break;

            case APP_RX_ERROR:
                app_state = APP_LOWPOWER;
                // We have received a Packet with a CRC error, send reply as if packet was correct
                if(is_master == true) {
                    // Send the next PING frame
                    memcpy(buffer, ping_msg, PING_PONG_SIZE);
                    sx1280_set_dio_irq_params(&sx1280_dev, tx_irq_mask, tx_irq_mask, SX128X_IRQ_RADIO_NONE, SX128X_IRQ_RADIO_NONE);
                    sx1280_send_payload(&sx1280_dev, buffer, buffer_size, (sx128x_tick_time_t){RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE});
                } else {
                    // Send the next PONG frame
                    memcpy(buffer, pong_msg, PING_PONG_SIZE);
                    sx1280_set_dio_irq_params(&sx1280_dev, tx_irq_mask, tx_irq_mask, SX128X_IRQ_RADIO_NONE, SX128X_IRQ_RADIO_NONE);
                    sx1280_send_payload(&sx1280_dev, buffer, buffer_size, (sx128x_tick_time_t){RX_TIMEOUT_TICK_SIZE, TX_TIMEOUT_VALUE});
                }
                break;

            case APP_TX_TIMEOUT:
                app_state = APP_LOWPOWER;
                sx1280_set_dio_irq_params(&sx1280_dev, rx_irq_mask, rx_irq_mask, SX128X_IRQ_RADIO_NONE, SX128X_IRQ_RADIO_NONE);
                sx1280_set_rx(&sx1280_dev, (sx128x_tick_time_t){RX_TIMEOUT_TICK_SIZE, RX_TIMEOUT_VALUE}); 
                break;

            case APP_LOWPOWER:
                break;

            default:
                // Set low power
                break;
        }
    }
}

void on_tx_done(void)
{
    app_state = APP_TX;
}

void on_rx_done(void)
{
    app_state = APP_RX;
}

void on_tx_timeout(void)
{
    app_state = APP_TX_TIMEOUT;
    printf( "<>>>>>>>>TXE\n\r" ); 
}

void on_rx_timeout(void)
{
    app_state = APP_RX_TIMEOUT;
}

void on_rx_error(sx128x_irq_error_code_t err_code)
{
    (void)err_code;

    app_state = APP_RX_ERROR;
    printf( "RXE<>>>>>>>>\n\r" ); 
}

void on_ranging_done(sx128x_irq_ranging_code_t val)
{
    (void)val;
}

void on_cad_done(bool cad_flag)
{
    (void)cad_flag;
}
