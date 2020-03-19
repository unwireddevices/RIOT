#include <string.h>
#include <math.h>

#include "sx128x.h"
#include "sx128x_internal.h"
#include "ranging_correct.h"

#include "lptimer.h"

#define ENABLE_DEBUG (0)
#include "debug.h"

#include "log.h"

lptimer_t op_timeout_timer;

/**
 * @brief Radio registers definition
 */
typedef struct {
    uint16_t      addr;                             //!< The address of the register
    uint8_t       value;                            //!< The value of the register
} sx128x_radio_registers_t;

/**
 * @brief Radio hardware registers initialization definition
 */
// { Address, RegValue }
#define SX128X_RADIO_INIT_REGISTERS_VALUE  { 0x00 }

/**
 * @brief Radio hardware registers initializations
 */
const sx128x_radio_registers_t radio_regs_init[] = {SX128X_RADIO_INIT_REGISTERS_VALUE};

/**
 * @brief Holds the internal operating mode of the radio
 */
static sx128x_radio_operating_modes_t operating_mode;

/**
 * @brief Stores the current packet type set in the radio
 */
static sx128x_radio_packet_types_t packet_type;

/**
 * @brief Stores the current LoRa bandwidth set in the radio
 */
static sx128x_radio_lora_bandwidths_t lora_bandwidth;

/**
 * @brief Holds the polling state of the driver
 */
static bool polling_mode;

/**
 * Hardware DIO IRQ callback initialization
 */
dio_irq_handler dio_irq[] = {sx1280_on_dio_irq};



/**
 * @brief Holds a flag raised on radio interrupt
 */
static bool irq_state;

static sx128x_radio_callbacks_t *radio_callbacks;

/**
 * @brief Compute the two's complement for a register of size lower than
 *        32bits
 *
 * @param [in]  num            The register to be two's complemented
 * @param [in]  bit_cnt        The position of the sign bit
 */
static int32_t sx1280_complement2(const int32_t num, const uint8_t bit_cnt);


static int32_t sx1280_complement2(const int32_t num, const uint8_t bit_cnt)
{
    int32_t ret_val = num;
    if(num >= (2 << (bit_cnt - 2))) {
        ret_val -= (2 << (bit_cnt - 1));
    }
    return ret_val;
}

void sx1280_init(sx128x_t *dev, const sx128x_params_t *params, sx128x_radio_callbacks_t *callbacks)
{
    radio_callbacks = callbacks;
    dev->params = *params;

    sx1280_hal_init(dev, dio_irq);
}

void sx1280_set_registers_default(const sx128x_t *dev)
{
    for(uint16_t i = 0; i < sizeof(radio_regs_init)/sizeof(sx128x_radio_registers_t); i++) {
        sx1280_hal_write_register(dev, radio_regs_init[i].addr, radio_regs_init[i].value);
    }
}

uint16_t sx1280_get_firmware_version(const sx128x_t *dev)
{
    uint16_t ret_val = (((uint16_t)(sx1280_hal_read_register(dev, SX128X_REG_LR_FIRMWARE_VERSION_MSB)) << 8) | 
                         (sx1280_hal_read_register(dev, SX128X_REG_LR_FIRMWARE_VERSION_MSB + 1)));
    return ret_val;
}

sx128x_radio_status_t sx1280_get_status(const sx128x_t *dev)
{
    uint8_t stat = 0;
    sx128x_radio_status_t status;

    sx1280_hal_read_command(dev, SX128X_RADIO_GET_STATUS, (uint8_t *)&stat, 1);
    status.Value = stat;
    return status;
}

sx128x_radio_operating_modes_t sx1280_get_opmode(void)
{
    return operating_mode;
}

void sx1280_set_sleep(const sx128x_t *dev, sx128x_sleep_params_t sleep_config)
{
    uint8_t sleep = (sleep_config.wakeup_rtc << 3) |
                    (sleep_config.instruction_ram_retention << 2) |
                    (sleep_config.data_buffer_retention << 1) |
                    (sleep_config.data_ram_retention);

    operating_mode = SX128X_MODE_SLEEP;
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_SLEEP, &sleep, 1);
}

void sx1280_set_standby(const sx128x_t *dev, sx128x_radio_standby_modes_t standby_config)
{
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_STANDBY, (uint8_t *)&standby_config, 1);
    if(standby_config == SX128X_STDBY_RC) {
        operating_mode = SX128X_MODE_STDBY_RC;
    } else {
        operating_mode = SX128X_MODE_STDBY_XOSC;
    }
}

void sx1280_set_fs(const sx128x_t *dev)
{
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_FS, 0, 0);
    operating_mode = SX128X_MODE_FS;
}

void sx1280_timeout_cb(void *arg) {
    sx128x_t *dev = (sx128x_t *)arg;

    sx1280_clear_irq_status(dev, SX128X_IRQ_RADIO_ALL);
    sx1280_set_standby(dev, SX128X_STDBY_XOSC);

    if((radio_callbacks != NULL) && (radio_callbacks->rx_timeout != NULL)) {
        radio_callbacks->rx_timeout();
    }
}

void sx1280_set_tx(const sx128x_t *dev, sx128x_tick_time_t timeout)
{
    uint8_t buf[3];

    buf[0] = timeout.step;
    buf[1] = (uint8_t)((timeout.nb_steps >> 8) & 0x00FF);
    buf[2] = (uint8_t)(timeout.nb_steps & 0x00FF);

    sx1280_clear_irq_status(dev, SX128X_IRQ_RADIO_ALL);

    // If the radio is doing ranging operations, then apply the specific calls
    // prior to SetTx
    if(sx1280_get_packet_type() == SX128X_PACKET_TYPE_RANGING) {
        sx1280_set_ranging_role(dev, SX128X_RADIO_RANGING_ROLE_MASTER);
    }
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_TX, buf, 3);
    operating_mode = SX128X_MODE_TX;
    
    op_timeout_timer.arg = (void *)dev;
    op_timeout_timer.callback = sx1280_timeout_cb;
    lptimer_set(&op_timeout_timer, 250); /* 250 ms maximum TX timeout */
}

void sx1280_set_rx(const sx128x_t *dev, sx128x_tick_time_t timeout)
{
    uint8_t buf[3];

    buf[0] = timeout.step;
    buf[1] = (uint8_t)((timeout.nb_steps >> 8) & 0x00FF);
    buf[2] = (uint8_t)(timeout.nb_steps & 0x00FF);

    sx1280_clear_irq_status(dev, SX128X_IRQ_RADIO_ALL);
    sx1280_set_standby(dev, SX128X_STDBY_XOSC);

    // If the radio is doing ranging operations, then apply the specific calls
    // prior to SetRx
    if(sx1280_get_packet_type() == SX128X_PACKET_TYPE_RANGING) {
        sx1280_set_ranging_role(dev, SX128X_RADIO_RANGING_ROLE_SLAVE);
    }
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_RX, buf, 3);
    operating_mode = SX128X_MODE_RX;
    
    uint32_t timeout_ms = 0;
    switch (timeout.step) {
        case SX128X_RADIO_TICK_SIZE_0015_US:
            timeout_ms = 5 + (timeout.nb_steps*15)/1000;
            break;
        case SX128X_RADIO_TICK_SIZE_0062_US:
            timeout_ms = 5 + (timeout.nb_steps*62)/1000;
            break;
        case SX128X_RADIO_TICK_SIZE_1000_US:
            timeout_ms = 5 + timeout.nb_steps;
            break;
        case SX128X_RADIO_TICK_SIZE_4000_US:
            timeout_ms = 5 + timeout.nb_steps*4;
            break;
        default:
            timeout_ms = 50;
            break;
    }

    op_timeout_timer.arg = (void *)dev;
    op_timeout_timer.callback = sx1280_timeout_cb;
    lptimer_set(&op_timeout_timer, timeout_ms);
}

void sx1280_set_rx_duty_cycle(const sx128x_t *dev, sx128x_radio_tick_sizes_t step, uint16_t nb_step_rx, uint16_t rx_nb_step_sleep)
{
    uint8_t buf[5];

    buf[0] = step;
    buf[1] = (uint8_t)((nb_step_rx >> 8) & 0x00FF);
    buf[2] = (uint8_t)(nb_step_rx & 0x00FF);
    buf[3] = (uint8_t)((rx_nb_step_sleep >> 8) & 0x00FF);
    buf[4] = (uint8_t)(rx_nb_step_sleep & 0x00FF);

    sx1280_hal_write_command(dev, SX128X_RADIO_SET_RXDUTYCYCLE, buf, 5);
    operating_mode = SX128X_MODE_RX;
}

void sx1280_set_cad(const sx128x_t *dev)
{
    sx1280_clear_irq_status(dev, SX128X_IRQ_RADIO_ALL);
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_CAD, 0, 0);
    operating_mode = SX128X_MODE_CAD;
}

void sx1280_set_tx_continuous_wave(const sx128x_t *dev)
{
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_TXCONTINUOUSWAVE, 0, 0);
}

void sx1280_set_tx_continuous_preamble(const sx128x_t *dev)
{
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_TXCONTINUOUSPREAMBLE, 0, 0);
}

void sx1280_set_packet_type(const sx128x_t *dev, sx128x_radio_packet_types_t pkt_type)
{
    // Save packet type internally to avoid questioning the radio
    packet_type = pkt_type;

    sx1280_hal_write_command(dev, SX128X_RADIO_SET_PACKETTYPE, (uint8_t *)&packet_type, 1);
}

sx128x_radio_packet_types_t sx1280_get_packet_type(void)
{
    return packet_type;
}

void sx1280_set_rf_frequency(const sx128x_t *dev, uint32_t frequency)
{
    uint8_t buf[3];
    uint32_t freq = 0;

    freq = (uint32_t)((double)frequency / (double)SX128X_FREQ_STEP);
    buf[0] = (uint8_t)((freq >> 16) & 0xFF);
    buf[1] = (uint8_t)((freq >> 8) & 0xFF);
    buf[2] = (uint8_t)(freq & 0xFF);

    sx1280_hal_write_command(dev, SX128X_RADIO_SET_RFFREQUENCY, buf, 3 );
}

void sx1280_set_tx_params(const sx128x_t *dev, int8_t power, sx128x_radio_ramp_times_t ramp_time)
{
    uint8_t buf[2];

    // The power value to send on SPI/UART is in the range [0..31] and the
    // physical output power is in the range [-18..13]dBm
    buf[0] = power + 18;
    buf[1] = (uint8_t)ramp_time;

    sx1280_hal_write_command(dev, SX128X_RADIO_SET_TXPARAMS, buf, 2);
}

void sx1280_set_cad_params(const sx128x_t *dev, sx128x_radio_lora_cad_symbols_t cad_symbol_num)
{
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_CADPARAMS, (uint8_t *)&cad_symbol_num, 1);
    operating_mode = SX128X_MODE_CAD;
}

void sx1280_set_buffer_base_addresses(const sx128x_t *dev, uint8_t tx_base_address, uint8_t rx_base_address)
{
    uint8_t buf[2];

    buf[0] = tx_base_address;
    buf[1] = rx_base_address;

    sx1280_hal_write_command(dev, SX128X_RADIO_SET_BUFFERBASEADDRESS, buf, 2);
}

void sx1280_set_modulation_params(const sx128x_t *dev, sx128x_modulation_params_t *modulation_params)
{
    uint8_t buf[3];

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if(packet_type != modulation_params->packet_type) {
        sx1280_set_packet_type(dev, modulation_params->packet_type);
    }

    switch(modulation_params->packet_type)
    {
        case SX128X_PACKET_TYPE_GFSK:
            buf[0] = modulation_params->params.gfsk.bitrate_bandwidth;
            buf[1] = modulation_params->params.gfsk.modulation_index;
            buf[2] = modulation_params->params.gfsk.modulation_shaping;
            break;

        case SX128X_PACKET_TYPE_LORA:
        case SX128X_PACKET_TYPE_RANGING:
            buf[0] = modulation_params->params.lora.spreading_factor;
            buf[1] = modulation_params->params.lora.bandwidth;
            buf[2] = modulation_params->params.lora.coding_rate;
            lora_bandwidth = modulation_params->params.lora.bandwidth;
            break;

        case SX128X_PACKET_TYPE_FLRC:
            buf[0] = modulation_params->params.flrc.bitrate_bandwidth;
            buf[1] = modulation_params->params.flrc.coding_rate;
            buf[2] = modulation_params->params.flrc.modulation_shaping;
            break;

        case SX128X_PACKET_TYPE_BLE:
            buf[0] = modulation_params->params.ble.bitrate_bandwidth;
            buf[1] = modulation_params->params.ble.modulation_index;
            buf[2] = modulation_params->params.ble.modulation_shaping;
            break;

        case SX128X_PACKET_TYPE_NONE:
            buf[0] = 0x00;
            buf[1] = 0x00;
            buf[2] = 0x00;
            break;
    }
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_MODULATIONPARAMS, buf, 3);
}

void sx1280_set_packet_params(const sx128x_t *dev, sx128x_packet_params_t *packet_params)
{
    uint8_t buf[7];

    // Check if required configuration corresponds to the stored packet type
    // If not, silently update radio packet type
    if(packet_type != packet_params->packet_type) {
        sx1280_set_packet_type(dev, packet_params->packet_type);
    }

    switch(packet_params->packet_type)
    {
        case SX128X_PACKET_TYPE_GFSK:
            buf[0] = packet_params->params.gfsk.preamble_length;
            buf[1] = packet_params->params.gfsk.sync_word_length;
            buf[2] = packet_params->params.gfsk.sync_word_match;
            buf[3] = packet_params->params.gfsk.header_type;
            buf[4] = packet_params->params.gfsk.payload_length;
            buf[5] = packet_params->params.gfsk.crc_length;
            buf[6] = packet_params->params.gfsk.whitening;
            break;

        case SX128X_PACKET_TYPE_LORA:
        case SX128X_PACKET_TYPE_RANGING:
            buf[0] = packet_params->params.lora.preamble_length;
            buf[1] = packet_params->params.lora.header_type;
            buf[2] = packet_params->params.lora.payload_length;
            buf[3] = packet_params->params.lora.crc_mode;
            buf[4] = packet_params->params.lora.invert_iq;
            buf[5] = 0x00;
            buf[6] = 0x00;
            break;

        case SX128X_PACKET_TYPE_FLRC:
            buf[0] = packet_params->params.flrc.preamble_length;
            buf[1] = packet_params->params.flrc.sync_word_length;
            buf[2] = packet_params->params.flrc.sync_word_match;
            buf[3] = packet_params->params.flrc.header_type;
            buf[4] = packet_params->params.flrc.payload_length;
            buf[5] = packet_params->params.flrc.crc_length;
            buf[6] = packet_params->params.flrc.whitening;
            break;

        case SX128X_PACKET_TYPE_BLE:
            buf[0] = packet_params->params.ble.connection_state;
            buf[1] = packet_params->params.ble.crc_field;
            buf[2] = packet_params->params.ble.ble_packet_type;
            buf[3] = packet_params->params.ble.whitening;
            buf[4] = 0x00;
            buf[5] = 0x00;
            buf[6] = 0x00;
            break;

        case SX128X_PACKET_TYPE_NONE:
            buf[0] = 0x00;
            buf[1] = 0x00;
            buf[2] = 0x00;
            buf[3] = 0x00;
            buf[4] = 0x00;
            buf[5] = 0x00;
            buf[6] = 0x00;
            break;
    }
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_PACKETPARAMS, buf, 7);
}

void sx1280_get_rx_buffer_status(const sx128x_t *dev, uint8_t *payload_length, uint8_t *rx_start_buffer_pointer)
{
    uint8_t status[2];

    sx1280_hal_read_command(dev, SX128X_RADIO_GET_RXBUFFERSTATUS, status, 2);

    // In case of LORA fixed header, the payloadLength is obtained by reading
    // the register REG_LR_PAYLOADLENGTH
    if((sx1280_get_packet_type() == SX128X_PACKET_TYPE_LORA) && 
       (sx1280_hal_read_register(dev, SX128X_REG_LR_PACKETPARAMS) >> 7 == 1))
    {
        *payload_length = sx1280_hal_read_register(dev, SX128X_REG_LR_PAYLOADLENGTH);
    }
    else if(sx1280_get_packet_type() == SX128X_PACKET_TYPE_BLE) {
        // In the case of BLE, the size returned in status[0] do not include the 2-byte length PDU header
        // so it is added there
        *payload_length = status[0] + 2;
    } else {
        *payload_length = status[0];
    }

    *rx_start_buffer_pointer = status[1];
}

void sx1280_get_packet_status(const sx128x_t *dev, sx128x_packet_status_t *pkt_status)
{
    uint8_t status[5];

    sx1280_hal_read_command(dev, SX128X_RADIO_GET_PACKETSTATUS, status, 5);

    pkt_status->packet_type = sx1280_get_packet_type();
    switch(pkt_status->packet_type)
    {
        case SX128X_PACKET_TYPE_GFSK:
            pkt_status->params.gfsk.rssi_avg = -status[0] / 2;
            pkt_status->params.gfsk.rssi_sync = -status[1] / 2;

            pkt_status->params.gfsk.error_status.sync_error = (status[2] >> 6) & 0x01;
            pkt_status->params.gfsk.error_status.length_error = (status[2] >> 5) & 0x01;
            pkt_status->params.gfsk.error_status.crc_error = (status[2] >> 4) & 0x01;
            pkt_status->params.gfsk.error_status.abort_error = (status[2] >> 3) & 0x01;
            pkt_status->params.gfsk.error_status.header_received = (status[2] >> 2) & 0x01;
            pkt_status->params.gfsk.error_status.packet_received = (status[2] >> 1) & 0x01;
            pkt_status->params.gfsk.error_status.packet_controler_busy = status[2] & 0x01;

            pkt_status->params.gfsk.tx_rx_status.rx_no_ack = (status[3] >> 5) & 0x01;
            pkt_status->params.gfsk.tx_rx_status.packet_sent = status[3] & 0x01;

            pkt_status->params.gfsk.sync_addr_status = status[4] & 0x07;
            break;

        case SX128X_PACKET_TYPE_LORA:
        case SX128X_PACKET_TYPE_RANGING:
            pkt_status->params.lora.rssi_pkt = -status[0] / 2;
            (status[1] < 128) ? (pkt_status->params.lora.snr_pkt = status[1] / 4) : 
                                (pkt_status->params.lora.snr_pkt = ((status[1] - 256) / 4));

            pkt_status->params.lora.error_status.sync_error = (status[2] >> 6) & 0x01;
            pkt_status->params.lora.error_status.length_error = (status[2] >> 5) & 0x01;
            pkt_status->params.lora.error_status.crc_error = (status[2] >> 4) & 0x01;
            pkt_status->params.lora.error_status.abort_error = (status[2] >> 3) & 0x01;
            pkt_status->params.lora.error_status.header_received = (status[2] >> 2) & 0x01;
            pkt_status->params.lora.error_status.packet_received = (status[2] >> 1) & 0x01;
            pkt_status->params.lora.error_status.packet_controler_busy = status[2] & 0x01;

            pkt_status->params.lora.tx_rx_status.rx_no_ack = (status[3] >> 5) & 0x01;
            pkt_status->params.lora.tx_rx_status.packet_sent = status[3] & 0x01;

            pkt_status->params.lora.sync_addr_status = status[4] & 0x07;
            break;

        case SX128X_PACKET_TYPE_FLRC:
            pkt_status->params.flrc.rssi_avg = -status[0] / 2;
            pkt_status->params.flrc.rssi_sync = -status[1] / 2;

            pkt_status->params.flrc.error_status.sync_error = (status[2] >> 6) & 0x01;
            pkt_status->params.flrc.error_status.length_error = (status[2] >> 5) & 0x01;
            pkt_status->params.flrc.error_status.crc_error = (status[2] >> 4) & 0x01;
            pkt_status->params.flrc.error_status.abort_error = (status[2] >> 3) & 0x01;
            pkt_status->params.flrc.error_status.header_received = (status[2] >> 2) & 0x01;
            pkt_status->params.flrc.error_status.packet_received = (status[2] >> 1) & 0x01;
            pkt_status->params.flrc.error_status.packet_controler_busy = status[2] & 0x01;

            pkt_status->params.flrc.tx_rx_status.rx_pid = (status[3] >> 6) & 0x03;
            pkt_status->params.flrc.tx_rx_status.rx_no_ack = (status[3] >> 5) & 0x01;
            pkt_status->params.flrc.tx_rx_status.rx_pid_err = (status[3] >> 4) & 0x01;
            pkt_status->params.flrc.tx_rx_status.packet_sent = status[3] & 0x01;

            pkt_status->params.flrc.sync_addr_status = status[4] & 0x07;
            break;

        case SX128X_PACKET_TYPE_BLE:
            pkt_status->params.ble.rssi_avg = -status[0] / 2;
            pkt_status->params.ble.rssi_sync = -status[1] / 2;

            pkt_status->params.ble.error_status.sync_error = (status[2] >> 6) & 0x01;
            pkt_status->params.ble.error_status.length_error = (status[2] >> 5) & 0x01;
            pkt_status->params.ble.error_status.crc_error = (status[2] >> 4) & 0x01;
            pkt_status->params.ble.error_status.abort_error = (status[2] >> 3) & 0x01;
            pkt_status->params.ble.error_status.header_received = (status[2] >> 2) & 0x01;
            pkt_status->params.ble.error_status.packet_received = (status[2] >> 1) & 0x01;
            pkt_status->params.ble.error_status.packet_controler_busy = status[2] & 0x01;

            pkt_status->params.ble.tx_rx_status.packet_sent = status[3] & 0x01;

            pkt_status->params.ble.sync_addr_status = status[4] & 0x07;
            break;

        case SX128X_PACKET_TYPE_NONE:
            // In that specific case, we set everything in the pktStatus to zeros
            // and reset the packet type accordingly
            memset(pkt_status, 0, sizeof(sx128x_packet_status_t));
            pkt_status->packet_type = SX128X_PACKET_TYPE_NONE;
            break;
    }
}

int8_t sx1280_get_rssi_inst(const sx128x_t *dev)
{
    uint8_t raw = 0;

    sx1280_hal_read_command(dev, SX128X_RADIO_GET_RSSIINST, &raw, 1);

    return (int8_t)(-raw / 2);
}

void sx1280_set_dio_irq_params(const sx128x_t *dev, uint16_t irq_mask, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask)
{
    uint8_t buf[8];

    buf[0] = (uint8_t)((irq_mask >> 8) & 0x00FF);
    buf[1] = (uint8_t)(irq_mask & 0x00FF);
    buf[2] = (uint8_t)((dio1_mask >> 8) & 0x00FF);
    buf[3] = (uint8_t)(dio1_mask & 0x00FF);
    buf[4] = (uint8_t)((dio2_mask >> 8) & 0x00FF);
    buf[5] = (uint8_t)(dio2_mask & 0x00FF);
    buf[6] = (uint8_t)((dio3_mask >> 8) & 0x00FF);
    buf[7] = (uint8_t)(dio3_mask & 0x00FF);

    sx1280_hal_write_command(dev, SX128X_RADIO_SET_DIOIRQPARAMS, buf, 8 );
}

uint16_t sx1280_get_irq_status(const sx128x_t *dev)
{
    uint8_t irq_status[2];

    sx1280_hal_read_command(dev, SX128X_RADIO_GET_IRQSTATUS, irq_status, 2);

    return ((irq_status[0] << 8) | irq_status[1]);
}

void sx1280_clear_irq_status(const sx128x_t *dev, uint16_t irq)
{
    uint8_t buf[2];

    buf[0] = (uint8_t)(((uint16_t)irq >> 8) & 0x00FF);
    buf[1] = (uint8_t)((uint16_t)irq & 0x00FF);
    sx1280_hal_write_command(dev, SX128X_RADIO_CLR_IRQSTATUS, buf, 2);
}

void sx1280_calibrate(const sx128x_t *dev, sx128x_calibration_params_t calib_param)
{
    uint8_t cal = (calib_param.adc_bulkp_enable << 5) |
                  (calib_param.adc_bulkn_enable << 4) |
                  (calib_param.adc_pulse_enable << 3) |
                  (calib_param.pll_enable << 2) |
                  (calib_param.rc13m_enable << 1) |
                  (calib_param.rc64k_enable);

    sx1280_hal_write_command(dev, SX128X_RADIO_CALIBRATE, &cal, 1);
}

void sx1280_set_regulator_mode(const sx128x_t *dev, sx128x_radio_regulator_modes_t mode)
{
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_REGULATORMODE, (uint8_t *)&mode, 1);
}

void sx1280_set_save_context(const sx128x_t *dev)
{
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_SAVECONTEXT, 0, 0);
}

void sx1280_set_auto_tx(const sx128x_t *dev, uint16_t time)
{
    uint16_t compensated_time = time - (uint16_t)AUTO_RX_TX_OFFSET;
    uint8_t buf[2];

    buf[0] = (uint8_t)((compensated_time >> 8) & 0x00FF);
    buf[1] = (uint8_t)(compensated_time & 0x00FF);

    sx1280_hal_write_command(dev, SX128X_RADIO_SET_AUTOTX, buf, 2);
}

void sx1280_stop_auto_tx(const sx128x_t *dev)
{
    uint8_t buf[2] = {0x00, 0x00};
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_AUTOTX, buf, 2);
}

void sx1280_set_auto_fs(const sx128x_t *dev, uint8_t enable)
{
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_AUTOFS, &enable, 1 );
}

void sx1280_set_long_preamble(const sx128x_t *dev, uint8_t enable)
{
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_LONGPREAMBLE, &enable, 1 );
}

void sx1280_set_payload(const sx128x_t *dev, uint8_t *buffer, uint8_t size)
{
    sx1280_hal_write_buffer(dev, 0x00, buffer, size);
}

uint8_t sx1280_get_payload(const sx128x_t *dev, uint8_t *buffer, uint8_t *size , uint8_t max_size)
{
    uint8_t offset;

    sx1280_get_rx_buffer_status(dev, size, &offset);
    if(*size > max_size) {
        return 1;
    }
    sx1280_hal_read_buffer(dev, offset, buffer, *size);
    return 0;
}

void sx1280_send_payload(const sx128x_t *dev, uint8_t *payload, uint8_t size, sx128x_tick_time_t timeout)
{
    sx1280_set_payload(dev, payload, size);
    sx1280_set_tx(dev, timeout);
}

uint8_t sx1280_set_sync_word(const sx128x_t *dev, uint8_t sync_word_idx, uint8_t *sync_word)
{
    uint16_t addr;
    uint8_t syncword_size = 0;

    switch(sx1280_get_packet_type())
    {
        case SX128X_PACKET_TYPE_GFSK:
            syncword_size = 5;
            switch(sync_word_idx)
            {
                case 1:
                    addr = SX128X_REG_LR_SYNCWORDBASEADDRESS1;
                    break;

                case 2:
                    addr = SX128X_REG_LR_SYNCWORDBASEADDRESS2;
                    break;

                case 3:
                    addr = SX128X_REG_LR_SYNCWORDBASEADDRESS3;
                    break;

                default:
                    return 1;
            }
            break;

        case SX128X_PACKET_TYPE_FLRC:
            // For FLRC packet type, the SyncWord is one byte shorter and
            // the base address is shifted by one byte
            syncword_size = 4;
            switch(sync_word_idx)
            {
                case 1:
                    addr = SX128X_REG_LR_SYNCWORDBASEADDRESS1 + 1;
                    break;

                case 2:
                    addr = SX128X_REG_LR_SYNCWORDBASEADDRESS2 + 1;
                    break;

                case 3:
                    addr = SX128X_REG_LR_SYNCWORDBASEADDRESS3 + 1;
                    break;

                default:
                    return 1;
            }
            break;

        case SX128X_PACKET_TYPE_BLE:
            // For Ble packet type, only the first SyncWord is used and its
            // address is shifted by one byte
            syncword_size = 4;
            switch(sync_word_idx)
            {
                case 1:
                    addr = SX128X_REG_LR_SYNCWORDBASEADDRESS1 + 1;
                    break;

                default:
                    return 1;
            }
            break;

        default:
            return 1;
    }
    sx1280_hal_write_registers(dev, addr, sync_word, syncword_size);
    return 0;
}

void sx1280_set_sync_word_error_tolerance(const sx128x_t *dev, uint8_t error_bits)
{
    error_bits = (sx1280_hal_read_register(dev, SX128X_REG_LR_SYNCWORDTOLERANCE) & 0xF0) | (error_bits & 0x0F);
    sx1280_hal_write_register(dev, SX128X_REG_LR_SYNCWORDTOLERANCE, error_bits);
}

void sx1280_set_crc_seed(const sx128x_t *dev, uint16_t seed)
{
    uint8_t val[2];

    val[0] = (uint8_t)(seed >> 8) & 0xFF;
    val[1] = (uint8_t)(seed  & 0xFF);

    switch(sx1280_get_packet_type())
    {
        case SX128X_PACKET_TYPE_GFSK:
        case SX128X_PACKET_TYPE_FLRC:
            sx1280_hal_write_registers(dev, SX128X_REG_LR_CRCSEEDBASEADDR, val, 2 );
            break;

        default:
            break;
    }
}

void sx1280_set_ble_access_address(const sx128x_t *dev, uint32_t access_address)
{
    sx1280_hal_write_register(dev, SX128X_REG_LR_BLE_ACCESS_ADDRESS, (access_address >> 24) & 0x000000FF);
    sx1280_hal_write_register(dev, SX128X_REG_LR_BLE_ACCESS_ADDRESS + 1, (access_address >> 16) & 0x000000FF);
    sx1280_hal_write_register(dev, SX128X_REG_LR_BLE_ACCESS_ADDRESS + 2, (access_address >> 8) & 0x000000FF);
    sx1280_hal_write_register(dev, SX128X_REG_LR_BLE_ACCESS_ADDRESS + 3, access_address & 0x000000FF);
}

void sx1280_set_ble_advertizer_access_address(const sx128x_t *dev)
{
    sx1280_set_ble_access_address(dev, SX128X_BLE_ADVERTIZER_ACCESS_ADDRESS);
}

void sx1280_set_crc_polynomial(const sx128x_t *dev, uint16_t polynomial)
{
    uint8_t val[2];

    val[0] = (uint8_t)(polynomial >> 8) & 0xFF;
    val[1] = (uint8_t)(polynomial  & 0xFF);

    switch(sx1280_get_packet_type())
    {
        case SX128X_PACKET_TYPE_GFSK:
        case SX128X_PACKET_TYPE_FLRC:
            sx1280_hal_write_registers(dev, SX128X_REG_LR_CRCPOLYBASEADDR, val, 2);
            break;

        default:
            break;
    }
}

void sx1280_set_whitening_seed(const sx128x_t *dev, uint8_t seed)
{
    switch(sx1280_get_packet_type())
    {
        case SX128X_PACKET_TYPE_GFSK:
        case SX128X_PACKET_TYPE_FLRC:
        case SX128X_PACKET_TYPE_BLE:
            sx1280_hal_write_register(dev, SX128X_REG_LR_WHITSEEDBASEADDR, seed);
            break;

        default:
            break;
    }
}

void sx1280_enable_manual_gain(const sx128x_t *dev)
{
    sx1280_hal_write_register(dev, SX128X_REG_ENABLE_MANUAL_GAIN_CONTROL, sx1280_hal_read_register(dev, SX128X_REG_ENABLE_MANUAL_GAIN_CONTROL) | SX128X_MASK_MANUAL_GAIN_CONTROL);
    sx1280_hal_write_register(dev, SX128X_REG_DEMOD_DETECTION, sx1280_hal_read_register(dev, SX128X_REG_DEMOD_DETECTION) & SX128X_MASK_DEMOD_DETECTION);
}

void sx1280_disable_manual_gain(const sx128x_t *dev)
{
    sx1280_hal_write_register(dev, SX128X_REG_ENABLE_MANUAL_GAIN_CONTROL, sx1280_hal_read_register(dev, SX128X_REG_ENABLE_MANUAL_GAIN_CONTROL) & ~SX128X_MASK_MANUAL_GAIN_CONTROL);
    sx1280_hal_write_register(dev, SX128X_REG_DEMOD_DETECTION, sx1280_hal_read_register(dev, SX128X_REG_DEMOD_DETECTION) | ~SX128X_MASK_DEMOD_DETECTION );
}

void sx1280_set_manual_gain_value(const sx128x_t *dev, uint8_t gain)
{
    sx1280_hal_write_register(dev, SX128X_REG_MANUAL_GAIN_VALUE, (sx1280_hal_read_register(dev, SX128X_REG_MANUAL_GAIN_VALUE) & SX128X_MASK_MANUAL_GAIN_VALUE) | gain);
}

void sx1280_set_lna_gain_setting(const sx128x_t *dev, const sx128x_radio_lna_settings_t lna_setting)
{
    switch(lna_setting)
    {
        case SX128X_LNA_HIGH_SENSITIVITY_MODE:
        {
            sx1280_hal_write_register(dev, SX128X_REG_LNA_REGIME, sx1280_hal_read_register(dev, SX128X_REG_LNA_REGIME) | SX128X_MASK_LNA_REGIME);
            break;
        }
        case SX128X_LNA_LOW_POWER_MODE:
        {
            sx1280_hal_write_register(dev, SX128X_REG_LNA_REGIME, sx1280_hal_read_register(dev, SX128X_REG_LNA_REGIME) & ~SX128X_MASK_LNA_REGIME);
            break;
        }
    }
}

void sx1280_set_ranging_id_length(const sx128x_t *dev, sx128x_radio_ranging_id_check_lengths_t length)
{
    switch(sx1280_get_packet_type())
    {
        case SX128X_PACKET_TYPE_RANGING:
            sx1280_hal_write_register(dev, SX128X_REG_LR_RANGINGIDCHECKLENGTH, ((((uint8_t)length) & 0x03) << 6) | (sx1280_hal_read_register(dev, SX128X_REG_LR_RANGINGIDCHECKLENGTH ) & 0x3F));
            break;

        default:
            break;
    }
}

void sx1280_set_device_ranging_address(const sx128x_t *dev, uint32_t address)
{
    uint8_t addr_array[] = {address >> 24, address >> 16, address >> 8, address};

    switch( sx1280_get_packet_type( ) )
    {
        case SX128X_PACKET_TYPE_RANGING:
            sx1280_hal_write_registers(dev, SX128X_REG_LR_DEVICERANGINGADDR, addr_array, 4);
            break;

        default:
            break;
    }
}

void sx1280_set_ranging_request_address(const sx128x_t *dev, uint32_t address)
{
    uint8_t addr_array[] = {address >> 24, address >> 16, address >> 8, address};

    switch(sx1280_get_packet_type())
    {
        case SX128X_PACKET_TYPE_RANGING:
            sx1280_hal_write_registers(dev, SX128X_REG_LR_REQUESTRANGINGADDR, addr_array, 4);
            break;

        default:
            break;
    }
}

int32_t sx1280_get_ranging_result(const sx128x_t *dev, sx128x_radio_ranging_result_types_t result_type)
{
    uint32_t val_lsb = 0;
    int32_t val = 0;

    switch(sx1280_get_packet_type())
    {
        case SX128X_PACKET_TYPE_RANGING:
            sx1280_set_standby(dev, SX128X_STDBY_XOSC);
            sx1280_hal_write_register(dev, 0x97F, sx1280_hal_read_register(dev, 0x97F) | (1 << 1)); // enable LORA modem clock
            sx1280_hal_write_register(dev, SX128X_REG_LR_RANGINGRESULTCONFIG, (sx1280_hal_read_register(dev, SX128X_REG_LR_RANGINGRESULTCONFIG) & SX128X_MASK_RANGINGMUXSEL) | ((((uint8_t)result_type) & 0x03) << 4));
            val_lsb = ((sx1280_hal_read_register(dev, SX128X_REG_LR_RANGINGRESULTBASEADDR) << 16) | 
                       (sx1280_hal_read_register(dev, SX128X_REG_LR_RANGINGRESULTBASEADDR + 1) << 8) | 
                       (sx1280_hal_read_register(dev, SX128X_REG_LR_RANGINGRESULTBASEADDR + 2)));
            sx1280_set_standby(dev, SX128X_STDBY_RC);

            // Convertion from LSB to distance. For explanation on the formula, refer to Datasheet of SX1280
            switch(result_type)
            {
                case SX128X_RANGING_RESULT_RAW:
                    // Convert the ranging LSB to distance in meter
                    // The theoretical conversion from register value to distance [m] is given by:
                    // distance [m] = ( complement2( register ) * 150 ) / ( 2^12 * bandwidth[MHz] ) )
                    // The API provide BW in [Hz] so the implemented formula is (complement2( register ) / bandwidth[Hz]) * A,
                    // where A = 150 / (2^12 / 1e6) = 36621.09
                    DEBUG("val_lsb is %08" PRIx32 "\n", val_lsb);
                    int32_t bw = sx1280_get_lora_bandwidth();
                    DEBUG("bw is %" PRIi32 "Hz\n", bw);
                    int32_t tws_compl = sx1280_complement2(val_lsb, 24) ;
                    DEBUG("tws_compl is %" PRIi32 "\n", tws_compl);
                    int32_t coefficient_a = (150 * 1000000) / (1 << 12);
                    DEBUG("coefficient_a is %" PRIi32 "\n", coefficient_a);
                    val = (int32_t)((int64_t)((int64_t)tws_compl * coefficient_a * 1000) / bw);
                    DEBUG("val is %" PRIi32 "meter\n", val);
                    break;

                case SX128X_RANGING_RESULT_AVERAGED:
                case SX128X_RANGING_RESULT_DEBIASED:
                case SX128X_RANGING_RESULT_FILTERED:
                    val = (double)val_lsb * 20.0 / 100.0;
                    break;

                default:
                    val = 0.0;
            }
            break;

        default:
            break;
    }
    return val;
}

uint8_t sx1280_get_ranging_power_delta_threshold_indicator(const sx128x_t *dev)
{
    sx1280_set_standby(dev, SX128X_STDBY_XOSC);
    sx1280_hal_write_register(dev, 0x97F, sx1280_hal_read_register(dev, 0x97F) | (1 << 1)); // enable LoRa modem clock
    sx1280_hal_write_register(dev, SX128X_REG_LR_RANGINGRESULTCONFIG, (sx1280_hal_read_register(dev, SX128X_REG_LR_RANGINGRESULTCONFIG) & SX128X_MASK_RANGINGMUXSEL) | ((((uint8_t)SX128X_RANGING_RESULT_RAW) & 0x03) << 4)); // Select raw results
    return sx1280_hal_read_register(dev, SX128X_REG_RANGING_RSSI);
}

void sx1280_set_ranging_calibration(const sx128x_t *dev, uint16_t cal)
{
    switch(sx1280_get_packet_type())
    {
        case SX128X_PACKET_TYPE_RANGING:
            sx1280_hal_write_register(dev, SX128X_REG_LR_RANGINGRERXTXDELAYCAL, (uint8_t)((cal >> 8) & 0xFF));
            sx1280_hal_write_register(dev, SX128X_REG_LR_RANGINGRERXTXDELAYCAL + 1, (uint8_t)((cal) & 0xFF));
            break;

        default:
            break;
    }
}

void sx1280_ranging_clear_filter_result(const sx128x_t *dev)
{
    uint8_t reg_val = sx1280_hal_read_register(dev, SX128X_REG_LR_RANGINGRESULTCLEARREG);

    // To clear result, set bit 5 to 1 then to 0
    sx1280_hal_write_register(dev, SX128X_REG_LR_RANGINGRESULTCLEARREG, reg_val | (1 << 5));
    sx1280_hal_write_register(dev, SX128X_REG_LR_RANGINGRESULTCLEARREG, reg_val & (~(1 << 5)));
}

void sx1280_ranging_set_filter_num_samples(const sx128x_t *dev, uint8_t num)
{
    // Silently set 8 as minimum value
    sx1280_hal_write_register(dev, SX128X_REG_LR_RANGINGFILTERWINDOWSIZE, (num < SX128X_DEFAULT_RANGING_FILTER_SIZE) ? SX128X_DEFAULT_RANGING_FILTER_SIZE : num);
}

int8_t sx1280_parse_hex_file_line(const sx128x_t *dev, char *line)
{
    uint16_t addr;
    uint16_t n;
    uint8_t code;
    uint8_t bytes[256];

    if(sx1280_get_hex_file_line_fields(line, bytes, &addr, &n, &code) != 0) {
        if(code == 0) {
            sx1280_hal_write_registers(dev, addr, bytes, n);
        }
        if(code == 1) { 
            // end of file
            //return 2;
        }
        if(code == 2) { 
            // begin of file
            //return 3;
        }
    } else {
        return 0;
    }
    return 1;
}

void sx1280_set_ranging_role(const sx128x_t *dev, sx128x_radio_ranging_roles_t role)
{
    uint8_t buf[1];

    buf[0] = role;
    sx1280_hal_write_command(dev, SX128X_RADIO_SET_RANGING_ROLE, &buf[0], 1);
}

int8_t sx1280_get_hex_file_line_fields(char *line, uint8_t *bytes, uint16_t *addr, uint16_t *num, uint8_t *code)
{
    uint16_t sum = 0;
    uint16_t len = 0;
    uint16_t cksum = 0;
    char *ptr;

    *num = 0;
    if(line[0] != ':') {
        return 0;
    }
    if(strlen(line) < 11) {
        return 0;
    }
    ptr = line + 1;
    if(!sscanf( ptr, "%02hx", &len)) {
        return 0;
    }
    ptr += 2;
    if((uint16_t)strlen(line) < (11 + (len * 2))) {
        return 0;
    }
    if(!sscanf(ptr, "%04hx", addr)) {
        return 0;
    }
    ptr += 4;
    if(!sscanf(ptr, "%02hhx", code)) {
        return 0;
    }
    ptr += 2;
    sum = (len & 255) + ((*addr >> 8) & 255) + (*addr & 255) + ((*code >> 8) & 255) + (*code & 255);
    while(*num != len) {
        if(!sscanf(ptr, "%02hhx", &bytes[*num])) {
            return 0;
        }
        ptr += 2;
        sum += bytes[*num] & 255;
        (*num)++;
        if(*num >= 256) {
            return 0;
        }
    }
    if(!sscanf(ptr, "%02hx", &cksum)) {
        return 0;
    }
    if(((sum & 255) + (cksum & 255)) & 255) {
        return 0; // checksum error
    }

    return 1;
}

int32_t sx1280_get_frequency_error(const sx128x_t *dev)
{
    uint8_t efe_raw[3] = {0};
    uint32_t efe = 0;
    int32_t efe_hz = 0;

    switch( sx1280_get_packet_type())
    {
        case SX128X_PACKET_TYPE_LORA:
        case SX128X_PACKET_TYPE_RANGING:
            efe_raw[0] = sx1280_hal_read_register(dev, SX128X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB);
            efe_raw[1] = sx1280_hal_read_register(dev, SX128X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 1);
            efe_raw[2] = sx1280_hal_read_register(dev, SX128X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MSB + 2);
            efe = (efe_raw[0] << 16) | (efe_raw[1] << 8) | efe_raw[2];
            efe &= SX128X_REG_LR_ESTIMATED_FREQUENCY_ERROR_MASK;

            // FrequencyError[Hz] = 1.55 x ( SignedFeiReading / 1600 / BW[kHz])
            DEBUG("Signed Fei Reading %08" PRIx32 "\n", efe);

            efe_hz = sx1280_complement2(efe, 20) * (sx1280_get_lora_bandwidth() / (1600 * 1000));
            DEBUG("Signed Fei Reading %08" PRIx32 "Hz\n", efe_hz);

            efe_hz = (1550 * efe_hz) / 1000;
            break;

        case SX128X_PACKET_TYPE_NONE:
        case SX128X_PACKET_TYPE_BLE:
        case SX128X_PACKET_TYPE_FLRC:
        case SX128X_PACKET_TYPE_GFSK:
            break;
    }

    return efe_hz;
}

void sx1280_set_polling_mode(void)
{
    polling_mode = true;
}

int32_t sx1280_get_lora_bandwidth(void)
{
    int32_t bw_value = 0;

    switch(lora_bandwidth)
    {
        case SX128X_LORA_BW_0200:
            bw_value = 203125;
            break;

        case SX128X_LORA_BW_0400:
            bw_value = 406250;
            break;

        case SX128X_LORA_BW_0800:
            bw_value = 812500;
            break;

        case SX128X_LORA_BW_1600:
            bw_value = 1625000;
            break;

        default:
            bw_value = 0;
    }
    return bw_value;
}

int32_t sx1280_get_ranging_correction_per_sf_bw_gain(const sx128x_radio_lora_spreading_factors_t sf, 
                                                     const sx128x_radio_lora_bandwidths_t bw, 
                                                     const int8_t gain)
{
    uint8_t sf_index = 0;
    uint8_t bw_index = 0;
    
    switch(sf){
        case SX128X_LORA_SF5:
            sf_index = 0;
            break;
        case SX128X_LORA_SF6:
            sf_index = 1;
            break;
        case SX128X_LORA_SF7:
            sf_index = 2;
            break;
        case SX128X_LORA_SF8:
            sf_index = 3;
            break;
        case SX128X_LORA_SF9:
            sf_index = 4;
            break;
        case SX128X_LORA_SF10:
            sf_index = 5;
            break;
        case SX128X_LORA_SF11:
            sf_index = 6;
            break;
        case SX128X_LORA_SF12:
            sf_index = 7;
            break;
    }
    switch(bw){
        case SX128X_LORA_BW_0400:
            bw_index = 0;
            break;
        case SX128X_LORA_BW_0800:
            bw_index = 1;
            break;
        case SX128X_LORA_BW_1600:
            bw_index = 2;
            break;
        default:
            break;
    }
    
    int32_t correction = ranging_correction_per_sf_bw_gain[sf_index][bw_index][gain];
    return correction;
}

double sx1280_compute_ranging_correction_polynome(const sx128x_radio_lora_spreading_factors_t sf, const sx128x_radio_lora_bandwidths_t bw, const double median)
{
    uint8_t sf_index = 0;
    uint8_t bw_index = 0;

    switch(sf){
        case SX128X_LORA_SF5:
            sf_index = 0;
            break;
        case SX128X_LORA_SF6:
            sf_index = 1;
            break;
        case SX128X_LORA_SF7:
            sf_index = 2;
            break;
        case SX128X_LORA_SF8:
            sf_index = 3;
            break;
        case SX128X_LORA_SF9:
            sf_index = 4;
            break;
        case SX128X_LORA_SF10:
            sf_index = 5;
            break;
        case SX128X_LORA_SF11:
            sf_index = 6;
            break;
        case SX128X_LORA_SF12:
            sf_index = 7;
            break;
    }
    switch(bw){
        case SX128X_LORA_BW_0400:
            bw_index = 0;
            break;
        case SX128X_LORA_BW_0800:
            bw_index = 1;
            break;
        case SX128X_LORA_BW_1600:
            bw_index = 2;
            break;
        default:
            break;
    }
    const sx128x_ranging_correction_polynomes_t *polynome = ranging_correction_polynomes_per_sf_bw[sf_index][bw_index];
    double corrected_value = 0.0;
    double correction_coeff = 0;
    for(uint8_t order = 0; order < polynome->order; order++){
        correction_coeff = polynome->coefficients[order] * pow(median, polynome->order - order - 1);
        corrected_value += correction_coeff;
    }
    return corrected_value;
}

void sx1280_set_interrupt_mode(void)
{
    polling_mode = false;
}

void sx1280_on_dio_irq(void *arg)
{

    sx128x_t *dev = (sx128x_t *)arg;
    /*
     * When polling mode is activated, it is up to the application to call
     * process_irqs( ). Otherwise, the driver automatically calls process_irqs( )
     * on radio interrupt.
     */
    if(polling_mode == true) {
        irq_state = true;
    } else {
        if (dev->cb) {
            dev->cb(dev->arg);
        }
    }
}

void sx1280_process_irqs(const sx128x_t *dev)
{
    sx128x_radio_packet_types_t packet_type = SX128X_PACKET_TYPE_NONE;

    if(sx1280_get_opmode() == SX128X_MODE_SLEEP) {
        return; // DIO glitch on V2b :-)
    }

    if(polling_mode == true) {
        if(irq_state == true) {
            __disable_irq();
            irq_state = false;
            __enable_irq();
        } else {
            return;
        }
    }

    packet_type = sx1280_get_packet_type();
    uint16_t irq_regs = sx1280_get_irq_status(dev);
    sx1280_clear_irq_status(dev, SX128X_IRQ_RADIO_ALL);

    switch(packet_type)
    {
        case SX128X_PACKET_TYPE_GFSK:
        case SX128X_PACKET_TYPE_FLRC:
        case SX128X_PACKET_TYPE_BLE:
            switch(operating_mode)
            {
                case SX128X_MODE_RX:
                    if((irq_regs & SX128X_IRQ_RX_DONE) == SX128X_IRQ_RX_DONE) {
                        if((irq_regs & SX128X_IRQ_CRC_ERROR) == SX128X_IRQ_CRC_ERROR) {
                            if((radio_callbacks != NULL) && (radio_callbacks->rx_error != NULL)) {
                                radio_callbacks->rx_error(SX128X_IRQ_CRC_ERROR_CODE);
                            }
                        } else if((irq_regs & SX128X_IRQ_SYNCWORD_ERROR) == SX128X_IRQ_SYNCWORD_ERROR) {
                            if((radio_callbacks != NULL) && (radio_callbacks->rx_error != NULL)) {
                                radio_callbacks->rx_error(SX128X_IRQ_SYNCWORD_ERROR_CODE);
                            }
                        } else {
                            if((radio_callbacks != NULL) && (radio_callbacks->rx_done != NULL)) {
                                radio_callbacks->rx_done();
                            }
                        }
                    }
                    if((irq_regs & SX128X_IRQ_SYNCWORD_VALID) == SX128X_IRQ_SYNCWORD_VALID) {
                        if((radio_callbacks != NULL) && (radio_callbacks->rx_sync_word_done != NULL)) {
                            radio_callbacks->rx_sync_word_done();
                        }
                    }
                    if((irq_regs & SX128X_IRQ_SYNCWORD_ERROR) == SX128X_IRQ_SYNCWORD_ERROR) {
                        if((radio_callbacks != NULL) && (radio_callbacks->rx_error != NULL)) {
                            radio_callbacks->rx_error(SX128X_IRQ_SYNCWORD_ERROR_CODE);
                        }
                    }
                    if((irq_regs & SX128X_IRQ_RX_TX_TIMEOUT) == SX128X_IRQ_RX_TX_TIMEOUT) {
                        if((radio_callbacks != NULL) && (radio_callbacks->rx_timeout != NULL)) {
                            radio_callbacks->rx_timeout();
                        }
                    }
                    if((irq_regs & SX128X_IRQ_TX_DONE) == SX128X_IRQ_TX_DONE) {
                        if((radio_callbacks != NULL) && (radio_callbacks->tx_done != NULL)) {
                            radio_callbacks->tx_done();
                        }
                    }
                    break;
                case SX128X_MODE_TX:
                    if((irq_regs & SX128X_IRQ_TX_DONE) == SX128X_IRQ_TX_DONE) {
                        if((radio_callbacks != NULL) && (radio_callbacks->tx_done != NULL)) {
                            radio_callbacks->tx_done( );
                        }
                    }
                    if((irq_regs & SX128X_IRQ_RX_TX_TIMEOUT) == SX128X_IRQ_RX_TX_TIMEOUT) {
                        if((radio_callbacks != NULL) && (radio_callbacks->tx_timeout != NULL)) {
                            radio_callbacks->tx_timeout();
                        }
                    }
                    break;
                default:
                    // Unexpected IRQ: silently returns
                    break;
            }
            break;
        case SX128X_PACKET_TYPE_LORA:
            switch(operating_mode)
            {
                case SX128X_MODE_RX:
                    lptimer_remove(&op_timeout_timer);
                    if((irq_regs & SX128X_IRQ_RX_DONE) == SX128X_IRQ_RX_DONE) {
                        if((irq_regs & SX128X_IRQ_CRC_ERROR) == SX128X_IRQ_CRC_ERROR) {
                            if((radio_callbacks != NULL) && (radio_callbacks->rx_error != NULL)) {
                                radio_callbacks->rx_error(SX128X_IRQ_CRC_ERROR_CODE);
                            }
                        } else {
                            if((radio_callbacks != NULL) && (radio_callbacks->rx_done != NULL)) {
                                radio_callbacks->rx_done();
                            }
                        }
                    }
                    if((irq_regs & SX128X_IRQ_HEADER_VALID) == SX128X_IRQ_HEADER_VALID) {
                        if((radio_callbacks != NULL) && (radio_callbacks->rx_header_done != NULL)) {
                            radio_callbacks->rx_header_done();
                        }
                    }
                    if((irq_regs & SX128X_IRQ_HEADER_ERROR) == SX128X_IRQ_HEADER_ERROR) {
                        if((radio_callbacks != NULL) && (radio_callbacks->rx_error != NULL)) {
                            radio_callbacks->rx_error( SX128X_IRQ_HEADER_ERROR_CODE );
                        }
                    }
                    if((irq_regs & SX128X_IRQ_RX_TX_TIMEOUT) == SX128X_IRQ_RX_TX_TIMEOUT) {
                        if((radio_callbacks != NULL) && (radio_callbacks->rx_timeout != NULL)) {
                            radio_callbacks->rx_timeout();
                        }
                    }
                    if((irq_regs & SX128X_IRQ_RANGING_SLAVE_REQUEST_DISCARDED) == SX128X_IRQ_RANGING_SLAVE_REQUEST_DISCARDED) {
                        if((radio_callbacks != NULL) && (radio_callbacks->rx_error != NULL)) {
                            radio_callbacks->rx_error(SX128X_IRQ_RANGING_ON_LORA_ERROR_CODE);
                        }
                    }
                    if((irq_regs & SX128X_IRQ_TX_DONE) == SX128X_IRQ_TX_DONE) {
                        if((radio_callbacks != NULL) && (radio_callbacks->tx_done != NULL)) {
                            radio_callbacks->tx_done();
                        }
                    }
                    break;
                case SX128X_MODE_TX:
                    lptimer_remove(&op_timeout_timer);
                    if((irq_regs & SX128X_IRQ_TX_DONE) == SX128X_IRQ_TX_DONE) {
                        if((radio_callbacks != NULL) && (radio_callbacks->tx_done != NULL)) {
                            radio_callbacks->tx_done( );
                        }
                    }
                    if((irq_regs & SX128X_IRQ_RX_TX_TIMEOUT) == SX128X_IRQ_RX_TX_TIMEOUT) {
                        if((radio_callbacks != NULL) && (radio_callbacks->tx_timeout != NULL)) {
                            radio_callbacks->tx_timeout();
                        }
                    }
                    break;
                case SX128X_MODE_CAD:
                    lptimer_remove(&op_timeout_timer);
                    if((irq_regs & SX128X_IRQ_CAD_DONE) == SX128X_IRQ_CAD_DONE) {
                        if((irq_regs & SX128X_IRQ_CAD_ACTIVITY_DETECTED) == SX128X_IRQ_CAD_ACTIVITY_DETECTED) {
                            if((radio_callbacks != NULL) && (radio_callbacks->cad_done != NULL)) {
                                radio_callbacks->cad_done(true);
                            }
                        } else {
                            if((radio_callbacks != NULL) && (radio_callbacks->cad_done != NULL)) {
                                radio_callbacks->cad_done(false);
                            }
                        }
                    } else if((irq_regs & SX128X_IRQ_RX_TX_TIMEOUT) == SX128X_IRQ_RX_TX_TIMEOUT) {
                        if((radio_callbacks != NULL) && (radio_callbacks->rx_timeout != NULL)) {
                            radio_callbacks->rx_timeout();
                        }
                    }
                    break;
                default:
                    // Unexpected IRQ: silently returns
                    break;
            }
            break;
        case SX128X_PACKET_TYPE_RANGING:
            switch( operating_mode )
            {
                // SX128X_MODE_RX indicates an IRQ on the Slave side
                case SX128X_MODE_RX:
                    lptimer_remove(&op_timeout_timer);
                    if((irq_regs & SX128X_IRQ_RANGING_SLAVE_REQUEST_DISCARDED) == SX128X_IRQ_RANGING_SLAVE_REQUEST_DISCARDED) {
                        if((radio_callbacks != NULL) && (radio_callbacks->ranging_done != NULL)) {
                            radio_callbacks->ranging_done(SX128X_IRQ_RANGING_SLAVE_ERROR_DISCARDED);
                        }
                    }
                    if((irq_regs & SX128X_IRQ_RANGING_SLAVE_REQUEST_VALID) == SX128X_IRQ_RANGING_SLAVE_REQUEST_VALID) {
                        if((radio_callbacks != NULL) && (radio_callbacks->ranging_done != NULL)) {
                            radio_callbacks->ranging_done(SX128X_IRQ_RANGING_SLAVE_VALID_CODE);
                        }
                    }
                    if((irq_regs & SX128X_IRQ_RANGING_SLAVE_RESPONSE_DONE) == SX128X_IRQ_RANGING_SLAVE_RESPONSE_DONE) {
                        if((radio_callbacks != NULL) && (radio_callbacks->ranging_done != NULL)) {
                            radio_callbacks->ranging_done(SX128X_IRQ_RANGING_SLAVE_VALID_CODE);
                        }
                    }
                    if((irq_regs & SX128X_IRQ_RX_TX_TIMEOUT) == SX128X_IRQ_RX_TX_TIMEOUT) {
                        if((radio_callbacks != NULL) && (radio_callbacks->ranging_done != NULL)) {
                            radio_callbacks->ranging_done(SX128X_IRQ_RANGING_SLAVE_ERROR_TIMEOUT);
                        }
                    }
                    if((irq_regs & SX128X_IRQ_HEADER_VALID) == SX128X_IRQ_HEADER_VALID) {
                        if((radio_callbacks != NULL) && (radio_callbacks->rx_header_done != NULL)) {
                            radio_callbacks->rx_header_done();
                        }
                    }
                    if((irq_regs & SX128X_IRQ_HEADER_ERROR) == SX128X_IRQ_HEADER_ERROR) {
                        if((radio_callbacks != NULL) && (radio_callbacks->rx_error != NULL)) {
                            radio_callbacks->rx_error(SX128X_IRQ_HEADER_ERROR_CODE);
                        }
                    }
                    break;
                // SX128X_MODE_TX indicates an IRQ on the Master side
                case SX128X_MODE_TX:
                    lptimer_remove(&op_timeout_timer);
                    if((irq_regs & SX128X_IRQ_RANGING_MASTER_RESULT_TIMEOUT) == SX128X_IRQ_RANGING_MASTER_RESULT_TIMEOUT) {
                        if((radio_callbacks != NULL) && (radio_callbacks->ranging_done != NULL)) {
                            radio_callbacks->ranging_done( SX128X_IRQ_RANGING_MASTER_ERROR_CODE );
                        }
                    }
                    if((irq_regs & SX128X_IRQ_RANGING_MASTER_RESULT_VALID) == SX128X_IRQ_RANGING_MASTER_RESULT_VALID) {
                        if((radio_callbacks != NULL) && (radio_callbacks->ranging_done != NULL)) {
                            radio_callbacks->ranging_done(SX128X_IRQ_RANGING_MASTER_VALID_CODE);
                        }
                    }
                    break;
                default:
                    // Unexpected IRQ: silently returns
                    break;
            }
            break;
        default:
            // Unexpected IRQ: silently returns
            break;
    }
}
