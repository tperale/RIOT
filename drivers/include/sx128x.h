/*
 * Copyright (C) 2016 Unwired Devices <info@unwds.com>
 *               2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @defgroup    drivers_sx128x Semtech SX1272 and SX1276 radios driver
 * @ingroup     drivers_netdev
 * @brief       Driver for Semtech SX1272 and SX1276 radios.
 *
 * This module contains the driver for radio devices of the Semtech sx128x
 * series (SX1272 and SX1276).
 * Only LoRa long range modem is supported at the moment.
 *
 * @{
 *
 * @file
 * @brief       Public interface for SX128X driver
 * @author      Eugene P. <ep@unwds.com>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

#ifndef SX128X_H
#define SX128X_H

#include "timex.h"
#include "ztimer.h"
#include "net/netdev.h"
#include "periph/gpio.h"
#include "periph/spi.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    SX128X device default configuration
 * @{
 */
#define SX128X_PACKET_TYPE_DEFAULT       (SX128X_PACKET_TYPE_LORA) /**< Use LoRa as default packet type */
#define SX128X_CHANNEL_DEFAULT           (2400UL)                  /**< Default channel frequency, 868.3MHz (Europe) */
#define SX128X_XTAL_FREQ                 (32000000UL)              /**< Internal oscillator frequency, 32MHz */
#define SX128X_RADIO_WAKEUP_TIME         (1U)                      /**< In milliseconds [ms] */

#define SX128X_CRYSTAL_FREQ (52.0)
#define SX128X_DIV_EXPONANT (18)

#define SX128X_TX_TIMEOUT_DEFAULT        (30 * MS_PER_SEC)      /**< TX timeout, 30s */
#define SX128X_RX_SINGLE                 (false)                /**< Single byte receive mode => continuous by default */
#define SX128X_RX_BUFFER_SIZE            (256)                  /**< RX buffer size */
#define SX128X_RADIO_TX_POWER            (12U)                  /**< Radio power in dBm */
#define SX128X_RADIO_TX_RAMP_TIME        (SX128X_TX_RADIO_RAMP_02_US) /**< Power amplifier ramp time */


#define SX128X_EVENT_HANDLER_STACK_SIZE  (2048U) /**< Stack size event handler */
#define SX128X_IRQ_DIO0                  (1<<0)  /**< DIO0 IRQ */
#define SX128X_IRQ_DIO1                  (1<<1)  /**< DIO1 IRQ */
#define SX128X_IRQ_DIO2                  (1<<2)  /**< DIO2 IRQ */
#define SX128X_IRQ_DIO3                  (1<<3)  /**< DIO3 IRQ */
#define SX128X_IRQ_DIO4                  (1<<4)  /**< DIO4 IRQ */
#define SX128X_IRQ_DIO5                  (1<<5)  /**< DIO5 IRQ */
/** @} */

/**
 * @defgroup drivers_sx128x_config     Semtech SX1272 and SX1276 driver compile configuration
 * @ingroup config_drivers_netdev
 * @{
 */
/**
 * @brief   GPIO mode of DIOx Pins.
 */
#ifndef SX128X_DIO_PULL_MODE
#define SX128X_DIO_PULL_MODE             (GPIO_IN_PD)
#endif
/** @} */

/**
 * @brief   SX128X initialization result.
 */
enum {
    SX128X_INIT_OK = 0,                /**< Initialization was successful */
    SX128X_ERR_SPI,                    /**< Failed to initialize SPI bus or CS line */
    SX128X_ERR_GPIOS,                  /**< Failed to initialize GPIOs */
    SX128X_ERR_NODEV                   /**< No valid device version found */
};

/**
 * @brief   Radio driver supported modems.
 */
enum {
    SX128X_PACKET_TYPE_GFSK = 0,       /**< FSK modem driver */
    SX128X_PACKET_TYPE_LORA,           /**< LoRa modem driver */
    SX128X_PACKET_TYPE_RANGING,        /**< Ranging modem driver */
    SX128X_PACKET_TYPE_FLRC,           /**< FLRC modem driver */
    SX128X_PACKET_TYPE_BLE,            /**< BLE modem driver */
};

/**
 * @brief   Radio driver internal state machine states definition.
 */
enum {
    SX128X_RF_IDLE = 0,                /**< Idle state */
    SX128X_RF_RX_RUNNING,              /**< Sending state */
    SX128X_RF_TX_RUNNING,              /**< Receiving state */
    SX128X_RF_CAD,                     /**< Channel activity detection state */
};

/**
 * @brief   Event types.
 */
enum {
    SX128X_RX_DONE = 0,                /**< Receiving complete */
    SX128X_TX_DONE,                    /**< Sending complete*/
    SX128X_RX_TIMEOUT,                 /**< Receiving timeout */
    SX128X_TX_TIMEOUT,                 /**< Sending timeout */
    SX128X_RX_ERROR_CRC,               /**< Receiving CRC error */
    SX128X_FHSS_CHANGE_CHANNEL,        /**< Channel change */
    SX128X_CAD_DONE,                   /**< Channel activity detection complete */
};

/**
 * @brief Power amplifier modes
 *
 * Default value is SX128X_PA_RFO.
 *
 * The power amplifier mode depends on the module hardware configuration.
 */
enum {
    SX128X_PA_RFO = 0,                 /**< RFO HF or RFO LF */
    SX128X_PA_BOOST,                   /**< Power amplifier boost (high power) */
};

/**
 * @name    SX128X device descriptor boolean flags
 * @{
 */
#define SX128X_LOW_DATARATE_OPTIMIZE_FLAG       (1 << 0)
#define SX128X_ENABLE_FIXED_HEADER_LENGTH_FLAG  (1 << 1)
#define SX128X_ENABLE_CRC_FLAG                  (1 << 2)
#define SX128X_CHANNEL_HOPPING_FLAG             (1 << 3)
#define SX128X_IQ_INVERTED_FLAG                 (1 << 4)
#define SX128X_RX_CONTINUOUS_FLAG               (1 << 5)
/** @} */

/**
 * @brief   LoRa configuration structure.
 */
typedef struct {
    uint16_t preamble_len;             /**< Length of preamble header */
    int8_t power;                      /**< Signal power */
    uint8_t bandwidth;                 /**< Signal bandwidth */
    uint8_t datarate;                  /**< Spreading factor rate, e.g datarate */
    uint8_t coderate;                  /**< Error coding rate */
    uint8_t freq_hop_period;           /**< Frequency hop period */
    uint8_t flags;                     /**< Boolean flags */
    uint32_t rx_timeout;               /**< RX timeout in milliseconds */
    uint32_t tx_timeout;               /**< TX timeout in milliseconds */
} sx128x_lora_settings_t;

/**
 * @brief   Radio settings.
 */
typedef struct {
    uint32_t channel;                  /**< Radio channel */
    uint8_t state;                     /**< Radio state */
    uint8_t modem;                     /**< Driver model (FSK or LoRa) */
    sx128x_lora_settings_t lora;       /**< LoRa settings */
} sx128x_radio_settings_t;

/**
 * @brief   SX128X internal data.
 */
typedef struct {
    /* Data that will be passed to events handler in application */
    ztimer_t tx_timeout_timer;         /**< TX operation timeout timer */
    ztimer_t rx_timeout_timer;         /**< RX operation timeout timer */
    uint32_t last_channel;             /**< Last channel in frequency hopping sequence */
    bool is_last_cad_success;          /**< Sign of success of last CAD operation (activity detected) */
} sx128x_internal_t;

/**
 * @brief   SX128X hardware and global parameters.
 */
typedef struct {
    spi_t spi;                         /**< SPI device */
    gpio_t nss_pin;                    /**< SPI NSS pin */
    gpio_t reset_pin;                  /**< Reset pin */
    gpio_t busy_pin;                   /**< Interrupt line busy */
    gpio_t dio1_pin;                   /**< Interrupt line DIO1 (Tx done, Rx timeout) */
    gpio_t dio2_pin;                   /**< Interrupt line DIO2 (FHSS channel change) */
    gpio_t dio3_pin;                   /**< Interrupt line DIO3 (CAD done) */
#if defined(SX128X_USE_TX_SWITCH) || defined(SX128X_USE_RX_SWITCH)
    gpio_t rx_switch_pin;              /**< Rx antenna switch */
    gpio_t tx_switch_pin;              /**< Tx antenna switch */
#endif
    uint8_t paselect;                  /**< Power amplifier mode (RFO or PABOOST) */
} sx128x_params_t;

/**
 * @brief   SX128X IRQ flags.
 */
typedef uint8_t sx128x_flags_t;

/**
 * @brief   SX128X device descriptor.
 * @extends netdev_t
 */
typedef struct {
    netdev_t netdev;                   /**< Netdev parent struct */
    sx128x_radio_settings_t settings;  /**< Radio settings */
    sx128x_params_t params;            /**< Device driver parameters */
    sx128x_internal_t _internal;       /**< Internal sx128x data used within the driver */
    sx128x_flags_t irq;                /**< Device IRQ flags */
} sx128x_t;

/**
 * @brief   Hardware IO IRQ callback function definition.
 */
typedef void (sx128x_dio_irq_handler_t)(sx128x_t *dev);

/**
 * @brief   Setup the SX128X
 *
 * @param[in] dev                      Device descriptor
 * @param[in] params                   Parameters for device initialization
 * @param[in] index                    Index of @p params in a global parameter struct array.
 *                                     If initialized manually, pass a unique identifier instead.
 */
void sx128x_setup(sx128x_t *dev, const sx128x_params_t *params, uint8_t index);

/**
 * @brief   Resets the SX128X
 *
 * @param[in] dev                      The sx128x device descriptor
 */
int sx128x_reset(const sx128x_t *dev);

/**
 * @brief   Initializes the transceiver.
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return result of initialization
 */
int sx128x_init(sx128x_t *dev);

/**
 * @brief   Initialize radio settings with default values
 *
 * @param[in] dev                      The sx128x device pointer
 */
void sx128x_init_radio_settings(sx128x_t *dev);

/**
 * @brief   Start a channel activity detection.
 *
 * @param[in] dev                      The sx128x device descriptor
 */
void sx128x_start_cad(sx128x_t *dev);

/**
 * @brief   Checks that channel is free with specified RSSI threshold.
 *
 * @param[in] dev                      The sx128x device structure pointer
 * @param[in] freq                     channel RF frequency
 * @param[in] rssi_threshold           RSSI threshold
 *
 * @return true if channel is free, false otherwise
 */
bool sx128x_is_channel_free(sx128x_t *dev, uint32_t freq, int16_t rssi_threshold);

/**
 * @brief   Reads the current RSSI value.
 *
 * @param[in] dev                      The sx128x device structure pointer
 *
 * @return the current value of RSSI (in dBm)
 */
int16_t sx128x_read_rssi(const sx128x_t *dev);

/**
 * @brief   Gets current state of transceiver.
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return radio state [RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
uint8_t sx128x_get_state(const sx128x_t *dev);

uint16_t sx128x_get_firmware_version(const sx128x_t *dev);

uint8_t sx128x_cmd_get_status(const sx128x_t *dev);

/**
 * @name    SX128X Radio Operation Modes
 * @{
 */
void sx128x_cmd_set_sleep(const sx128x_t *dev, uint8_t config);
void sx128x_cmd_set_standby(const sx128x_t *dev, uint8_t config);
void sx128x_cmd_set_tx(const sx128x_t *dev, uint8_t period_base, uint16_t period_base_count);
void sx128x_cmd_set_rx(const sx128x_t *dev, uint8_t period_base, uint16_t period_base_count);
void sx128x_cmd_set_rx_duty_cycle(const sx128x_t *dev, uint8_t period_base, uint16_t period_base_count, uint16_t sleep_period_base_count);
void sx128x_cmd_set_cad(const sx128x_t *dev);
/** @} */

/**
 * @name    SX128X Radio Configuration Modes
 * @{
 */
void sx128x_cmd_set_packet_type(sx128x_t *dev, uint8_t packet_type);
uint8_t sx128x_cmd_get_packet_type(const sx128x_t *dev);
void sx128x_cmd_set_regulator_mode(sx128x_t *dev, uint8_t mode);
void sx128x_cmd_set_frequency(const sx128x_t *dev, uint32_t freq);
void sx128x_cmd_set_tx_params(const sx128x_t *dev, int8_t power, uint8_t ramp_time);
void sx128x_cmd_set_cad_params(const sx128x_t *dev, uint8_t symbol_num);
void sx128x_cmd_set_buffer_base_address(const sx128x_t *dev, uint8_t tx_address, uint8_t rx_address);
void sx128x_cmd_set_modulation_params(sx128x_t *dev, uint8_t param1, uint8_t param2, uint8_t param3);
void sx128x_cmd_set_packet_params(sx128x_t *dev, uint8_t param1, uint8_t param2, uint8_t param3, uint8_t param4, uint8_t param5, uint8_t param6, uint8_t param7);
/** @} */

/**
 * @name    SX128X Communication status information
 * @{
 */
uint8_t sx128x_cmd_get_rx_buffer_status(const sx128x_t *dev);
uint8_t sx128x_cmd_get_packet_status(const sx128x_t *dev);
uint8_t sx128x_cmd_get_rssi_inst(const sx128x_t *dev);
/** @} */

/**
 * @name    SX128X IRQ Handling
 * @{
 */
void sx128x_cmd_set_dio_irq_params(const sx128x_t *dev, uint16_t dio1_mask, uint16_t dio2_mask, uint16_t dio3_mask);
uint16_t sx128x_cmd_get_irq_status(const sx128x_t *dev);
void sx128x_cmd_clear_irq_status(const sx128x_t *dev, uint16_t irq_mask);
/** @} */


/**
 * @brief   Sets current state of transceiver.
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] state                    The new radio state
 *
 * @return radio state [RF_IDLE, RF_RX_RUNNING, RF_TX_RUNNING]
 */
void sx128x_set_state(sx128x_t *dev, uint8_t state);

/**
 * @brief   Configures the radio with the given modem.
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] modem                    Modem to be used [0: FSK, 1: LoRa]
 */
void sx128x_set_modem(sx128x_t *dev, uint8_t modem);

/**
 * @brief   Gets the synchronization word.
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return The synchronization word
 */
uint8_t sx128x_get_syncword(const sx128x_t *dev);

/**
 * @brief   Sets the synchronization word.
 *
 * @param[in] dev                     The sx128x device descriptor
 * @param[in] syncword                The synchronization word
 */
void sx128x_set_syncword(sx128x_t *dev, uint8_t syncword);

/**
 * @brief   Gets the channel RF frequency.
 *
 * @param[in]  dev                     The sx128x device descriptor
 *
 * @return The channel frequency
 */
uint32_t sx128x_get_channel(const sx128x_t *dev);

/**
 * @brief   Sets the channel RF frequency.
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] freq                     Channel RF frequency
 */
void sx128x_set_channel(sx128x_t *dev, uint32_t freq);

/**
 * @brief   Computes the packet time on air in milliseconds.
 *
 * @pre     Can only be called if sx128x_init_radio_settings has already
 *          been called.
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] pkt_len                  The received packet payload length
 *
 * @return computed air time (ms) for the given packet payload length
 */
uint32_t sx128x_get_time_on_air(const sx128x_t *dev, uint8_t pkt_len);

/**
 * @brief   Sets the radio in sleep mode
 *
 * @param[in] dev                      The sx128x device descriptor
 */
void sx128x_set_sleep(sx128x_t *dev);

/**
 * @brief   Sets the radio in stand-by mode
 *
 * @param[in] dev                      The sx128x device descriptor
 */
void sx128x_set_standby(sx128x_t *dev);

/**
 * @brief   Sets the radio in reception mode.
 *
 * @param[in] dev                      The sx128x device descriptor
 */
void sx128x_set_rx(sx128x_t *dev);

/**
 * @brief   Sets the radio in transmission mode.
 *
 * @param[in] dev                      The sx128x device descriptor
 */
void sx128x_set_tx(sx128x_t *dev);

/**
 * @brief   Gets the maximum payload length.
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return The maximum payload length
 */
uint8_t sx128x_get_max_payload_len(const sx128x_t *dev);

/**
 * @brief   Sets the maximum payload length.
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] maxlen                   Maximum payload length in bytes
 */
void sx128x_set_max_payload_len(const sx128x_t *dev, uint8_t maxlen);

/**
 * @brief   Gets the SX128X operating mode
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return The actual operating mode
 */
uint8_t sx128x_get_op_mode(const sx128x_t *dev);

/**
 * @brief   Sets the SX128X operating mode
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] op_mode                  The new operating mode
 */
void sx128x_set_op_mode(const sx128x_t *dev, uint8_t op_mode);

/**
 * @brief   Gets the SX128X bandwidth
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return the bandwidth
 */
uint8_t sx128x_get_bandwidth(const sx128x_t *dev);

/**
 * @brief   Sets the SX128X bandwidth
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] bandwidth                The new bandwidth
 */
void sx128x_set_bandwidth(sx128x_t *dev, uint8_t bandwidth);

/**
 * @brief   Gets the SX128X LoRa spreading factor
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return the spreading factor
 */
uint8_t sx128x_get_spreading_factor(const sx128x_t *dev);

/**
 * @brief   Sets the SX128X LoRa spreading factor
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] sf                       The spreading factor
 */
void sx128x_set_spreading_factor(sx128x_t *dev, uint8_t sf);

/**
 * @brief   Gets the SX128X LoRa coding rate
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return the current LoRa coding rate
 */
uint8_t sx128x_get_coding_rate(const sx128x_t *dev);

/**
 * @brief   Sets the SX128X LoRa coding rate
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] coderate                 The LoRa coding rate
 */
void sx128x_set_coding_rate(sx128x_t *dev, uint8_t coderate);

/**
 * @brief   Checks if the SX128X LoRa RX single mode is enabled/disabled
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return the LoRa single mode
 */
bool sx128x_get_rx_single(const sx128x_t *dev);

/**
 * @brief   Enable/disable the SX128X LoRa RX single mode
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] single                   The LoRa RX single mode
 */
void sx128x_set_rx_single(sx128x_t *dev, bool single);

/**
 * @brief   Checks if the SX128X CRC verification mode is enabled
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return the LoRa single mode
 */
bool sx128x_get_crc(const sx128x_t *dev);

/**
 * @brief   Enable/Disable the SX128X CRC verification mode
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] crc                      The CRC check mode
 */
void sx128x_set_crc(sx128x_t *dev, bool crc);

/**
 * @brief   Gets the SX128X frequency hopping period
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return the frequency hopping period
 */
uint8_t sx128x_get_hop_period(const sx128x_t *dev);

/**
 * @brief   Sets the SX128X frequency hopping period
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] hop_period               The frequency hopping period
 */
void sx128x_set_hop_period(sx128x_t *dev, uint8_t hop_period);

/**
 * @brief   Gets the SX128X LoRa fixed header length mode
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return the LoRa implicit mode
 */
bool sx128x_get_fixed_header_len_mode(const sx128x_t *dev);

/**
 * @brief   Sets the SX128X to fixed header length mode (explicit mode)
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] mode                     The header mode
 */
void sx128x_set_fixed_header_len_mode(sx128x_t *dev, bool mode);

/**
 * @brief   Gets the SX128X payload length
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return the payload length
 */
uint8_t sx128x_get_payload_length(const sx128x_t *dev);

/**
 * @brief   Sets the SX128X payload length
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] len                      The payload len
 */
void sx128x_set_payload_length(sx128x_t *dev, uint8_t len);

/**
 * @brief   Gets the SX128X TX radio power
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return the radio power
 */
uint8_t sx128x_get_tx_power(const sx128x_t *dev);

/**
 * @brief   Sets the SX128X transmission power
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] power                    The TX power
 */
void sx128x_set_tx_power(sx128x_t *dev, int8_t power);

/**
 * @brief   Gets the SX128X preamble length
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return the preamble length
 */
uint16_t sx128x_get_preamble_length(const sx128x_t *dev);

/**
 * @brief   Sets the SX128X LoRa preamble length
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] preamble                 The LoRa preamble length
 */
void sx128x_set_preamble_length(sx128x_t *dev, uint16_t preamble);

/**
 * @brief   Sets the SX128X LoRa symbol timeout
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] timeout                  The LoRa symbol timeout
 */
void sx128x_set_symbol_timeout(sx128x_t *dev, uint16_t timeout);

/**
 * @brief   Sets the SX128X RX timeout
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] timeout                  The RX timeout
 */
void sx128x_set_rx_timeout(sx128x_t *dev, uint32_t timeout);

/**
 * @brief   Sets the SX128X TX timeout
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] timeout                  The TX timeout
 */
void sx128x_set_tx_timeout(sx128x_t *dev, uint32_t timeout);

/**
 * @brief   Checks if the SX128X LoRa inverted IQ mode is enabled/disabled
 *
 * @param[in] dev                      The sx128x device descriptor
 *
 * @return the LoRa IQ inverted mode
 */
bool sx128x_get_iq_invert(const sx128x_t *dev);

/**
 * @brief   Enable/disable the SX128X LoRa IQ inverted mode
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] iq_invert                The LoRa IQ inverted mode
 */
void sx128x_set_iq_invert(sx128x_t *dev, bool iq_invert);

/**
 * @brief   Sets the SX128X LoRa frequency hopping mode
 *
 * @param[in] dev                      The sx128x device descriptor
 * @param[in] freq_hop_on              The LoRa frequency hopping mode
 */
void sx128x_set_freq_hop(sx128x_t *dev, bool freq_hop_on);

#ifdef __cplusplus
}
#endif

#endif /* SX128X_H */
/** @} */
