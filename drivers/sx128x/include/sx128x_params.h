/*
 * Copyright (C) 2017 Inria
 *               2017 Inria Chile
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx128x
 * @{
 * @file
 * @brief       Default configuration for SX128X driver
 *
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

#ifndef SX128X_PARAMS_H
#define SX128X_PARAMS_H

#include "board.h"
#include "sx128x.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters for the SX128X driver
 *          Pins are adapted to STM32 Nucleo-64 boards.
 * @{
 */
#ifndef SX128X_PARAM_SPI
#define SX128X_PARAM_SPI (SPI_DEV(1))
#endif

#ifndef SX128X_PARAM_SPI_NSS
#define SX128X_PARAM_SPI_NSS GPIO_PIN(0, 4) /* D10 */
#endif

#ifndef SX128X_PARAM_RESET
#define SX128X_PARAM_RESET GPIO_PIN(1, 0) /* A0 */
#endif

#ifndef SX128X_PARAM_BUSY
#define SX128X_PARAM_BUSY GPIO_PIN(2, 1) /* D2 */
#endif

#ifndef SX128X_PARAM_DIO1
#define SX128X_PARAM_DIO1 GPIO_PIN(0, 6) /* D3 */
#endif

#ifndef SX128X_PARAM_DIO2
#define SX128X_PARAM_DIO2 GPIO_PIN(1, 5) /* D4 */
#endif

#ifndef SX128X_PARAM_DIO3
#define SX128X_PARAM_DIO3 GPIO_PIN(1, 4) /* D5 */
#endif



/* #ifndef SX128X_PARAM_SPI */
/* #define SX128X_PARAM_SPI (SPI_DEV(0)) */
/* #endif */

/* #ifndef SX128X_PARAM_SPI_NSS */
/* #define SX128X_PARAM_SPI_NSS GPIO_PIN(1, 6) /1* D10 *1/ */
/* #endif */

/* #ifndef SX128X_PARAM_RESET */
/* #define SX128X_PARAM_RESET GPIO_PIN(0, 0) /1* A0 *1/ */
/* #endif */

/* #ifndef SX128X_PARAM_BUSY */
/* #define SX128X_PARAM_BUSY GPIO_PIN(0, 1) /1* D2 *1/ */
/* #endif */

/* #ifndef SX128X_PARAM_DIO1 */
/* #define SX128X_PARAM_DIO1 GPIO_PIN(1, 3) /1* D3 *1/ */
/* #endif */

/* #ifndef SX128X_PARAM_DIO2 */
/* #define SX128X_PARAM_DIO2 GPIO_PIN(1, 5) /1* D4 *1/ */
/* #endif */

/* #ifndef SX128X_PARAM_DIO3 */
/* #define SX128X_PARAM_DIO3 GPIO_PIN(1, 4) /1* D5 *1/ */
/* #endif */

#ifndef SX128X_PARAM_PASELECT
#define SX128X_PARAM_PASELECT (SX128X_PA_RFO)
#endif

#ifndef SX128X_PARAM_TX_SWITCH
#define SX128X_PARAM_TX_SWITCH GPIO_UNDEF
#endif

#ifndef SX128X_PARAM_RX_SWITCH
#define SX128X_PARAM_RX_SWITCH GPIO_UNDEF
#endif

#ifndef SX128X_PARAMS
#if defined(SX128X_USE_TX_SWITCH) || defined(SX128X_USE_RX_SWITCH)
#define SX128X_PARAMS                                                          \
  {                                                                            \
    .spi = SX128X_PARAM_SPI, .nss_pin = SX128X_PARAM_SPI_NSS,                  \
    .reset_pin = SX128X_PARAM_RESET, .busy_pin = SX128X_PARAM_BUSY,            \
    .dio1_pin = SX128X_PARAM_DIO1, .dio2_pin = SX128X_PARAM_DIO2,              \
    .dio3_pin = SX128X_PARAM_DIO3, .rx_switch_pin = SX128X_PARAM_RX_SWITCH,    \
    .tx_switch_pin = SX128X_PARAM_TX_SWITCH, .paselect = SX128X_PARAM_PASELECT \
  }
#else
#define SX128X_PARAMS                                                          \
  {                                                                            \
    .spi = SX128X_PARAM_SPI, .nss_pin = SX128X_PARAM_SPI_NSS,                  \
    .reset_pin = SX128X_PARAM_RESET, .busy_pin = SX128X_PARAM_BUSY,            \
    .dio1_pin = SX128X_PARAM_DIO1, .dio2_pin = SX128X_PARAM_DIO2,              \
    .dio3_pin = SX128X_PARAM_DIO3, .paselect = SX128X_PARAM_PASELECT           \
  }
#endif
#endif
/**@}*/

/**
 * @brief   SX128X configuration
 */
static const sx128x_params_t sx128x_params[] = {SX128X_PARAMS};

#ifdef __cplusplus
}
#endif

#endif /* SX128X_PARAMS_H */
/** @} */
