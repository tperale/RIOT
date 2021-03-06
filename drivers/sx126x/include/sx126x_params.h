/*
 * Copyright (C) 2021 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx126x
 *
 * @{
 * @file
 * @brief       Default configuration
 *
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 */

#ifndef SX126X_PARAMS_H
#define SX126X_PARAMS_H

#include "board.h"
#include "sx126x.h"
#include "sx126x_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @name    Set default configuration parameters
 *
 * Default values are adapted for mbed shield used with to nucleo64 boards
 * @{
 */
#ifndef SX126X_PARAM_SPI
#define SX126X_PARAM_SPI                    SPI_DEV(0)
#endif

#ifndef SX126X_PARAM_SPI_NSS
#define SX126X_PARAM_SPI_NSS                GPIO_PIN(0, 8)  /* D7 */
#endif

#ifndef SX126X_PARAM_RESET
#define SX126X_PARAM_RESET                  GPIO_PIN(0, 0)  /* A0 */
#endif

#ifndef SX126X_PARAM_BUSY
#define SX126X_PARAM_BUSY                   GPIO_PIN(1, 3)  /* D3 */
#endif

#ifndef SX126X_PARAM_DIO1
#define SX126X_PARAM_DIO1                   GPIO_PIN(1, 4)  /* D5 */
#endif

#ifndef SX126X_PARAM_REGULATOR
#define SX126X_PARAM_REGULATOR              SX126X_REG_MODE_DCDC
#endif

#define SX126X_PARAMS             { .spi = SX126X_PARAM_SPI,      \
                                    .nss_pin = SX126X_PARAM_SPI_NSS,  \
                                    .reset_pin = SX126X_PARAM_RESET,    \
                                    .busy_pin = SX126X_PARAM_BUSY,      \
                                    .dio1_pin = SX126X_PARAM_DIO1,      \
                                    .regulator = SX126X_PARAM_REGULATOR }
/**@}*/

/**
 * @brief   Configuration struct
 */
static const sx126x_params_t sx126x_params[] =
{
    SX126X_PARAMS
};

#ifdef __cplusplus
}
#endif

#endif /* SX126X_PARAMS_H */
/** @} */
