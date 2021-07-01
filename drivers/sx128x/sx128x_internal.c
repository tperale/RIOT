/*
 * Copyright (c) 2016 Unwired Devices <info@unwds.com>
 *               2017 Inria Chile
 *               2017 Inria
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sx128x
 * @{
 * @file
 * @brief       implementation of internal functions for sx128x
 *
 * @author      Eugene P. <ep@unwds.com>
 * @author      José Ignacio Alamos <jose.alamos@inria.cl>
 * @author      Alexandre Abadie <alexandre.abadie@inria.fr>
 * @}
 */
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <inttypes.h>

#include "periph/spi.h"
#include "ztimer.h"
#include "xtimer.h"

#include "net/lora.h"

#include "sx128x.h"
#include "sx128x_registers.h"
#include "sx128x_internal.h"
#include "sx128x_params.h"

#define ENABLE_DEBUG 1
#include "debug.h"

#define SX128X_SPI_SPEED    (SPI_CLK_400KHZ)
#define SX128X_SPI_MODE     (SPI_MODE_0)

static void sx128x_wait_busy(const sx128x_t *dev) {
    uint64_t now = xtimer_now_usec64();
    const uint32_t timeout = 20000;
    while (gpio_read(dev->params.busy_pin) > 1) {
        if ((now + timeout) < xtimer_now_usec64()) {
            DEBUG("[sx128x] busy pin timeout\n");
            break;
        }
    }
}

void sx128x_reg_write(const sx128x_t *dev, uint16_t addr, uint8_t data)
{
    sx128x_reg_write_burst(dev, addr, &data, 1);
}

uint8_t sx128x_reg_read(const sx128x_t *dev, uint16_t addr)
{
    return sx128x_reg_read_burst(dev, addr);
}

void sx128x_cmd_burst(const sx128x_t *dev, uint8_t cmd, uint8_t *in, uint8_t in_size, uint8_t *out, uint8_t out_size)
{
    sx128x_wait_busy(dev);

    gpio_clear(dev->params.nss_pin);

    uint8_t ret = spi_transfer_byte(dev->params.spi, SPI_CS_UNDEF, true, cmd);
    if (ret & SX128X_STATUS_COMMAND_STATUS_PROCESSING_ERROR) {
        /* return; */
    }

    for (uint8_t i = 0; i < in_size; i++) {
        spi_transfer_byte(dev->params.spi, SPI_CS_UNDEF, true, in[i]);
    }
    for (uint8_t i = 0; i < out_size; i++) {
        spi_transfer_byte(dev->params.spi, SPI_CS_UNDEF, true, out[i]);
    }

    gpio_set(dev->params.nss_pin);

    sx128x_wait_busy(dev);
}

void sx128x_reg_write_burst(const sx128x_t *dev, uint16_t reg, uint8_t *buffer,
                            uint8_t size)
{
    uint8_t reg_addr[2] = { (reg >> 8) & 0xFF, (reg & 0xFF) };
    /* spi_acquire(dev->params.spi, SPI_CS_UNDEF, SX128X_SPI_MODE, SX128X_SPI_SPEED); */

    gpio_clear(dev->params.nss_pin);
    spi_transfer_byte(dev->params.spi, SPI_CS_UNDEF, true, SX128X_CMD_WRITE_REG);
    spi_transfer_bytes(dev->params.spi, SPI_CS_UNDEF, true, reg_addr, NULL, sizeof(uint16_t));
    spi_transfer_bytes(dev->params.spi, SPI_CS_UNDEF, false, buffer, NULL, size);
    gpio_set(dev->params.nss_pin);

    /* spi_release(dev->params.spi); */
}

uint8_t sx128x_reg_read_burst(const sx128x_t *dev, uint16_t reg)
{
    uint8_t ret;
    uint8_t reg_addr[2] = { (reg >> 8) & 0xFF, (reg & 0xFF) };
    /* spi_acquire(dev->params.spi, SPI_CS_UNDEF, SX128X_SPI_MODE, SX128X_SPI_SPEED); */

    gpio_clear(dev->params.nss_pin);
    spi_transfer_byte(dev->params.spi, SPI_CS_UNDEF, true, SX128X_CMD_READ_REG);
    spi_transfer_bytes(dev->params.spi, SPI_CS_UNDEF, true, reg_addr, NULL, sizeof(uint16_t));
    spi_transfer_bytes(dev->params.spi, SPI_CS_UNDEF, false, NULL, &ret, 1);
    gpio_set(dev->params.nss_pin);

    /* spi_release(dev->params.spi); */
    return ret;
}

void sx128x_write_fifo(const sx128x_t *dev, uint8_t *buffer, uint8_t size)
{
    uint8_t cmd = SX128X_CMD_WRITE_BUF;
    uint8_t offset = 0;

    /* spi_acquire(dev->params.spi, SPI_CS_UNDEF, SX128X_SPI_MODE, SX128X_SPI_SPEED); */

    gpio_clear(dev->params.nss_pin);
    spi_transfer_bytes(dev->params.spi, SPI_CS_UNDEF, true, &cmd, NULL, 1);
    spi_transfer_bytes(dev->params.spi, SPI_CS_UNDEF, true, &offset, NULL, 1);
    spi_transfer_bytes(dev->params.spi, SPI_CS_UNDEF, false, buffer, NULL, size);
    gpio_set(dev->params.nss_pin);

    /* spi_release(dev->params.spi); */
}

void sx128x_read_fifo(const sx128x_t *dev, uint8_t *buffer, uint8_t size)
{
    uint8_t addr[3] = {0, 0, 0};
    sx128x_cmd_burst(dev, SX128X_CMD_READ_BUF, addr, 3, buffer, size);
}
