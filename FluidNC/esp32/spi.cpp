// Copyright (c) 2022 Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Driver/spi.h"

#include <SPI.h>
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "src/Logging.h"

bool spi_init_bus(pinnum_t sck_pin, pinnum_t miso_pin, pinnum_t mosi_pin) {
    // Start the SPI bus with the pins defined here.  Once it has been started,
    // those pins "stick" and subsequent attempts to restart it with defaults
    // for the miso, mosi, and sck pins are ignored
    SPI.begin(sck_pin, miso_pin, mosi_pin);  // CS is defined by each device

    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = mosi_pin,
        .miso_io_num     = miso_pin,
        .sclk_io_num     = sck_pin,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4000,
    };

    // Depends on the chip variant
#define SPI_DMA_CHAN 1
    esp_err_t err = spi_bus_initialize(HSPI_HOST, &bus_cfg, 1);
    if (err) {
        log_debug("spi_bus_initialize failed: " << esp_err_to_name(err));
        return true;
    }
    return false;
}

void spi_deinit_bus() {
    spi_bus_free(HSPI_HOST);
    SPI.end();
}

void spi_unregister_device(spidev_t handle) {
    spi_bus_remove_device(spi_device_handle_t(handle));
}

spidev_t spi_register_device(pinnum_t cs_pin, uint8_t spi_mode, int hz) {
    esp_err_t           err;
    spi_device_handle_t handle;

    spi_device_interface_config_t dev_config = {
        .command_bits     = 0,
        .address_bits     = 0,
        .dummy_bits       = 0,
        .mode             = spi_mode,
        .duty_cycle_pos   = 0,
        .cs_ena_pretrans  = 0,
        .cs_ena_posttrans = 0,
        .clock_speed_hz   = hz,  ///< Clock speed, divisors of 80MHz, in Hz. See ``SPI_MASTER_FREQ_*``.
        .input_delay_ns   = 0,
        .spics_io_num     = cs_pin,
        .flags            = 0,
        .queue_size       = 1,
        .pre_cb           = NULL,
        .post_cb          = NULL,
    };

    err = spi_bus_add_device(HSPI_HOST, &dev_config, &handle);

    if (err) {
        log_error("Cannot register SPI device");
        return spidev_t(NULL);
    }
    return spidev_t(handle);
}

bool spi_transfer(spidev_t devid, uint8_t* outbuf, uint8_t* inbuf, size_t len) {
    spi_transaction_t transaction = {
        .flags     = 0,
        .cmd       = 0,
        .addr      = 0,
        .length    = len * 8,  // Data length in bits
        .rxlength  = 0,        // Rx data length in bits, 0 means same as length
        .user      = 0,
        .tx_buffer = outbuf,
        .rx_buffer = inbuf,
    };
    esp_err_t err = spi_device_polling_transmit(spi_device_handle_t(devid), &transaction);
    return err != ESP_OK;
}
bool spi_send_cmd_addr(spidev_t devid, uint16_t cmd, uint8_t cmdbits, uint32_t addr, uint8_t addrbits, uint8_t dummybits) {
    spi_transaction_ext_t transaction {
        .base {
            .flags     = SPI_TRANS_VARIABLE_CMD | SPI_TRANS_VARIABLE_ADDR | SPI_TRANS_VARIABLE_DUMMY,
            .cmd       = cmd,
            .addr      = addr,
            .length    = 0,
            .rxlength  = 0,
            .user      = 0,
            .tx_buffer = 0,
            .rx_buffer = NULL,
        },
        .command_bits = cmdbits,
        .address_bits = addrbits,
        .dummy_bits   = dummybits,
    };
    esp_err_t err = spi_device_polling_transmit(spi_device_handle_t(devid), (spi_transaction_t*)&transaction);
    if (err) {
        log_debug("spi_send failed: " << esp_err_to_name(err));
    }
    return err;
}
bool spi_receive(spidev_t devid, uint8_t dummybits, uint8_t* inbuf, size_t inbits) {
    spi_transaction_ext_t transaction = {
        .base {
            .flags     = 0,
            .cmd       = 0,
            .addr      = 0,
            .length    = inbits,  // Data length in bits
            .rxlength  = inbits,
            .user      = 0,
            .tx_buffer = NULL,
            .rx_buffer = inbuf,
        },
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits   = dummybits,
    };
    esp_err_t err = spi_device_polling_transmit(spi_device_handle_t(devid), (spi_transaction_t*)&transaction);
    if (err) {
        log_debug("spi_receive failed: " << esp_err_to_name(err));
    }
    return err;
}
