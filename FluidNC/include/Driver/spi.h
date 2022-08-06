// Copyright (c) 2022 Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "src/Pins/PinDetail.h"  // pinnum_t

bool spi_init_bus(pinnum_t sck_pin, pinnum_t miso_pin, pinnum_t mosi_pin);
void spi_deinit_bus();

typedef void* spidev_t;

// Returns devid or NULL
spidev_t spi_register_device(pinnum_t cs_pin, uint8_t mode, int hz);
void     spi_unregister_device(spidev_t devid);

bool spi_transfer(spidev_t devid, uint8_t* outbuf, uint8_t* inbuf, size_t len);
bool spi_receive(spidev_t devid, uint8_t dummybits, uint8_t* inbuf, size_t inbits);
bool spi_send_cmd_addr(spidev_t devid, uint16_t cmd, uint8_t cmdbits, uint32_t addr, uint8_t addrbits, uint8_t dummybits);
