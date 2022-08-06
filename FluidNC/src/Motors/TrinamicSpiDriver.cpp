// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

/*
    This is used for Trinamic SPI controlled stepper motor drivers.
*/

#include "TrinamicSpiDriver.h"
#include "../Machine/MachineConfig.h"
#include <TMCStepper.h>  // https://github.com/teemuatlut/TMCStepper
#include <atomic>

namespace MotorDrivers {

    pinnum_t TrinamicSpiDriver::_daisy_chain_cs_id = 255;
    spidev_t TrinamicSpiDriver::_daisy_chain_spi_devid;
    uint8_t  TrinamicSpiDriver::_spi_index_mask = 0;
    uint8_t  TrinamicSpiDriver::_spi_index_max  = 0;

    void TrinamicSpiDriver::init() {}

    void TrinamicSpiDriver::afterParse() {
        if (_daisy_chain_cs_id == 255) {
            // Either it is not a daisy chain or this is the first daisy-chained TMC in the config file
            Assert(_cs_pin.defined(), "TMC cs_pin: pin must be configured");

            // This next line might be unnecessary
            _cs_pin.setAttr(Pin::Attr::Output | Pin::Attr::InitialOn);

            _spi_devid = spi_register_device(_cs_pin.getNative((Pin::Capabilities::Output)), 3, 4000000);

            if (_spi_index != -1) {
                // This is the first daisy-chained TMC in the config file
                // Do the cs pin mapping now and record the ID in daisy_chain_cs_id
                _cs_mapping        = PinMapper(_cs_pin);
                _daisy_chain_cs_id = _cs_mapping.pinId();
                set_bitnum(_spi_index_mask, _spi_index);
                _daisy_chain_spi_devid = _spi_devid;

            } else {
                // The TMC SPI is not daisy-chained.  Every such instance uses index 1
                // so the
                _spi_index = 1;
            }
        } else {
            // This is another - not the first - daisy-chained TMC
            Assert(_cs_pin.undefined(), "For daisy-chained TMC, cs_pin: pin must be configured only once");
            Assert(_spi_index != -1, "spi_index: must be configured on all daisy-chained TMCs");
            Assert(bitnum_is_false(_spi_index_mask, _spi_index), "spi_index: must be unique among all daisy-chained TMCs");
            set_bitnum(_spi_index_mask, _spi_index);
            if (_spi_index > _spi_index_max) {
                _spi_index_max = _spi_index;
            }
            _spi_devid = _daisy_chain_spi_devid;
        }
    }

    uint8_t TrinamicSpiDriver::setupSPI() {
        _has_errors = false;

        auto spiConfig = config->_spi;
        Assert(spiConfig && spiConfig->defined(), "SPI bus is not configured. Cannot initialize TMC driver.");

        uint8_t cs_id;
        if (_daisy_chain_cs_id != 255) {
            cs_id = _daisy_chain_cs_id;
        } else {
            _cs_pin.setAttr(Pin::Attr::Output | Pin::Attr::InitialOn);
            _cs_mapping = PinMapper(_cs_pin);
            cs_id       = _cs_mapping.pinId();
        }

        return cs_id;
    }

    void TrinamicSpiDriver::finalInit() {
        _has_errors = false;

        link = List;
        List = this;

        // Display the stepper library version message once, before the first
        // TMC config message.  Link is NULL for the first TMC instance.
        if (!link) {
            log_debug("TMCStepper Library Ver. 0x" << String(TMCSTEPPER_VERSION, HEX));
        }

        config_message();

        // After initializing all of the TMC drivers, create a task to
        // display StallGuard data.  List == this for the final instance.
        if (List == this) {
            xTaskCreatePinnedToCore(readSgTask,    // task
                                    "readSgTask",  // name for task
                                    4096,          // size of task stack
                                    this,          // parameters
                                    1,             // priority
                                    NULL,
                                    SUPPORT_TASK_CORE  // must run the task on same core
            );
        }
    }

    /*
    This is the startup message showing the basic definition
    */
    void TrinamicSpiDriver::config_message() {
        log_info("    " << name() << " Step:" << _step_pin.name() << " Dir:" << _dir_pin.name() << " CS:" << _cs_pin.name()
                        << " Disable:" << _disable_pin.name() << " Index:" << _spi_index << " R:" << _r_sense);
    }

    uint8_t TrinamicSpiDriver::toffValue() {
        if (_disabled) {
            return _toff_disable;
        }
        return _mode == TrinamicMode::StealthChop ? _toff_stealthchop : _toff_coolstep;
    }

};

// This is for possibly-daisy-chained TMC chips.  To send a packet
// to a chip somewhere in the chain, you must send enough dummy bits
// afterward to push the packet past previous chips in the chain.
// To receive, you must first discard data from later chips in the chain.
const size_t           packetBytes = 5;
constexpr const size_t packetBits  = packetBytes * 8;
void                   tmc_spi_send(spidev_t devid, uint8_t reg, uint32_t data, int index) {
    // index == 1 is the first chip in the chain
    size_t beforeChips  = index - 1;
    size_t dummyOutBits = beforeChips * packetBits;
    spi_send_cmd_addr(devid, reg, 8, data, 32, dummyOutBits);
}
void tmc_spi_write(spidev_t devid, uint8_t reg, uint32_t data, int index) {
    tmc_spi_send(devid, reg | 0x80, data, index);
}
uint32_t tmc_spi_receive(spidev_t devid, int index, int max_index) {
    size_t afterChips  = max_index - index;
    size_t dummyInBits = afterChips * packetBits;

    uint8_t in[packetBytes];
    spi_receive(devid, dummyInBits, in, packetBits);

    uint8_t status = in[0];

    uint32_t data = (uint32_t)in[1] << 24;
    data          = (uint32_t)in[2] << 16;
    data          = (uint32_t)in[3] << 8;
    data          = (uint32_t)in[4];
    return data;
};
uint32_t tmc_spi_read(spidev_t devid, uint8_t reg, int index, int max_index) {
    tmc_spi_send(devid, reg, 0, index);
    return tmc_spi_receive(devid, index, max_index);
}
