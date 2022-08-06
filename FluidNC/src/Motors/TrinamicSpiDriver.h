// Copyright (c) 2020 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include "TrinamicBase.h"
#include "../Pin.h"
#include "../PinMapper.h"

#include "Driver/spi.h"

#include <cstdint>

const int NORMAL_TCOOLTHRS = 0xFFFFF;  // 20 bit is max
const int NORMAL_THIGH     = 0;

namespace MotorDrivers {

    class TrinamicSpiDriver : public TrinamicBase {
    public:
        TrinamicSpiDriver() = default;

        // Overrides for inherited methods
        virtual void init() override;
        //bool         set_homing_mode(bool ishoming) override;

        // Configuration handlers:
        void afterParse() override;

        void validate() const override { StandardStepper::validate(); }

        void group(Configuration::HandlerBase& handler) override {
            TrinamicBase::group(handler);

            handler.item("cs_pin", _cs_pin);
            handler.item("spi_index", _spi_index, -1, 127);

            handler.item("run_mode", _run_mode, trinamicModes);
            handler.item("homing_mode", _homing_mode, trinamicModes);
            handler.item("stallguard", _stallguard, -64, 63);
            handler.item("stallguard_debug", _stallguardDebugMode);
            handler.item("toff_coolstep", _toff_coolstep, 2, 15);
        }

    protected:
        Pin       _cs_pin;  // The chip select pin (can be the same for daisy chain)
        int32_t   _spi_index = -1;
        const int _spi_freq  = 100000;

        void config_message() override;

        uint8_t setupSPI();
        void    finalInit();

        bool    reportTest(uint8_t result);
        uint8_t toffValue();

        spidev_t _spi_devid;

    private:
        static pinnum_t _daisy_chain_cs_id;
        static spidev_t _daisy_chain_spi_devid;
        static uint8_t  _spi_index_mask;
        static uint8_t  _spi_index_max;

        PinMapper _cs_mapping;

        void trinamic_test_response();
        void trinamic_stepper_enable(bool enable);

        void     spi_first(uint8_t cmd, uint32_t data);
        void     spi_write(uint8_t reg, uint32_t data);
        uint32_t spi_read(uint8_t reg);
    };

};

// The following lets us replace TMCStepper's use of the Arduino SPI class
// with our own driver that is based on platform-level APIs.  TMC2130Stepper
// defines virtual methods read() and write() to access the SPI bus.  We
// create a TMC2130Spi class that derives from TMC2130Stepper with our own
// overrides of its read() and write() methods.

void     tmc_spi_send(spidev_t devid, uint8_t reg, uint32_t data, int index);
uint32_t tmc_spi_receive(spidev_t devid, int index, int max_index);
void     tmc_spi_write(spidev_t devid, uint8_t reg, uint32_t data, int index);
uint32_t tmc_spi_read(spidev_t devid, uint8_t reg, int index, int max_index);

class TMC2130Spi : public TMC2130Stepper {
    spidev_t _spi_devid;

public:
    TMC2130Spi(uint8_t cs_id, float r_sense, int32_t spi_index, spidev_t spi_devid) :
        TMC2130Stepper(cs_id, r_sense, spi_index), _spi_devid(spi_devid) {}

    // XXX we should just avoid calling this and set the speed when we init the spidev
    void setSPISpeed(uint32_t speed) {}

protected:
    void     write(uint8_t reg, uint32_t data) override { tmc_spi_write(_spi_devid, reg, data, link_index); }
    uint32_t read(uint8_t reg) override { return tmc_spi_read(_spi_devid, reg, link_index, chain_length); };
    // TMC2130Stepper::begin() calls pinMode(_pinCS, OUTPUT); and switchCSpin(HIGH);
    // The pinMode call is not weak, but it is weak in Arduino
    void switchCSpin(bool state) {}
};

// TMC5160Stepper derives from TMC2130Stepper directly, without going through
// our TMC2130Spi, so we must duplicate the overrides.

class TMC5160Spi : public TMC5160Stepper {
    spidev_t _spi_devid;

public:
    TMC5160Spi(uint8_t cs_id, float r_sense, int32_t spi_index, spidev_t spi_devid) :
        TMC5160Stepper(cs_id, r_sense, spi_index), _spi_devid(spi_devid) {}

    void setSPISpeed(uint32_t speed) {}

protected:
    void     write(uint8_t reg, uint32_t data) override { tmc_spi_write(_spi_devid, reg, data, link_index); }
    uint32_t read(uint8_t reg) override { return tmc_spi_read(_spi_devid, reg, link_index, chain_length); }
    void     switchCSpin(bool state) {}
};
