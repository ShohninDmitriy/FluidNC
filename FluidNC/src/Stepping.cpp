// #include "Driver/i2s_out.h"
#include "EnumItem.h"
#include "Stepping.h"
#include "Machine/MachineConfig.h"  // config

#include <atomic>

step_engine_t* step_engines = NULL;  // Linked list of stepping engines

step_engine_t* find_engine(const char* name) {
    for (step_engine_t* p = step_engines; p; p = p->link) {
        // Initial substring match, handles different forms of I2S
        if (strncmp(name, p->name, strlen(p->name)) == 0) {
            return p;
        }
    }
    return NULL;
}

namespace Machine {

    // fStepperTimer should be an integer divisor of the bus speed, i.e. of fTimers
    const int ticksPerMicrosecond = Stepping::fStepperTimer / 1000000;

    int Stepping::_engine = RMT_ENGINE;

    int Stepping::_n_active_axes = 0;

    bool   Stepping::_switchedStepper = false;
    size_t Stepping::_segments        = 12;

    uint32_t Stepping::_idleMsecs           = 255;
    uint32_t Stepping::_pulseUsecs          = 4;
    uint32_t Stepping::_directionDelayUsecs = 0;
    uint32_t Stepping::_disableDelayUsecs   = 0;

    step_engine_t* Stepping::step_engine;

    const EnumItem stepTypes[] = { { Stepping::TIMED, "Timed" },
                                   { Stepping::RMT_ENGINE, "RMT" },
                                   { Stepping::I2S_STATIC, "I2S_static" },
                                   { Stepping::I2S_STREAM, "I2S_stream" },
                                   EnumItem(Stepping::RMT_ENGINE) };

    void Stepping::afterParse() {
        const char* name = stepTypes[_engine].name;
        step_engine      = find_engine(name);
        Assert(step_engine, "Cannot find stepping engine for %s", name);
        Assert(strcmp("I2S", name) || config->_i2so, "I2SO bus must be configured for this stepping type");
    }

    void Stepping::init() {
        log_info("Stepping:" << stepTypes[_engine].name << " Pulse:" << _pulseUsecs << "us Dsbl Delay:" << _disableDelayUsecs
                             << "us Dir Delay:" << _directionDelayUsecs << "us Idle Delay:" << _idleMsecs << "ms");

        uint32_t actual = step_engine->init(_directionDelayUsecs, _pulseUsecs);
        if (actual != _pulseUsecs) {
            log_warn("stepping/pulse_us adjusted to " << actual);
        }
        // Prepare stepping interrupt callbacks.  The one that is actually
        // used is determined by timerStart() and timerStop()

        // Setup a timer for direct stepping
        stepTimerInit(fStepperTimer, Stepper::pulse_func);

        // Register pulse_func with the I2S subsystem
        // This could be done via the linker.
        //        i2s_out_set_pulse_callback(Stepper::pulse_func);

        Stepper::init();
    }

}

Stepping::motor_t* Stepping::axis_motors[MAX_N_AXIS][MAX_MOTORS_PER_AXIS] = { nullptr };

void Stepping::assignMotor(int axis, int motor, int step_pin, bool step_invert, int dir_pin, bool dir_invert) {
    if (axis >= _n_active_axes) {
        _n_active_axes = axis + 1;
    }
    step_pin = step_engine->init_step_pin(step_pin, step_invert);

    motor_t* m               = new motor_t;
    axis_motors[axis][motor] = m;
    m->step_pin              = step_pin;
    m->step_invert           = step_invert;
    m->dir_pin               = dir_pin;
    m->dir_invert            = dir_invert;
    m->blocked               = false;
    m->limited               = false;
}

int Stepping::axis_steps[MAX_N_AXIS] = { 0 };

bool* Stepping::limit_var(int axis, int motor) {
    auto m = axis_motors[axis][motor];
    return m ? &(m->limited) : nullptr;
}

void Stepping::block(int axis, int motor) {
    auto m = axis_motors[axis][motor];
    if (m) {
        m->blocked = true;
    }
}

void Stepping::unblock(int axis, int motor) {
    auto m = axis_motors[axis][motor];
    if (m) {
        m->blocked = false;
    }
}

void Stepping::limit(int axis, int motor) {
    auto m = axis_motors[axis][motor];
    if (m) {
        m->limited = true;
    }
}
void Stepping::unlimit(int axis, int motor) {
    auto m = axis_motors[axis][motor];
    if (m) {
        m->limited = false;
    }
}

void IRAM_ATTR Stepping::step(uint8_t step_mask, uint8_t dir_mask) {
    // Set the direction pins, but optimize for the common
    // situation where the direction bits haven't changed.
    static uint8_t previous_dir_mask = 255;  // should never be this value
    if (previous_dir_mask == 255) {
        // Set all the direction bits the first time
        previous_dir_mask = ~dir_mask;
    }

    if (dir_mask != previous_dir_mask) {
        for (size_t axis = 0; axis < _n_active_axes; axis++) {
            bool dir     = bitnum_is_true(dir_mask, axis);
            bool old_dir = bitnum_is_true(previous_dir_mask, axis);
            if (dir != old_dir) {
                for (size_t motor = 0; motor < MAX_MOTORS_PER_AXIS; motor++) {
                    auto m = axis_motors[axis][motor];
                    if (m) {
                        step_engine->set_dir_pin(m->dir_pin, dir ^ m->dir_invert);
                    }
                }
            }
            // Some stepper drivers need time between changing direction and doing a pulse.
            step_engine->finish_dir();
        }
        previous_dir_mask = dir_mask;
    }

    // Turn on step pulses for motors that are supposed to step now
    for (size_t axis = 0; axis < _n_active_axes; axis++) {
        if (bitnum_is_true(step_mask, axis)) {
            auto increment = bitnum_is_true(dir_mask, axis) ? -1 : 1;
            axis_steps[axis] += increment;
            for (size_t motor = 0; motor < MAX_MOTORS_PER_AXIS; motor++) {
                auto m = axis_motors[axis][motor];
                if (m && !m->blocked && !m->limited) {
                    step_engine->set_step_pin(m->step_pin, !m->step_invert);
                }
            }
        }
    }
    step_engine->finish_step();
}

// Turn all stepper pins off
void IRAM_ATTR Stepping::unstep() {
    if (step_engine->start_unstep()) {
        return;
    }
    for (size_t axis = 0; axis < _n_active_axes; axis++) {
        for (size_t motor = 0; motor < MAX_MOTORS_PER_AXIS; motor++) {
            auto m = axis_motors[axis][motor];
            if (m) {
                step_engine->set_step_pin(m->step_pin, m->step_invert);
            }
        }
    }
    step_engine->finish_unstep();
}

void Stepping::reset() {}
void Stepping::beginLowLatency() {}
void Stepping::endLowLatency() {}

// Called only from step()
void IRAM_ATTR Stepping::waitDirection() {}

// Called only from Stepper::pulse_func when a new segment is loaded
// The argument is in units of ticks of the timer that generates ISRs
void IRAM_ATTR Stepping::setTimerPeriod(uint16_t timerTicks) {
    // stepTimerSetTicks((uint32_t)timerTicks * 3 / 10);
    stepTimerSetTicks((uint32_t)timerTicks / 2);
}

// Called only from Stepper::wake_up which is not used in ISR context
void Stepping::startTimer() {
    stepTimerStart();
}
// Called only from Stepper::stop_stepping, used in both ISR and foreground contexts
void IRAM_ATTR Stepping::stopTimer() {
    stepTimerStop();
}

void Stepping::group(Configuration::HandlerBase& handler) {
    handler.item("engine", _engine, stepTypes);
    handler.item("idle_ms", _idleMsecs, 0, 10000000);  // full range
    handler.item("pulse_us", _pulseUsecs, 0, 30);
    handler.item("dir_delay_us", _directionDelayUsecs, 0, 10);
    handler.item("disable_delay_us", _disableDelayUsecs, 0, 1000000);  // max 1 second
    handler.item("segments", _segments, 6, 20);
}

uint32_t Stepping::maxPulsesPerSec() {
    return step_engine->max_pulses_per_sec();
}
