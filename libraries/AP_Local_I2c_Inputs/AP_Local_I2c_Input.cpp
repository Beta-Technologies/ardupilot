// Talking to an MBAD01_08 rotary encoder using i2c
//   Maximum clock frequency 400kHz
//   Latency 200us to 400us
//   16 bits per revolution

#include <AP_Local_I2c_Inputs/AP_Local_I2c_Input.h>
#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <utility>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

AP_Local_I2c_Input::AP_Local_I2c_Input(uint8_t addr):
    i2c_addr(addr),
    _last_update_ms(0),
    _counter(0),
    _raw_encoder(0)
{
    i2c_addr = addr;
    _offset = 1000.0f;
    // 1 revolution is 65535 (16 bits per revolution)
    // trying: 60 degree deflection is 1000 us equivalent
    _ratio = 1000.0f / (65535.0f / (360.0f / 60.0f));
    sem = hal.util->new_semaphore();
}

AP_Local_I2c_Input::~AP_Local_I2c_Input()
{
    delete sem;
}

// modeled on AP_Airspeed_MS4525.cpp
bool AP_Local_I2c_Input::init(void) {

    const struct {
        uint8_t bus;
        uint8_t addr;
    } addresses[] = {
        { 1, i2c_addr },
        { 0, i2c_addr },
        { 2, i2c_addr },
    };
    bool found = false;

    for (uint8_t i=0; i<ARRAY_SIZE(addresses); i++) {
        _dev = hal.i2c_mgr->get_device(addresses[i].bus, addresses[i].addr);
        if (!_dev) {
            continue;
        }
        if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
            continue;
        }

        // lots of retries during probe
        _dev->set_retries(10);
    
        read();

        _dev->get_semaphore()->give();

        found = (_last_update_ms != 0);
        if (found) {
            printf("I2C_MBAD01: Found encoder on bus %u address 0x%02x\n", addresses[i].bus, addresses[i].addr);
            break;
        }
    }

    if (!found) {
        printf("I2C_MBAD01: no encoder found\n");
        return false;
    }

    // drop to 2 retries for runtime
    _dev->set_retries(2);
    
    //_dev->register_periodic_callback(20000,
    //    FUNCTOR_BIND_MEMBER(&AP_Airspeed_MS4525::_timer, void));
    return true;
}

void AP_Local_I2c_Input::read()
{
    _measure();
    hal.scheduler->delay(10);
    _collect();
}

// start a measurement
void AP_Local_I2c_Input::_measure()
{
    // if less than 250us have elapsed since the end of the previous response
    // we should wait. (from data sheet)
    _measurement_started_ms = 0;
    uint8_t cmd = '1'; // 0x31 for single position data request
    if (_dev->transfer(&cmd, 1, nullptr, 0)) {
        _measurement_started_ms = AP_HAL::millis();
    }
}

// read the values from the sensor
void AP_Local_I2c_Input::_collect()
{
    const struct {
        uint8_t header;
        // big-endian, left-aligned
        uint8_t absolute_position[3];
        uint16_t encoder_status;
        // should be 0xEF
        uint8_t footer;
    } data = {
        0, {0,0,0}, 0, 0
    };

    _measurement_started_ms = 0;

    if (!_dev->transfer(nullptr, 0, (uint8_t *)&data, sizeof(data))) {
        return;
    }

    if (data.footer != 0xEF) {
        // error, we didn't get the expected footer
        return;
    }

    if (data.encoder_status & (1<<9)) {
        // bit 9 indicates any error. position is not valid
        // detailed status in bits 0 through 7
        if (data.encoder_status & 1) {
            // position data changed too fast
        }
        else if (data.encoder_status & (1<<1)) {
            // magnetic pattern error. Metal particles or stray magnetic field present.
        }
        else if (data.encoder_status & (1<<2)) {
            // system error in the circuitry or inconsistent calibartion data
        }
        else if (data.encoder_status & (1<<3)) {
            // Power supply error - voltage out of range
        }
        else if (data.encoder_status & (1<<4)) {
            // Readhead temperature is out of range
        }
        else if (data.encoder_status & (1<<5)) {
            // signal lost. readhead is out of alignment
        }
        else if (data.encoder_status & (1<<6)) {
            // signal amplitude low. Distance between head and ring is too large
        }
        else if (data.encoder_status & (1<<6)) {
            // signal amplitude high. Distance between head and ring is too small
        }
        return;
    }

    int32_t position = 0;
    // bit twiddling is unchecked
    position |= data.absolute_position[0] << 16;
    position |= data.absolute_position[1] << 8;
    position |= data.absolute_position[2];

    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _raw_encoder = position;
        _counter++;
        sem->give();
    }
    
    _last_update_ms = AP_HAL::millis();
}

