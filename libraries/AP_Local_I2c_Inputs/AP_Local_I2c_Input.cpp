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

extern const AP_HAL::HAL &hal;

AP_Local_I2c_Input::AP_Local_I2c_Input(uint8_t addr):
    i2c_addr(addr),
    _last_update_ms(0),
    _counter(0),
    _raw_encoder(0)
{
    _offset = 1000.0f;
    // 1 revolution is 65535 (16 bits per revolution)
    _ratio = 1000.0f / 65535.0f;
    sem = NULL;
}

AP_Local_I2c_Input::~AP_Local_I2c_Input()
{
    delete sem;
}

// modeled on AP_Airspeed_MS4525.cpp
bool AP_Local_I2c_Input::init(void) {

    if (sem == NULL) 
    {
        sem = hal.util->new_semaphore();
    }

    const struct {
        uint8_t bus;
        uint8_t addr;
    } addresses[] = {
        { 0, i2c_addr },
        { 1, i2c_addr },
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
        _dev->set_retries(25);
    
        read_position();

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

/** 
 * The output doesn't seem to correspond to the datasheet we have for the Aksim MBA sensor
 * What we do see is a continuous stream (we don't need to send a '1' read command).
 * Bytes 0, 1: First two bytes are position, big-endian
 * Byte 2 is status
 *   0x06: head is too close to ring
 *   0x00: good spacing
 *   0x05: head is too far from ring
 *   0x08: head is much too far (position will be FF FF)
 * Byte 3: is detailed status, can't figure out the pattern yet
 * Byte 4: changes wildly with any movement
 */
void AP_Local_I2c_Input::read_position()
{
    _error = "";
    uint8_t data[5];

    _measurement_started_ms = 0;

    if (!_dev->transfer(nullptr, 0, (uint8_t *)&data, 5)) {
        return;
    }
    // hal.console->printf("Bytes: %02X %02X -  %02X %02X - %02X", 
    //     data[0], data[1], data[2], data[3], data[4]);

    uint8_t encoder_status = data[2];
    _healthy = encoder_status == 0x00;

    if (!_healthy) {
        if (encoder_status == 0x06) {
            _error = "Read head is too close to ring.";
        } else if (encoder_status == 0x05) {
            _error = "Read head is too far from ring.";
        } else if (encoder_status == 0x08) {
            _error = "Read head is MUCH too far from ring. Position invalid";
        }
    }

    int32_t position = 0;
    // bit twiddling is unchecked
    position |= ((uint32_t)data[0]) << 8;
    position |= data[1];

    if (sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        _raw_encoder = position;
        _counter++;
        _last_update_ms = AP_HAL::millis();
        sem->give();
    }
}

/*
The data sheet would have us expect the following error bits:

    if (encoder_status & (1<<9)) {
        // bit 9 indicates any error. position is not valid
        // detailed status in bits 0 through 7
        if (encoder_status & 1) {
            _error = "position data changed too fast";
        }
        else if (encoder_status & (1<<1)) {
            _error = "magnetic pattern error. Metal particles or stray magnetic field present.";
        }
        else if (encoder_status & (1<<2)) {
            _error = "system error in the circuitry or inconsistent calibartion data";
        }
        else if (encoder_status & (1<<3)) {
            _error = "Power supply error - voltage out of range";
        }
        else if (encoder_status & (1<<4)) {
            _error = "Readhead temperature is out of range";
        }
        else if (encoder_status & (1<<5)) {
            _error = "signal lost. readhead is out of alignment";
        }
        else if (encoder_status & (1<<6)) {
            _error = "signal amplitude low. Distance between head and ring is too large";
        }
        else if (encoder_status & (1<<7)) {
            _error = "signal amplitude high. Distance between head and ring is too small";
        }
        return;
    }

*/

