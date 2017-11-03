// Talking to an MBAD01_08 rotary encoder using i2c
//   Maximum clock frequency 400kHz
//   Latency 200us to 400us
//   16 bits per revolution

#include <AP_Local_Inputs/AP_Local_I2c_Input.h>
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
//            printf("I2C_MBAD01: Found encoder on bus %u address 0x%02x\n", addresses[i].bus, addresses[i].addr);
            break;
        }
    }

    if (!found) {
//        printf("I2C_MBAD01: no encoder found\n");
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
    uint8_t data[5];

    _measurement_started_ms = 0;

    if (!_dev->transfer(nullptr, 0, (uint8_t *)&data, 5)) {
        return;
    }

    bool error = (bool)(data[2] & (1 << 3));
    bool warning = (bool)(data[2] & (1 << 2));

    bool readHeadNear = (bool)(data[2] & (1 << 1));
    bool readHeadFar = (bool)(data[2] & 1);
    bool misaligned = (bool)(data[3] & (1 << 7));
    bool temperature = (bool)(data[3] & (1 << 6));
    bool powerError = (bool)(data[3] & (1 << 5));
    bool systemError = (bool)(data[3] & (1 << 4));
    bool patternError = (bool)(data[3] & (1 << 3));
    bool acceleration = (bool)(data[3] & (1 << 2));
    //uint8_t crc = data[4];

    _error = "";

    if (error || warning) {
        const char* prefix = "";
        const char* message = "";

        if (error) {
            prefix = "ERROR-- position invalid:";
        } else if (warning) {
            prefix = "Warning:";
        }

        if (readHeadNear) {
            message = " Read head is too close to ring.";
        } else if (readHeadFar) {
            message = "Read head is too far from ring.";
        } else if (acceleration) {
            message = "Position data changed too fast";
        } else if (patternError) {
            message = "magnetic pattern error. Metal particles or stray magnetic field present.";
        } else if (systemError) {
            message = "system error in the circuitry or inconsistent calibartion data";
        } else if (powerError) {
            message = "Power supply error - voltage out of range";
        } else if (temperature) {
            message = "Read head temperature is out of range";
        } else if (misaligned) {
            message = "Signal lost. readhead is out of alignment";
        }

        snprintf(error_message, 200, "%s %s", prefix, message);
        _error = error_message;
    }

    // if (error) {
    //     // should we 'failsafe' to mid-range? 
    //     // we probably need a parameter for each sensor that gives us 
    //     // the value to use when there's an error.
    //     raw_encoder = 32767;
    //     return;
    // }

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

void AP_Local_I2c_Input::change_address(uint8_t new_address)
{
    uint8_t data[3];
    data[0] = 'a';
    data[1] = new_address;
    data[2] = 'a';

    if (!_dev->transfer(nullptr, 0, (uint8_t *)&data, 3)) {
        return;
    }
    i2c_addr = new_address;
}

uint8_t AP_Local_I2c_Input::calculateCrc(uint8_t *buffer, uint8_t size)  
{  
    uint32_t t;
    uint8_t icrc = 1;
    uint8_t numBytes = size - 1;
    
    t = buffer[0];  
    while (numBytes--)
    {
        t = buffer[icrc++] ^ _CrcTable[t];
    }  
    return _CrcTable[t];
}

uint8_t AP_Local_I2c_Input::_CrcTable [256] = {  
    0x00, 0x97, 0xB9, 0x2E, 0xE5, 0x72, 0x5C, 0xCB, 0x5D, 0xCA, 0xE4, 0x73, 0xB8, 0x2F, 0x01, 0x96,
    0xBA, 0x2D, 0x03, 0x94, 0x5F, 0xC8, 0xE6, 0x71, 0xE7, 0x70, 0x5E, 0xC9, 0x02, 0x95, 0xBB, 0x2C,
    0xE3, 0x74, 0x5A, 0xCD, 0x06, 0x91, 0xBF, 0x28, 0xBE, 0x29, 0x07, 0x90, 0x5B, 0xCC, 0xE2, 0x75,
    0x59, 0xCE, 0xE0, 0x77, 0xBC, 0x2B, 0x05, 0x92, 0x04, 0x93, 0xBD, 0x2A, 0xE1, 0x76, 0x58, 0xCF,
    0x51, 0xC6, 0xE8, 0x7F, 0xB4, 0x23, 0x0D, 0x9A, 0x0C, 0x9B, 0xB5, 0x22, 0xE9, 0x7E, 0x50, 0xC7,
    0xEB, 0x7C, 0x52, 0xC5, 0x0E, 0x99, 0xB7, 0x20, 0xB6, 0x21, 0x0F, 0x98, 0x53, 0xC4, 0xEA, 0x7D,
    0xB2, 0x25, 0x0B, 0x9C, 0x57, 0xC0, 0xEE, 0x79, 0xEF, 0x78, 0x56, 0xC1, 0x0A, 0x9D, 0xB3, 0x24,
    0x08, 0x9F, 0xB1, 0x26, 0xED, 0x7A, 0x54, 0xC3, 0x55, 0xC2, 0xEC, 0x7B, 0xB0, 0x27, 0x09, 0x9E,
    0xA2, 0x35, 0x1B, 0x8C, 0x47, 0xD0, 0xFE, 0x69, 0xFF, 0x68, 0x46, 0xD1, 0x1A, 0x8D, 0xA3, 0x34,
    0x18, 0x8F, 0xA1, 0x36, 0xFD, 0x6A, 0x44, 0xD3, 0x45, 0xD2, 0xFC, 0x6B, 0xA0, 0x37, 0x19, 0x8E,
    0x41, 0xD6, 0xF8, 0x6F, 0xA4, 0x33, 0x1D, 0x8A, 0x1C, 0x8B, 0xA5, 0x32, 0xF9, 0x6E, 0x40, 0xD7,
    0xFB, 0x6C, 0x42, 0xD5, 0x1E, 0x89, 0xA7, 0x30, 0xA6, 0x31, 0x1F, 0x88, 0x43, 0xD4, 0xFA, 0x6D,
    0xF3, 0x64, 0x4A, 0xDD, 0x16, 0x81, 0xAF, 0x38, 0xAE, 0x39, 0x17, 0x80, 0x4B, 0xDC, 0xF2, 0x65,
    0x49, 0xDE, 0xF0, 0x67, 0xAC, 0x3B, 0x15, 0x82, 0x14, 0x83, 0xAD, 0x3A, 0xF1, 0x66, 0x48, 0xDF,
    0x10, 0x87, 0xA9, 0x3E, 0xF5, 0x62, 0x4C, 0xDB, 0x4D, 0xDA, 0xF4, 0x63, 0xA8, 0x3F, 0x11, 0x86,
    0xAA, 0x3D, 0x13, 0x84, 0x4F, 0xD8, 0xF6, 0x61, 0xF7, 0x60, 0x4E, 0xD9, 0x12, 0x85, 0xAB, 0x3C
};
