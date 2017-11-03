#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/utility/OwnPtr.h>
#include <AP_HAL/I2CDevice.h>
#include <GCS_MAVLink/GCS_MAVLink.h>

#define MBAD01_STOCK_I2C_ADDR 0x18

class AP_Local_I2c_Input
{
public:
    AP_Local_I2c_Input(uint8_t addr = MBAD01_STOCK_I2C_ADDR);
    ~AP_Local_I2c_Input(void);

    bool init(void);

    // read the i2c source and update _raw_encoder
    void read_position(void);

    // change to the new address, and updates this device to use it
    void change_address(uint8_t new_addresss);

    const char* last_error(void) { return _error; }

    void set_i2c_addr(uint8_t addr) {
        i2c_addr = addr;
    }

    void set_offset(uint16_t offset) {
        _offset = offset;
    }

    void set_ratio(float ratio) {
        _ratio = ratio;
    }
    
    uint8_t get_i2c_addr() {
        return i2c_addr;
    }

    // return the unfiltered encoder value in m/s
    uint32_t get_raw_encoder(void) const {
        return _raw_encoder;
    }

    // return the current encoder value in 1000 - 2000 PWM range
    uint32_t get_encoder(void) const {
        return MIN(MAX((int)(_raw_encoder * _ratio + _offset + 0.5f), 1000), 2000);
    }

    // return the current offset
    float get_offset(void) const {
        return _offset;
    }

    float get_ratio(void) const {
        return _ratio;
    }

    float get_counter(void) const {
        return _counter;
    }

    // return health status of sensor
    bool healthy(void) const { return _healthy; }

    // return time in ms of last update
    uint32_t last_update_ms(void) const { return _last_update_ms; }

    //static const struct AP_Param::GroupInfo var_info[];

private:
    const char *    _error;
    char error_message[200];

    uint8_t         i2c_addr;
    int32_t         _raw_encoder;

    // offset and ratio to get it in the range we want
    AP_Float        _offset;
    AP_Float        _ratio;

    // state 
    bool		    _healthy:1;
    uint8_t         _counter;
    uint32_t        _measurement_started_ms;
    uint32_t        _last_update_ms;

    AP_HAL::Semaphore *sem;    
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    uint8_t calculateCrc(uint8_t *buffer, uint8_t size);
    static uint8_t _CrcTable [256];
};
