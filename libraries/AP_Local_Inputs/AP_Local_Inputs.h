#pragma once

#include "AP_Local_I2c_Input.h"

class AP_Local_Inputs {
public:
    AP_Local_Inputs();
	static AP_Local_Inputs create() { return AP_Local_Inputs{}; }

    constexpr AP_Local_Inputs(AP_Local_Inputs &&other) = default;

    /* Do not allow copies */
    AP_Local_Inputs(const AP_Local_Inputs &other) = delete;
    AP_Local_Inputs &operator=(const AP_Local_Inputs&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    void start(void);
	void update(void);

private:

    int16_t scale_raw(uint16_t read, uint16_t low, uint16_t high);

    bool started;

    AP_Local_I2c_Input *aileron_encoder;
    AP_Local_I2c_Input *elevator_encoder;
    AP_Local_I2c_Input *throttle_encoder;
    AP_Local_I2c_Input *rudder_encoder;
    AP_Local_I2c_Input *tilt_encoder;

    // encoder parameters
    AP_Int8 _ail_address;
    AP_Int32 _ail_low;
    AP_Int32 _ail_high;

    AP_Int8 _ele_address;
    AP_Int32 _ele_low;
    AP_Int32 _ele_high;

    AP_Int8 _thr_address;
    AP_Int32 _thr_low;
    AP_Int32 _thr_high;

    AP_Int8 _rud_address;
    AP_Int32 _rud_low;
    AP_Int32 _rud_high;

    AP_Int8 _tilt_address;
    AP_Int32 _tilt_low;
    AP_Int32 _tilt_high;
};
