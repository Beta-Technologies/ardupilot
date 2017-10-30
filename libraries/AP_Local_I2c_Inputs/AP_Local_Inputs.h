#pragma once

#include "AP_Local_I2c_Input.h"

class AP_Local_Inputs {
public:
	static AP_Local_Inputs create() { return AP_Local_Inputs{}; }

    constexpr AP_Local_Inputs(AP_Local_Inputs &&other) = default;

    /* Do not allow copies */
    AP_Local_Inputs(const AP_Local_Inputs &other) = delete;
    AP_Local_Inputs &operator=(const AP_Local_Inputs&) = delete;

    static const struct AP_Param::GroupInfo var_info[];

    void start(void);
	void update(void);

private:
    AP_Local_Inputs();

    AP_Local_I2c_Input *aileron_encoder;
    AP_Local_I2c_Input *elevator_encoder;
    AP_Local_I2c_Input *throttle_encoder;
    AP_Local_I2c_Input *rudder_encoder;
    AP_Local_I2c_Input *tilt_encoder;

    // encoder parameters
    AP_Int8 _ail_address;
    AP_Int16 _ail_offset;
    AP_Float _ail_ratio;

    AP_Int8 _ele_address;
    AP_Int16 _ele_offset;
    AP_Float _ele_ratio;

    AP_Int8 _thr_address;
    AP_Int16 _thr_offset;
    AP_Float _thr_ratio;

    AP_Int8 _rud_address;
    AP_Int16 _rud_offset;
    AP_Float _rud_ratio;

    AP_Int8 _tilt_address;
    AP_Int16 _tilt_offset;
    AP_Float _tilt_ratio;
};
