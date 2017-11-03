// 2017 Beta Technologies Inc.
// 
// This handles the parameters for our local i2c inputs.
//

#include <AP_HAL/AP_HAL.h>
#include "AP_Local_Inputs.h"

extern const AP_HAL::HAL& hal;

AP_Local_Inputs::AP_Local_Inputs(void):
 started(false),
 aileron_encoder(NULL),
 elevator_encoder(NULL),
 throttle_encoder(NULL),
 rudder_encoder(NULL),
 tilt_encoder(NULL)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_Local_Inputs::start(void)
{
    AP_Local_I2c_Input *encoder;

    // aileron
    encoder = new AP_Local_I2c_Input(_ail_address);
    if (encoder->init()) {
        aileron_encoder = encoder;
        hal.console->printf("BETA: found aileron encoder.");
    }

    // elevator
    encoder = new AP_Local_I2c_Input(_ele_address);
    if (encoder->init()) {
        elevator_encoder = encoder;
        hal.console->printf("BETA: found elevator encoder.");
    }

    // throttle
    encoder = new AP_Local_I2c_Input(_thr_address);
    if (encoder->init()) {
        throttle_encoder = encoder;
        hal.console->printf("BETA: found throttle encoder.");
    }

    // rudder
    encoder = new AP_Local_I2c_Input(_rud_address);
    if (encoder->init()) {
        rudder_encoder = encoder;
        hal.console->printf("BETA: found rudder encoder.");
    }

    // tilt
    encoder = new AP_Local_I2c_Input(_tilt_address);
    if (encoder->init()) {
        tilt_encoder = encoder;
        hal.console->printf("BETA: found tilt encoder.");
    }
}


void AP_Local_Inputs::update(void)
{
    if (!started)
    {
        start();
        started = true;
        return;
    }

    int16_t channels[11];

    // mid-range defaults
    for (uint8_t i = 0; i < 5; i++) {
        channels[i] = 1500;
    }
    // low throttle default
    channels[2] = 950;

    if (aileron_encoder != NULL) {
        aileron_encoder->read_position();

        uint16_t raw_encoder = aileron_encoder->get_raw_encoder();
        channels[0] = scale_raw(raw_encoder, _ail_low, _ail_high);
    }

    if (elevator_encoder != NULL) {
        elevator_encoder->read_position();
        uint16_t raw_encoder = elevator_encoder->get_raw_encoder();
        channels[1] = scale_raw(raw_encoder, _ele_low, _ele_high);
    }

    if (throttle_encoder != NULL) {
        throttle_encoder->read_position();

        uint16_t raw_encoder = throttle_encoder->get_raw_encoder();
        channels[2] = scale_raw(raw_encoder, _thr_low, _thr_high);
    }

    if (rudder_encoder != NULL) {
        rudder_encoder->read_position();

        uint16_t raw_encoder = rudder_encoder->get_raw_encoder();
        channels[3] = scale_raw(raw_encoder, _rud_low, _rud_high);
    }

    if (tilt_encoder != NULL) {
        tilt_encoder->read_position();

        uint16_t raw_encoder = tilt_encoder->get_raw_encoder();
        channels[4] = scale_raw(raw_encoder, _tilt_low, _tilt_high);
    }

    // sentinel / not used
    for (uint8_t i = 5; i < 11; i++) {
        channels[i] = 0xFFFF;
    }
    
    hal.rcin->set_overrides(channels, 10);
}

int16_t AP_Local_Inputs::scale_raw(uint16_t read, uint16_t low, uint16_t high)
{
    float ratio = 1000.0f / ((float)high - (float)low);
    int16_t scaled = (int16_t)(((float)read - low) * ratio);
    return MIN(MAX(scaled + 1000, 1000), 2000);
}

const AP_Param::GroupInfo AP_Local_Inputs::var_info[] = {
    // @Param: AIL_ADDR
    // @DisplayName: i2c address for aileron encoder
    // @Description: The i2c address of the encoder that we use for Aileron inputs. 
    // @Range: 0 255
    // @Increment: 1
    AP_GROUPINFO("AIL_ADDR", 0, AP_Local_Inputs, _ail_address, 0x20),

    // @Param: AIL_LOW
    // @DisplayName: aileron -- lowest raw value expected
    // @Description: This value will map to a PWM of 1000
    // @Range: 0, 65535
    // @Increment: 1
    AP_GROUPINFO("AIL_LOW", 1, AP_Local_Inputs, _ail_low, 0),

    // @Param: AIL_HIGH
    // @DisplayName: aileron -- higest raw value expected
    // @Description: Highest value we expect from the encoder
    // @Range: 0, 65535
    // @Increment: 1
    AP_GROUPINFO("AIL_HIGH", 2, AP_Local_Inputs, _ail_high, 65535),


    // @Param: ELE_ADDR
    // @DisplayName: i2c address for elevator encoder
    // @Description: The i2c address of the encoder that we use for elevator inputs. 
    // @Range: 0 255
    // @Increment: 1
    AP_GROUPINFO("ELE_ADDR", 3, AP_Local_Inputs, _ele_address, 0x21),

    // @Param: ELE_LOW
    // @DisplayName: elevator encoder low
    // @Description: This value will map to a PWM of 1000
    // @Range: 0, 65535
    // @Increment: 1
    AP_GROUPINFO("ELE_LOW", 4, AP_Local_Inputs, _ele_low, 0),

    // @Param: ELE_HIGH
    // @DisplayName: elevator encoder ratio
    // @Description: This value will map to a PWM of 2000
    // @Range: 0, 65535
    // @Increment: 1
    AP_GROUPINFO("ELE_HIGH", 5, AP_Local_Inputs, _ele_high, 65535),


    // @Param: THR_ADDR
    // @DisplayName: i2c address for throttle encoder
    // @Description: The i2c address of the encoder that we use for throttle inputs. 
    // @Range: 0 255
    // @Increment: 1
    AP_GROUPINFO("THR_ADDR", 6, AP_Local_Inputs, _thr_address, 0x22),

    // @Param: THR_LOW
    // @DisplayName: throttle encoder offset
    // @Description: This value will map to a PWM of 1000
    // @Range: 0, 65535
    // @Increment: 1
    // @RebootRequired: True
    AP_GROUPINFO("THR_LOW", 7, AP_Local_Inputs, _thr_low, 0),

    // @Param: THR_HIGH
    // @DisplayName: throttle encoder ratio
    // @Description: This value will map to a PWM of 2000
    // @Range: 0, 65535
    // @Increment: 1
    AP_GROUPINFO("THR_HIGH", 8, AP_Local_Inputs, _thr_high, 65535),


    // @Param: RUD_ADDR
    // @DisplayName: i2c address for rudder encoder
    // @Description: The i2c address of the encoder that we use for rudder inputs. 
    // @Range: 0 255
    // @Increment: 1
    AP_GROUPINFO("RUD_ADDR", 9, AP_Local_Inputs, _rud_address, 0x23),

    // @Param: RUD_LOW
    // @DisplayName: rudder encoder offset
    // @Description: This value will map to a PWM of 1000
    // @Range: 0, 65535
    // @Increment: 1
    AP_GROUPINFO("RUD_LOW", 10, AP_Local_Inputs, _rud_low, 0),

    // @Param: RUD_HIGH
    // @DisplayName: rudder encoder ratio
    // @Description: This value will map to a PWM of 2000
    // @Range: 0, 65535
    // @Increment: 1
    AP_GROUPINFO("RUD_HIGH", 11, AP_Local_Inputs, _rud_high, 65535),


    // @Param: TILT_ADDR
    // @DisplayName: i2c address for tilt encoder
    // @Description: The i2c address of the encoder that we use for tilt inputs. 
    // @Range: 0 255
    // @Increment: 1
    AP_GROUPINFO("TILT_ADDR", 12, AP_Local_Inputs, _tilt_address, 0x24),

    // @Param: TILT_LOW
    // @DisplayName: tilt encoder offset
    // @Description: This value will map to a PWM of 1000
    // @Range: 0, 65535
    // @Increment: 1
    AP_GROUPINFO("TILT_LOW", 13, AP_Local_Inputs, _tilt_low, 0),

    // @Param: TILT_HIGH
    // @DisplayName: tilt encoder ratio
    // @Description: This value will map to a PWM of 2000
    // @Range: 0, 65535
    // @Increment: 1
    AP_GROUPINFO("TILT_HIGH", 14, AP_Local_Inputs, _tilt_high, 65535),

    AP_GROUPEND
};
