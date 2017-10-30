// 2017 Beta Technologies Inc.
// 
// Front-end driver for accessing one driver per AP_Local_I2c_Input which reads values from an encoder
// on the i2c bus and uses rcinput->set_override to provide its value to the flight controller.
//

#include <AP_HAL/AP_HAL.h>
#include "AP_Local_Inputs.h"

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

AP_Local_Inputs::AP_Local_Inputs(void):
 aileron_encoder(NULL),
 elevator_encoder(NULL),
 throttle_encoder(NULL),
 rudder_encoder(NULL),
 tilt_encoder(NULL)
 {

}

void AP_Local_Inputs::start(void)
{
    AP_Local_I2c_Input *encoder;

    // aileron
    encoder = new AP_Local_I2c_Input(_thr_address);
    if (encoder->init()) {
        encoder->set_offset(_thr_offset);
        encoder->set_ratio(_thr_ratio);
        aileron_encoder = encoder;
    }

    // elevator
    encoder = new AP_Local_I2c_Input(_ele_address);
    if (encoder->init()) {
        encoder->set_offset(_ele_offset);
        encoder->set_ratio(_ele_ratio);
        elevator_encoder = encoder;
    }

    // throttle
    encoder = new AP_Local_I2c_Input(_thr_address);
    if (encoder->init()) {
        encoder->set_offset(_thr_offset);
        encoder->set_ratio(_thr_ratio);
        throttle_encoder = encoder;
    }

    // rudder
    encoder = new AP_Local_I2c_Input(_rud_address);
    if (encoder->init()) {
        encoder->set_offset(_rud_offset);
        encoder->set_ratio(_rud_ratio);
        rudder_encoder = encoder;
    }

    // tilt
    encoder = new AP_Local_I2c_Input(_tilt_address);
    if (encoder->init()) {
        encoder->set_offset(_tilt_offset);
        encoder->set_ratio(_tilt_ratio);
        tilt_encoder = encoder;
    }
}

void AP_Local_Inputs::update(void)
{
    int16_t channels[11];

    // mid-range defaults
    for (uint8_t i = 0; i < 5; i++) {
        channels[i] = 1500;
    }
    // low throttle default
    channels[2] = 950;

    if (aileron_encoder != NULL) {
        aileron_encoder->read_position();
        channels[0] = aileron_encoder->get_encoder();
    }

    if (elevator_encoder != NULL) {
        elevator_encoder->read_position();
        channels[1] = elevator_encoder->get_encoder();
    }

    if (throttle_encoder != NULL) {
        throttle_encoder->read_position();
        channels[2] = throttle_encoder->get_encoder();
    }

    if (rudder_encoder != NULL) {
        rudder_encoder->read_position();
        channels[3] = rudder_encoder->get_encoder();
    }

    if (tilt_encoder != NULL) {
        tilt_encoder->read_position();
        channels[4] = tilt_encoder->get_encoder();
    }

    // sentinel / not used
    for (uint8_t i = 5; i < 11; i++) {
        channels[i] = 0xFFFF;
    }

    hal.rcin->set_overrides(channels, 10);
}

const AP_Param::GroupInfo AP_Local_Inputs::var_info[] = {
    // @Param: THR_I2C_ADDRESS
    // @DisplayName: i2c address for aileron encoder
    // @Description: The i2c address of the encoder that we use for Aileron inputs. 
    // @Range: 0 255
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("THR_I2C_ADDRESS",        0, AP_Local_Inputs, _thr_address, 1),

    // @Param: THR_OFFSET
    // @DisplayName: aileron encoder offset
    // @Description: The encoder gives us values in the 0-65535 range. This value is multiplied by ratio, then offset is added to achieve our target output value.
    // @Range: -3278 32767
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("THR_OFFSET",        0, AP_Local_Inputs, _thr_offset, 1),

    // @Param: THR_RATIO
    // @DisplayName: aileron encoder offset
    // @Description: The encoder gives us values in the 0-65535 range. This value is multiplied by ratio, then offset is added to achieve our target output value.
    // @Range: -3278 32767
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("THR_RATIO",        0, AP_Local_Inputs, _thr_ratio, 1),


    // @Param: ELE_I2C_ADDRESS
    // @DisplayName: i2c address for elevator encoder
    // @Description: The i2c address of the encoder that we use for elevator inputs. 
    // @Range: 0 255
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ELE_I2C_ADDRESS",        0, AP_Local_Inputs, _ele_address, 1),

    // @Param: ELE_OFFSET
    // @DisplayName: elevator encoder offset
    // @Description: The encoder gives us values in the 0-65535 range. This value is multiplied by ratio, then offset is added to achieve our target output value.
    // @Range: -3278 32767
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ELE_OFFSET",        0, AP_Local_Inputs, _ele_offset, 1),

    // @Param: ELE_RATIO
    // @DisplayName: elevator encoder ratio
    // @Description: The encoder gives us values in the 0-65535 range. This value is multiplied by ratio, then offset is added to achieve our target output value.
    // @Range: -3278 32767
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ELE_RATIO",        0, AP_Local_Inputs, _ele_ratio, 1),


    // @Param: THR_I2C_ADDRESS
    // @DisplayName: i2c address for throttle encoder
    // @Description: The i2c address of the encoder that we use for throttle inputs. 
    // @Range: 0 255
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("THR_I2C_ADDRESS",        0, AP_Local_Inputs, _thr_address, 1),

    // @Param: THR_OFFSET
    // @DisplayName: throttle encoder offset
    // @Description: The encoder gives us values in the 0-65535 range. This value is multiplied by ratio, then offset is added to achieve our target output value.
    // @Range: -3278 32767
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("THR_OFFSET",        0, AP_Local_Inputs, _thr_offset, 1),

    // @Param: THR_RATIO
    // @DisplayName: throttle encoder ratio
    // @Description: The encoder gives us values in the 0-65535 range. This value is multiplied by ratio, then offset is added to achieve our target output value.
    // @Range: -3278 32767
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("THR_RATIO",        0, AP_Local_Inputs, _thr_ratio, 1),


    // @Param: RUD_I2C_ADDRESS
    // @DisplayName: i2c address for rudder encoder
    // @Description: The i2c address of the encoder that we use for rudder inputs. 
    // @Range: 0 255
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("RUD_I2C_ADDRESS",        0, AP_Local_Inputs, _rud_address, 1),

    // @Param: RUD_OFFSET
    // @DisplayName: rudder encoder offset
    // @Description: The encoder gives us values in the 0-65535 range. This value is multiplied by ratio, then offset is added to achieve our target output value.
    // @Range: -3278 32767
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("RUD_OFFSET",        0, AP_Local_Inputs, _rud_offset, 1),

    // @Param: RUD_RATIO
    // @DisplayName: rudder encoder ratio
    // @Description: The encoder gives us values in the 0-65535 range. This value is multiplied by ratio, then offset is added to achieve our target output value.
    // @Range: -3278 32767
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("RUD_RATIO",        0, AP_Local_Inputs, _rud_ratio, 1),


    // @Param: TILT_I2C_ADDRESS
    // @DisplayName: i2c address for tilt encoder
    // @Description: The i2c address of the encoder that we use for tilt inputs. 
    // @Range: 0 255
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TILT_I2C_ADDRESS",        0, AP_Local_Inputs, _tilt_address, 1),

    // @Param: TILT_OFFSET
    // @DisplayName: tilt encoder offset
    // @Description: The encoder gives us values in the 0-65535 range. This value is multiplied by ratio, then offset is added to achieve our target output value.
    // @Range: -3278 32767
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TILT_OFFSET",        0, AP_Local_Inputs, _tilt_offset, 1),

    // @Param: TILT_RATIO
    // @DisplayName: tilt encoder ratio
    // @Description: The encoder gives us values in the 0-65535 range. This value is multiplied by ratio, then offset is added to achieve our target output value.
    // @Range: -3278 32767
    // @Increment: 1
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("TILT_RATIO",        0, AP_Local_Inputs, _tilt_ratio, 1),

    AP_GROUPEND
};
