//
// Simple test and address changes for the Beta Technologies hall-effect encoders using the i2c driver.
// (C) 2017  Anna Svagzdys, Jim Carroll
// 
// This uses the i2c drivers to talk to Hall-effect MBAD01_08 rotary encoders that will be used to 
// measure the pilot's inputs from the cockpit.
//

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Local_I2c_Inputs/AP_Local_I2c_Input.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

// encoder-specific config
//static AP_Local_I2c_Input rotary_encoder = AP_Local_I2c_Input();
AP_Local_I2c_Input rotary_encoder;
static AP_BoardConfig BoardConfig = AP_BoardConfig::create();


void setup(void);
void loop(void);

static void initialize_encoder(void);
static void read_encoder(void);
static void display_values(void);
static void run_test(void);

void setup(void)
{
    // setup any board specific drivers
    BoardConfig.init();

    hal.console->printf("AP_Local_I2c_Inputs startup...\n");

    hal.console->printf("Setup Complete.\n\n");    
}

void initialize_encoder(void) 
{
    if (!rotary_encoder.init()) {
         // print the error
        hal.console->printf("Init error.\n");
    } else {
        hal.console->printf("Init complete.\n");
    }
    hal.console->printf("Last error: %s.\n", rotary_encoder.last_error());
}

void read_encoder(void) 
{
    rotary_encoder.read_position();
}

void display_values(void) 
{
    hal.console->printf("Encoder position Raw: %i  Scaled: %i    %s \n",
       rotary_encoder.get_raw_encoder(),
       rotary_encoder.get_encoder(),
       rotary_encoder.last_error());
}

void run_test(void)
{
    while (hal.console->available()) {
        hal.console->read();
    }

    while (!hal.console->available()) {
        read_encoder();
        display_values();
        //hal.console->printf("Last error: %s.\n", rotary_encoder.lastError());
        hal.scheduler->delay(200);
    }

    while (hal.console->available()) {
        hal.console->read();
    }
}

void loop(void)
{
    int16_t user_input;

    hal.console->printf("\n");
    hal.console->printf("%s\n",
    "Menu (press enter after selection):\n"
    "    i) initialize encoder\n"
    "    r) read  encoder\n"
    "    t) continous display test\n"
    "    b) reboot");

    // wait for user input
    while (!hal.console->available()) {
        hal.scheduler->delay(20);
    }

    // read in user input
    while (hal.console->available()) {
        user_input = hal.console->read();

        if (user_input == 'i' || user_input == 'I') {
            initialize_encoder();
        }

        if (user_input == 'r' || user_input == 'R') {
            read_encoder();
            display_values();
        }

        if (user_input == 't' || user_input == 'T') {
            run_test();
        }

        if (user_input == 'b' || user_input == 'B') {
            hal.scheduler->reboot(false);
        }
    }
}

AP_HAL_MAIN();
