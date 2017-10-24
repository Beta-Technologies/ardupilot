//
// Simple test and address changes for the Beta Technologies hall-effect encoders using the i2c driver.
// (C) 2017  Anna Svagzdys, Jim Carroll
// 
// This uses the i2c drivers to talk to Hall-effect MBAD01_08 rotary encoders that will be used to 
// measure the pilot's inputs from the cockpit.
//

#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_AP_Local_I2c_Inputs/AP_Local_I2c_Inputs.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>

const AP_HAL::HAL &hal = AP_HAL::get_HAL();

// encoder-specific config
static rotary_encoder = AP_Local_I2c_Input::create();
static AP_BoardConfig BoardConfig = AP_BoardConfig::create();


void setup(void);
void loop(void);

static void read_encoder(void);
static void display_values(void);

void setup(void)
{
    // setup any board specific drivers
    BoardConfig.init();

    hal.console->printf("AP_Local_I2c_Inputs startup...\n");

    rotary_encoder.init();

    char *error = rotary_encoder.error();
    if (error != NULL) {
        hal.console->printf("Error: %s.\n\n", error);
    } else {
        // display initial values
        display_values();
    }


    hal.console->printf("Setup Complete.\n\n");
}

void read_encoder(void) 
{
    rotary_encoder.read();
}

void display_values(void) 
{
    hal.console->printf("Encoder position Raw: %i  Scaled: %i\n",
       rotary_encoder.get_raw_encoder(),
       rotary_encoder.get_encoder());
}

void loop(void)
{
    int16_t user_input;

    hal.console->printf("\n");
    hal.console->printf("%s\n",
    "Menu (press enter after selection):\n"
    "    s) sample and display encoder values\n"
    "    r) reboot");

//    "    t) sample continuously\n"

    // wait for user input
    while (!hal.console->available()) {
        hal.scheduler->delay(20);
    }

    // read in user input
    while (hal.console->available()) {
        user_input = hal.console->read();

        if (user_input == 's' || user_input == 'S') {
            read_encorder();
            display_values();
        }

        // if (user_input == 't' || user_input == 'T') {
        //     run_test();
        // }

        if (user_input == 'r' || user_input == 'R') {
            hal.scheduler->reboot(false);
        }
    }
}

static void sample_and_display(void)
{
    
}


static void display_offsets_and_scaling()
{


    const Vector3f &accel_offsets = ins.get_accel_offsets();
    const Vector3f &accel_scale = ins.get_accel_scale();
    const Vector3f &gyro_offsets = ins.get_gyro_offsets();

    // display results
    hal.console->printf("\nAccel Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                        (double)accel_offsets.x,
                        (double)accel_offsets.y,
                        (double)accel_offsets.z);
    hal.console->printf("Accel Scale X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                        (double)accel_scale.x,
                        (double)accel_scale.y,
                        (double)accel_scale.z);
    hal.console->printf("Gyro Offsets X:%10.8f \t Y:%10.8f \t Z:%10.8f\n",
                        (double)gyro_offsets.x,
                        (double)gyro_offsets.y,
                        (double)gyro_offsets.z);
}

static void run_test()
{
    Vector3f accel;
    Vector3f gyro;
    uint8_t counter = 0;
    static uint8_t accel_count = ins.get_accel_count();
    static uint8_t gyro_count = ins.get_gyro_count();
    static uint8_t ins_count = MAX(accel_count, gyro_count);

    // flush any user input
    while (hal.console->available()) {
        hal.console->read();
    }

    // clear out any existing samples from ins
    ins.update();

    // loop as long as user does not press a key
    while (!hal.console->available()) {
        // wait until we have a sample
        ins.wait_for_sample();

        // read samples from ins
        ins.update();

        // print each accel/gyro result every 50 cycles
        if (counter++ % 50 != 0) {
            continue;
        }

        // loop and print each sensor
        for (uint8_t ii = 0; ii < ins_count; ii++) {
            char state;

            if (ii > accel_count - 1) {
                // No accel present
                state = '-';
            } else if (ins.get_accel_health(ii)) {
                // Healthy accel
                state = 'h';
            } else {
                // Accel present but not healthy
                state = 'u';
            }

            accel = ins.get_accel(ii);

            hal.console->printf("%u - Accel (%c) : X:%6.2f Y:%6.2f Z:%6.2f norm:%5.2f",
                                ii, state, (double)accel.x, (double)accel.y, (double)accel.z,
                                (double)accel.length());

            gyro = ins.get_gyro(ii);

            if (ii > gyro_count - 1) {
                // No gyro present
                state = '-';
            } else if (ins.get_gyro_health(ii)) {
                // Healthy gyro
                state = 'h';
            } else {
                // Gyro present but not healthy
                state = 'u';
            }

            hal.console->printf("   Gyro (%c) : X:%6.2f Y:%6.2f Z:%6.2f\n",
                                state, (double)gyro.x, (double)gyro.y, (double)gyro.z);
        }
    }

    // clear user input
    while (hal.console->available()) {
        hal.console->read();
    }
}

AP_HAL_MAIN();
