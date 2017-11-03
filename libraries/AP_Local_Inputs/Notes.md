

How are the basic RCInput channels initialized?

i2c example (the AP prefix is for ArduPlane):
    - libraries/AP_Airspeed/AP_Airspeed_MS4525.cpp

How does the Pixhawk Cube detect the receiver connected?

Our library that uses the i2c driver would be
```
   libraries/AP_Local_i2c_input
```   

we'll create a Sketch in examples to exercise the i2c and do address changes on the i2c devices.

```
Where are parameters defined?
   For instance for the chute library TERRAIN_ENABLE is set up in
There's a TERRAIN group defined in
   ardupilot\libraries\SITL\SITL.cpp
   AP_GROUPINFO("TERRAIN",       34, SITL,  terrain_enable, 1),
                (name, idx, clazz, element, def: 1, flags: 0)
and then we have the AP_SUBGROUPINFO in
   ardupilot\ArduPlane\Parameters.cpp
    // @Param: TERRAIN_FOLLOW
    // @DisplayName: Use terrain following
    // @Description: This enables terrain following for CRUISE mode, FBWB mode, ...
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    GSCALAR(terrain_follow, "TERRAIN_FOLLOW",  0),
```

Exercises
---------

build and run the Sketch:
```
  ardupilot\libraries\AP_InertialSensor\examples\INS_generic/INS_generic.cpp
using
  ./waf build --target examples/INS_generic
```
which creates
```
   build/px4-v3/examples/INS_generic.px4
```

which can be programmed with Mission Platter Load custom firmware
Then, in Termanal connect using APM, then press t to test.
press enter to stop the test.
You can see this menu in the INS_generic.cpp void loop(void) function.

Our first Sketch
----------------

We'll use a similar menu to do i2c address changes.

creating:
```
   AP_Local_Inputs/examples/EncoderTest/ReadEncoder.cpp
   AP_Local_Inputs/examples/EncoderTest/wscript
   AP_Local_Inputs/AP_Local_I2c_Input.h
   AP_Local_Inputs/AP_Local_I2c_Input.cpp
      Represent one i2c device with its own address, which defaluts to 0x18
Also needed to add the library to 
   ardupilot\ArduPlane\wscript   
```


to build:
```
$ ./waf configure --board px4-v3 --check-cxx-compiler g++
$ ./waf plane
$ ./waf build --target examples/EncoderTest
creates
   build/px4-v3/examples/EncoderTest.px4
```


The i2c interface described on page 22 of the datasheet gives us all of the position and error decoding specifications. We're using Datasheet_Aksim_MBA.pdf

