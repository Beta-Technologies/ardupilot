

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
   AP_Local_I2c_Inputs/examples/EncoderTest/ReadEncoder.cpp
   AP_Local_I2c_Inputs/examples/EncoderTest/wscript
   AP_Local_I2c_Inputs/AP_Local_I2c_Input.h
   AP_Local_I2c_Inputs/AP_Local_I2c_Input.cpp
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

The hall effect sensor position output doesn't seem to match anything in the data sheet.
What we did was set it to read bytes and display them in hex format. Fiddling with the sensor
we see the following output.

```
  The output doesn't seem to correspond to the datasheet we have for the Aksim MBA sensor
  What we do see is a continuous stream (we don't need to send a '1' read command).
  Bytes 0, 1: First two bytes are position, big-endian
  Byte 2 is status
    0x06: head is too close to ring
    0x00: good spacing
    0x05: head is too far from ring
    0x08: head is much too far (position will be FF FF)
  Byte 3: is detailed status, can't figure out the pattern yet
  Byte 4: changes wildly with any movement
```


