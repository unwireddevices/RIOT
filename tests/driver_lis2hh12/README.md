# About
This is a manual test application for the LIS2HH12 accelerometer driver.

# Usage
This test application will initialize the accelerometer with its default
parameters (as defined in `drivers/lis2hh12/include/lis2hh12_params.h`:
 - `LIS2HH12_PARAM_SCALE`:  100Hz
 - `LIS2HH12_PARAM_RATE`:   2G

After initialization, the sensor reads the acceleration values every 100ms
and prints them to the STDOUT.
