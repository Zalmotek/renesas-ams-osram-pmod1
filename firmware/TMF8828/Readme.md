# TMF8828

Datasheet can be found [here](https://ams-osram.com/products/sensors/distance-proximity-sensors/ams-tmf8828-configurable-8x8-multi-zone-time-of-flight-sensor#Datasheets).

## Methods

#### factoryCalibration 

This method allows the user to switch between 3 pre-defined configurations:  
- Configuration 0: Period 132 ms, KiloIterations = 250
- Configuration 1: Period 264 ms, KiloIteratiosn = 500
- Configuration 2: Period 528 ms, KiloIterations = 1000
  
You can modify the configurations (e.g. choose different period or Kilo iterations) in the application source file, recompile and upload your 
own configuration in the *tmf8828_calib.h* file. 

The factory calibration method takes as arguments an integer value between 0 and 2 corresponding to the desired configuration. 

#### startMeasuring

This method puts the sensor in measurement mode. Note that factory calibration must be done before using this method. 

#### update 

The Update method takes as arguments two integer arrays correspondings to the confidence values and distane values acquired by the sensor. This method returns the number of points in which the measurement took place.

### Code example to run factory calibration, start the measurement and print the acquired values via RTT. 

    TMF8828 sensor;
    sensor.factoryCalibration(1); 
    sensor.startMeasuring();
    int conf[32], dist[32];
    while (true) {
        int cnt = sensor.update(conf, dist);
        for (int i = 0; i < cnt; i++) {
            APP_PRINT("conf: %d, dist: %d\n", conf[i], dist[i]);
        }
        if (cnt) {
            APP_PRINT("\n");
        }
    }