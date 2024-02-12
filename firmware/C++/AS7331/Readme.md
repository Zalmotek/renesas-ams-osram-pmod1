# AS7331

Datasheet can be found [here](https://ams-osram.com/products/sensors/ambient-light-color-spectral-sensors/ams-as7331-spectral-uv-sensor#Datasheets).

## Methods

#### Power Up
This method wakes up the device. 

#### Reset
This method software resets the device.

#### getChipID

This method gets the chipID.

#### setConfigurationMode

This method puts the sensor in configuration mode.

#### init

This function takes 6 arguments: 
1. mmode

Can have 2 possible values: 0x00 for continuous mode and 0x01 for one-time measurement.

2. cclk

Can have 4 possible values: 0x00 for 1024, 0x01 for 2047, 0x02 for 4096 and 0x03 for 8192.

3. sb

Can have 2 possible values: 0x01 to enable standby or 0x00 to disable standby.

4. breakTime
5. gain
6. time

breakTime gain and time are variables that impact the sensor's sesnitivity to light and depending on the conditions in which the measurement takes place, the datasheet must be consulted as to pick appropiate values.

To replicate the experiment in the video demo, which takes place in an office space, the following values were picked:

*breakTime = 40;*

*gain = 8;* 

*timeMs = 9;*

#### setMeasurementMode

This method puts the sensor in measurement mode.

#### getStatus

This method checks the status of the device and returns it.

#### readAllData

This method takes as argument an array and fills it with the UV-A, UV-b, UV-C, and Temperature values. 


### Code example to power up, configure, and initialize the device, followed by entering measuring mode and printing the values via RTT. 
    MMODE   mmode = AS7331_CONT_MODE;
    CCLK    cclk  = AS7331_1024;
    uint8_t sb    = 0x01;
    uint8_t breakTime = 40;
    uint8_t gain = 8;
    uint8_t timeMs = 9;
    float scaleFactorA = 304.69f / ((float)(1 << (11 - gain))) / ((float)(1 << timeMs)/1024.0f) / 1000.0f;
    float scaleFactorB = 398.44f / ((float)(1 << (11 - gain))) / ((float)(1 << timeMs)/1024.0f) / 1000.0f;
    float scaleFactorC = 191.41f / ((float)(1 << (11 - gain))) / ((float)(1 << timeMs)/1024.0f) / 1000.0f;

    uint16_t tempData= 0, UVAData = 0, UVBData = 0, UVCData = 0, allData[4] = {0, 0, 0, 0};
    float temp_C = 0;
    int status;


    I2Cdev i2c_0;
    AS7331 AS7331(&i2c_0);

    AS7331.powerUp();
	AS7331.reset();
	delay(100);
	uint8_t AS7331_ID = AS7331.getChipID();
	//APP_PRINT("ID: %d\n", AS7331_ID);

	// check if AS7331 has acknowledged
	if(AS7331_ID == 0x21)  {
		AS7331.setConfigurationMode();
		AS7331.init(mmode, cclk, sb, breakTime, gain, timeMs);
		delay(100); // let sensor settle
		AS7331.setMeasurementMode();
	}
	else {
		APP_ERR_PRINT("Wrong id: %d\n", AS7331_ID);
	}

	while (true) {
		status = AS7331.getStatus();
		//APP_PRINT("Current status: %d\n", status);

		if (status & 0x0008) {
			AS7331.readAllData(allData); 
			tempData = allData[0];
			UVAData  = allData[1];
			UVBData  = allData[2];
			UVCData  = allData[3];

			APP_PRINT("Raw counts\n");
			APP_PRINT("AS7331 UVA = %d\n", UVAData);
			APP_PRINT("AS7331 UVB = %d\n", UVBData);
			APP_PRINT("AS7331 UVC = %d\n\n", UVCData);

			APP_PRINT("Scaled UV data\n");
			APP_PRINT("AS7331 UVA (uW/cm^2)= %d", (int)((float)(UVAData)*scaleFactorA));
			APP_PRINT(".%d\n", (int)((float)(UVAData) * scaleFactorA * 100 ) % 100);
			APP_PRINT("AS7331 UVB (uW/cm^2)= %d", (int)((float)(UVBData)*scaleFactorB));
			APP_PRINT(".%d\n", (int)((float)(UVBData) * scaleFactorB * 100) % 100);
			APP_PRINT("AS7331 UVC (uW/cm^2)= %d", (int)((float)(UVCData)*scaleFactorC));
			APP_PRINT(".%d\n\n", (int)((float)(UVCData) * scaleFactorC * 100)  % 100);

			temp_C = tempData * 0.05f - 66.9f;
			APP_PRINT("AS7331 Temperature = %d", (int)temp_C);
			APP_PRINT(".%d C\n\n", (int)((float)(temp_C) * 100)  % 100);
		}
	}


