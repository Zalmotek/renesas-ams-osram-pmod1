# AS7343
Datasheet can be found [here](https://ams-osram.com/products/sensors/ambient-light-color-spectral-sensors/ams-as7343-spectral-sensor#Datasheets).

## Methods

#### begin
This method sets up the hardware and initializez the I2C bus.

#### setATIME
This method sets the integration time step count.

#### setASTEP
This method sets the integration time step size.

#### setGAIN
This method sets the ADC gain multiplier.

#### enableLED
This method enables the  control of an attached LED on the LDR pin.

#### setLED
This method sets the current limit for the LED.

#### readALLChannels
This method takes an array as an argument and fills it with the current measurements for Spectral channels F1-8, Clear and NIR.

### Code sample to set-up the sensor, turn on the LED at 15/255 strength and begin measurement:


    if (!as7343.begin()){
	    APP_ERR_PRINT("Could not find AS7343\n");
	    while (1) { delay(10); }
	  }

	as7343.setATIME(100);
	as7343.setASTEP(999);
	as7343.setGain(AS7343_GAIN_256X);

	as7343.enableLED(true);
	as7343.setLEDCurrent(15);

	uint16_t readings[12];

	while(true) {
		if (!as7343.readAllChannels(readings)){
		    APP_ERR_PRINT("Error reading all channels!");
		  }
		else {
			APP_PRINT("Success\n");
			delay(1000);
		}

		  APP_PRINT("ADC0/F1 415nm : %d\n", readings[0]);
		  APP_PRINT("ADC1/F2 445nm : %d\n", readings[1]);
		  APP_PRINT("ADC2/F3 480nm : %d\n", readings[2]);
		  APP_PRINT("ADC3/F4 515nm : %d\n", readings[3]);
		  APP_PRINT("ADC0/F5 555nm : %d\n", readings[6]);

		  APP_PRINT("ADC1/F6 590nm : %d\n", readings[7]);
		  APP_PRINT("ADC2/F7 630nm : %d\n", readings[8]);
		  APP_PRINT("ADC3/F8 680nm : %d\n", readings[9]);
		  APP_PRINT("ADC4/Clear    : %d\n", readings[10]);
		  APP_PRINT("ADC5/NIR      : %d\n\n", readings[11]);
	}