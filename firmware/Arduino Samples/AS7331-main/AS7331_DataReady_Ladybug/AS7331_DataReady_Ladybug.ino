/* 1/28/23 Copyright Tlera Corporation
 *  
 *  Created by Kris Winer   
 *  
 *  The AS7331 is a three-channel UV light sensor with separate photodiodes sensitive to UVA, UVB, and UVC
 *  light radiation.
 *  
 *  Library may be used freely and without limit with attribution.
 *  
 */
#include "AS7331.h"

#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use

I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

bool SerialDebug = true;

// battery voltage monitor definitions
float VDDA, VBAT, VBUS, Temperature;

//AS7331 definitions
//#define AS7331_intPin   8    // interrupt pin definitions 

// Specify sensor parameters //
MMODE   mmode = AS7331_CONT_MODE;  // choices are modes are CONT, CMD, SYNS, SYND
CCLK    cclk  = AS7331_1024;      // choices are 1.024, 2.048, 4.096, or 8.192 MHz
uint8_t sb    = 0x01;             // standby enabled 0x01 (to save power), standby disabled 0x00                    
uint8_t breakTime = 40;           // sample timeMs == 8 us x breaktimeMs (0 - 255, or 0 - 2040 us range), CONT or SYNX modes

uint8_t gain = 8; // ADCGain = 2^(11-gain), by 2s, 1 - 2048 range,  0 < gain = 11 max, default 10
uint8_t timeMs = 9; // 2^time in ms, so 0x07 is 2^6 = 64 ms, 0 < time = 15 max, default  6

// sensitivities at 1.024 MHz clock
float lsbA = 304.69f / ((float)(1 << (11 - gain))) / ((float)(1 << timeMs)/1024.0f) / 1000.0f;  // uW/cm^2
float lsbB = 398.44f / ((float)(1 << (11 - gain))) / ((float)(1 << timeMs)/1024.0f) / 1000.0f;
float lsbC = 191.41f / ((float)(1 << (11 - gain))) / ((float)(1 << timeMs)/1024.0f) / 1000.0f;

// Logic flags to keep track of device states
uint16_t tempData= 0, UVAData = 0, UVBData = 0, UVCData = 0, allData[4] = {0, 0, 0, 0};
float temp_C = 0;
//bool AS7331_Ready_flag = false;
uint16_t status = 0;

AS7331 AS7331(&i2c_0); // instantiate AS7331 class


void setup()
{
  /* Enable USB UART */
  Serial.begin(115200);
//  Serial.blockOnOverrun(false);  
  delay(4000);
  Serial.println("Serial enabled!");

//  pinMode(AS7331_intPin, INPUT);  // define AS7331 data ready interrupt

  /* initialize two wire bus */
  I2C_BUS.begin();                // Set master mode, default on SDA/SCL for STM32L4
  I2C_BUS.setClock(400000);       // I2C frequency at 400 kHz
  delay(1000);

  Serial.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                // should detect AS7331 at 0x14 and BME280 at 0x77
  delay(1000);

   AS7331.powerUp();
   AS7331.reset();      // software reset before initialization
   delay(100); 

  // Read the AS7331 Chip ID register, this is a good test of communication
  Serial.println("AS7331 accelerometer...");
  byte AS7331_ID = AS7331.getChipID();  // Read CHIP_ID register for AS7331
  Serial.print("AS7331 "); Serial.print("I AM "); Serial.print(AS7331_ID, HEX); Serial.print(" I should be "); Serial.println(0x21, HEX);
  Serial.println(" ");
  delay(1000); 

  if(AS7331_ID == 0x21) // check if AS7331 has acknowledged
   {
   Serial.println("AS7331 is online..."); Serial.println(" ");

   Serial.print("Sensitivity channel A = "); Serial.print(lsbA, 5); Serial.println(" uW/cm^2");
   Serial.print("Sensitivity channel B = "); Serial.print(lsbB, 5); Serial.println(" uW/cm^2");
   Serial.print("Sensitivity channel C = "); Serial.print(lsbC, 5); Serial.println(" uW/cm^2");
   
   AS7331.setConfigurationMode();
   AS7331.init(mmode, cclk, sb, breakTime, gain, timeMs);
   delay(100); // let sensor settle
   AS7331.setMeasurementMode();

  }
  else 
  {
   if(AS7331_ID != 0x21) Serial.println(" AS7331 not functioning!");
   while(1){}; // wait here forever for a power cycle
  }

//  attachInterrupt(AS7331_intPin, myinthandler, RISING);  // attach data ready INT pin output of AS7331

  status = AS7331.getStatus(); // reset interrupt flag before entering main loop
  
}/* end of setup */


void loop()
{
  /* AS7331 data ready detect*/
//  if(AS7331_Ready_flag)
//  {
//   AS7331_Ready_flag = false;    // clear the ready flag  

   status = AS7331.getStatus();

//   // Error handling
//   if(status & 0x0080) Serial.println("overflow of internal time reference!");
//   if(status & 0x0040) Serial.println("overflow of measurement register(s)!");
//   if(status & 0x0020) Serial.println("overflow of internal conversion channel(s)!");
//   if(status & 0x0010) Serial.println("measurement results overwritten!");
//   if(status & 0x0004) Serial.println("measurement in progress!");
  
   if(status & 0x0008) {  // when data ready
   // Serial.print("status = 0x"); Serial.println(status, HEX);

   // UVAData = AS7331.readUVAData(); // read data in four separate I2C transactions takes 590 us 
   // UVBData = AS7331.readUVBData();  
   // UVCData = AS7331.readUVCData();
   // tempData = AS7331.readTempData();

   AS7331.readAllData(allData); // burst read data in one I2C transaction takes 281 us
   tempData = allData[0];
   UVAData  = allData[1];
   UVBData  = allData[2];
   UVCData  = allData[3];
//   }
   
   Serial.println("Raw counts");
   Serial.print("AS7331 UVA = "); Serial.println(UVAData);  
   Serial.print("AS7331 UVB = "); Serial.println(UVBData);  
   Serial.print("AS7331 UVC = "); Serial.println(UVCData);  
   Serial.println(" ");
   
   Serial.println("Scaled UV data");
   Serial.print("AS7331 UVA (uW/cm^2)= "); Serial.println((float)(UVAData)*lsbA);  
   Serial.print("AS7331 UVB (uW/cm^2)= "); Serial.println((float)(UVBData)*lsbB);  
   Serial.print("AS7331 UVC (uW/cm^2)= "); Serial.println((float)(UVCData)*lsbC);  
   Serial.println(" ");

   temp_C = tempData * 0.05f - 66.9f;
   Serial.print("AS7331 Temperature = "); Serial.print(temp_C, 2); Serial.println(" C");
   Serial.println(" ");

  } /* end of AS7331 data ready interrupt handling*/
}  /* end of loop*/

/* Useful functions */
//void myinthandler()
//{
//  AS7331_Ready_flag = true; 
//}
