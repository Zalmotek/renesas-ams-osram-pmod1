#include "AS7331.h"
#include "SparkFun_TMF882X_Library.h"
#include "AMS_OSRAM_AS7343.h"
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>

#define DELAY 4000
#define REFRESH 100

#ifdef ESP8266
#define STMPE_CS 16
#define TFT_CS   0
#define TFT_DC   15
#define SD_CS    2
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
#define STMPE_CS 32
#define TFT_CS   15
#define TFT_DC   33
#define SD_CS    14
#elif defined(TEENSYDUINO)
#define TFT_DC   10
#define TFT_CS   4
#define STMPE_CS 3
#define SD_CS    8
#elif defined(ARDUINO_STM32_FEATHER)
#define TFT_DC   PB4
#define TFT_CS   PA15
#define STMPE_CS PC7
#define SD_CS    PC5
#elif defined(ARDUINO_NRF52832_FEATHER)  /* BSP 0.6.5 and higher! */
#define TFT_DC   11
#define TFT_CS   31
#define STMPE_CS 30
#define SD_CS    27
#elif defined(ARDUINO_MAX32620FTHR) || defined(ARDUINO_MAX32630FTHR)
#define TFT_DC   P5_4
#define TFT_CS   P5_3
#define STMPE_CS P3_3
#define SD_CS    P3_2
#else
// Anything else, defaults!
#define STMPE_CS 6
#define TFT_CS   9
#define TFT_DC   10
#define SD_CS    5
#endif

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
float lsbA = 304.69f / ((float)(1 << (11 - gain))) / ((float)(1 << timeMs) / 1024.0f) / 1000.0f; // uW/cm^2
float lsbB = 398.44f / ((float)(1 << (11 - gain))) / ((float)(1 << timeMs) / 1024.0f) / 1000.0f;
float lsbC = 191.41f / ((float)(1 << (11 - gain))) / ((float)(1 << timeMs) / 1024.0f) / 1000.0f;

// Logic flags to keep track of device states
uint16_t tempData = 0, UVAData = 0, UVBData = 0, UVCData = 0, allData[4] = {0, 0, 0, 0};
float temp_C = 0;
//bool AS7331_Ready_flag = false;
uint16_t status = 0;

AMS_OSRAM_AS7343 as7343;

SparkFun_TMF882X  myTMF882X;
static struct tmf882x_msg_meas_results myResults;

AS7331 AS7331(&i2c_0); // instantiate AS7331 class

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

void setup()
{
  /* Enable USB UART */
  Serial.begin(115200);
  Serial.println("Serial enabled!");

  /* initialize two wire bus */
  I2C_BUS.begin();                // Set master mode, default on SDA/SCL for STM32L4
  I2C_BUS.setClock(400000);       // I2C frequency at 400 kHz
  delay(100);

  AS7331.powerUp();
  AS7331.reset();      // software reset before initialization
  delay(100);

  // Read the AS7331 Chip ID register, this is a good test of communication
  Serial.println("AS7331 accelerometer...");
  byte AS7331_ID = AS7331.getChipID();  // Read CHIP_ID register for AS7331
  Serial.print("AS7331 "); Serial.print("I AM "); Serial.print(AS7331_ID, HEX); Serial.print(" I should be "); Serial.println(0x21, HEX);
  Serial.println(" ");
  delay(100);

  if (AS7331_ID == 0x21) // check if AS7331 has acknowledged
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
    if (AS7331_ID != 0x21) Serial.println(" AS7331 not functioning!");
    while (1) {}; // wait here forever for a power cycle
  }

  status = AS7331.getStatus(); // reset interrupt flag before entering main loop

  if (!myTMF882X.begin())
  {
    Serial.println("Error - The TMF882X failed to initialize - is the board connected?");
    while (1);
  }
  else {
    Serial.println("TMF882X started.");
  }

  if (!as7343.begin()) {
    Serial.println("Could not find as7343");
    while (1) {
      delay(10);
    }
  }

  as7343.setATIME(100);
  as7343.setASTEP(999);
  as7343.setGain(AS7343_GAIN_256X);

  tft.begin();
  tft.setRotation(3);
  tft.drawRect(0, 0, 320, 240, ILI9341_WHITE);
  tft.fillRect(0, 0, 320, 240, ILI9341_WHITE);
}

void AS7343Data() {
  uint16_t readings[20];
  if (!as7343.readAllChannels(readings)) {
    Serial.println("Error reading all channels!");
    return;
  }

  tft.drawRect(0, 0, 320, 240, ILI9341_WHITE);
  tft.fillRect(0, 0, 320, 240, ILI9341_WHITE);

  tft.setTextColor(0x2A13, ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(220, 220);
  tft.print("Zalmotek");

  tft.setCursor(10, 20);
  tft.print("AS7343 data:");
  tft.setCursor(10, 40);
  tft.print("415nm: "); tft.print(readings[0]);
  tft.setCursor(10, 60);
  tft.print("445nm: "); tft.print(readings[1]);
  tft.setCursor(10, 80);
  tft.print("480nm: "); tft.print(readings[2]);
  tft.setCursor(10, 100);
  tft.print("515nm: "); tft.print(readings[3]);
  tft.setCursor(10, 120);
  tft.print("555nm: "); tft.print(readings[6]);
  tft.setCursor(10, 140);
  tft.print("590nm: "); tft.print(readings[7]);
  tft.setCursor(10, 160);
  tft.print("630nm: "); tft.print(readings[8]);
  tft.setCursor(10, 180);
  tft.print("680nm: "); tft.print(readings[9]);

  long long start = millis();
  while (millis() - start < DELAY) {
    //delay(REFRESH);
    if (!as7343.readAllChannels(readings)) {
      Serial.println("Error reading all channels!");
      return;
    }
    tft.setCursor(10, 40);
    tft.print("415nm: "); tft.print(readings[0]);
    tft.setCursor(10, 60);
    tft.print("445nm: "); tft.print(readings[1]);
    tft.setCursor(10, 80);
    tft.print("480nm: "); tft.print(readings[2]);
    tft.setCursor(10, 100);
    tft.print("515nm: "); tft.print(readings[3]);
    tft.setCursor(10, 120);
    tft.print("555nm: "); tft.print(readings[6]);
    tft.setCursor(10, 140);
    tft.print("590nm: "); tft.print(readings[7]);
    tft.setCursor(10, 160);
    tft.print("630nm: "); tft.print(readings[8]);
    tft.setCursor(10, 180);
    tft.print("680nm: "); tft.print(readings[9]);
  }
}

void TMF8828Data() {
  tft.drawRect(0, 0, 240, 320, ILI9341_WHITE);
  tft.fillRect(0, 0, 240, 320, ILI9341_WHITE);
  
  tft.setTextColor(0x2A13, ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(220, 220);
  tft.print("Zalmotek");

  if (myTMF882X.startMeasuring(myResults))
  {
    tft.setCursor(10, 20);
    tft.print("TMF8828 data:");
    tft.setCursor(10, 40);
    tft.print("ch1: "); tft.print(myResults.results[0].distance_mm); tft.print("mm");
    tft.setCursor(10, 60);
    tft.print("ch2: "); tft.print(myResults.results[1].distance_mm); tft.print("mm");
    tft.setCursor(10, 80);
    tft.print("ch3: "); tft.print(myResults.results[2].distance_mm); tft.print("mm");
    tft.setCursor(10, 100);
    tft.print("ch4: "); tft.print(myResults.results[3].distance_mm); tft.print("mm");
    tft.setCursor(10, 120);
    tft.print("ch5: "); tft.print(myResults.results[4].distance_mm); tft.print("mm");
    tft.setCursor(10, 140);
    tft.print("ch6: "); tft.print(myResults.results[5].distance_mm); tft.print("mm");
    tft.setCursor(10, 160);
    tft.print("ch7: "); tft.print(myResults.results[6].distance_mm); tft.print("mm");
    tft.setCursor(10, 180);
    tft.print("ch8: "); tft.print(myResults.results[7].distance_mm); tft.print("mm");
    tft.setCursor(10, 200);
    tft.print("ch9: "); tft.print(myResults.results[8].distance_mm); tft.print("mm");
  }
  
  long long start = millis();
  while (millis() - start < DELAY) {
    //delay(REFRESH);
    if (myTMF882X.startMeasuring(myResults))
    {
      tft.setCursor(10, 40);
      tft.print("ch1: "); tft.print(myResults.results[0].distance_mm); tft.print("mm     ");
      tft.setCursor(10, 60);
      tft.print("ch2: "); tft.print(myResults.results[1].distance_mm); tft.print("mm     ");
      tft.setCursor(10, 80);
      tft.print("ch3: "); tft.print(myResults.results[2].distance_mm); tft.print("mm     ");
      tft.setCursor(10, 100);
      tft.print("ch4: "); tft.print(myResults.results[3].distance_mm); tft.print("mm     ");
      tft.setCursor(10, 120);
      tft.print("ch5: "); tft.print(myResults.results[4].distance_mm); tft.print("mm     ");
      tft.setCursor(10, 140);
      tft.print("ch6: "); tft.print(myResults.results[5].distance_mm); tft.print("mm     ");
      tft.setCursor(10, 160);
      tft.print("ch7: "); tft.print(myResults.results[6].distance_mm); tft.print("mm     ");
      tft.setCursor(10, 180);
      tft.print("ch8: "); tft.print(myResults.results[7].distance_mm); tft.print("mm     ");
      tft.setCursor(10, 200);
      tft.print("ch9: "); tft.print(myResults.results[8].distance_mm); tft.print("mm     ");
    }
  }
}

void AS7331Data() {
  status = AS7331.getStatus();
  if (status & 0x0008) {

    AS7331.readAllData(allData);
    tempData = allData[0];
    UVAData  = allData[1];
    UVBData  = allData[2];
    UVCData  = allData[3];
  }
  temp_C = tempData * 0.05f - 66.9f;
  
  tft.drawRect(0, 0, 320, 240, ILI9341_WHITE);
  tft.fillRect(0, 0, 320, 240, ILI9341_WHITE);

  tft.setTextColor(0x2A13, ILI9341_WHITE);
  tft.setTextSize(2);
  tft.setCursor(220, 220);
  tft.print("Zalmotek");
  tft.setTextSize(4);

  tft.setCursor(10, 20);
  tft.print("AS7331 data:");
  tft.setCursor(10, 60);
  tft.print("UVA: "); tft.print((float)(UVAData)*lsbA); //tft.print("uW/cm^2");
  tft.setCursor(10, 100);
  tft.print("UVB: "); tft.print((float)(UVBData)*lsbB); //tft.print("uW/cm^2");
  tft.setCursor(10, 140);
  tft.print("UVC: "); tft.print((float)(UVCData)*lsbC); //tft.print("uW/cm^2");
  tft.setCursor(10, 180);
  tft.print("temp: "); tft.print((int)temp_C); 
  tft.setTextSize(2);
  tft.print("o");
  tft.setTextSize(4);
  tft.print("C");

  long long start = millis();
  while (millis() - start < DELAY) {
    //delay(REFRESH);
    status = AS7331.getStatus();
    if (status & 0x0008) {
  
      AS7331.readAllData(allData);
      tempData = allData[0];
      UVAData  = allData[1];
      UVBData  = allData[2];
      UVCData  = allData[3];
    }
    temp_C = tempData * 0.05f - 66.9f;

    tft.setCursor(10, 60);
    tft.print("UVA: "); tft.print((float)(UVAData)*lsbA); //tft.print("uW/cm^2");
    tft.setCursor(10, 100);
    tft.print("UVB: "); tft.print((float)(UVBData)*lsbB); //tft.print("uW/cm^2");
    tft.setCursor(10, 140);
    tft.print("UVC: "); tft.print((float)(UVCData)*lsbC); //tft.print("uW/cm^2");
    tft.setCursor(10, 180);
    tft.print("temp: "); tft.print((int)temp_C); 
    tft.setTextSize(2);
    tft.print("o");
    tft.setTextSize(4);
    tft.print("C");
  }
}

void loop()
{
  AS7343Data();
  TMF8828Data();
  AS7331Data();
}
