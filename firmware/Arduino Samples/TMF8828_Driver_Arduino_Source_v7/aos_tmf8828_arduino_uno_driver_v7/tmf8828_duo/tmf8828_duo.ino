/*
 *****************************************************************************
 * Copyright by ams OSRAM AG                                                 *
 * All rights are reserved.                                                  *
 *                                                                           *
 * IMPORTANT - PLEASE READ CAREFULLY BEFORE COPYING, INSTALLING OR USING     *
 * THE SOFTWARE.                                                             *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT         *
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS         *
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT  *
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,     *
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT          *
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,     *
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY     *
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT       *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE     *
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.      *
 *****************************************************************************
 */
//
// tmf8828 duo - use 2 tmf8828
//

// ---------------------------------------------- includes ----------------------------------------

#include <Wire.h>
#include "registers_i2c.h"
#include "tmf8828.h"
#include "tmf8828_calib.h"


// ---------------------------------------------- defines -----------------------------------------

// for 2 tmf8828 the maximum amount of data cannot be transferred with 115200 baud
#define UART_BAUD_RATE              1000000UL
#define VERSION                     6

// arduino uno can only do 400KHz I2C
#define I2C_CLK_SPEED               400000

// how much logging we want
#define MY_LOG_LEVEL                LOG_LEVEL_INFO

// tmf states
#define TMF8828_STATE_DISABLED      0
#define TMF8828_STATE_STANDBY       1     
#define TMF8828_STATE_STOPPED       2
#define TMF8828_STATE_MEASURE       3
#define TMF8828_STATE_ERROR         4

// support 2 tmf8828, with adjacent i2c slave addresses
#define I2C_SLAVE_ADDR_A            (TMF8828_SLAVE_ADDR+1)      // A device get new address
#define I2C_SLAVE_ADDR_B            (TMF8828_SLAVE_ADDR)

// convenience macros to access the 2 instances as pointers
#define TMF8828_A               ( &(tmf8828[0]) )
#define TMF8828_B               ( &(tmf8828[1]) )

// start the 2nd tmf8828 to measure after the first one only after this time, to allow the 
// arduino to readout the results fast enough
#define TMF8828_DELAY_FOR_START_OF_2ND    33000UL

// ---------------------------------------------- constants -----------------------------------------

// for each configuration specifiy a period in milli-seconds
const uint16_t configPeriod[3] = {132, 264, 528};

// for each configuration specify the number of Kilo Iterations (Kilo = 1024)
const uint16_t configKiloIter[3] = {250, 500, 1000};


// ---------------------------------------------- variables -----------------------------------------

tmf8828Driver tmf8828[ 2 ];      // two instances of tmf8828

extern uint8_t logLevel;          // for i2c logging in shim 

int8_t stateTmf8828;              // current state of the device 
int8_t configNr;                  // this sample application has only a few configurations it will loop through, the variable keeps track of that
int8_t clkCorrectionOn;           // if non-zero clock correction is on
int8_t dumpHistogramOn;           // if non-zero, dump all histograms


// ---------------------------------------------- function declaration ------------------------------

// Select one of the available configurations and configure the device accordingly.
void configure( );

// Function to print the content of these registers, len should be a multiple of 8
void printRegisters( uint8_t regAddr, uint16_t len, char seperator, uint8_t calibId );

// Print the current state (stateTmf8828) in a readable format
void printState( );

// Function prints a help screen
void printHelp( );

// Function checks the UART for received characters and interprets them
void serialInput( );

// Arduino setup function is only called once at startup. Do all the HW initialisation stuff here.
void setup( );

// Arduino main loop function, is executed cyclic
void loop( );


// ---------------------------------------------- functions -----------------------------------------

// wrap through the available configurations and configure the device accordingly.
void configure ( )
{
  configNr = configNr + 1;
  if ( configNr > 2 )
  {
    configNr = 0;     // wrap around
  }
  // always re-configure both devices
  if (  APP_SUCCESS_OK == tmf8828Configure( TMF8828_A, configPeriod[configNr], configKiloIter[configNr], dumpHistogramOn )  
     && APP_SUCCESS_OK == tmf8828Configure( TMF8828_B, configPeriod[configNr], configKiloIter[configNr], dumpHistogramOn )
     )
  {
    PRINT_STR( "#Conf" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "Period=" );
    PRINT_INT( configPeriod[configNr] );
    PRINT_STR( "ms" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "KIter=" );
    PRINT_INT( configKiloIter[configNr] );
  }
  else
  {
    PRINT_STR( "#Err#" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "Config" );      
  }
  PRINT_LN( );
}

// Print the current state (stateTmf8828) in a readable format
// Both tmf8828 should always be in the same state, we only have one state variable
void printState ( )
{
  PRINT_STR( "state=" );
  switch ( stateTmf8828 )
  {
    case TMF8828_STATE_DISABLED: PRINT_STR( "disabled" ); break;
    case TMF8828_STATE_STANDBY: PRINT_STR( "standby" ); break;
    case TMF8828_STATE_STOPPED: PRINT_STR( "stopped" ); break;
    case TMF8828_STATE_MEASURE: PRINT_STR( "measure" ); break;
    case TMF8828_STATE_ERROR: PRINT_STR( "error" ); break;   
    default: PRINT_STR( "???" ); break;
  }
  PRINT_LN( );
}

// print registers either as c-struct or plain
void printRegisters ( uint8_t slaveAddr, uint8_t regAddr, uint16_t len, char seperator, uint8_t calibId )
{
  uint8_t buf[8];
  uint16_t i;
  if ( seperator == ',' )
  {
    PRINT_STR( "const PROGMEM uint8_t tmf8828_calib_" );
    PRINT_INT( calibId );
    PRINT_STR( "[] = {" );
    PRINT_LN( );
  }
  for ( i = 0; i < len; i += 8 )            // if len is not a multiple of 8, we will print a bit more registers ....
  {
    uint8_t * ptr = buf;    
    i2c_rx( slaveAddr, regAddr, buf, 8 );
    if ( seperator == ' ' )
    {
      PRINT_STR( "0x" );
      PRINT_INT_HEX( regAddr );
      PRINT_STR( ": " );
    }
    PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator ); 
    PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator ); 
    PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator ); 
    PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator ); 
    PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator ); 
    PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator ); 
    PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator ); 
    PRINT_STR( " 0x" ); PRINT_INT_HEX( *ptr++ ); PRINT_CHAR( seperator ); 
    PRINT_LN( );
    regAddr = regAddr + 8;
  }
  if ( seperator == ',' )
  {
    PRINT_STR( "};" );
    PRINT_LN( );
  }
}

// -------------------------------------------------------------------------------------------------------------
// Function prints a help screen
void printHelp ( )
{
  PRINT_STR( "TMF8828 Arduino=" );
  PRINT_INT( VERSION ); 
#if 0                                             // a baudrate of 1Mbaud requires more stack, so need to free some space on RAM
  PRINT_LN( ); PRINT_STR( "h ... help " );
  PRINT_LN( ); PRINT_STR( "e ... enable device and download FW" );
  PRINT_LN( ); PRINT_STR( "d ... disable device" );
  PRINT_LN( ); PRINT_STR( "w ... wakeup" );
  PRINT_LN( ); PRINT_STR( "p ... power down" );
  PRINT_LN( ); PRINT_STR( "m ... measure" );
  PRINT_LN( ); PRINT_STR( "s ... stop measure" );
  PRINT_LN( ); PRINT_STR( "c ... change conf" );
  PRINT_LN( ); PRINT_STR( "f ... do fact calib" );
  PRINT_LN( ); PRINT_STR( "l ... load fact calib" );
  PRINT_LN( ); PRINT_STR( "r ... restore fact calib from file" );  
  PRINT_LN( ); PRINT_STR( "x ... clock corr on/off" );
  PRINT_LN( ); PRINT_STR( "a ... dump registers" );
  PRINT_LN( ); PRINT_STR( "z ... histogram" );
#endif
  PRINT_LN( );
}

// function that enables one tmf8828 and downloads the fw and switches to 8x8 mode
static int8_t enableOneTmf8828 ( tmf8828Driver * driver )
{
  tmf8828Enable( driver );
  delay_in_microseconds( ENABLE_TIME_MS * 1000 );
  tmf8828ClkCorrection( driver, clkCorrectionOn ); 
  tmf8828SetLogLevel( driver, MY_LOG_LEVEL );
  tmf8828Wakeup( driver );
  if ( tmf8828IsCpuReady( driver, CPU_READY_TIME_MS ) )
  {
    if ( tmf8828DownloadFirmware( driver ) == BL_SUCCESS_OK ) 
    {
      if ( APP_SUCCESS_OK != tmf8828SwitchTo8x8Mode( driver ) )
      {
        PRINT_STR( "#Err" );
        PRINT_CHAR( SEPERATOR );
        PRINT_STR( "no tmf8828" );
        PRINT_LN( );
      }
      return APP_SUCCESS_OK;
    }
  }
  return APP_ERROR_CMD;                                         // something severe went wrong, cannot use 2 tmf8828
}

// function will return the factory calibration matching the id
static const uint8_t * getPrecollectedFactoryCalibration ( uint8_t id )
{
  const uint8_t * factory_calib = tmf8828_calib_0;
  switch ( id )
  {
    case 1: factory_calib = tmf8828_calib_1; break;      
    case 2: factory_calib = tmf8828_calib_2; break;      
    case 3: factory_calib = tmf8828_calib_3; break;      
    case 4: factory_calib = tmf8828_calib_4; break;      
    case 5: factory_calib = tmf8828_calib_5; break;      
    case 6: factory_calib = tmf8828_calib_6; break;      
    case 7: factory_calib = tmf8828_calib_7; break;      
  } // default is calib 0
  return factory_calib;
}

// perform factory calibration for one tmf8828
static void factoryCalibrationOneTmf8828 ( tmf8828Driver * driver )
{
  tmf8828Configure( driver, 1, 4000, 0 );    // no histogram dumping in factory calibration allowed, 4M iterations for factory calibration recommended
  tmf8828ResetFactoryCalibration( driver );
  if (  APP_SUCCESS_OK == tmf8828FactoryCalibration( driver )      // walk through all 4 calibration  
     && APP_SUCCESS_OK == tmf8828FactoryCalibration( driver )
     && APP_SUCCESS_OK == tmf8828FactoryCalibration( driver )
     && APP_SUCCESS_OK == tmf8828FactoryCalibration( driver )
     )
  {
    PRINT_STR( "Fact calib" );
  }
  else
  {
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "fact calib" );
    PRINT_LN( );
  }   
  PRINT_LN( ); 
  tmf8828Configure( driver, configPeriod[configNr], configKiloIter[configNr], dumpHistogramOn );
}

static void loadFactoryCalibrationOneTmf8828 ( tmf8828Driver * driver, uint8_t idx )
{
  tmf8828ResetFactoryCalibration( driver );
  tmf8828LoadConfigPageFactoryCalib( driver );
  printRegisters( driver->i2cSlaveAddress, 0x20, 0xE0-0x20, ',', idx + 0 );  
  tmf8828WriteConfigPage( driver );                // advance to next calib page
  tmf8828LoadConfigPageFactoryCalib( driver );
  printRegisters( driver->i2cSlaveAddress, 0x20, 0xE0-0x20, ',', idx + 1 );  
  tmf8828WriteConfigPage( driver );                // advance to next calib page
  tmf8828LoadConfigPageFactoryCalib( driver );
  printRegisters( driver->i2cSlaveAddress, 0x20, 0xE0-0x20, ',', idx + 2 );  
  tmf8828WriteConfigPage( driver );                // advance to next calib page
  tmf8828LoadConfigPageFactoryCalib( driver );
  printRegisters( driver->i2cSlaveAddress, 0x20, 0xE0-0x20, ',', idx + 3 );  
  tmf8828WriteConfigPage( driver );                // advance to next calib page
}

static void setStoredFactoryCalibration ( tmf8828Driver * driver, uint8_t idx )
{
  if (  APP_SUCCESS_OK == tmf8828ResetFactoryCalibration( driver )                                             // First reset, then load all 4 calib pages
     && APP_SUCCESS_OK == tmf8828SetStoredFactoryCalibration( driver, getPrecollectedFactoryCalibration( idx + 0 ) )
     && APP_SUCCESS_OK == tmf8828SetStoredFactoryCalibration( driver, getPrecollectedFactoryCalibration( idx + 1 ) )
     && APP_SUCCESS_OK == tmf8828SetStoredFactoryCalibration( driver, getPrecollectedFactoryCalibration( idx + 2 ) )
     && APP_SUCCESS_OK == tmf8828SetStoredFactoryCalibration( driver, getPrecollectedFactoryCalibration( idx + 3 ) )
     )             
  {   
    PRINT_STR( "Set fact calib" );
  }
  else
  {
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "loadCalib" );  
  }
  PRINT_LN( );
}

// -------------------------------------------------------------------------------------------------------------

// Function checks the UART for received characters and interprets them
void serialInput ( )
{
  while ( Serial.available() )
  {
    char rx = Serial.read();
    if ( rx < 33 || rx >=126 ) // skip all control characters and DEL  
    {
      continue; // nothing to do here
    }
    else
    { 
      if ( rx == 'h' )
      {
          printHelp(); 
      }
      else if ( rx == 'c' ) // show and use next configuration
      {
        if ( stateTmf8828 == TMF8828_STATE_STOPPED )
        {
          configure( );
        }
      }
      else if ( rx == 'e' ) // enable
      {  
        if ( stateTmf8828 == TMF8828_STATE_DISABLED )
        {
          stateTmf8828 = TMF8828_STATE_ERROR;                                                   // if one of below checks fail, we go to error state
          if ( APP_SUCCESS_OK == enableOneTmf8828( TMF8828_A ) )
          {
            if ( APP_SUCCESS_OK == tmf8828ChangeI2CAddress( TMF8828_A, I2C_SLAVE_ADDR_A ) )     // tmf8828 A now listens to new i2c slave address
            {
              if ( APP_SUCCESS_OK == enableOneTmf8828( TMF8828_B ) )
              {
                stateTmf8828 = TMF8828_STATE_STOPPED;
                configNr = 2; // do a wrap around
                configure();// do a default configuration for arduino uno
                printHelp();                                                                    // prints on UART usage and waits for user input on serial
              }
            }
            else
            {
              PRINT_STR( "#Err" );
              PRINT_CHAR( SEPERATOR );
              PRINT_STR( "I2C address changes failed" );
              PRINT_LN( );
            }
          }
        } // else devices is already enabled
      }
      else if ( rx == 'd' )       // disable
      {  
        tmf8828Disable( TMF8828_A );
        tmf8828Disable( TMF8828_B );
        stateTmf8828 = TMF8828_STATE_DISABLED;
      }
      else if ( rx == 'w' )       // wakeup
      {
        if ( stateTmf8828 == TMF8828_STATE_STANDBY )
        {
          tmf8828Wakeup( TMF8828_A );
          tmf8828Wakeup( TMF8828_B );
          if ( tmf8828IsCpuReady( TMF8828_A, CPU_READY_TIME_MS ) &&  tmf8828IsCpuReady( TMF8828_B, CPU_READY_TIME_MS ) )
          {
            stateTmf8828 = TMF8828_STATE_STOPPED;
          }
          else
          {
            stateTmf8828 = TMF8828_STATE_ERROR;
          }
        }
      }
      else if ( rx == 'p' )       // power down
      {
        if ( stateTmf8828 == TMF8828_STATE_MEASURE )      // stop a measurement first
        {
          tmf8828StopMeasurement( TMF8828_A );
          tmf8828StopMeasurement( TMF8828_B );
          tmf8828DisableInterrupts( TMF8828_A, 0xFF );               // just disable all
          tmf8828DisableInterrupts( TMF8828_B, 0xFF );               // just disable all
          stateTmf8828 = TMF8828_STATE_STOPPED;
        }
        if ( stateTmf8828 == TMF8828_STATE_STOPPED )
        {
          tmf8828Standby( TMF8828_A );
          tmf8828Standby( TMF8828_B );
          stateTmf8828 = TMF8828_STATE_STANDBY;
        }
      }
      else if ( rx == 'm' )
      {  
        if ( stateTmf8828 == TMF8828_STATE_STOPPED )
        {
          tmf8828ClrAndEnableInterrupts( TMF8828_A, TMF8828_APP_I2C_RESULT_IRQ_MASK | TMF8828_APP_I2C_RAW_HISTOGRAM_IRQ_MASK );
          tmf8828ClrAndEnableInterrupts( TMF8828_B, TMF8828_APP_I2C_RESULT_IRQ_MASK | TMF8828_APP_I2C_RAW_HISTOGRAM_IRQ_MASK );
          tmf8828StartMeasurement( TMF8828_A );
          // Arduino uno needs some time to read the results and publish them
          delay_in_microseconds( TMF8828_DELAY_FOR_START_OF_2ND );        // WARNING: start the 2nd tmf8828 with a delay, to give time for readout of results
          tmf8828StartMeasurement( TMF8828_B );
          stateTmf8828 = TMF8828_STATE_MEASURE;
        }
      }
      else if ( rx == 's' )
      {
        if ( stateTmf8828 == TMF8828_STATE_MEASURE || stateTmf8828 == TMF8828_STATE_STOPPED )
        {
          tmf8828StopMeasurement( TMF8828_A );
          tmf8828StopMeasurement( TMF8828_B );
          tmf8828DisableInterrupts( TMF8828_A, 0xFF );               // just disable all
          tmf8828DisableInterrupts( TMF8828_B, 0xFF );               // just disable all
          stateTmf8828 = TMF8828_STATE_STOPPED;
        }
      }
      else if ( rx == 'f' )
      {  
        if ( stateTmf8828 == TMF8828_STATE_STOPPED )
        {
          factoryCalibrationOneTmf8828( TMF8828_A );
          factoryCalibrationOneTmf8828( TMF8828_B );
        }
      }
      else if ( rx == 'l' )
      {  
        if ( stateTmf8828 == TMF8828_STATE_STOPPED )
        {
          loadFactoryCalibrationOneTmf8828( TMF8828_A, 0 );   // 1st tmf8828 has calib records 0..3
          loadFactoryCalibrationOneTmf8828( TMF8828_B, 4 );   // 2nd tmf8828 has calib records 4..7
        }
      }
      else if ( rx == 'r' )
      {
        if ( stateTmf8828 == TMF8828_STATE_STOPPED )
        {
          setStoredFactoryCalibration( TMF8828_A, 0 );
          setStoredFactoryCalibration( TMF8828_B, 4 );
        }
      }
      else if ( rx == 'z' )
      {
        if ( stateTmf8828 == TMF8828_STATE_STOPPED )
        {
          dumpHistogramOn = dumpHistogramOn + 1;       // select histogram dump on/off, and type of histogram dumping
          if ( dumpHistogramOn > (TMF8828_COM_HIST_DUMP__histogram__electrical_calibration_24_bit_histogram + TMF8828_COM_HIST_DUMP__histogram__raw_24_bit_histogram) )
          {
            dumpHistogramOn = 0; // is off again
          }
          tmf8828Configure( TMF8828_A, configPeriod[configNr], configKiloIter[configNr], dumpHistogramOn );   
          tmf8828Configure( TMF8828_B, configPeriod[configNr], configKiloIter[configNr], dumpHistogramOn );   
          PRINT_STR( "Histogram is " );
          PRINT_INT( dumpHistogramOn );
          PRINT_LN( );
        }
      }
      else if ( rx == 'a' )
      {  
        if ( stateTmf8828 != TMF8828_STATE_DISABLED )
        {
          printRegisters( TMF8828_A->i2cSlaveAddress, 0x00, 256, ' ', 0 );  
          printRegisters( TMF8828_B->i2cSlaveAddress, 0x00, 256, ' ', 0 );  
        }
      }
      else if ( rx == 'x' )
      {
        clkCorrectionOn = !clkCorrectionOn;       // toggle clock correction on/off  
        tmf8828ClkCorrection( TMF8828_A, clkCorrectionOn );
        tmf8828ClkCorrection( TMF8828_B, clkCorrectionOn );
        PRINT_STR( "Clk corr is " );
        PRINT_INT( clkCorrectionOn );
        PRINT_LN( );
      }
      else 
      {
        PRINT_STR( "#Err" );
        PRINT_CHAR( SEPERATOR );
        PRINT_STR( "Cmd " );
        PRINT_CHAR( rx );
        PRINT_LN( );
      }
    }
    printState();
  }
}

// -------------------------------------------------------------------------------------------------------------

// Arduino setup function is only called once at startup. Do all the HW initialisation stuff here.
void setup ( )
{
  stateTmf8828 = TMF8828_STATE_DISABLED;
  configNr = 0;                             // rotate through the given configurations
  clkCorrectionOn = 1;                      // default set it on
  dumpHistogramOn = 0;                      // default is off
  tmf8828Initialise( TMF8828_A, ENABLE_PIN, INTERRUPT_PIN );
  tmf8828Initialise( TMF8828_B, ALT_ENABLE_PIN, ALT_INTERRUPT_PIN );    // 2nd tmf8828 uses different enable and interrupt pin
  logLevel = LOG_LEVEL_INFO;                                             //i2c logging only

  // configure ENABLE pin and interupt pin
  PIN_OUTPUT( ENABLE_PIN );
  PIN_OUTPUT( ALT_ENABLE_PIN );                     // 2nd tmf8828
  PIN_INPUT( INTERRUPT_PIN );
  PIN_INPUT( ALT_INTERRUPT_PIN );                   // 2nd tmf8828

  // start serial
  Serial.end( );                                      // this clears any old pending data 
  Serial.begin( UART_BAUD_RATE );

  // start i2c
  Wire.begin();
  Wire.setClock( I2C_CLK_SPEED );

  // cold-start of both tmf8828s
  tmf8828Disable( TMF8828_A );                     
  tmf8828Disable( TMF8828_B );                      
  delay_in_microseconds(CAP_DISCHARGE_TIME_MS * 1000); // wait for a proper discharge of the cap
  printHelp();
}


static int8_t readResultsAndHistogramsOneTmf8828( tmf8828Driver * driver, uint8_t * interruptStatus )
{
  int8_t res = APP_SUCCESS_OK;
  uint8_t intStatus = 0;
  intStatus = tmf8828GetAndClrInterrupts( driver, TMF8828_APP_I2C_RESULT_IRQ_MASK | TMF8828_APP_I2C_ANY_IRQ_MASK | TMF8828_APP_I2C_RAW_HISTOGRAM_IRQ_MASK );   // always clear also the ANY interrupt
  if ( intStatus & TMF8828_APP_I2C_RESULT_IRQ_MASK )                      // check if a result is available (ignore here the any interrupt)
  {
    res = tmf8828ReadResults( driver );
  }
  if ( intStatus & TMF8828_APP_I2C_RAW_HISTOGRAM_IRQ_MASK )
  {
    res = tmf8828ReadHistogram( driver );                                              // read a (partial) raw histogram
  }
  *interruptStatus |= intStatus;
  return res;
}

// Arduino main loop function, is executed cyclic
void loop ( )
{
  int8_t res = APP_SUCCESS_OK;
  uint8_t interruptStatus = 0;
  uint8_t intStatus = 0;
  serialInput();                                                            // handle any keystrokes from UART

  if ( stateTmf8828 == TMF8828_STATE_STOPPED || stateTmf8828 == TMF8828_STATE_MEASURE )
  {
    res |= readResultsAndHistogramsOneTmf8828( TMF8828_A, &interruptStatus );
    intStatus |= interruptStatus; 
    res |= readResultsAndHistogramsOneTmf8828( TMF8828_B, &interruptStatus ); 
    intStatus |= interruptStatus; 
  }

  if ( res != APP_SUCCESS_OK )                         // in case that fails there is some error in programming or on the device, this should not happen
  {
    tmf8828StopMeasurement( TMF8828_A );
    tmf8828StopMeasurement( TMF8828_B );
    tmf8828DisableInterrupts( TMF8828_A, 0xFF );
    tmf8828DisableInterrupts( TMF8828_B, 0xFF );
    stateTmf8828 = TMF8828_STATE_STOPPED;
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "inter" );
    PRINT_CHAR( SEPERATOR );
    PRINT_INT( intStatus );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "but no data" );
    PRINT_LN( );
  }
}
