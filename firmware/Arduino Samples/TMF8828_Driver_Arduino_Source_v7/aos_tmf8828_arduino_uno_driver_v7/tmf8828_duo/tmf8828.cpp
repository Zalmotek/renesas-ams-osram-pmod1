/*
 *****************************************************************************
 * Copyright by ams OSRAM AG                                                       *
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

//  simple tmf8828 driver

// --------------------------------------------------- includes --------------------------------

#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "tmf8828.h"
#include "tmf8828_image.h"
#include "registers_i2c.h"
#include "tmf8828_shim.h"


// --------------------------------------------------- defines --------------------------------

#define TMF8828_COM_APP_ID                                  0x0   // register address
#define TMF8828_COM_APP_ID__application                     0x3   // measurement application id
#define TMF8828_COM_APP_ID__bootloader                      0x80  // bootloader application id

#define TMF8828_COM_TMF8828_MODE                            0x10 // mode register is either 0x00 == tmf8820/1 or 0x08 == tmf8828                 
#define TMF8828_COM_TMF8828_MODE__mode__TMF8821             0    // the device is operating in 3x3/3x6/4x4 (TMF8820/TMF8821) mode       
#define TMF8828_COM_TMF8828_MODE__mode__TMF8828             8

// --------------------------------------------------- bootloader -----------------------------

#define TMF8X2X_BL_MAX_DATA_SIZE                  0x80  // Number of bytes that can be written or read with one BL command
#define TMF8828_COM_CMD_STAT                      0x08

#define TMF8828_COM_CMD_STAT__bl_cmd_ok           0x00  
#define TMF8828_COM_CMD_STAT__bl_cmd_errors       0x0F  // all success/error are below or equal to this number
#define TMF8828_COM_CMD_STAT__bl_cmd_ramremap     0x11  // Bootloader command to remap the vector table into RAM (Start RAM application).
#define TMF8828_COM_CMD_STAT__bl_cmd_r_ram        0x40  // Read from BL RAM.
#define TMF8828_COM_CMD_STAT__bl_cmd_w_ram        0x41  // Write to BL RAM.
#define TMF8828_COM_CMD_STAT__bl_cmd_addr_ram     0x43  // Set address pointer in RAM for Read/Write to BL RAM.

#define BL_HEADER           2     // bootloader header is 2 bytes
#define BL_MAX_DATA_PAYLOAD 128   // bootloader data payload can be up to 128
#define BL_FOOTER           1     // bootloader footer is 1 byte

// Bootloader maximum wait sequences
#define BL_CMD_SET_ADDR_TIMEOUT_MS    1
#define BL_CMD_W_RAM_TIMEOUT_MS       1
#define BL_CMD_RAM_REMAP_TIMEOUT_MS   1

// wait for version readout, to switch from ROM to RAM (and have the version published on I2C)
#define APP_PUBLISH_VERSION_WAIT_TIME_MS 10

// --------------------------------------------------- application ----------------------------

// application status, we check only for ok or accepted, everything between 2 and 15 (inclusive) 
// is an error
#define TMF8828_COM_CMD_STAT__stat_ok                       0x0  // Everything is okay
#define TMF8828_COM_CMD_STAT__stat_accepted                 0x1  // Everything is okay too, send sop to halt ongoing command

// application commands
#define TMF8828_COM_CMD_STAT__cmd_measure                             0x10  // Start a measurement
#define TMF8828_COM_CMD_STAT__cmd_stop                                0xff  // Stop a measurement
#define TMF8828_COM_CMD_STAT__cmd_write_config_page                   0x15  // Write the active config page
#define TMF8828_COM_CMD_STAT__cmd_load_config_page_common             0x16  // Load the common config page
#define TMF8828_COM_CMD_STAT__cmd_load_config_page_factory_calib      0x19  // Load the factory calibration config page
#define TMF8828_COM_CMD_STAT__cmd_stat__CMD_RESET_FACTORY_CALIBRATION 0x1F // Manually reset the factory calibration. Only supported if 8x8_measurements = 1.                      
#define TMF8828_COM_CMD_STAT__cmd_factory_calibration                 0x20  // Perform a factory calibration

#define TMF8828_COM_CMD_STAT__cmd_i2c_slave_address                   0x21  // change I2C address

#define TMF8828_COM_CMD_STAT__cmd_stat__CMD_SWITCH_TMF8821_MODE       0x65 // Switch to 3x3/3x6/4x4 mode. The device will need to be re-configured after this command. Only supported if 8x8_measurements = 1.  
#define TMF8828_COM_CMD_STAT__cmd_stat__CMD_SWITCH_TMF8828_MODE       0x6C // Switch to 8x8 mode. The device will need to be re-configured after this command. Only supported if 8x8_measurements = 1. 

// configuration page addresses and defines
#define TMF8828_COM_PERIOD_MS_LSB                           0x24  // period in milliseconds
#define TMF8828_COM_PERIOD_MS_MSB                           0x25
#define TMF8828_COM_KILO_ITERATIONS_LSB                     0x26  // Kilo (1024) iterations
#define TMF8828_COM_KILO_ITERATIONS_MSB                     0x27
#define TMF8828_COM_SPAD_MAP_ID                             0x34  // configure the SPAD map id, with some example maps
#define TMF8828_COM_SPAD_MAP_ID__map_last                   0x15  // maximum allowed spad map id, for tmf8828 only 15 is allowed
#define TMF8X2X_COM_HIST_DUMP                               0x39  // 0 ... all off, 1 ... raw histograms, 2 ... ec histograms
#define TMF8X2X_COM_I2C_SLAVE_ADDRESS                       0x3b  // register that holds the 7-bit shifted slave address
#define TMF8X2X_COM_ALG_SETTING_0                           0x35  // register that holds the algorithm settings

// Application maximum wait sequences
#define APP_CMD_LOAD_CONFIG_TIMEOUT_MS                      3
#define APP_CMD_WRITE_CONFIG_TIMEOUT_MS                     3
#define APP_CMD_MEASURE_TIMEOUT_MS                          5
#define APP_CMD_STOP_TIMEOUT_MS                             25
#define APP_CMD_FACTORY_CALIB_TIMEOUT_MS                    2000
#define APP_CMD_I2C_SLAVE_ADDRESS_TIMEOUT_MS                1
#define APP_CMD_SWITCH_MODE_CMD_TIMEOUT_MS                  1     // timeout until command is accepted
#define APP_CMD_SWITCH_MODE_TIMEOUT_MS                      10    // mode switch is achieved through a reset of the device (mode is at warm-start decided)

// -------------------------------------------------------- some checks --------------------------------------------

// check that we can read a complete result page also in the dataBuffer
#define DATA_BUFFER_SIZE                  (TMF8828_COM_CONFIG_FACTORY_CALIB__factory_calibration_size)

#if ( ( (BL_HEADER + BL_MAX_DATA_PAYLOAD + BL_FOOTER + 1) > DATA_BUFFER_SIZE ) || ( (TMF8828_COM_CONFIG_RESULT__measurement_result_size) > DATA_BUFFER_SIZE ) )
  #error "Increase data buffer size"
#endif


// clock correction pairs index calculation
#define CLK_CORRECTION_IDX_MODULO( x )    ( (x) & ( (CLK_CORRECTION_PAIRS)-1 ) )

// how accurate the calculation is going to be. The higher the accuracy the less far apart are
// the pairs allowed. An 8 precision means that the factor is 1/256 accurate.
#define CALC_PRECISION                                                  8
// Need this to add to the corrected distance before shifting right
#define HALF_CALC_PRECISION                                             ( 1 << ((CALC_PRECISION) - 1 ) )
#define CALC_DISTANCE_CORR_FACTOR( hostTickDiff, tmf8828TickDiff )      ( ( ( (hostTickDiff) * (TMF8828_TICKS_PER_US) ) << (CALC_PRECISION) ) / ( (tmf8828TickDiff) * (HOST_TICKS_PER_US) ) ) 
// Round before performing the division (right shift), make sure it is a logical shift right and not an arithmetical shift right
#define CALC_DISTANCE( distance, hostTickDiff, tmf8828TickDiff )        ( ( (uint32_t)( (distance) * CALC_DISTANCE_CORR_FACTOR( hostTickDiff, tmf8828TickDiff ) + (HALF_CALC_PRECISION) ) ) >> (CALC_PRECISION) )

// Find the maximum distance values to avoid mathematical errors due to overflow 
#define MAX_HOST_DIFF_VALUE                                             ( ( 0xFFFFFFFFUL / (TMF8828_TICKS_PER_US) ) >> CALC_PRECISION )
#define MAX_TMF8828_DIFF_VALUE                                          ( ( 0xFFFFFFFFUL / (HOST_TICKS_PER_US) )

// Saturation macro for 16-bit
#define SATURATE16( v )                                                 ( (v) > 0xFFFF ? 0xFFFF : (v) )

// For TMF882x sys ticks to be valid the LSB must be set.
#define TMF8828_SYS_TICK_IS_VALID( tick )                               ( (tick) & 1 )

// -------------------------------------------------------- variables ----------------------------------------------

uint8_t dataBuffer[ DATA_BUFFER_SIZE ];           // transfer/receive buffer

// -------------------------------------------------------- functions ----------------------------------------------

static void tmf8828ResetClockCorrection( tmf8828Driver * driver );

void tmf8828Initialise ( tmf8828Driver * driver, uint8_t enablePin, uint8_t interruptPin )
{
  tmf8828ResetClockCorrection( driver );
  driver->enablePin = enablePin;
  driver->interruptPin = interruptPin;
  driver->i2cSlaveAddress = TMF8828_SLAVE_ADDR;
  driver->clkCorrectionEnable = 1;                  // default is on
  driver->logLevel = LOG_LEVEL_ERROR;
}

// Function to overwrite the default log level
void tmf8828SetLogLevel ( tmf8828Driver * driver, uint8_t level )
{
  driver->logLevel = level;
}

// Function to set clock correction on or off.
// enable ... if <>0 clock correction is enabled (default)
// enable ... if ==0 clock correction is disabled
void tmf8828ClkCorrection ( tmf8828Driver * driver, uint8_t enable )
{
  driver->clkCorrectionEnable = !!enable;
}

// Function executes a reset of the device 
void tmf8828Reset ( tmf8828Driver * driver ) 
{
  dataBuffer[0] = RESETREASON__soft_reset__MASK;
  i2c_tx( driver->i2cSlaveAddress, RESETREASON, dataBuffer, 1 );
  if ( driver->logLevel >= LOG_LEVEL_VERBOSE ) 
  {
    PRINT_STR( "reset" );
    PRINT_LN( );
  }
  tmf8828ResetClockCorrection( driver );
}

// Function sets the enable PIN high
void tmf8828Enable ( tmf8828Driver * driver ) 
{
  PIN_HIGH( driver->enablePin );
  tmf8828Initialise( driver, driver->enablePin, driver->interruptPin );           // when enable gets high, the HW resets to default slave addr
}

// Function clears the enable PIN (=low)
void tmf8828Disable ( tmf8828Driver * driver ) 
{
  PIN_LOW( driver->enablePin );
}

// Function checks if the CPU becomes ready within the given time
int8_t tmf8828IsCpuReady ( tmf8828Driver * driver, uint8_t waitInMs )
{
  i2c_rx( driver->i2cSlaveAddress, ENABLE, dataBuffer, 1 );        // Need to read it twice after a PON=0, so do it 1 additional time 
  do 
  {
    dataBuffer[0] = 0;                                        // clear before reading
    i2c_rx( driver->i2cSlaveAddress, ENABLE, dataBuffer, 1 );      // read the enable register to determine cpu ready
    if ( !!( dataBuffer[0] & ENABLE__cpu_ready__MASK ) )
    {
      if ( driver->logLevel >= LOG_LEVEL_VERBOSE )
      {
        PRINT_STR( "CPU ready" );
        PRINT_LN( );
      }
      return 1;                                               // done                  
    }
    else if ( waitInMs )                                      // only wait until it is the last time we go through the loop, that would be a waste of time to wait again
    {
      delay_in_microseconds( 1000 );
    }
  } while ( waitInMs-- );
  if ( driver->logLevel >= LOG_LEVEL_ERROR )
  {
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "CPU not ready" );
    PRINT_LN( );
  }
  return 0;                                                 // cpu did not get ready
}

// Function attemps a wakeup of the device 
void tmf8828Wakeup ( tmf8828Driver * driver ) 
{
  dataBuffer[0] = 0;                                         // clear before reading
  i2c_rx( driver->i2cSlaveAddress, ENABLE, dataBuffer, 1 );      // read the enable register to dermine power state
  if ( ( dataBuffer[0] & ENABLE__cpu_ready__MASK ) == 0 )                  
  {
    dataBuffer[0] = dataBuffer[0] | ENABLE__pon__MASK;      // make sure to keep the remap bits
    i2c_tx( driver->i2cSlaveAddress, ENABLE, dataBuffer, 1 );    // set PON bit in enable register
    if ( driver->logLevel >= LOG_LEVEL_VERBOSE ) 
    {
      PRINT_STR( "PON=1" );
      PRINT_LN( );
    }
  }
  else
  {
    if ( driver->logLevel >= LOG_LEVEL_VERBOSE ) 
    {
      PRINT_STR( "awake ENABLE=0x" );
      PRINT_INT_HEX( dataBuffer[0] );
      PRINT_LN( );
    }  
  }
}

// Function puts the device in standby state
void tmf8828Standby ( tmf8828Driver * driver ) 
{
  dataBuffer[0] = 0;                                                   // clear before reading
  i2c_rx( driver->i2cSlaveAddress, ENABLE, dataBuffer, 1 );      // read the enable register to determine power state
  if ( ( dataBuffer[0] & ENABLE__cpu_ready__MASK ) != 0 )                  
  {
    dataBuffer[0] = dataBuffer[0] & ~ENABLE__pon__MASK;                         // clear only the PON bit
    i2c_tx( driver->i2cSlaveAddress, ENABLE, dataBuffer, 1 );   // clear PON bit in enable register
    if ( driver->logLevel >= LOG_LEVEL_VERBOSE ) 
    {
      PRINT_STR( "PON=0" );
      PRINT_LN( );
    }
  }
  else
  {
    if ( driver->logLevel >= LOG_LEVEL_VERBOSE ) 
    {
      PRINT_STR( "standby ENABLE=0x" );
      PRINT_INT_HEX( dataBuffer[0] );
      PRINT_LN( );
    }  
  }
}

// function to check if a register has a specific value
static int8_t tmf8828CheckRegister ( tmf8828Driver * driver, uint8_t regAddr, uint8_t expected, uint8_t len, uint16_t timeoutInMs )
{
  uint8_t i;
  uint32_t t = get_sys_tick();
  do 
  {
    dataBuffer[0] = ~expected;
    i2c_rx( driver->i2cSlaveAddress, regAddr, dataBuffer, len );
    if ( dataBuffer[0] == expected )
    {
      return APP_SUCCESS_OK; 
    }
    else if ( timeoutInMs )                             // do not wait if timeout is 0
    {
      delay_in_microseconds(1000);  
    }
  } while ( timeoutInMs-- > 0 );
  if ( driver->logLevel >= LOG_LEVEL_ERROR ) 
  {
    t = get_sys_tick() - t;
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "timeout " );
    PRINT_INT( t );
    PRINT_STR( " reg=0x" );
    PRINT_INT_HEX( regAddr );
    for ( i = 0; i < len; i++ )
    {
      PRINT_STR( " 0x" );
      PRINT_INT_HEX( dataBuffer[i] );
    }
    PRINT_LN( );
  }      
  return APP_ERROR_TIMEOUT;        // error timeout
}


// --------------------------------------- bootloader ------------------------------------------

// calculate the checksum according to bootloader spec
static uint8_t tmf8828BootloaderChecksum ( uint8_t * data, uint8_t len )      
{
  uint8_t sum = 0;
  while ( len-- > 0 )
  {
    sum += *data;
    data++;
  }
  sum = sum ^ 0xFF;
  return sum;
}

// execute command to set the RAM address pointer for RAM read/writes
static int8_t tmf8828BootloaderSetRamAddr ( tmf8828Driver * driver, uint16_t addr )
{
  dataBuffer[0] = TMF8828_COM_CMD_STAT__bl_cmd_addr_ram;
  dataBuffer[1] = 2;
  dataBuffer[2] = (uint8_t)addr;        // LSB of addr
  dataBuffer[3] = (uint8_t)(addr>>8);    // MSB of addr
  dataBuffer[4] = tmf8828BootloaderChecksum( dataBuffer, 4 );
  i2c_tx( driver->i2cSlaveAddress, TMF8828_COM_CMD_STAT, dataBuffer, 5 );
  return tmf8828CheckRegister( driver, TMF8828_COM_CMD_STAT, TMF8828_COM_CMD_STAT__bl_cmd_ok, 3, BL_CMD_SET_ADDR_TIMEOUT_MS );      // many BL errors only have 3 bytes 
}

// execute command to write a chunk of data to RAM
static int8_t tmf8828BootloaderWriteRam ( tmf8828Driver * driver, uint8_t len )
{
  dataBuffer[0] = TMF8828_COM_CMD_STAT__bl_cmd_w_ram;
  dataBuffer[1] = len;
  dataBuffer[BL_HEADER+len] = tmf8828BootloaderChecksum( dataBuffer, BL_HEADER+len );
  i2c_tx( driver->i2cSlaveAddress, TMF8828_COM_CMD_STAT, dataBuffer, BL_HEADER+len+BL_FOOTER );
  return tmf8828CheckRegister( driver, TMF8828_COM_CMD_STAT,TMF8828_COM_CMD_STAT__bl_cmd_ok, 3, BL_CMD_W_RAM_TIMEOUT_MS );    // many BL errors only have 3 bytes 
}

// execute command RAM remap to address 0 and continue running from RAM 
static int8_t tmf8828BootloaderRamRemap ( tmf8828Driver * driver, uint8_t appId )
{
  int8_t stat;
  dataBuffer[0] = TMF8828_COM_CMD_STAT__bl_cmd_ramremap;
  dataBuffer[1] = 0;
  dataBuffer[BL_HEADER] = tmf8828BootloaderChecksum( dataBuffer, BL_HEADER );
  i2c_tx( driver->i2cSlaveAddress, TMF8828_COM_CMD_STAT, dataBuffer, BL_HEADER+BL_FOOTER );
  delay_in_microseconds( APP_PUBLISH_VERSION_WAIT_TIME_MS * 1000 );
  // ram remap -> the bootloader will not answer to this command if successfull, so check the application id register instead
  stat = tmf8828CheckRegister( driver, TMF8828_COM_APP_ID, appId, 4, BL_CMD_RAM_REMAP_TIMEOUT_MS );  // tmf8828 application has 4 verion bytes
  if ( driver->logLevel >= LOG_LEVEL_INFO )
  {
    PRINT_STR( "#Vers" );
    PRINT_CHAR( SEPERATOR );
    PRINT_INT( dataBuffer[0] );
    PRINT_CHAR(  '.' );
    PRINT_INT( dataBuffer[1] );
    PRINT_CHAR(  '.' );
    PRINT_INT( dataBuffer[2] );
    PRINT_CHAR(  '.' );
    PRINT_INT( dataBuffer[3] );
    PRINT_LN( );
  }
  return stat;
}

// download the image file to RAM
int8_t tmf8828DownloadFirmware ( tmf8828Driver * driver ) 
{
  uint32_t idx = 0;
  int8_t stat = BL_SUCCESS_OK;
  uint8_t chunkLen;
  if ( driver->logLevel >= LOG_LEVEL_VERBOSE ) 
  {
    PRINT_STR( "Image addr=0x" );
    PRINT_INT_HEX( tmf8828_image_start );
    PRINT_STR( " len=" );
    PRINT_INT( tmf8828_image_length );
    for ( idx = 0; idx < 16; idx++ )
    {
      PRINT_STR( " 0x" );
      PRINT_INT_HEX( READ_PROGRAM_MEMORY_BYTE( tmf8828_image + idx ) );      // read from program memory space
    }
    PRINT_LN( );
  }
  stat = tmf8828BootloaderSetRamAddr( driver, tmf8828_image_start );
  idx = 0;  // start again at the image begin
  while ( stat == BL_SUCCESS_OK && idx < tmf8828_image_length )
  {
      if ( driver->logLevel >= LOG_LEVEL_VERBOSE )
      {
        PRINT_STR( "Download addr=0x" );
        PRINT_INT_HEX( idx );
        PRINT_LN( );
      }
      for( chunkLen=0; chunkLen < BL_MAX_DATA_PAYLOAD && idx < tmf8828_image_length; chunkLen++, idx++ )
      {
        dataBuffer[BL_HEADER + chunkLen] = READ_PROGRAM_MEMORY_BYTE( tmf8828_image + idx );              // read from code memory into local ram buffer
      }
      stat = tmf8828BootloaderWriteRam( driver, chunkLen );
  }
  if ( stat == BL_SUCCESS_OK )
  {
    stat = tmf8828BootloaderRamRemap( driver, TMF8828_COM_APP_ID__application );      // if you load a test-application this may have another ID
    if ( stat == BL_SUCCESS_OK )
    {
      if ( driver->logLevel >= LOG_LEVEL_INFO )
      {
        PRINT_STR( "FW downloaded" );
        PRINT_LN( );
      }
      return stat;
    }
  }
  if ( driver->logLevel >= LOG_LEVEL_ERROR ) 
  {
    PRINT_STR( "#Err" );
    PRINT_CHAR( SEPERATOR );
    PRINT_STR( "FW downl or REMAP" );
    PRINT_LN( );
  }
  return stat;
}

// --------------------------------------- application -----------------------------------------

// Reset clock correction calculation
static void tmf8828ResetClockCorrection ( tmf8828Driver * driver )
{
  uint8_t i;
  driver->clkCorrectionIdx = 0;                      // reset clock correction
  for ( i = 0; i < CLK_CORRECTION_PAIRS; i++ )
  {
    driver->hostTicks[ i ] = 0;
    driver->tmf8828Ticks[ i ] = 0;                  // initialise the tmf8828Ticks to a value that has the LSB cleared -> can identify that these are no real ticks
  }
  if ( driver->logLevel & LOG_LEVEL_CLK_CORRECTION )
  {
    PRINT_STR( "ClkCorr reset" );
    PRINT_LN( );
  }
}

// Add a host tick and a tmf8828 tick to the clock correction list.
static void tmf8828ClockCorrectionAddPair ( tmf8828Driver * driver, uint32_t hostTick, uint32_t tmf8828Tick )
{
  if ( TMF8828_SYS_TICK_IS_VALID( tmf8828Tick ) )                             // only use ticks if tmf8828Tick has LSB set
  {
    driver->clkCorrectionIdx = CLK_CORRECTION_IDX_MODULO( driver->clkCorrectionIdx + 1 );     // increment and take care of wrap-over
    driver->hostTicks[ driver->clkCorrectionIdx ] = hostTick;
    driver->tmf8828Ticks[ driver->clkCorrectionIdx ] = tmf8828Tick;
  }
  else if ( driver->logLevel & LOG_LEVEL_CLK_CORRECTION )
  {
    PRINT_STR( "ClkCorr ticks invalid " );      // this can happen if the host did read out the data very, very fast,
    PRINT_INT( tmf8828Tick );                   // and the device was busy handling other higher priority interrupts
    PRINT_LN( );                                // The device does always set the LSB of the sys-tick to indicate that
  }                                                 // The device does always set the LSB of the sys-tick to indicate that
}                                                   // the device did set the sys-tick.

// execute command to load a given config page
static int8_t tmf8828LoadConfigPage ( tmf8828Driver * driver, uint8_t pageCmd )
{
  int8_t stat;
  dataBuffer[0] = pageCmd;
  i2c_tx( driver->i2cSlaveAddress, TMF8828_COM_CMD_STAT, dataBuffer, 1 );                                                      // instruct device to load page
  stat = tmf8828CheckRegister( driver, TMF8828_COM_CMD_STAT, TMF8828_COM_CMD_STAT__stat_ok, 1, APP_CMD_LOAD_CONFIG_TIMEOUT_MS );  // check that load command is completed
  if ( stat == APP_SUCCESS_OK )
  {
    stat = tmf8828CheckRegister( driver, TMF8828_COM_CONFIG_RESULT, pageCmd, 1, 0 );                                              // check that correct config page is loaded, no more waiting
  }
  return stat;
}

// Convert 4 bytes in little endian format into an uint32_t  
uint32_t tmf8828GetUint32 ( uint8_t * data ) 
{
  uint32_t t =    data[ 3 ];
  t = (t << 8 ) + data[ 2 ];
  t = (t << 8 ) + data[ 1 ];
  t = (t << 8 ) + data[ 0 ];
  return t;
}

// Function that executes command to write any configuration page
int8_t tmf8828WriteConfigPage ( tmf8828Driver * driver )
{
  int8_t stat;
  dataBuffer[0] = TMF8828_COM_CMD_STAT__cmd_write_config_page;
  i2c_tx( driver->i2cSlaveAddress, TMF8828_COM_CMD_STAT, dataBuffer, 1 );                                                      // instruct device to load page
  stat = tmf8828CheckRegister( driver, TMF8828_COM_CMD_STAT, TMF8828_COM_CMD_STAT__stat_ok, 1, APP_CMD_WRITE_CONFIG_TIMEOUT_MS ); // check that write command is completed
  return stat;
}

// Function to load the common config page into I2C ram that the host can read/write it via I2C
int8_t tmf8828LoadConfigPageCommon ( tmf8828Driver * driver )
{
  return tmf8828LoadConfigPage( driver, TMF8828_COM_CMD_STAT__cmd_load_config_page_common );
}

// Function to load the factory calibration config page into I2C ram that the host can read/write it via I2C
int8_t tmf8828LoadConfigPageFactoryCalib ( tmf8828Driver * driver )
{
  return tmf8828LoadConfigPage( driver, TMF8828_COM_CMD_STAT__cmd_load_config_page_factory_calib );
}

// function to change the I2C address of the device
int8_t tmf8828ChangeI2CAddress ( tmf8828Driver * driver, uint8_t newI2cSlaveAddress )
{
  uint8_t oldAddr = driver->i2cSlaveAddress;
  int8_t stat = tmf8828LoadConfigPageCommon( driver );          // first load the page, then only overwrite the registers you want to change 
  if ( stat == APP_SUCCESS_OK )
  {
    dataBuffer[0] = newI2cSlaveAddress << 1;          // i2c slave address is shifted into the upper 7 bits of the 8-bit register
    i2c_tx( driver->i2cSlaveAddress, TMF8X2X_COM_I2C_SLAVE_ADDRESS, dataBuffer, 1 );
    stat = tmf8828WriteConfigPage( driver );                 //  write the config page back
    if ( stat == APP_SUCCESS_OK )
    {
      dataBuffer[0] = TMF8828_COM_CMD_STAT__cmd_i2c_slave_address;
      i2c_tx( driver->i2cSlaveAddress, TMF8828_COM_CMD_STAT, dataBuffer, 1 );      // instruct device to change i2c address
      driver->i2cSlaveAddress = newI2cSlaveAddress;                                 // from now on try to read from new address
      stat = tmf8828CheckRegister( driver, TMF8828_COM_CMD_STAT, TMF8828_COM_CMD_STAT__stat_ok, 1, APP_CMD_I2C_SLAVE_ADDRESS_TIMEOUT_MS ); // check that command ok
     }
  }
  if ( stat != APP_SUCCESS_OK )
  {
    driver->i2cSlaveAddress = oldAddr;                                            // switch failed, use old address again
    if ( driver->logLevel >= LOG_LEVEL_ERROR ) 
    {
      PRINT_STR( "#Err" );
      PRINT_CHAR( SEPERATOR );
      PRINT_STR( "I2C-Addr " );
      PRINT_INT( stat );
      PRINT_LN( );
    }
  }
  return stat; 
}

// configure device according to given parameters
int8_t tmf8828Configure ( tmf8828Driver * driver, uint16_t periodInMs, uint16_t kiloIterations, uint8_t dumpHistogram )
{
  int8_t stat = APP_ERROR_PARAM;
  stat = tmf8828LoadConfigPageCommon( driver );          // first load the page, then only overwrite the registers you want to change 
  if ( stat == APP_SUCCESS_OK )
  {
    dataBuffer[0] = (uint8_t)periodInMs;            // lsb
    dataBuffer[1] = (uint8_t)(periodInMs>>8);       // msb
    dataBuffer[2] = (uint8_t)kiloIterations;        // lsb  - kilo iterations are right behind the period so we can write with one i2c tx
    dataBuffer[3] = (uint8_t)(kiloIterations>>8);   // msb
    i2c_tx( driver->i2cSlaveAddress, TMF8828_COM_PERIOD_MS_LSB, dataBuffer, 4 );
    dataBuffer[0] = dumpHistogram & 0x3;            // only raw histograms and/or EC histograms
    i2c_tx( driver->i2cSlaveAddress, TMF8X2X_COM_HIST_DUMP, dataBuffer, 1 );
    /* show distance results with extended confidence range
	   report distances -> 0x04
	   switch on extended confidence range -> 0x80
	   0x04 | 0x80 -> 0x84 
	   please see TMF8828 data sheet for details
	*/
	dataBuffer[0] = 0x84;   
    i2c_tx( driver->i2cSlaveAddress, TMF8X2X_COM_ALG_SETTING_0, dataBuffer, 1 );
    stat = tmf8828WriteConfigPage( driver );               // as a last step write the config page back
  }
  if ( stat != APP_SUCCESS_OK )
  {
    if ( driver->logLevel >= LOG_LEVEL_ERROR ) 
    {
      PRINT_STR( "#Err" );
      PRINT_CHAR( SEPERATOR );
      PRINT_STR( "Config " );
      PRINT_INT( stat );
      PRINT_LN( );
    }
  }
  return stat;
}

// Function to reset the factory calibration. Call this function before providing the 4 factory
// calibration pages for tmf8828.
int8_t tmf8828ResetFactoryCalibration ( tmf8828Driver * driver )
{
  dataBuffer[0] = TMF8828_COM_CMD_STAT__cmd_stat__CMD_RESET_FACTORY_CALIBRATION;
  i2c_tx( driver->i2cSlaveAddress, TMF8828_COM_CMD_STAT, dataBuffer, 1 );                                                      // instruct device to load page
  return tmf8828CheckRegister( driver, TMF8828_COM_CMD_STAT, TMF8828_COM_CMD_STAT__stat_ok, 1, APP_CMD_WRITE_CONFIG_TIMEOUT_MS ); // check that command is done
}

// Function to execute a factory calibration
int8_t tmf8828FactoryCalibration ( tmf8828Driver * driver )
{
  dataBuffer[0] = TMF8828_COM_CMD_STAT__cmd_factory_calibration;
  i2c_tx( driver->i2cSlaveAddress, TMF8828_COM_CMD_STAT, dataBuffer, 1 );                                                      // instruct device to load page
  return tmf8828CheckRegister( driver, TMF8828_COM_CMD_STAT, TMF8828_COM_CMD_STAT__stat_ok, 1, APP_CMD_FACTORY_CALIB_TIMEOUT_MS ); // check that factory calib command is done
}

// Function to write a pre-stored calibration page
int8_t tmf8828SetStoredFactoryCalibration ( tmf8828Driver * driver, const uint8_t * calibPage )
{
  int8_t status = tmf8828LoadConfigPageFactoryCalib( driver );
  if ( APP_SUCCESS_OK == status )
  {
    dataBuffer[0] = READ_PROGRAM_MEMORY_BYTE( calibPage );
    if ( dataBuffer[0] == TMF8828_COM_CMD_STAT__cmd_load_config_page_factory_calib )
    {
      for ( uint8_t i = 1; i < TMF8828_COM_CONFIG_FACTORY_CALIB__factory_calibration_size; i++ )
      {
        dataBuffer[i] = READ_PROGRAM_MEMORY_BYTE( calibPage + i );
      }
      // actually write the calibration data
      i2c_tx( driver->i2cSlaveAddress, TMF8828_COM_CONFIG_RESULT, dataBuffer, TMF8828_COM_CONFIG_FACTORY_CALIB__factory_calibration_size );
      // issue the write page command
      return tmf8828WriteConfigPage( driver );
    }
    else
    {
      status = APP_ERROR_NO_CALIB_PAGE;
    }
  }
  return status;
}

// function starts a measurement
int8_t tmf8828StartMeasurement ( tmf8828Driver * driver ) 
{
  tmf8828ResetClockCorrection( driver );                                                                                         // clock correction only works during measurements
  dataBuffer[0] = TMF8828_COM_CMD_STAT__cmd_measure;
  i2c_tx( driver->i2cSlaveAddress, TMF8828_COM_CMD_STAT, dataBuffer, 1 );                                                      // instruct device to load page
  return tmf8828CheckRegister( driver, TMF8828_COM_CMD_STAT, TMF8828_COM_CMD_STAT__stat_accepted, 1, APP_CMD_MEASURE_TIMEOUT_MS ); // check that measure command is accepted
}

// function stops a measurement
int8_t tmf8828StopMeasurement ( tmf8828Driver * driver ) 
{
  dataBuffer[0] = TMF8828_COM_CMD_STAT__cmd_stop;
  i2c_tx( driver->i2cSlaveAddress, TMF8828_COM_CMD_STAT, dataBuffer, 1 );                                                      // instruct device to load page
  return tmf8828CheckRegister( driver, TMF8828_COM_CMD_STAT, TMF8828_COM_CMD_STAT__stat_ok, 1, APP_CMD_STOP_TIMEOUT_MS );         // check that stop command is accepted
}

// function to switch to one of the 2 modes
static int8_t tmf8828SwitchToMode ( tmf8828Driver * driver, uint8_t modeCmd, uint8_t mode )
{
  int8_t status;
  dataBuffer[0] = modeCmd;
  i2c_tx( driver->i2cSlaveAddress, TMF8828_COM_CMD_STAT, dataBuffer, 1 );                                                      // instruct device to load page
  status = tmf8828CheckRegister( driver, TMF8828_COM_CMD_STAT, TMF8828_COM_CMD_STAT__stat_ok, 1, APP_CMD_SWITCH_MODE_CMD_TIMEOUT_MS );         // check that switch command is accepted
  // now wait for switch to happen -> a system reset of device, mode is choosen at cold-start/warm-start only
  if ( status == APP_SUCCESS_OK )
  {
    status = tmf8828CheckRegister( driver, TMF8828_COM_TMF8828_MODE, mode, 1, APP_CMD_SWITCH_MODE_TIMEOUT_MS );         
  }
  return status;
}

// function switches to 8x8 mode
int8_t tmf8828SwitchTo8x8Mode ( tmf8828Driver * driver )
{
  return tmf8828SwitchToMode( driver, TMF8828_COM_CMD_STAT__cmd_stat__CMD_SWITCH_TMF8828_MODE, TMF8828_COM_TMF8828_MODE__mode__TMF8828 );
}

// function switches to legacy mode
int8_t tmf8828SwitchToLegacyMode ( tmf8828Driver * driver )
{
  return tmf8828SwitchToMode( driver, TMF8828_COM_CMD_STAT__cmd_stat__CMD_SWITCH_TMF8821_MODE, TMF8828_COM_TMF8828_MODE__mode__TMF8821 );
}

// function reads and clears the specified interrupts
uint8_t tmf8828GetAndClrInterrupts ( tmf8828Driver * driver, uint8_t mask )
{
  uint8_t setInterrupts;
  dataBuffer[0] = 0;
  i2c_rx( driver->i2cSlaveAddress, INT_STATUS, dataBuffer, 1 );            // read interrupt status register
  setInterrupts = dataBuffer[0] & mask;
  if ( setInterrupts )
  {
    dataBuffer[0] = dataBuffer[0] & mask;                             // clear only those that were set when we read the register, and only those we want to know
    i2c_tx( driver->i2cSlaveAddress, INT_STATUS, dataBuffer, 1 );       // clear interrupts by pushing a 1 to status register
  }
  return setInterrupts;
}

// function clears and enables the specified interrupts
void tmf8828ClrAndEnableInterrupts ( tmf8828Driver * driver, uint8_t mask )
{
  dataBuffer[0] = 0xFF;                                               // clear all interrupts  
  i2c_tx( driver->i2cSlaveAddress, INT_STATUS, dataBuffer, 1 );         // clear interrupts by pushing a 1 to status register
  dataBuffer[0] = 0;
  i2c_rx( driver->i2cSlaveAddress, INT_ENAB, dataBuffer, 1 );            // read current enabled interrupts 
  dataBuffer[0] = dataBuffer[0] | mask;                             // enable those in the mask, keep the others if they were enabled
  i2c_tx( driver->i2cSlaveAddress, INT_ENAB, dataBuffer, 1 );
}

// function disables the specified interrupts
void tmf8828DisableInterrupts ( tmf8828Driver * driver, uint8_t mask )
{
  dataBuffer[0] = 0;
  i2c_rx( driver->i2cSlaveAddress, INT_ENAB, dataBuffer, 1 );            // read current enabled interrupts 
  dataBuffer[0] = dataBuffer[0] & ~mask;                            // clear only those in the mask, keep the others if they were enabled
  i2c_tx( driver->i2cSlaveAddress, INT_ENAB, dataBuffer, 1 );
  dataBuffer[0] = mask; 
  i2c_tx( driver->i2cSlaveAddress, INT_STATUS, dataBuffer, 1 );         // clear interrupts by pushing a 1 to status register
}

// function reads the result page (if there is none the function returns an error, else success)
int8_t tmf8828ReadResults ( tmf8828Driver * driver ) 
{
  uint32_t hTick;            // get the sys-tick just before the I2C rx
  dataBuffer[0] = 0;
  hTick = get_sys_tick( );            // get the sys-tick just before the I2C rx
  i2c_rx( driver->i2cSlaveAddress, TMF8828_COM_CONFIG_RESULT, dataBuffer, TMF8828_COM_CONFIG_RESULT__measurement_result_size );
  if ( dataBuffer[0] == TMF8828_COM_CONFIG_RESULT__measurement_result )
  {
    uint32_t tTick = tmf8828GetUint32( dataBuffer + RESULT_REG( SYS_TICK_0 ) );
    tmf8828ClockCorrectionAddPair( driver, hTick, tTick );
    print_results( driver, dataBuffer, TMF8828_COM_CONFIG_RESULT__measurement_result_size );
    return APP_SUCCESS_OK;
  }
  return APP_ERROR_NO_RESULT_PAGE;
}

// Correct the distance based on the clock correction pairs 
uint16_t tmf8828CorrectDistance ( tmf8828Driver * driver, uint16_t distance )
{
  if ( driver->clkCorrectionEnable )
  {
    uint8_t idx = driver->clkCorrectionIdx;                                                 // last inserted
    uint8_t idx2 = CLK_CORRECTION_IDX_MODULO( idx + CLK_CORRECTION_PAIRS - 1 );     // oldest available
    if ( TMF8828_SYS_TICK_IS_VALID( driver->tmf8828Ticks[ idx ] ) && TMF8828_SYS_TICK_IS_VALID( driver->tmf8828Ticks[ idx2 ] ) )    // only do a correction if both tmf8828 ticks are valid 
    { 
      uint32_t hDiff = driver->hostTicks[ idx ] - driver->hostTicks[ idx2 ];
      uint32_t tDiff = driver->tmf8828Ticks[ idx ] - driver->tmf8828Ticks[ idx2 ];
      if ( tDiff > 0 && hDiff > 0 )                                                 // do not multiply or divide by 0
      {
        uint32_t d = distance;
        d = CALC_DISTANCE( d, hDiff, tDiff );
        distance = SATURATE16( d );
      }
    }
  }
  return distance;
}

// Function to read histograms and print them on UART. 
int8_t tmf8828ReadHistogram ( tmf8828Driver * driver )
{
  dataBuffer[0] = 0;
  i2c_rx( driver->i2cSlaveAddress, TMF8828_COM_CONFIG_RESULT, dataBuffer, TMF8828_COM_HISTOGRAM_PACKET_SIZE );
  if ( ( dataBuffer[0] & TMF8828_COM_OPTIONAL_SUBPACKET_HEADER_MASK ) == TMF8828_COM_OPTIONAL_SUBPACKET_HEADER_MASK ) // histograms must have MSB set
  {
    print_histogram( driver, dataBuffer, TMF8828_COM_HISTOGRAM_PACKET_SIZE );
    return APP_SUCCESS_OK;
  }
  return APP_ERROR_NO_RESULT_PAGE;
}
