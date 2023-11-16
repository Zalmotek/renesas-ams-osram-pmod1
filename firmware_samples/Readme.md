# TMF8828

## UART and command line interpreter  

UART commands
- h ... print help 
- d ... disable device
- w ... wakeup device
- e ... enable device and download FW
- p ... power down device
- m ... start measure
- s ... stop measure
- c ... use next configuration
- f ... do fact. calibration
- l ... load fact. calibration page
- r ... restore fact. calibration from file
- x ... clock correction on/off toggle
- a ... dump all registers
- z ... histogram dumping on/off
- i ... i2c addr. change 

To use the command line commands, navigate to hal_entry.cpp and go to the** hal_entry function** and find the **commands[]** char array.
To use the UART commands please fill this array with the desired commands and make sure you call the serial input function which passes two parameters: the commands array and the number of characters in said array. 

EG: To enable device and start measuring, this section will look like this:

```
void hal_entry(void){
    char commands[]={'e','m'};
    serialInput(commands,2);
    while(true){loop();}
}
```

## Command line interpreter application  

The command line interpreter application mimics a simple host application. It allows the user to switch to the next of 3 pre-defined configurations:  
- Configuration 0: Period 132 ms, KiloIterations = 250
- Configuration 1: Period 264 ms, KiloIteratiosn = 500
- Configuration 2: Period 528 ms, KiloIterations = 1000

You can modify the configurations (e.g. choose different period or Kilo iterations) in the application source file, recompile and download your 
own configuration application to the Arduino. 

Note that the factory calibration record must match both - the device serial number and the SPAD map id to be valid. I.e. if factory calibration is only valid for 8x8 mode as this is a special SPAD map ID.


## Examples  

### Power up device  

Before the device can be used the host must power the device. The arduino uno setup function will pull the 
enable line to the tmf8828 low. I.e. the tmf8828 will be powered down.
Enter the character
- e  
followed by ENTER to enable the device, the driver will automatically download the the firmware patch file to the tmf8828 RAM. The arduino uno will publish the FW version in the terminal:  

E.g. version 3.224.18.19


### Power up device and do measurements  

Type the following commands on UART:
- e 
- m 

The header for the results looks like this:  
 \#Obj,i2c_slave_address,result_number,temperature,number_valid_results,device_ticks,distance0_in_mm,confidence0,distance1_in_mm,confidence1,distance2_in_mm,confidence2, ...  

You will see measurement result records on the UART for the default configuration.  
\#Obj,65,1,23,12,229155,1815,64,1805,105,1823,71,832,22,807,41,840,27,1872,27,1854,45,1868,35,0,0,0 ...  
\#Obj,65,2,23,12,387365,1797,65,1788,98,1809,70,821,22,802,37,832,27,1863,28,1833,38,1860,34,0,0,0 ...  


# Factory calibration  


Factory calibration must be done for each SPAD map and also for each device. It is recommended to use 4000K iterations for factory calibration.
So the application will configure the tmf8828 for 4000K iterations, execute factory calibration and reconfigure the device to the original
number of iterations afterwards.

The simplest way is to do a live factory calibration. I.e. do the following steps:
1. Connect your arduino uno and tmf8828 to the PC via USB
2. Start a terminal program 
3. Configure and connect the terminal program to the arduino uno
4. Make sure you have a kind of cover glass on top of the tmf8828. I used some transparent plasic foil on top of my tmf8828. This is needed for the crosstalk.
5. Make sure there is no object in front of the tmf8828 within 40 cm.
6. enter the following commands in your terminal console:  
    - e 
    - f  
7. Now check if your device is factory calibrated:
    - m 
    - s 
    - a 
8. Now check if register 0x07 has the value 0x00 (factory calibration is accepted) or 0x31 (factory calibration is missing == not accepted).

If the device still reports 0x31 make sure that you have a cover glass and that there is no object in front of the device closer than 40 cm.


## Crosstalk readout

The crosstalk per channel is part of the factory calibration page. 
A simple way to do this is the following:
1. Perform a factory calibration = f
2. Load the factory calibration page = l
3. Print all registers = a

Read the cross talk values for the channels according to the data-sheet from the registers.

E.g. 
channel 1 = 0x60-0x63 is a 32-bit value in little-endian format
channel 2 = 0x64-0x67 is a 32-bit value in little-endian format
channel 3 = 0x68-0x6b is a 32-bit value in little-endian format
channel 4 = 0x6c-0x6f is a 32-bit value in little-endian format

In below register dump snippet the crosstalk values for channel 1 to 4 are encoded as little endian 32-bit values :
0x60:  0x2  0x0  0x0  0x0  0x2  0x0  0x0  0x0   
0x68:  0x58  0x1  0x0  0x0  0x2  0x0  0x0  0x0   
  
channel 1 crosstalk = 2   
channel 2 crosstalk = 2   
channel 3 crosstalk = 0x158 = 344   
channel 4 crosstalk = 2   


There is python script: **talk_to_arduino_factory_calibration.py** that will perform a factory calibration and does calculate
the crosstalk for up to 20 channels. Channel 0 and Channel 10 will always have crosstalk 0. Channels 11 to 19 will have the 
predefined values of 4000 for non-time-multiplexed configurations.

Also note that channels that are not used in a configuration will have a crosstalk outside the limits stated by the
optical design guide. Check the ams-OSRAM documenation for details about the optical design guide.

Here is a sample output from that script:    
Factory calibration was done with 4000 KiloIterations  
Crosstalk values:                                           
channel_0 = 0, normalized to 550K iterations = 0            
channel_1 = 69124, normalized to 550K iterations = 9504     
channel_2 = 37188, normalized to 550K iterations = 5113     
channel_3 = 37989, normalized to 550K iterations = 5223     
channel_4 = 71095, normalized to 550K iterations = 9775     
channel_5 = 92202, normalized to 550K iterations = 12677    
channel_6 = 25961, normalized to 550K iterations = 3569     
channel_7 = 23181, normalized to 550K iterations = 3187     
channel_8 = 33549, normalized to 550K iterations = 4612     
channel_9 = 1, normalized to 550K iterations = 0            
channel_10 = 0, normalized to 550K iterations = 0           
channel_11 = 64789, normalized to 550K iterations = 8908    
channel_12 = 25674, normalized to 550K iterations = 3530    
channel_13 = 20376, normalized to 550K iterations = 2801    
channel_14 = 33893, normalized to 550K iterations = 4660    
channel_15 = 53080, normalized to 550K iterations = 7298    
channel_16 = 37825, normalized to 550K iterations = 5200    
channel_17 = 45156, normalized to 550K iterations = 6208    
channel_18 = 84454, normalized to 550K iterations = 11612    
channel_19 = 1, normalized to 550K iterations = 0   
Factory calibration success  


If you have e.g. no coverglass on your tmf8828 the factory calibration will fail and the crosstalk values are invalid too. Readout may look like this
(notice that the factory calibraiton status is set to a value <>0, which indicates a failure):    
Crosstalk values:                                           
channel_0 = 0, normalized to 550K iterations = 0            
channel_1 = 1, normalized to 550K iterations = 0     
channel_2 = 2, normalized to 550K iterations = 0                                   
...   
channel_19 = 1, normalized to 550K iterations = 0     
Factory calibration failed with status=49    


**Important note:** Factory calibration may have completed successfully, and the crosstalk values may still be outside the 
allowed limits as specified in the ams-OSRAM optical design guide (ODG). 
If they are outside the specified limits for a given SPAD mask configuration the performance of the device
can be reduced.


## Factory calibration generation and storing it for Arduino Uno  

**For the Arduino Uno example the factory calibration is compiled in. The real application should write this to a file  and read it from the file**, instead of manually copying the factory calibration into the source file tmf8828_calib.c and having to recompile and download the new application to the Arduino Uno.


The provided factory calibration (in file tmf8828_calib.c) only matches for my tmf8828. 
For the Arduino Uno to get your own please do the following steps:
1. Connect your arduino uno and tmf8828 to the PC via USB
2. Start a terminal program 
3. Configure and connect the terminal program to the arduino uno
4. Make sure you have a kind of cover glass on top of the tmf8828. I used some transparent plasic foil on top of my tmf8828. This is needed for the crosstalk.
5. Make sure there is no object in front of the tmf8828 within 40 cm.
6. enter the following commands in your terminal console:  
    - e 
    - f 
    - l 
    - c 
    - f 
    - l 
    - c 
    - f 
    - l 
7. Now store the 3 C-structure records from the terminal window into the tmf8828_calib.c file (tmf8828_calib_0[], tmf8828_calib_1[] and tmf8828_calib_2[])
8. Save the file
9. Recompile your arduino project and download it to the arduino

The c will use the next available configuration and with the f you do a factory calibration for each of the configurations. 
The l will load the factory calibration page and print the tmf8828_calib_ structure on the terminal, to be copied to the file tmf8828_calib.c. 

Now you should be able to use the factory calibration. 


## Factory calibration verification  

Do the following to verify that the factory calibration is accepted by the device:  
1. Connect your arduino uno and tmf8828 to the PC via USB
2. Start a terminal program 
3. Configure and connect the terminal program to the arduino uno
4. enter the following commands in your terminal console:
    - e 
    - r 
    - m 
    - s 
    - a 
5. Now check if register 0x07 has the value 0x00 (factory calibration is accepted) or 0x31 (factory calibration is missing == not accepted).
You can do the same for the other 2 configurations. 
6. enter the following commands in your terminal console:
    - c 
    - r 
    - m 
    - s 
    - a 
7. Now check if register 0x07 has the value 0x00 (factory calibration is accepted) or 0x31 (factory calibration is missing == not accepted).
8. enter the following commands in your terminal console:
    - c 
    - r 
    - m 
    - s 
    - a 
9. Now check if register 0x07 has the value 0x00 (factory calibration is accepted) or 0x31 (factory calibration is missing == not accepted).  


## I2C slave address changing

There is a simple python script **talk_to_arduino_i2c_change_address.py** that will instruct the device to change its slave address
from default 65 to 66 and back to 65. The change is visible if you observe the I2C bus with e.g. an I2C analyzer


# Histogram dumping  

The device can also dump 24-bit histograms. The order the arduino uno driver reports these histograms is exactly the same as the tmf882x reports them on I2C.  

There are 2 types of histograms available:  
- raw histograms: these are dumped if the arduino uno reports: Histogram is 1
- electrical calibration histograms: these are dumped if the arduino uno reports: Histogram is 2
- both histograms: these are dumped if the arduino uno reports: Histogram is 3  

Type z and ENTER until you have the desired value for histogram dumping (0,1,2,3). 
For all histograms, the first number after the marker (\#Raw, \#Cal) will give the channel and which of the 3 bytes that build a 24-bit value this histogram contains.  
There are always 10 histograms reported. 
- Number 0 == TDC0, Channel0, Byte0 (LSB)
- Number 1 == TDC0, Channel1, Byte0 (LSB)
- ... 
- Number 9 == TDC4, Channel1, Byte0 (LSB)
- Number 10 == TDC0, Channel0, Byte1 (mid-byte)
- Number 11 == TDC0, Channel1, Byte1 (mid-byte)
- ...
- Number 19 == TDC4, Channel1, Byte1 (mid-byte)
- Number 20 == TDC0, Channel0, Byte2 (MSB)
- Number 21 == TDC0, Channel1, Byte2 (MSB)
- ...
- Number 29 == TDC4, Channel1, Byte2 (MSB)
 
The header for the raw histograms looks like this:  
 \#Raw,i2c_slave_address,Tdc_Channel_Byte_number,bin0_byte_value,bin1_byte_value,bin2_byte_value, ...  

The raw histograms will look like this:  
 \#Raw,65,0,2,2,2,2,2,2,2,3,1,0,1,3,4,72,183,176,132,246,198,58,199,117,160,235,148 ...  
 \#Raw,65,1,136,168,199,125,120,137,114,171,165,136,152,133,141,145,194,193,172,170 ...  
 \#Raw,65,2,114,95,130,145,111,100,96,114,67,79,116,137,83,111,155,117,136,122,116 ...  

The header for the calibration histograms looks like this:  
 \#Cal,i2c_slave_address,Tdc_Channel_Byte_number,bin0_byte_value,bin1_byte_value,bin2_byte_value, ...  

The electrical calibration histograms will look like this:
 \#Cal,65,0,0,0,0,0,0,0,0,0,0 ...,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,115,2,118,156,15,0,0 ...  
 \#Cal,65,1,0,0,0,0,0,0,0,0,0,0,...,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,242,1,56,72,28,5,0,0 ...  


There is python script: **talk_to_arduino_combine_histograms.py** that combines the read in raw histograms and raw electrical calibration histograms.
The script dumps all received data that starts with a \# 

Note that this script does not check that the histograms have been received from multiple I2C slaves. The i2c_slave_address field is ignored. The
script assumes that only one tmf882x is producing the histograms.

The combined histograms are named like this:  
- combined raw histograms: RCo for Raw Combined
- combined electrical calibration histograms: CCo for Calibration Combined

The combined histograms have the TDC + Channel index combined like the histograms dumped by the EVM.
There are always 10 combined histograms reported. 
- Number 0 == TDC0, Channel0,
- Number 1 == TDC0, Channel1
- Number 2 == TDC1, Channel0
- Number 3 == TDC1, Channel1
- ...
- Number 9 == TDC4, Channel1

The header for the raw combined histograms looks like this (note that now the Tdc+Channel is part of the first field, and the bin values are 24-bits):  
 \#RCo*Tdc_Channel*,bin0_value,bin1_value,bin2_value, ...  

E.g. raw histograms combined:  
 \#RCo0,3,0,1,4,1,1,0,4,4,3,2,3,1,103,16624,24971,8194, ...  
 \#RCo1,296,288,299,301,306,300,304,311,297,279,288,307, ...  
 
The header for the calibration combined histograms looks like this (note that now the Tdc+Channel is part of the first field, and the bin values are 24-bits):  
 \#CCo*Tdc_Channel*,bin0_value,bin1_value,bin2_value, ...  

E.g. calibration histograms combined  
 \#CCo0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, ...  