# -*- coding: utf-8 -*-
"""
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
 
Simple script converts arduino uno tmf882x raw histograms in combined ones

Format of histograms:
#Raw,<i2c_slave_addr>,<histogram_channel>,<data0>,<data1>, ...

"""

# Importing Libraries
import csv
import serial
import time


TMF882X_CHANNELS = 10
TMF882X_BINS = 128
TMF882X_SKIP_FIELDS = 3                         # skip first 3 fields
TMF882X_IDX_FIELD = (TMF882X_SKIP_FIELDS-1)
# this is the array for summing up each channel 
rawSum = [[0 for _ in range(TMF882X_BINS)] for _ in range(TMF882X_CHANNELS)]
eclSum = [[0 for _ in range(TMF882X_BINS)] for _ in range(TMF882X_CHANNELS)]


def write(x):
    """
    Write a single byte to the arduino
    Args:
        x (character): Must be one of the commands the arduino program understands, h,e,d,w,p,m,s,c,f,l,a,x.
    """
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)

def read():
    """
    Read a single line from the serial port. It is from the arduino.
    Returns:
        data (bytes): the read line from the arduino.
    """
    data = arduino.readline()
    if ( len(data) > 0 ):
        if ( data[0] != b'#'[0] ):
            print( data )
    return data

def waitForArduinoStartTalk():
    """
    Wait until Arduino starts talking.    
    Returns:
        data (bytes): the read line from the arduino.
    """
    data = b''                                              # make an empty list
    while ( len(data) == 0 ):                               # wait for the device to start talking == download completed
        data = read()
    return data

def waitForArduinoStopTalk():    
    """
    Wait until Arduino stops talking.    
    """
    data = b'0'                                             # make a non-empty list
    while ( len(data) > 0 ):                                # wait for the device to stop talking == help screen printed
        data = read()
       
def readWrite( csvwriter, number_of_results : int ):
    """
    Read lines from serial and print them to the CSV file. return when number_of_results
    records have been received    
    Args:
        csvwriter (csv.writer): the csv writer that is used to dump to the output file.
        number_of_results (int): dump records until this number of result records have been received.
    """
    records = 0
    while ( records < number_of_results ):                  # wait for the device to stop talking
        data = read()
        data = data.decode('utf-8')
        data = data.replace('\r','')                    # remove unwanted \r and \n to not have them 
        data = data.replace('\n','')                    # in the CSV file as unwanted chars
        row = data.split(',')
        if ( len(row) > 0 and row[0][0] == '#'):
            csvout.writerow( row )                   # dump all lines that start with a hashtag
            if ( row[0] ==  '#Obj' ):
                rowtowrite = row 
                records = records + 1
                print( "Number of result records received=", records )
            elif ( row[0] == '#Raw' and len(row) == TMF882X_BINS+TMF882X_SKIP_FIELDS ):
                # skip the I2C slave address field
                idx = int(row[TMF882X_IDX_FIELD])
                if ( idx >= 0 and idx <= 9 ):
                    for col in range(TMF882X_BINS):
                        rawSum[idx][col] = int( row[TMF882X_SKIP_FIELDS+col] )                                      # LSB is only assignement
                elif ( idx >= 10 and idx <= 19 ):
                    idx = idx - 10
                    for col in range(TMF882X_BINS):
                        rawSum[idx][col] = rawSum[idx][col] + int(row[TMF882X_SKIP_FIELDS+col]) * 256               # mid
                elif ( idx >= 20 and idx <= 29 ):
                    idx = idx - 20
                    for col in range(TMF882X_BINS):
                        rawSum[idx][col] = rawSum[idx][col] + int(row[TMF882X_SKIP_FIELDS+col]) * 256 * 256         # MSB
                    rowtowrite = ["#RCo"+str( idx )] + rawSum[idx] 
                    csvout.writerow( rowtowrite )                                                                   # dump this combined histogram too  
            elif ( row[0] == '#Cal' and len(row) == TMF882X_BINS+TMF882X_SKIP_FIELDS ):
                # skip the I2C slave address field
                idx = int(row[TMF882X_IDX_FIELD])
                if ( idx >= 0 and idx <= 9 ):
                    for col in range(TMF882X_BINS):
                        rawSum[idx][col] = int( row[TMF882X_SKIP_FIELDS+col] )                                      # LSB is only assignement
                elif ( idx >= 10 and idx <= 19 ):
                    idx = idx - 10
                    for col in range(TMF882X_BINS):
                        rawSum[idx][col] = rawSum[idx][col] + int(row[TMF882X_SKIP_FIELDS+col]) * 256               # mid
                elif ( idx >= 20 and idx <= 29 ):
                    idx = idx - 20
                    for col in range(TMF882X_BINS):
                        rawSum[idx][col] = rawSum[idx][col] + int(row[TMF882X_SKIP_FIELDS+col]) * 256 * 256         # MSB
                    rowtowrite = ["#CCo"+str( idx )] + rawSum[idx] 
                    csvout.writerow( rowtowrite )                                                                   # dump this combined histogram too  
 
if ( True ):                                                        # 
    number_of_results = 10*4 # multiply by 4 for 8x8
    CSV_FILE = "./tmf8828_arduino_uno_file_with_histogram.csv"

    f = open( CSV_FILE, 'w', encoding='UTF8', newline='' )
    f.write( "sep=,\n")    
    f.write( "Tmf8828 Arduino Uno \n");

    #  get arduino to read in    
    arduino = serial.Serial(port='COM27', baudrate=115200, timeout=0.1)

    #dump csv
    csvout = csv.writer( f, delimiter=',')
    
    # give arduinto time to wakeup
    while ( True ):
        data = read()
        if ( len(data) > 0 ):
            print( "started receiving ... ")
            break                                           # leave waiting loop
        else:
            time.sleep( 1.0 )
            write( 'h' )                                    # try to get device talking
            
    waitForArduinoStopTalk()                                # wait for the device to stop talking
    print( "Now we can instruct the device to do something ")

    write( 'd' )                                            # disable device for a clean start
    waitForArduinoStopTalk()

    print( "Enable device and download fw ----------------------- ")
    write( 'e' )                                            # enable device
    waitForArduinoStartTalk()                               # wait for the device to start talking
    waitForArduinoStopTalk()                                # wait for the device to stop talking

    print( "Dump also raw+calibration histograms ---------------- ")
    write( 'z' )                                            # raw histograms
    waitForArduinoStopTalk()                                # wait for the device to stop talking
    write( 'z' )                                            # calibration histograms
    waitForArduinoStopTalk()                                # wait for the device to stop talking
    write( 'z' )                                            # raw+calibration histograms
    waitForArduinoStopTalk()                                # wait for the device to stop talking
       
    print( "Now start measurements ")
    write( 'm' )                                            # start measurements

    readWrite( csvout, number_of_results )                  # read in xx results and store in csv format    
    
    write( 's' )                                            # stop measurements
    waitForArduinoStopTalk()                                # wait for the device to stop talking
    
    print( "Received {} results".format(number_of_results) )
    
    write( 'd' )                                            # disable device
    waitForArduinoStopTalk()                                # wait for the device to stop talking

    f.close()                                               # close file
    arduino.close()                                         # close serial port
    print( "End of program")
    
