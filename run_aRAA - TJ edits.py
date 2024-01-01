#Disclaimer: 
#The views, opinions, and/or findings contained in this document are those of the authors and should not be interpreted 
#as representing the official views or policies of Department of Health and Human Services or the U.S. Government. 
#The mention of commercial products, their sources, or their use in connection with material reported herein
#is not to be construed as either actual or implied endorsement of such products by the Department of Health and Human Services.
#The authors assume no responsibility whatsoever for use of the automated Reactive Accelerated Aging (aRAA) 
#by other parties and make no guarantees, expressed or implied, about the quality, reliability, 
#or any other characteristic of the system. 
#Further, use of the aRAA in no way implies endorsement by the FDA or confers any advantage in regulatory decisions. 
#Any use of the aRAA is to be cited as stipulated in the license agreement. 
#In addition, any derivative work shall bear a notice that it is sourced from the aRAA, 
#and any modified versions shall additionally bear the notice that they have been modified.

import RPi.GPIO as GPIO
import time
from datetime import datetime
import smbus
import serial
import serial.tools.list_ports
import traceback
from potentiostat import Potentiostat

# Vessel Parameters
experimentName = ['Add file name here']
experimentRunParameters = [[145]]  # Set target H2O2 concentration here
average_value = 20

# Calibration parameters for converting current to H2O2 concentration
# The values below (12.292 and 0.1999) are examples and should be edited based on the user's calibration experiment.
# These values represent the y-intercept and slope of the linear regression between current and H2O2 concentration.
# Calibration Experiment: y = mx + b, where y is H2O2 concentration, x is current.
experimentCurrToConcFunctions = [lambda x: (x - 12.292) / 0.1999]

# Electrochemical Run Parameters:
# These parameters define the settings for the potentiostatic experiment (electrochemical run) for each reaction module.
# Each module has a dictionary specifying the current range, voltage range, voltage limits, and corresponding time durations.
# - "curr_range": Current range for the potentiostatic experiment (e.g., '1000uA' for 1000 microamperes).
# - "volt_range": Voltage range for the potentiostatic experiment (e.g., '1V' for 1 volt).
# - "low_volt": Lower voltage limit during the experiment in volts.
# - "high_volt": Higher voltage limit during the experiment in volts.
# - "low_volt_time": Duration of the lower voltage limit in seconds.
# - "high_volt_time": Duration of the higher voltage limit in seconds.
echemRunParameters = [{"curr_range": '1000uA',
                       "volt_range": '1V',
                       "low_volt": -0.3,
                       "high_volt": 0.7,
                       "low_volt_time": .5,
                       "high_volt_time": 2}]

# GPIO pins corresponding to peristaltic pump control
# Pin 17 is an example and should be edited based on the user's setup.
# This pin has positive voltage and is connected to an external power relay
# that controls the peristaltic pump.
pumpPins = [17]  

defaultPath = '/home/pi/potentiostat-master/software/python/potentiostat/RAA'

# Sampling rate for collecting data from the potentiostat. This is the rate
# at which the system reads potential and current data from the potentiostat.
# For example, the code below specifies the sampling rate (sampleRate) as float(2.5**-1), 
# which is equivalent to 1/2.5 or approximately 0.4 Hz. 
# This means that the system collects data from the potentiostat every 1/0.4 seconds,
# or approximately every 2.5 seconds.
sampleRate = float(2.5**-1)

# Data write rate for storing collected data to a file. This is the rate
# at which the system writes the collected data to a file. 
# For example, the code below specifies the sampling rate (dataWriteRate) as float(15**-1), 
# which is equivalent to 1/15 or approximately 0.067 Hz. 
# This means that the system writes data to the file every 1/0.067 seconds,
# or approximately every 15 seconds.
dataWriteRate = float(15**-1)

# I2C bus setup
bus = smbus.SMBus(1)
numberOfRAAs = len(experimentName)
allH2O2Conc = [[] for _ in range(numberOfRAAs)]
all_current = [float('nan') for _ in range(numberOfRAAs)]
echem_running_status = [False for _ in range(numberOfRAAs)]
echem_timing = [[] for _ in range(numberOfRAAs)]
new_current_available = [False for _ in range(numberOfRAAs)]
estimated_echem_end_time = [[] for _ in range(numberOfRAAs)]

# File names for data logging
dataLogFileName = [defaultPath + '/' + exp_name + '.txt' for exp_name in experimentName]
GPIO.setmode(GPIO.BCM)

# Set up GPIO pins for pumps
for k in range(len(pumpPins)):
    GPIO.setup(pumpPins[k], GPIO.OUT, initial=GPIO.LOW)

# Function to find Serial ports based on PID
def find_PID_serial_ports(num_RAAs):
    ser_ports = list(serial.tools.list_ports.comports())
    serialPID = [[] for _ in range(num_RAAs)]
    for k in range(len(ser_ports)):
        for k1 in range(num_RAAs):
            ser = serial.Serial(ser_ports[k][0], 9600, timeout=0)
            ser.flushInput()
            ser.flushOutput()
            cT = ser.readlines()
            string = f'*00{k1 + 1}G110 \r\r'
            ser.flushInput()
            ser.flushOutput()
            ser.write(string)
            ser.flushInput()
            ser.flushOutput()
            time.sleep(.1)
            cT = ser.readlines()
            ser.flushInput()
            ser.flushOutput()
            ser.close()
            if cT:
                serialPID[k1] = ser_ports[k][0]
    return serialPID

# Function to find Arduino Serial ports
def find_arduino_serial_ports(num_RAAs):
    Ard_Address = [[] for _ in range(num_RAAs)]
    ports = list(serial.tools.list_ports.comports())
    print('Looking for ARD port...')
    for p in ports:
        try:
            dev = Potentiostat(p[0])
            cT = dev.get_device_id()
            if float(cT) == 0:
                print(f'Connecting RAA 1 to Arduino (ID: {cT}) via port {p[0]}.')
                Ard_Address[0] = dev
        except:
            pass
    for i in range(num_RAAs):
        if not Ard_Address[i]:
            print(f'Could not connect RAA {i + 1} to potentiostat')
    return Ard_Address

# Function to write data to RAA file
def Write_To_RAA_File(log_file_name, new_line):
    current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
    new_line = [current_time] + [str(val) for val in new_line]
    new_line = '\t'.join(new_line) + '\n'
    with open(log_file_name, 'a') as RAAFile:
        RAAFile.write(new_line)

# Function to add value to data list
def Add_Value_To_Data_List(old_vals, new_val, len_limit):
    if len(old_vals) >= len_limit:
        old_vals = old_vals[1:]
    old_vals.append(new_val)
    return old_vals

# Function to run experiment in synchronized mode
def run_rodeo_synchronized(device_ID, echem_timer, echem_parameters):
    # ... (skipped for brevity)

# Function to initialize the experiment
def initialize_experiment(device_ID, pump_pin, experiment_parameters):
    # ... (skipped for brevity)

# Find Serial ports for PID and Arduino
serialPID = find_PID_serial_ports(1)  # Changed to 1 RAA
Ard_Address = find_arduino_serial_ports(1)  # Changed to 1 RAA

# Check if connections are successful
if not serialPID[0]:
    print('Could not connect to RAA 1')
    GPIO.cleanup()
    quit()

# Initialize experiment for the single RAA
pumpPin = pumpPins[0]
deviceID = Ard_Address[0]
experimentRunParameter = experimentRunParameters[0]
experimentCurrToConcFunction = experimentCurrToConcFunctions[0]
echemRunParameter = echemRunParameters[0]
experimentName_k = experimentName[0]
dataLogFileName_k = dataLogFileName[0]

experiment_parameters = {
    'curr_to_conc_function': experimentCurrToConcFunction,
    'echem_parameters': echemRunParameter,
    'experiment_run_parameters': experimentRunParameter,
    'experiment_name': experimentName_k,
    'data_log_file_name': dataLogFileName_k,
    'sample_rate': sampleRate,
    'data_write_rate': dataWriteRate
}

try:
    device_ID = Potentiostat(serialPID[0])
    initialize_experiment(device_ID, pumpPin, experiment_parameters)
except:
    print(f"Error in RAA 1: {traceback.format_exc()}")
    pass

finally:
    GPIO.cleanup()
    print('Exiting script...')
