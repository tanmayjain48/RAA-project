# Import necessary modules
from potentiostat import Potentiostat
import time
import serial
import serial.tools.list_ports
import traceback

# Define experiment parameters
potentiostat_ID = 0
curr_range = '1000uA'
volt_range = '1V'
low_voltage = -0.3
low_time = 0.5
high_voltage = 0.7
high_time = 2
sample_period = 0.1

# Define output file for results
filename = 'Calibration experiment.txt'

try:
    # Initialize potentiostat device
    dev = None
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        print 'Checking port %s'%(str(p))
        try:
            dev_temp = Potentiostat(p[0])
            cT = dev_temp.get_device_id()
            if cT == potentiostat_ID:
                dev = dev_temp
            print 'Connected...'
        except:
            print 'Skipping...'
            pass

    if dev:
        # Configure potentiostat for chronoamperometry
        print 'Running chronoamperometry on potentiostat with device ID: %s'%(str(dev.get_device_id()))
        dev.set_all_elect_connected(True)
        dev.set_volt_range(volt_range)
        dev.set_curr_range(curr_range)

        # Define variables for data collection
        list_length = 5
        curr_list = None

        while True:
            stTime = time.time()
            run_pot = True
            hold_period = False
            sample_time = time.time()
            num_samples = 0
            total_curr = 0

            # Cycle for low_time + high_time
            dev.set_volt(low_voltage)
            while run_pot:
                if time.time() - stTime > low_time:
                    # Start holding high voltage
                    if not hold_period:
                        dev.set_volt(high_voltage)
                        hold_period = True

                    # Sample every sample_period seconds
                    if time.time() - stTime > low_time + (high_time/2):
                        if time.time() - sample_time > sample_period:
                            total_curr += dev.get_curr()
                            num_samples += 1
                            sample_time = time.time()
                
                # Exit cycle
                if time.time() - stTime > low_time + high_time:
                    run_pot = False

            # Calculate moving average of sampled currents
            if curr_list is None:
                curr_list = [total_curr/num_samples]
            elif len(curr_list) < list_length:
                curr_list = curr_list + [total_curr/num_samples]
            else:
                curr_list = curr_list[1:len(curr_list)] + [total_curr/num_samples]
            curr = sum(curr_list)/len(curr_list)

            # Write current value to the file
            with open(filename, 'a') as fp:
                fp.write("%f\n" % curr)
            print "%f" % (curr)
    else:
        print 'Could not connect potentiostat with device ID: %s'%(str(potentiostat_ID))
except Exception as e:
    # Print traceback if an exception occurs
    traceback.print_exc()
finally:
    # Ensure that the test is stopped on all available ports
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        try:
            dev = Potentiostat(p[0])
            dev.stop_test()
        except:
            pass
