from potentiostat import Potentiostat
import time
import serial
import serial.tools.list_ports
import traceback

potentiostat_ID = 0
curr_range = '1000uA'  # Name of current range for test [-10uA, +10uA]
volt_range = '1V'
low_voltage = -0.3
low_time = 0.5
high_voltage = 0.7
high_time = 2
sample_period = .1

filename = '2023-04-18-calibration-aRAA4-70C-120rpm-1000uA-V7-800mM.txt'

try: 
    dev = None
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        print 'Checking port %s'%(str(p))
        try:
            dev_temp = Potentiostat(p[0])
            cT = dev_temp.get_device_id()
            if cT == potentiostat_ID:
                dev = dev_temp
            print 'Binded...'
        except:
            print 'Skipping...'
            pass

    if dev:
        print 'Running chronoamperometry on potentiostat with device ID: %s'%(str(dev.get_device_id()))
        dev.set_all_elect_connected(True)
        dev.set_volt_range(volt_range)
        dev.set_curr_range(curr_range)
        ##dev.stop_test()

        list_length = 5
        curr_list = None
        while True:
            stTime = time.time()
            run_pot = True
            hold_period = False
##            sample_period = .1
            sample_time = time.time()
            num_samples = 0
            total_curr = 0

            # cycle for low_time + high_time
#           print "Holding Low"
            dev.set_volt(low_voltage)
            while run_pot:
                if time.time() - stTime > low_time:
                    # start holoding high voltage
                    if not hold_period:
#                        print "Holding High"
                        dev.set_volt(high_voltage)
                        hold_period = True

                    # sample every sample_period seconds
                    if time.time() - stTime > low_time + (high_time/2):
                        if time.time() - sample_time > sample_period:
#                            print dev.get_curr()
                            total_curr += dev.get_curr()
                            num_samples += 1
                            sample_time = time.time()
                
                # exit cycle
                if time.time() - stTime > low_time + high_time:
                    run_pot = False

            # keep moving average of sampled currents
            if curr_list is None:
                curr_list = [total_curr/num_samples]
            elif len(curr_list) < list_length:
                curr_list = curr_list + [total_curr/num_samples]
            else:
                curr_list = curr_list[1:len(curr_list)] + [total_curr/num_samples]
            curr = sum(curr_list)/len(curr_list)
            # write to the file
            with open(filename, 'a') as fp:
				fp.write("%f\n"%curr)
            print "%f"%(curr)
    else:
        print 'Could not connect potentiostat with device ID: %s'%(str(potentiostat_ID))
except:
    traceback.print_exc()
    pass
finally:
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        try:
            dev = Potentiostat(p[0])
            dev.stop_test()
        except:
            pass
