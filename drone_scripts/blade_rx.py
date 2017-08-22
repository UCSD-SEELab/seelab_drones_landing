'''
This will be a class to do basic receive operations on the BladeRF running
the 1.9.1 firmware. 

It will work on a laptop (Windows at least) or drone, and the user
will just have to identify whether he/she is using USB2/3 and if machine is
drone or laptop. To start it will just interface with BladeRF CLI. Good
documentation can be found here: 
    https://github.com/Nuand/bladeRF/tree/master/host/utilities/bladeRF-cli
    
Do not attempt to use this without a basic understanding of how RF stuff works
or you may damage the board. At the very least read through this:
https://github.com/Nuand/bladeRF/wiki/Getting-Started%3A-Verifying-Basic-Device-Operation#Loading_the_FPGA


General overview:
Likely the only truely useful stuff in here will be loading the FPGA image if used
on a raspberry pi (when gnuradio and bladerf software installed on windows the
image will load automatically). 
'''

interface = 'USB2' # make this automatic. Hi-Speed vs SuperSpeed
machine =   'drone'
default_fpga = '/usr/share/adafruit/webide/repositories/seelab_drones/drone_scripts/hostedx40.rbf'
verbosity = 'critical'

import os
import subprocess
import time
import sys
import gnuradio
import csv

if machine == 'laptop':
    prefix = 'bladeRF-cli'
    serial = 'f0da8b1365ac17c3a22d33e1130f2087'
elif machine == 'drone':
    prefix = 'bladeRF-cli'
    serial = '155002437db445798af3a5437a906ba2'

class blade_rf_sdr():
    
    def __init__(self, mode):
        self.blade_rf = 1
        num_attempts = 10                   # num times to try to find blade
        delay = 3                           # seconds to delay between searches
        if not self.find_device(num_attempts, delay):
            sys.exit("No BladeRF found. Restart program")
        
        self.load_fpga()
        if machine == 'drone':
            # Experimental, may not yet work
            subprocess.check_output([prefix, '-v', verbosity])
            # print('drone')
    
	# find device connected to usb
    def find_device(self, num_attempts, delay):
        while True:
            num_attempts = num_attempts - 1
            try:
                output = subprocess.check_output([prefix, '-p'])
                print("BladeRF found")
                return True
            except:
                print("Error connecting to device")
            if num_attempts == 0:
                return False
            time.sleep(delay)
    
    
    def open_blade(self, id):
        print("may be useless")
    
    # send a "command" sort of operation to the bladeRF
    def send_command(self, cmd):
        output = None
        args = cmd.split(" ")
        args.insert(0, prefix)
        try:
            output = subprocess.check_output(args)
        except:
            print("Error sending '" + cmd + "' to bladeRF")
    
    # send an "execution" sort of operation to the bladeRF
	# Look in documentation for distinction between exec/cmd if you care
    def send_exec(self, cmd):
        exec_cmd = [prefix, '-e', cmd]
        try:
            return subprocess.check_output(exec_cmd)
        except:
            print("Error sending '" + cmd + "' to bladeRF")
            # this is arbitrary but could be used for something in the future
            return("Errno " + str(5))
    
    
    def is_idle(self, mode):
        '''This function checks if the bladeRF is done transmitting or receiving
        by polling its status.
        Input: mode (string)
            'rx' - check if the bladeRF is done receiving
            'tx' - check if the bladeRF is done transmitting
        Output: result (boolean)
            True -  done with operation
            False - not done with operation or error checking               '''
        idle_cmd = mode
        result = self.send_exec(idle_cmd)
        if 'State: Idle' in result:
            print(mode + ' idle')
            return True
        elif 'State: Running' in result:
            print(mode + ' running')
            return False
        else:
            print('error checking idle status')
            return False
    
    
    def load_fpga(self, external = True, filepath = None):
        '''This function loads the FPGA image. Always run at bladeRF boot unless
        you have configured it to autoload on powerup'''
        
        print("Loading FPGA...")
        if external:
            self.send_command('-l' + " " + default_fpga)
        else:
            if not filepath:
                self.fpga_path = default_fpga
            else:
                self.fpga_path = filepath
            self.send_command('-l' + " " + self.fpga_path)
    
    #sets the sample rate
    def set_sample_rate(self, sample_freq):
        #sample_cmd = [prefix, '-e', 'set', 'samplerate', 'rx', str(sample_freq) + 'M']
        sample_cmd = ('set samplerate rx ' + str(sample_freq) + 'M')
        self.send_exec(sample_cmd)
    
    
    def set_bandwidth(self, mode, bandwidth):
        '''This function sets rx and/or tx bandwidth on the bladeSDR.
        Input 1 ('mode')      - which bandwidth to set
            - 'rx'  - set receiver bandwidth
            - 'tx'  - set transmitter bandwidth
            - 'all' - set both receive and transmit bandwidth
        
        Input 2 ('bandwidth') - bandwidth (number in MHz)
        '''
        if mode == 'rx' or mode == 'all':
            sample_cmd = ('set bandwidth rx ' + str(sample_freq) + 'M')
            self.send_exec(sample_cmd)
        if mode == 'tx' or mode == 'all':
            sample_cmd = ('set bandwidth tx ' + str(sample_freq) + 'M')
            self.send_exec(sample_cmd)


    def set_center_freq(self, mode, center_freq):
        '''Pass in center freq in MHz'''
        if mode == 'rx' or mode == 'all':
            freq_cmd = ('set frequency rx ' + str(center_freq) + 'M')
            # print("Setting rx center frequency to " + str(center_freq))
            self.send_exec(freq_cmd)
        if mode == 'tx' or mode == 'all':
            freq_cmd = ('set frequency tx ' + str(center_freq) + 'M')
            # print("Setting tx center frequency to " + str(center_freq))
            self.send_exec(freq_cmd)


    def set_amplifier_gain(self, amplifier, gain):
        '''This function sets the gain of the Rx and Tx amplifiers on the
        bladeRF. It can take multiple arguments as a list, but the number of
        elements of 'amplifier' must be equal to the number of elements of
        'gain' and the two must correspond (ie gain[1] is the gain value for
        amplifier[1])
        
        'Rx Amplifier Name' [gain range, in dB] (Rx or Tx amp):
           'lnagain' [0, 3, 6]    (Rx)
           'rxvga1'  [5 -> 30]    (Rx)
           'rxvga2'  [0 -> 30]    (Rx)
           'txvga1'  [-35 -> -4]  (Tx)
           'txvga2'  [0 -> 25]    (Tx)
        '''
        num_args = len(amplifier)
        if num_args != len(gain):
            sys.exit("Incorrect data provided to set_amplifier_gain function")
        for i in range (0, num_args):
            gain_cmd = ('set ' + amplifier[i] + ' ' + str(gain[i]))
            self.send_exec(gain_cmd)
            # print(gain_cmd)
    
    
    def rx_samples(self, n, file_format = None, filepath = None):
        '''
        Not pretty but it works
        Returns the fft data received by the bladeRF
        '''
        rx_cfg_exec = ('rx config file='+filepath + ' format='+file_format + 
        ' n=' + n + '; rx start; rx wait; rx')
        self.send_exec(rx_cfg_exec)
        return self.list_from_csv(filepath)


    def list_from_csv(self, filepath):
        '''This returns a list of the magnitudes for each of nfft points from
        the .csv file output by the bladeRF.
        
        Note that bladeRF outputs a column of I and a column of Q. We take the
        magnitude of these (~voltage, not ~power) and return the list'''
        with open(filepath, 'rb') as f:
            reader = csv.reader(f)
            data_list = list(reader)
        mag_list = [None] * len(data_list)
        for i in range(0, len(data_list)):
            mag_list[i] = (int(data_list[i][0])**2 + int(data_list[i][1])**2)**0.5
        return mag_list


    def run(self, sdr):
        # sdr.open_blade('1')
        # sdr.set_sample_rate(6)
        self.set_amplifier_gain(['lnagain', 'rxvga1', 'rxvga2'], [0, 5, 0])
        filename = '/usr/share/adafruit/webide/repositories/bladerf/BladeRX/trial.csv'
        self.rx_samples('100K', 'csv', filename)

if __name__ == '__main__':
    sdr = blade_rf_sdr(1)
    sdr.set_center_freq('all', 446.5)
    sdr.run(sdr)