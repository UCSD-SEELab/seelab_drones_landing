'''
For use with Nuand BladeRF. This script will scan the frequencies listed in fc
list and perform basic statistics to estimate which channel has the lowest
utilization. It will return the channel to use and the data it scanned.

We will be using 440 MHz, 925 MHz, and 1270 MHz to talk on since these are in
HAM or ISM bands.

NOTE: You must have a HAM radio license to do any transmitting!

TODO: find a way to suppress the bladeRF/GNU Radio output for cleaner terminal

General overview:
This script basically runs the show when it comes to bladeRF FSK communication.
Only one drone should be the master and theoretically any number can be slaves.
In this current basic implementation only the master gets to decide which freq.
to communicate on. It then sends out a message to inform the slave drones which
frequency they should listen on. Once a slave drone successfully gets a message
to change frequency it will hop over to the new frequency (that the master
determined to be "optimal"). After the master has determined an optimal frequency
it will transmit a message to switch to that frequency (at it's current/previous
frequency) for a set amount of time ("tx_trans_time", in seconds), then switch over
to the new freq. Each time the master transmits it will broadcast for "tx_time"
seconds and then go to sleep. Each slave will listen for "rx_time" seconds
between sleeping. You are free to mess around with these, but I found these
times to work well so that the drones don't "lose" each other (never exchange
messages on the right freq) in basic indoor and outdoor testing.
'''

### Only enable one or the other, not both (cannot be master and slave)!
MASTER = False                              # master determines freq to tx on
SLAVE = True
BEACON_RX = False                           # check signal strength from beacon
BEACON_FREQ = 440                           # only if BEACON_RX

import blade_rx as blade
import os
import subprocess
import numpy as np
from time import sleep
import time
import threading
from pubsub import pub
import tx_2400_r2
import rx_2400_r2
import extract_bits

SAVE = False
DATABASE = True
FILENAME = "blade_2_5mhz.txt"


mhz = 1000000
khz = 1000.0
RX_MIN_FREQ = 300 * mhz
np.set_printoptions(precision=4)

fs = 0.4                                     # 0.4 MHz, 400 kHz
f1 = 440
f2 = 925
f3 = 1270
fc = f1                                      # default frequency in MHz
current_freq_rx = fc
current_freq_tx = fc
tx_time = 7                                  # num seconds to tx msg for
rx_time = 6                                  # num seconds to rx msg for
tx_trans_time = 30                           # seconds before switch freq
lnagain = 6
rxvga1 = 30
rxvga2 = 30
txvga1 = -4
txvga2 = 25
NFFT = 1024
NUM_DECIMAL = 3
SCAN_RES = 1

# rxSDR is an old name used for the rtl. Change it if you want future person
class rxSDR(threading.Thread):

    def __init__(self):
        super(rxSDR, self).__init__()
        self._master_delay = 7
        self._slave_delay = 8
        self.daemon = True
        # Configure SDR parameters
        self.sdr = blade.blade_rf_sdr(1)         # init bladeRF, load FPGA
        self.setGainDefaults()
        # self.setFc(fc, 'mhz', 'rx')
        self.setFs(fs, 'mhz')
        
        self.start()


    def setGainDefaults(self): # input gain in dB, "auto" by default
        gain_list = ['lnagain', 'rxvga1', 'rxvga2', 'txvga1', 'txvga2']
        gain_vals = [lnagain, rxvga1, rxvga2, txvga1, txvga2]
        self.sdr.set_amplifier_gain(gain_list, gain_vals)


    def setFc(self, Fc, unit = "mhz", mode='rx'):
        '''Auto converted to MHz later so pass in MHz now. This is messy I'll
        clean it up if I have time'''
        '''
        if (unit == "hz"):             Fc = Fc
        elif (unit == "khz"):          Fc = Fc * khz
        else:                          Fc = Fc * mhz
        '''
        if (Fc < (RX_MIN_FREQ / mhz)):
            print("Error. Fc too low. Setting to " + str(RX_MIN_FREQ))
            Fc = RX_MIN_FREQ / mhz

        self.sdr.set_center_freq(mode, Fc)


    def setFs(self, Fs, unit = "hz"):     # default to Hz
        '''This should be good to go. Sloppy for now'''
        # if (unit == "khz"): self.sdr.set_sample_rate(Fs * khz)
        # elif (unit == "mhz"): self.sdr.set_sample_rate(Fs * mhz)
        # else: self.sdr.set_sample_rate(Fs)
        self.sdr.set_sample_rate(Fs)


    def getFrequencies(self, nfft):
        '''This should be properly updated. Report back after test'''
        # Get width number of raw samples so the number of frequency bins is
		# the same as the display width.  Add two because there will be mean/DC
		# values in the results which are ignored.
        filename = '/usr/share/adafruit/webide/repositories/seelab_drones/database_files/trial.csv'
        samples = self.sdr.rx_samples('1K', 'csv', filename)
        hw_time = np.hamming(nfft)
        # fft and take abs() to get frequency bin magnitudes
        freqs = np.absolute(np.fft.fft(np.multiply(samples, hw_time)))
        # ignore the mea/DC values at the ends
        # freqs = freqs[1:-1]
        # fftshift to have 0 freqs at center
        freqs = np.fft.fftshift(freqs)
        # convert to  dB
        freqs = 20.0*np.log10(freqs)
        return freqs
    
    def _callback(self, sdr_data):
        '''Send SDR scan data to subscribers'''
        pub.sendMessage("sensor-messages.sdr-data", arg1=sdr_data)

    
    def get_reading(self, fc_list):
        '''
        This function scans the spectrum given and returns the data
        Returns:
        
        data - [[metadata], nfft data points]; list of fft data from scan
        best_channel[1] - frequency (in MHz) to switch to
        '''

        if SAVE:
            with open(FILENAME, 'a') as file:
                file.write('fs,%f,mhz,nfft,%d'%(fs,NFFT)+'\n')
        now = time.time()
        i = 0
        data = []
        # store best channel data in form [avg of fft, frequency in MHz]
        best_channel = [3000, fc]
        
        if not BEACON_RX:
            for x in fc_list:
                data.append([])
                self.setFc(x, 'mhz', 'rx')
            
                if ((time.time() - now) < 0.025): time.sleep(0.025 - (time.time()-now)) # allow PLL to settle
            
                freqs = self.getFrequencies(NFFT)
            
                if SAVE:
                    with open(FILENAME, 'a') as f:
                        f.write(','.join(map(str, np.round(freqs, NUM_DECIMAL))) + '\n')
                if DATABASE:
                    params = ['freq',x,'fs',fs,'mhz','nfft',NFFT]
                    freqs = freqs.tolist()
                    freqs.insert(0, params)
            
                data[i].append(freqs)
                i = i + 1
                fft_avg = sum(freqs[1:NFFT+1]) / float(len(freqs[1:NFFT+1]))
                if fft_avg < best_channel[0]:
                    best_channel[0] = fft_avg
                    best_channel[1] = x

            # print("Scan required " + str(time.time() - now) + " seconds")
            data.insert(0, ['best_channel', best_channel[1]])
            return data, best_channel[1]
        else:
            # Best channel will be scanned channel
            self.setFc(BEACON_FREQ, 'mhz', 'rx')
            freqs = self.getFrequencies(NFFT)
            params = ['BEACON_RX', 'freq', BEACON_FREQ, 'fs', fs, 'mhz', 'nfft', NFFT]
            freqs = freqs.tolist()
            freqs = freqs[int(0.4 * NFFT):int(0.6*NFFT)]
            fft_avg = sum(freqs) / float(len(freqs))
            freqs.insert(0, params)
            data = freqs
            data.insert(0, ['avg', fft_avg])
            return data, BEACON_FREQ

                
    def send_channel_info(self, next_freq):
        '''This function blasts out the file for the next frequency to switch
        to, on the current frequency'''
        
        print('')
        print("%%% Sending message to switch to " + str(next_freq) + " MHz")
        print('')
        
        if next_freq == 925:
            file = '_send_f2.bin'
        elif next_freq == 1270:
            file = '_send_f3.bin'
        else: # then its 440 MHz
            file = '_send_f1.bin'
        
        # Don't worry about the first two args, can figure out if you look in
        # the GNU Radio script if you really care
        print("##### Transmitting on: " + str(current_freq_tx) + ' MHz')
        # print('Next transmission on: ' + str(next_freq) + ' MHz')
        print('')
        tx_2400_r2.main(None, None, tx_time, current_freq_tx*mhz, file)


    def change_tx_channel(self, next_freq):
        '''Send command to change transmitting frequency to the new one.
        This is called after some delay to make sure nearby drones have had a
        chance to "hear" about the frequency change. Future SEELab members can
        add some ack upon frequency change to eliminate the need for this.'''
        global current_freq_tx
        print('')
        print("@@@@@ Changing tx freq from %d to %d MHz"%
                (current_freq_tx, next_freq))
        print('')
        current_freq_tx = next_freq
        
    
    def change_rx_channel(self, next_freq):
        '''Same as above, but for Rx channel'''
        global current_freq_rx
        print("Changing rx freq from %d to %d MHz"%(current_freq_rx, next_freq))
        current_freq_rx = next_freq

    def receive_channel_info(self):
	    '''Listen for a message at current freq and then check the record to
		see if it successfully got a message'''
        print('')
        print("#####Listening for msg at: " + str(current_freq_rx) + ' MHz...')
        print('')
        rx_2400_r2.main(None, None, rx_time, current_freq_rx*mhz)
        output = extract_bits.main()
        # print(output)
        return output

    
    def run(self):
        '''Note that this is a bit ugly. "next_channel" is what we would predict
        we should communicate on next. In this setting though that is controlled
        by the master device, so if a slave drone it doesn't matter. For slave
        drones we just need to worry about what channel to receive on, so we use
        "next_freq"'''
        fc_list = [f1, f2, f3]
        start_time = time.time()
        while True:
            data, next_channel = self.get_reading(fc_list)
            
            if data is not None:                     # if successful
                if MASTER:
                    data.insert(0, ['c:', current_freq_tx, 'n:', next_channel])
                if SLAVE:
                    data.insert(0, ['c_rx:', current_freq_rx])
                self._callback(data)                 # send the data to be logged
            
            if MASTER:
                self.send_channel_info(next_channel)
                if time.time() - start_time >= tx_trans_time:
                    self.change_tx_channel(next_channel)
                    start_time = time.time()
                time.sleep(self._master_delay)
            
            if SLAVE:
                next_freq = self.receive_channel_info()
                if next_freq != -1:
                    self.change_rx_channel(next_freq)
                time.sleep(self._slave_delay)
            
            if BEACON_RX:
                print("#####Avg power in band: " + str(data[0][1]))
                time.sleep(2)
