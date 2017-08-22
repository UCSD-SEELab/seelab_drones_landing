'''
Written by Stephen Wayne

Inspired by the work of Tony DiCola at adafruit and [whoever wrote rtlsdr]

rxSDR is a class to interact with a XXXXX SDR receiver. I may add spectrum
visualization down the road if I figure it out and feel it's necessary.

Using Adafruit's SDR Receiver USB stick - 
   RTL2832 w/ R820T

Highest safe sample rate is 2.56MS/s
Uses about 300mA @ 5v
Samples are unsigned 8-bit so (num - 127) = actual I/Q

Can change frequency about 40 times/second. 300/sec if we dgaf about PLL lock

Can tune 22 to 1870 MHz

Super awesome resource:
http://www.superkuh.com/rtlsdr.html

This version of the file tries to use threading to integrate with drone

General overview:
This function uses the RTL SDR (cheap one) to scan the frequencies listed
in "fc_list" and save the fft (frequency spectrum) to the database or a
text file. This can be good for debugging communication or learning how
to do some of this stuff, but probably won't be useful for other practical
purposes. It is something I developed while learning how to do SDR stuff,
so making something like this on your own may be a good learning tool.
'''

import rtlsdr as rtl
import numpy as np
from time import sleep
import time
import threading
from pubsub import pub

SAVE = False
DATABASE = True
FILENAME = "fullRange_2_5mhz.txt"

mhz = 1000000.0
khz = 1000.0
RX_MIN_FREQ = 22 * mhz
np.set_printoptions(precision=4)

fcLow = 100
fcHigh = 200
fc = 30
fs = 2.5
bw = 300
gain = 'auto'
NFFT = 1024
NUM_DECIMAL = 3
SCAN_RES = 1

class rxSDR(threading.Thread):

    def __init__(self):
        super(rxSDR, self).__init__()
        self._delay = 10
        self.daemon = True
        # Configure SDR parameters
        self.sdr = rtl.RtlSdr()
        self.setGain(gain)
        self.setFc(fc, "mhz")
        self.setFs(fs, "mhz")
        
        self.start()
		
	
    def setBwKhz(self, bw_khz):     # this is useless
        self.sdr.set_bandwidth(bw_khz*khz)

    def setGain(self, gain_dB = "auto"): # input gain in dB, "auto" by default
        self.sdr.set_gain(gain_dB)

    def getGain(self):                   # get tuner gain in dB
        return self.sdr.get_gain()

    def getGains(self, unit = ".1dB"):  # get gain from each stage of receiver (default to output in 0.1dB)
        gains = self.sdr.get_gains()
        if (unit == "dB"):
            gainsdB = [i / 10.0 for i in gains]
            return gainsdB
        else: return gains

    def printGain(self):                 # print receiver gain
        print("gain=" + str(self.getGain()) + " dB")

    def printGains(self, unit = ".1dB"):   # print out gains. defaults to 0.1dB
        gains = self.getGains(unit)
        if (unit == "dB"):
            print("gains (dB):" + "\n" + str(gains))
        else: print("gains (0.1dB): " + "\n" + str(gains))

    def setFc(self, Fc, unit = "mhz"):     # set center frequency (default to mhz)
        if (unit == "hz"):             Fc = Fc
        elif (unit == "khz"):          Fc = Fc * khz
        else:                          Fc = Fc * mhz
        if (Fc < RX_MIN_FREQ):
            print("Error. Fc too low. Setting to " + str(RX_MIN_FREQ))
            Fc = RX_MIN_FREQ
        self.sdr.set_center_freq(Fc)

    def getFc(self, unit = "mhz"):      # get center frequency. defaults to mhz
        freq_hz = self.sdr.get_center_freq()
        if (unit == "hz"):      return freq_hz
        elif (unit == "khz"):   return freq_hz / khz
        else:                   return freq_hz / mhz

    def printFc(self, unit = "mhz"):
        if ((unit != "khz") and (unit != "hz")): unit = "mhz"
        print("Fc=" + str(self.getFc(unit)) + " " + unit)

    def setFs(self, Fs, unit = "hz"):     # set sample frequency (default to Hz)
        if (unit == "khz"): self.sdr.set_sample_rate(Fs * khz)
        elif (unit == "mhz"): self.sdr.set_sample_rate(Fs * mhz)
        else: self.sdr.set_sample_rate(Fs)

    def getFs(self, unit):          # get sample rate. defaults to mhz
        freq_hz = self.sdr.get_sample_rate()
        if (unit == "hz"): return freq_hz
        elif (unit == "khz"): return freq_hz / khz
        else: return freq_hz / mhz

    def printFs(self, unit):
        if ((unit != "hz") and (unit != "khz")): unit = "mhz"
        print("Fs=" + str(self.getFs(unit)) + " " + unit)

    # inspired by Adafruit's model.py get_data function
    def getFrequencies(self, nfft):    # get spectrum (fft) of the current center frequency
        # Get width number of raw samples so the number of frequency bins is
		# the same as the display width.  Add two because there will be mean/DC
		# values in the results which are ignored.
        samples = self.sdr.read_samples(nfft)
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
    
    def _callback(self, sdr_data):     # how we communicate messages between threads
        pub.sendMessage("sensor-messages.sdr-data", arg1=sdr_data)
    
    def get_reading(self, fc_list):
        '''
        Get the spectrum of freqs in fc_list, store to database and/or file
        '''

        # radio = rxSDR(30, fs, bw, gain)  # radio on RPi
        # fc_list = np.linspace(fcLow, fcHigh, ((fcHigh - fcLow)/(SCAN_RES*fs) + 1))
        # radio = self.sdr

        if SAVE:
            with open(FILENAME, 'a') as file:
                file.write('fl,%f,fh,%f,fs,%f,mhz,gain,%s,nfft,%d,scan,%f'%(fcLow,fcHigh,fs,gain,NFFT,SCAN_RES)+'\n')
        now = time.time()
        i = 0
        data = []
        for x in fc_list:
            data.append([])
            self.setFc(x, "mhz")
            if ((time.time() - now) < 0.025): time.sleep(0.025 - (time.time()-now)) # allow PLL to settle
            freqs = self.getFrequencies(NFFT)
            if SAVE:
                with open(FILENAME, 'a') as f:
                    f.write(','.join(map(str, np.round(freqs, NUM_DECIMAL))) + '\n')
            if DATABASE:
                params = ['freq',x,'fs',fs,'mhz','gain',gain,'nfft',NFFT]
                freqs = freqs.tolist()
                freqs.insert(0, params)
            data[i].append(freqs)
            i = i + 1
        return data
        # print("Scan required " + str(time.time() - now) + " seconds")

    
    def run(self):
        fc_list = np.linspace(fcLow, fcHigh, ((fcHigh - fcLow)/(SCAN_RES*fs) + 1))
        while True:
            data = self.get_reading(fc_list)  # get the frequency data
            if data is not None:                     # if successful
                self._callback(data)                 # send the data to be logged
            time.sleep(self._delay)