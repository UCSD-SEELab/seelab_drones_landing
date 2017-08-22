'''
Inspired by the work of Tony DiCola at adafruit and [whoever wrote rtlsdr()]

rxSDR is a class to interact with a XXXXX SDR receiver. I may add spectrum
visualization down the road if I figure it out and feel it's necessary.

Actually, real time visualization while flying sounds like a great idea!

Using work from "-----"

Using Adafruit's SDR Receiver USB stick - 
   RTL2832 w/ R820T

Highest safe sample rate is 2.56MS/s
Uses about 300mA @ 5v
Samples are unsigned 8-bit so (num - 127) = actual I/Q

Can change frequency about 40 times/second. 300/sec if we dgaf about PLL lock

Can tune 22 to 1870 MHz

Super awesome resource:
http://www.superkuh.com/rtlsdr.html

Future work:
-- Set frequency offset at runtime based on known radio source
-- Idiot proof: ensure cant be set too low/high, etc
'''

import rtlsdr as rtl
import numpy as np
from time import sleep

mhz = 1000000.0
khz = 1000.0
RX_MIN_FREQ = 22 * mhz

class rxSDR:

    def __init__(self, fc, fs, bw, gain):
        self.sdr = rtl.RtlSdr()
        sleep(1)
        self.setGain(gain)
        self.setFc(fc, "mhz")
        self.setFs(fs, "mhz")
        # self.setBwKhz(bw)

    def setBwKhz(self, bw_khz):     # does not work for some reason?
        self.sdr.set_bandwidth(bw_khz*khz)

    def setGain(self, gain_dB = "auto"): # input gain in dB, "auto" by default
        self.sdr.set_gain(gain_dB)

    def getGain(self):                   # get tuner gain in dB
        return self.sdr.get_gain()

    def getGains(self, unit = ".1dB"):  # default to output in 0.1dB
        gains = self.sdr.get_gains()
        if (unit == "dB"):
            gainsdB = [i / 10.0 for i in gains]
            return gainsdB
        else: return gains

    def printGain(self):
        print("gain=" + str(self.getGain()) + " dB")

    def printGains(self, unit = ".1dB"):   # print out gains. defaults to 0.1dB
        gains = self.getGains(unit)
        if (unit == "dB"):
            print("gains (dB):" + "\n" + str(gains))
        else: print("gains (0.1dB): " + "\n" + str(gains))

    def setFc(self, Fc, unit = "mhz"):
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

    def setFs(self, Fs, unit = "hz"):     # default to Hz
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
    def getFrequencies(self, nfft):
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