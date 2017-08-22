#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Rx 2400 R2
# Generated: Sun Mar 19 20:12:40 2017
# 
# General comments on this file:
# This file is used to interface with the bladeRF to receive
# data at 2530 bits per second (approx. 2.4k baud, hence the
# file name). The meat of this file was generated using GNU
# Radio from a simple FSK receiver that I designed 
# (use that file as a reference to learn how to do future
# designs - it is titled *.grc wheree * is something similar
# to the title of this file). I made a few modifications to
# this file (many of which are detailed in bladerf_readme.txt),
# and you will have to make similar modifications to future
# designs to make it work on the Raspberry Pi and interface
# with existing software. The mods I made are by no means the
# best way to do things, so feel free to be creative. The
# most important one for running this on a Pi is to load the
# FPGA image (using stuff from blade_rx.py) - you cannot
# really interface with BladeRF without.
# 
# Note: the baud rate in here is different than the baud rate
# in the transmitter file. This is mostly an experimental
# result. FSK clock recovery was more successful in field
# testing using 2530 vs 2490. When you design transceiver
# systems you will likely need to do similar testing to make
# necessary tweaks to make communication more reliable.
##################################################

from gnuradio import analog
from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import math
import osmosdr
import time
import blade_rx
import subprocess


class rx_2400_r2(gr.top_block):

    def __init__(self, center_freq=440000000):
        gr.top_block.__init__(self, "Rx 2400 R2")

        ##################################################
        # Parameters
        ##################################################
        self.center_freq = center_freq

        ##################################################
        # Variables
        ##################################################
        self.sps = sps = 10
        self.baud_rate = baud_rate = 2530
        self.samp_rate_tx = samp_rate_tx = 400000
        self.samp_rate = samp_rate = int(baud_rate * sps)
        self.rx_vga_gain = rx_vga_gain = 35
        self.rx_lna_gain = rx_lna_gain = 6
        self.quad_gain = quad_gain = 8
        self.freq = freq = center_freq

        ##################################################
        # Blocks
        ##################################################
        self.osmosdr_source_0 = osmosdr.source( args="numchan=" + str(1) + " " + "bladerf=0" )
        self.osmosdr_source_0.set_sample_rate(samp_rate_tx)
        self.osmosdr_source_0.set_center_freq(freq, 0)
        self.osmosdr_source_0.set_freq_corr(0, 0)
        self.osmosdr_source_0.set_dc_offset_mode(1, 0)
        self.osmosdr_source_0.set_iq_balance_mode(1, 0)
        self.osmosdr_source_0.set_gain_mode(False, 0)
        self.osmosdr_source_0.set_gain(rx_lna_gain, 0)
        self.osmosdr_source_0.set_if_gain(0, 0)
        self.osmosdr_source_0.set_bb_gain(rx_vga_gain, 0)
        self.osmosdr_source_0.set_antenna("", 0)
        self.osmosdr_source_0.set_bandwidth(1500000, 0)
          
        self.low_pass_filter_0 = filter.fir_filter_ccf(1, firdes.low_pass(
        	1, samp_rate_tx, 6000, 6000, firdes.WIN_HAMMING, 6.76))
        self.fir_filter_xxx_0 = filter.fir_filter_fff(16, (firdes.low_pass(1.0, baud_rate*sps, baud_rate, 0.25*baud_rate)))
        self.fir_filter_xxx_0.declare_sample_delay(0)
        self.digital_pfb_clock_sync_xxx_0 = digital.pfb_clock_sync_fff(sps, 2*3.14159265/100, (firdes.low_pass(1.0, baud_rate*sps, baud_rate, 0.25*baud_rate)), 32, 16, 1.5, 1)
        self.digital_binary_slicer_fb_0 = digital.binary_slicer_fb()
        self.blocks_file_sink_0_0 = blocks.file_sink(gr.sizeof_char*1, "/usr/share/adafruit/webide/repositories/seelab_drones/data/_out.bin", False)
        self.blocks_file_sink_0_0.set_unbuffered(True)
        self.blocks_add_const_vxx_0 = blocks.add_const_vff((0, ))
        self.analog_quadrature_demod_cf_0 = analog.quadrature_demod_cf(quad_gain)
        self.analog_pwr_squelch_xx_0 = analog.pwr_squelch_cc(-70, 1e-4, 0, True)

        ##################################################
        # Connections
        ##################################################
        self.connect((self.analog_pwr_squelch_xx_0, 0), (self.low_pass_filter_0, 0))    
        self.connect((self.analog_quadrature_demod_cf_0, 0), (self.blocks_add_const_vxx_0, 0))    
        self.connect((self.blocks_add_const_vxx_0, 0), (self.digital_pfb_clock_sync_xxx_0, 0))    
        self.connect((self.digital_binary_slicer_fb_0, 0), (self.blocks_file_sink_0_0, 0))    
        self.connect((self.digital_pfb_clock_sync_xxx_0, 0), (self.fir_filter_xxx_0, 0))    
        self.connect((self.fir_filter_xxx_0, 0), (self.digital_binary_slicer_fb_0, 0))    
        self.connect((self.low_pass_filter_0, 0), (self.analog_quadrature_demod_cf_0, 0))    
        self.connect((self.osmosdr_source_0, 0), (self.analog_pwr_squelch_xx_0, 0))    

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.set_freq(self.center_freq)

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.set_samp_rate(int(self.baud_rate * self.sps))
        self.digital_pfb_clock_sync_xxx_0.update_taps((firdes.low_pass(1.0, self.baud_rate*self.sps, self.baud_rate, 0.25*self.baud_rate)))
        self.fir_filter_xxx_0.set_taps((firdes.low_pass(1.0, self.baud_rate*self.sps, self.baud_rate, 0.25*self.baud_rate)))

    def get_baud_rate(self):
        return self.baud_rate

    def set_baud_rate(self, baud_rate):
        self.baud_rate = baud_rate
        self.set_samp_rate(int(self.baud_rate * self.sps))
        self.digital_pfb_clock_sync_xxx_0.update_taps((firdes.low_pass(1.0, self.baud_rate*self.sps, self.baud_rate, 0.25*self.baud_rate)))
        self.fir_filter_xxx_0.set_taps((firdes.low_pass(1.0, self.baud_rate*self.sps, self.baud_rate, 0.25*self.baud_rate)))

    def get_samp_rate_tx(self):
        return self.samp_rate_tx

    def set_samp_rate_tx(self, samp_rate_tx):
        self.samp_rate_tx = samp_rate_tx
        self.low_pass_filter_0.set_taps(firdes.low_pass(1, self.samp_rate_tx, 6000, 6000, firdes.WIN_HAMMING, 6.76))
        self.osmosdr_source_0.set_sample_rate(self.samp_rate_tx)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate

    def get_rx_vga_gain(self):
        return self.rx_vga_gain

    def set_rx_vga_gain(self, rx_vga_gain):
        self.rx_vga_gain = rx_vga_gain
        self.osmosdr_source_0.set_bb_gain(self.rx_vga_gain, 0)

    def get_rx_lna_gain(self):
        return self.rx_lna_gain

    def set_rx_lna_gain(self, rx_lna_gain):
        self.rx_lna_gain = rx_lna_gain
        self.osmosdr_source_0.set_gain(self.rx_lna_gain, 0)

    def get_quad_gain(self):
        return self.quad_gain

    def set_quad_gain(self, quad_gain):
        self.quad_gain = quad_gain
        self.analog_quadrature_demod_cf_0.set_gain(self.quad_gain)

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.osmosdr_source_0.set_center_freq(self.freq, 0)


def argument_parser():
    parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
    parser.add_option(
        "-f", "--center-freq", dest="center_freq", type="intx", default=440000000,
        help="Set center_freq [default=%default]")
    return parser


def main(top_block_cls=rx_2400_r2, options=None, rx_time=5, freq=None):
    #subprocess.check_output(['bladeRF-cli', '-v', 'verbose'])
    top_block_cls=rx_2400_r2
    if options is None:
        options, _ = argument_parser().parse_args()
    
    if freq is not None:
        options.center_freq = freq
    tb = top_block_cls(center_freq=options.center_freq)
    tb.start()
    start_time = time.time()
    
    while (time.time() - start_time < rx_time):
        time.sleep(0.5)
    '''
    try:
        raw_input('Press Enter to quit: ')
    except EOFError:
        pass'''
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
