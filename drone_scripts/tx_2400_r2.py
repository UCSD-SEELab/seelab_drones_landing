#!/usr/bin/env python2
# -*- coding: utf-8 -*-
##################################################
# GNU Radio Python Flow Graph
# Title: Tx 2400 R2
# Generated: Sun Mar 19 20:12:58 2017
# 
# General comments on this file:
# This file is used to interface with the bladeRF to transmit
# data at 2490 bits per second (approx. 2.4k baud, hence the
# file name). The meat of this file was generated using GNU
# Radio from a simple FSK transmitter that I designed 
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
##################################################

from gnuradio import blocks
from gnuradio import digital
from gnuradio import eng_notation
from gnuradio import filter
from gnuradio import gr
from gnuradio.eng_option import eng_option
from gnuradio.filter import firdes
from optparse import OptionParser
import osmosdr
import time
# import blade_rx


class tx_2400_r2(gr.top_block):

    def __init__(self, center_freq=440000000, filename="_send.bin"):
        gr.top_block.__init__(self, "Tx 2400 R2")

        ##################################################
        # Parameters
        ##################################################
        self.center_freq = center_freq
        self.filename = filename

        ##################################################
        # Variables
        ##################################################
        self.sps = sps = 10
        self.prefix = prefix = '/usr/share/adafruit/webide/repositories/bladerf/BladeRX/'
        self.baud_rate = baud_rate = 2490
        self.txvga2 = txvga2 = 25
        self.txvga1 = txvga1 = -4
        self.samp_rate_tx = samp_rate_tx = 400000
        self.samp_rate = samp_rate = int(baud_rate * sps)
        self.freq = freq = center_freq
        self.filepath = filepath = prefix + filename

        ##################################################
        # Blocks
        ##################################################
        self.rational_resampler_xxx_0 = filter.rational_resampler_ccc(
                interpolation=samp_rate_tx,
                decimation=samp_rate,
                taps=None,
                fractional_bw=None,
        )
        self.osmosdr_sink_0 = osmosdr.sink( args="numchan=" + str(1) + " " + "bladerf=0" )
        self.osmosdr_sink_0.set_sample_rate(samp_rate_tx)
        self.osmosdr_sink_0.set_center_freq(freq, 0)
        self.osmosdr_sink_0.set_freq_corr(0, 0)
        self.osmosdr_sink_0.set_gain(txvga2, 0)
        self.osmosdr_sink_0.set_if_gain(0, 0)
        self.osmosdr_sink_0.set_bb_gain(txvga1, 0)
        self.osmosdr_sink_0.set_antenna("", 0)
        self.osmosdr_sink_0.set_bandwidth(1500000, 0)
          
        self.digital_gfsk_mod_0 = digital.gfsk_mod(
        	samples_per_symbol=sps,
        	sensitivity=1.0,
        	bt=1,
        	verbose=False,
        	log=False,
        )
        self.blocks_file_source_0 = blocks.file_source(gr.sizeof_char*1, filepath, True)

        ##################################################
        # Connections
        ##################################################
        self.connect((self.blocks_file_source_0, 0), (self.digital_gfsk_mod_0, 0))    
        self.connect((self.digital_gfsk_mod_0, 0), (self.rational_resampler_xxx_0, 0))    
        self.connect((self.rational_resampler_xxx_0, 0), (self.osmosdr_sink_0, 0))    

    def get_center_freq(self):
        return self.center_freq

    def set_center_freq(self, center_freq):
        self.center_freq = center_freq
        self.set_freq(self.center_freq)

    def get_filename(self):
        return self.filename

    def set_filename(self, filename):
        self.filename = filename
        self.set_filepath(self.prefix + self.filename)

    def get_sps(self):
        return self.sps

    def set_sps(self, sps):
        self.sps = sps
        self.set_samp_rate(int(self.baud_rate * self.sps))

    def get_prefix(self):
        return self.prefix

    def set_prefix(self, prefix):
        self.prefix = prefix
        self.set_filepath(self.prefix + self.filename)

    def get_baud_rate(self):
        return self.baud_rate

    def set_baud_rate(self, baud_rate):
        self.baud_rate = baud_rate
        self.set_samp_rate(int(self.baud_rate * self.sps))

    def get_txvga2(self):
        return self.txvga2

    def set_txvga2(self, txvga2):
        self.txvga2 = txvga2
        self.osmosdr_sink_0.set_gain(self.txvga2, 0)

    def get_txvga1(self):
        return self.txvga1

    def set_txvga1(self, txvga1):
        self.txvga1 = txvga1
        self.osmosdr_sink_0.set_bb_gain(self.txvga1, 0)

    def get_samp_rate_tx(self):
        return self.samp_rate_tx

    def set_samp_rate_tx(self, samp_rate_tx):
        self.samp_rate_tx = samp_rate_tx
        self.osmosdr_sink_0.set_sample_rate(self.samp_rate_tx)

    def get_samp_rate(self):
        return self.samp_rate

    def set_samp_rate(self, samp_rate):
        self.samp_rate = samp_rate

    def get_freq(self):
        return self.freq

    def set_freq(self, freq):
        self.freq = freq
        self.osmosdr_sink_0.set_center_freq(self.freq, 0)

    def get_filepath(self):
        return self.filepath

    def set_filepath(self, filepath):
        self.filepath = filepath
        self.blocks_file_source_0.open(self.filepath, True)


def argument_parser():
    parser = OptionParser(option_class=eng_option, usage="%prog: [options]")
    parser.add_option(
        "-f", "--center-freq", dest="center_freq", type="intx", default=440000000,
        help="Set center_freq [default=%default]")
    parser.add_option(
        "-n", "--filename", dest="filename", type="string", default="_send.bin",
        help="Set filename [default=%default]")
    return parser


def main(top_block_cls=tx_2400_r2, options=None, tx_time=5, freq=None, fn=None):
    top_block_cls=tx_2400_r2
    # blade_rx.blade_rf_sdr(1)
    if options is None:
        options, _ = argument_parser().parse_args()

    if freq is not None and fn is not None:
        options.center_freq = freq
        options.filename=fn

    tb = top_block_cls(center_freq=options.center_freq, filename=options.filename)
    tb.start()
    start_time = time.time()
    
    while (time.time() - start_time < tx_time):
        time.sleep(0.5)
    
    tb.stop()
    tb.wait()


if __name__ == '__main__':
    main()
