For files created in gnuradio you must 'manually' load the fpga.
Do this with:

'import blade_rx' near the import statements and:

'blade_rx.blade_rf_sdr(1)' to the very top of main function

also in main for tx: change to
def main(top_block_cls=tx_2400_r2, options=None, tx_time=5, freq=None, fn=None):
    top_block_cls=tx_2400_r2
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


also in main for rx:
def main(top_block_cls=rx_2400_r2, options=None, rx_time=5, freq=None):
    top_block_cls=rx_2400_r2
    if options is None:
        options, _ = argument_parser().parse_args()
    
    if freq is not None:
        options.center_freq = freq
    tb = top_block_cls(center_freq=options.center_freq)
    tb.start()
    start_time = time.time()
    
    while (time.time() - start_time < rx_time):
        print(rx)
    '''
    try:
        raw_input('Press Enter to quit: ')
    except EOFError:
        pass'''
    tb.stop()
    tb.wait()