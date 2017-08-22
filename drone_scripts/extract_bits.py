# -*- coding: utf-8 -*-
"""
This will extract received data from a received data file. It is basically
just glorified string parsing. Logic and efficiency can be improved in the
future but for now it works well enough.

If parsing large files look into using "nmap" to lower resource requirements.

This is tailored to the simple yet reliable messages that I was transmitting.
It may be useful for you to learn a little about how to parse/structure messages,
but the way that I designed messages is not the way that it should be done now
that limited time is less of a concern.

Note: "average value" just means the number of "1"s in a binary string divided
by the length of that string. Not what you want to do in the future but it
added reliability for me since I really only had time for one full scale
field test.

String 1 is '\Gee\\See\' in binary
    -- corresponds to 440 MHz, average value is 0.50
String 2 is '@!BADA$$!@' in binary
    -- corresponds to 925 MHz, average value is 0.225
String 3 is '?wow==wow?' in binary
    -- corresponds to 1270 MHz, average value is 0.725

@author: whatn
"""
SEND = False
RECEIVE = True
VERBOSE = False
BASIC = True
PACKET_LEN = 80
HD_LIMIT = 5
# Strings designed to represent the 3 different frequencies
f1_str = '01011100010001110110010101100101010111000101110001010011011001010110010101011100'
f2_str = '01000000001000010100001001000001010001000100000100100100001001000010000101000000'
f3_str = '00111111011101110110111101110111001111010011110101110111011011110111011100111111'
rx_str = [f1_str, f2_str, f3_str]
# the three frequencies, in MHz
f1 = 440
f2 = 925
f3 = 1270
rx_freqs = [f1, f2, f3]
# the "average value" of the strings representing the three frequencies
rx_avgs = [0.50, 0.225, 0.725]

def hamming(data1, data2):
    '''Compute the hamming distance. Return -1 if different lengths.
	Note that this is not very useful as it does not shift bits left or
	right, it only checks if they match up or not. Clock recovery is a
	common issue with message exchange (you may sample a little too fast
	and see the same bit twice - two 0s instead of one 0 - or you may
	sample a bit too slow and miss the bit. It would be better if
	this incorporated shifting. Really only useful to check if the message
	fully matches or not.'''
    if len(data1) != len(data2):
        return -1
    else:
        return sum(b1 != b2 for b1, b2 in zip(data1, data2))


def check_next_packet(msg_start, preamble, data):
    '''Check for a new message packet later in the file of received
	data. Return -1 if none'''
    new_msg_start = data.find(preamble, msg_start) + len(preamble)
    if (new_msg_start - len(preamble)) == -1:
        return -1
    return new_msg_start


def hex_to_bits(packet):
    data_hex = [elem.encode('hex') for elem in packet]
    data_str = ''
    data_list = []
    for byte in data_hex:
        if byte == '00':
            data_str += '0'
            data_list.append(0)
        elif byte == '01':
            data_str += '1'
            data_list.append(1)
    return data_str, data_list


def get_packet(msg_start, postamble, data):
    '''Look for a packet in the received data file.'''
    msg_end = data.find(postamble, msg_start)
    if msg_end == -1:
        return -1, -1
    else:
        packet = data[msg_start:msg_end]
    data_str, data_list = hex_to_bits(packet)
    return data_str, data_list


def check_str(extracted_str, extracted_list):
    '''Checks string against known responses. If hamming distance is less than
    5 for a message we assume that is correct. If it is greater than 5 then we
    calculate the "average" of the string of bits [(# of 1s) / string length]
    to estimate which message was sent
    
    NOTE: This should only be called if preamble found in message
    RETURNS: Next frequency to switch to (in mhz)
    '''
    idx = 0
    lowest_hd = [PACKET_LEN, idx]
    for string in rx_str:
        hd = hamming(extracted_str, string)
        if hd < lowest_hd[0]:
            lowest_hd = [hd, idx]
            if VERBOSE:
                print('Hamming distance=' + str(hd) + ' to string ' + string)
        idx += 1
    
    if lowest_hd[0] <= HD_LIMIT:
        freq = rx_freqs[lowest_hd[1]]
        if VERBOSE or BASIC:
            print('')
            print("%%%(S) Received cmd to change freq to " + str(freq) + " MHz")
            print('')
    else:
        if VERBOSE: print("String unusable. Taking average value...")
        avg = sum(extracted_list) / float(len(extracted_list))
        closest_val = min(rx_avgs, key=lambda x:abs(x-avg))
        freq_idx = rx_avgs.index(closest_val)
        freq = rx_freqs[freq_idx]
        if VERBOSE or BASIC:
            print('')
            print("%%%(A) Received cmd to change freq to " + str(freq) + " MHz")
            print('')
    return freq


#To use on PC just change the file paths
def main():
    '''
	This basically parses through a file of hex/binary data received by
	the bladeRF and checks if it can find an appropriate packet. If it
	sees a proper preamble then it knows where to look, and if not then
	it knows there is no fully correct packet in the received data. It has_key
	basic data processing to attempt to get a packet even if transmission is not
	perfect. Again this is applicable to the system I used to test but you will
	probably want to write your own version of this tailored to whatever
	modifications you may make.
	
	Preamble is a bit sequence that preceeds a packet and a postamble is a
	bit sequence that follows the packet. If message exchange is completely
	successful you will find the data to be exchanged between the pre-
	and postamble.
	'''
    if SEND:
        preamble = '\x01\x01\x01\x01'
        postamble = '\x3C\x3C\x3C\x3C'
        data = open('_send.bin', 'r').read()
    elif RECEIVE:
        preamble = '\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x01\x00\x00\x00\x00\x00\x00\x00\x01'
        postamble = '\x00\x00\x01\x01\x01\x01\x00\x00\x00\x00\x01\x01\x01\x01\x00\x00\x00\x00\x01\x01\x01\x01\x00\x00\x00\x00\x01\x01\x01\x01\x00\x00'
        postamble = preamble
        data = open('/usr/share/adafruit/webide/repositories/seelab_drones/data/_out.bin', 'r').read()
    
	# index of start of message
    msg_start = check_next_packet(0, preamble, data)
    num_iter = 1
    
	# if there might be a message
    if msg_start != -1:
        data_str, data_list = get_packet(msg_start, postamble, data)
        # print(data_str)
        if data_str == -1 or len(data_str) != PACKET_LEN:
            packet = data[msg_start:msg_start + PACKET_LEN]
            data_str_bak, data_list_bak = hex_to_bits(packet)
            #print(data_str_bak)
            if VERBOSE: print("Data error. Checking for more packets")
            while data_str == -1 or len(data_str) != PACKET_LEN:
                num_iter += 1
                msg_start = check_next_packet(msg_start, preamble, data)
                if msg_start == -1:
                    # no more new packets found
                    break
                data_str, data_list = get_packet(msg_start, postamble, data)
            
            if data_str == -1 or len(data_str) != PACKET_LEN or msg_start == -1:
                # If we must use the first string found
                if VERBOSE: print("No usable packets found. Using original")
                data_list_bak = data_list_bak[0:PACKET_LEN]
                data_list = data_list_bak
                data_str_bak = data_str_bak[0:PACKET_LEN]
                data_str = data_str_bak
            
            ratio = sum(data_list) / float(len(data_list))
            if VERBOSE: print("Iterations: " + str(num_iter))
        else:
            # if successful on first packet
            ratio = sum(data_list) / float(len(data_list))
        
		# verbose just to help debug program and data transmission
        if VERBOSE:
            print('Ratio of 1s to 0s: ' + str(ratio))
            print("Hamming distance: " + str(hamming(data_str, expected_str)))
            print(data_str)
        
		# best guess at next frequency to hop to
        freq = check_str(data_str, data_list)
    else:
        print("Warning: no data packet found")
        freq = -1
    return freq

main()