#!/usr/bin/env python
'''
gdp_access.py
'''

import threading
import json
import time
import pprint

import gdp

DEBUG_PRINT = True


class Signpost(object):
    '''Signpost object holds most recent readings for each Signpost and 
       generates alerts when a Signpost '''
    def __init__(self, name):
        print('Signpost: Initializing signpost %s' % (name))
        self.pp = pprint.PrettyPrinter(indent=4)
        self.name = name
        self.mac_addr = None
        self.loc = None
        self.f_anomaly = 0
        self.last_ts = None
        self.audio = {'ts':None, 
                      'recno':None}
        self.gps = {'ts':None,
                    'recno':None}
        self.energy = {'ts':None,
                       'recno':None}
        self.radio = {'ts':None,
                      'recno':None}
        self.microwave_radar = {'ts':None,
                                'recno':None}
        self.ambient = {'ts':None,
                        'recno':None}
        self.air_quality = {'ts':None,
                            'recno':None}
        self.anomalies = {'ts':None,
                            'recno':None}
        
    def update(self, msg_in, device='Unknown'):
        print('Signpost: Updating signpost %s' % (self.name))
        if ('data' in msg_in):
            try:        
                msg_data = json.loads(msg_in['data'])
            except:
                print('Signpost: Failed to parse message')
                return
        else:
            return
            
        if ('ts' in msg_in):
            if ('tv_sec' in msg_in['ts']):
                if (self.last_ts < msg_in['ts']['tv_sec']):
                    self.last_ts = msg_in['ts']['tv_sec']
                    print('Signpost.update: %s has new recent message at %d' % (self.name, self.last_ts))
            else:
                self.last_ts = None
                return
        else:
            self.last_ts = None
            return
        
        msg_data['device'] = device
        
        if ((self.mac_addr == None) and ('_meta' in msg_data)):
            if ('device_id' in '_meta'):
                self.mac_addr = msg_data['_meta']['device_id']
        
        if (msg_data['device'] == 'signpost_audio_frequency'):
            self._process_audio_freq(msg_data)
            
        elif (msg_data['device'] == 'signpost_gps'):
            self._process_gps(msg_data)
            
        elif (msg_data['device'] == 'signpost_energy'):
            self._process_energy(msg_data)
            
        elif (msg_data['device'] == 'signpost_radio'):
            self._process_radio(msg_data)
            
        elif (msg_data['device'] == 'signpost_microwave_radar'):
            self._process_microwave_radar(msg_data)
            
        elif (msg_data['device'] == 'signpost_ambient'):
            self._process_ambient(msg_data)
            
        elif (msg_data['device'] == 'signpost_ucsd_air_quality'):
            self._process_ucsd_air_quality(msg_data)
        
        elif (msg_data['device'] == 'anomalies'):
            self._process_anomalies(msg_data)
    
    def _process_audio_freq(self, msg):
        '''Process data from the Audio Frequency Module sent to the GDP
           https://github.com/lab11/signpost-software/blob/master/docs/DataSchemas.md
           TODO Process data and only save relevant portions.'''
           
        print('Signpost._process_audio_freq: Processing message')
        if (not(msg['device'] == 'signpost_audio_frequency')):
            return
            
        if ('ts' in msg):
            self.audio['ts'] = msg['ts']['tv_sec']
        else:
            self.audio['ts'] = None
        
        if ('recno' in msg):
            self.audio['recno'] = msg['recno']
        else:
            self.audio['recno'] = None
        
        for key, val in msg.iteritems(): 
            self.audio[key] = val
            
        if (DEBUG_PRINT):
            self.pp.pprint(self.audio)
        print('Signpost._process_audio_freq:   Finished!')
        
        
    def _process_gps(self, msg):
        '''Process data from the GPS Module sent to the GDP
           https://github.com/lab11/signpost-software/blob/master/docs/DataSchemas.md
           TODO Process data and only save relevant portions.'''
           
        print('Signpost._process_gps: Processing message')
        if (not(msg['device'] == 'signpost_gps')):
            return
        
        if ('ts' in msg):
            self.gps['ts'] = msg['ts']['tv_sec']
        else:
            self.gps['ts'] = None
        
        if ('recno' in msg):
            self.gps['recno'] = msg['recno']
        else:
            self.gps['recno'] = None
    
        gps_lat = None
        gps_lon = None
        
        if (msg['latitude_direction']['value'] == 'N'):
            gps_lat = msg['latitude']['value']
        elif (msg['latitude_direction']['value'] == 'S'):
            gps_lat = -msg['latitude']['value']
            
        if (msg['longitude_direction']['value'] == 'E'):
            gps_lon = msg['longitude']['value']
        elif (msg['longitude_direction']['value'] == 'W'):
            gps_lon = -msg['longitude']['value']
        
        if (gps_lat and gps_lon):
            self.loc = [gps_lat, gps_lon]
            print('Signpost._process_gps: Loc is %s!' % (self.loc)) 
            
        for key, val in msg.iteritems(): 
            self.gps[key] = val
            
        if (DEBUG_PRINT):
            self.pp.pprint(self.gps)
        print('Signpost._process_gps:   Finished!')
        
        
    def _process_energy(self, msg):
        '''Process data from the Energy Module sent to the GDP
           https://github.com/lab11/signpost-software/blob/master/docs/DataSchemas.md
           TODO Process data and only save relevant portions.''' 
           
        print('Signpost._process_energy: Processing message')  
        if (not(msg['device'] == 'signpost_energy')):
            return
            
        if ('ts' in msg):
            self.energy['ts'] = msg['ts']['tv_sec']
        else:
            self.energy['ts'] = None
        
        if ('recno' in msg):
            self.energy['recno'] = msg['recno']
        else:
            self.energy['recno'] = None
        
        for key, val in msg.iteritems(): 
            self.energy[key] = val
        
        if (DEBUG_PRINT):
            self.pp.pprint(self.energy)
        print('Signpost._process_energy:   Finished!')
        
        
    def _process_radio(self, msg):
        '''Process data from the Radio Module sent to the GDP
           https://github.com/lab11/signpost-software/blob/master/docs/DataSchemas.md
           TODO Process data and only save relevant portions.'''
           
        print('Signpost._process_radio: Processing message')  
        if (not(msg['device'] == 'signpost_radio')):
            return
        
        if ('ts' in msg):
            self.radio['ts'] = msg['ts']['tv_sec']
        else:
            self.radio['ts'] = None
        
        if ('recno' in msg):
            self.radio['recno'] = msg['recno']
        else:
            self.radio['recno'] = None
        
        for key, val in msg.iteritems(): 
            self.radio[key] = val
        
        if (DEBUG_PRINT):
            self.pp.pprint(self.radio)
        print('Signpost._process_radio:   Finished!')
        
        
    def _process_microwave_radar(self, msg):
        '''Process data from the Microwave Radar Module sent to the GDP
           https://github.com/lab11/signpost-software/blob/master/docs/DataSchemas.md
           TODO Process data and only save relevant portions.'''
           
        print('Signpost._process_microwave_radar: Processing message')   
        if (not(msg['device'] == 'signpost_microwave_radar')):
            return
        
        if ('ts' in msg):
            self.microwave_radar['ts'] = msg['ts']['tv_sec']
        else:
            self.microwave_radar['ts'] = None
        
        if ('recno' in msg):
            self.microwave_radar['recno'] = msg['recno']
        else:
            self.microwave_radar['recno'] = None
        
        for key, val in msg.iteritems(): 
            self.microwave_radar[key] = val

        if (DEBUG_PRINT):
            self.pp.pprint(self.microwave_radar)
        print('Signpost._process_microwave_radar:   Finished!')
        
        
    def _process_ambient(self, msg):
        '''Process data from the Ambient Sensing Module sent to the GDP
           https://github.com/lab11/signpost-software/blob/master/docs/DataSchemas.md
           TODO Process data and only save relevant portions.'''
         
        print('Signpost._process_ambient: Processing message')   
        if (not(msg['device'] == 'signpost_ambient')):
            return
        
        if ('ts' in msg):
            self.ambient['ts'] = msg['ts']['tv_sec']
        else:
            self.ambient['ts'] = None
        
        if ('recno' in msg):
            self.ambient['recno'] = msg['recno']
        else:
            self.ambient['recno'] = None
        
        for key, val in msg.iteritems(): 
            self.ambient[key] = val
                
        if (DEBUG_PRINT):
            self.pp.pprint(self.ambient)
        print('Signpost._process_ambient:   Finished!')
        
        
    def _process_ucsd_air_quality(self, msg):
        '''Process data from the UCSD Air Quality module sent to the GDP
           https://github.com/lab11/signpost-software/blob/master/docs/DataSchemas.md
           TODO Process data and only save relevant portions.'''
           
        print('Signpost._process_ucsd_air_quality: Processing message')   
        if (not(msg['device'] == 'signpost_ucsd_air_quality')):
            return
        
        if ('ts' in msg):
            self.air_quality['ts'] = msg['ts']['tv_sec']
        else:
            self.air_quality['ts'] = None
        
        if ('recno' in msg):
            self.air_quality['recno'] = msg['recno']
        else:
            self.air_quality['recno'] = None
        
        for key, val in msg.iteritems(): 
            self.air_quality[key] = val

        if (DEBUG_PRINT):
            self.pp.pprint(self.air_quality)
        print('Signpost._process_ucsd_air_quality:   Finished!')
        

    def _process_anomalies(self, msg):
        '''Process data from the anomalies stream provided by Duc at UIUC'''
           
        print('Signpost._process_anomalies: Processing message')   
        if (not(msg['device'] == 'anomalies')):
            return
        
        if ('ts' in msg):
            self.anomalies['ts'] = msg['ts']['tv_sec']
        else:
            self.anomalies['ts'] = None
        
        if ('recno' in msg):
            self.anomalies['recno'] = msg['recno']
        else:
            self.anomalies['recno'] = None
        
        for key, val in msg.iteritems(): 
            self.anomalies[key] = val

        if ('anomaly' in msg):
            if (msg['anomaly']['value'] == 1):
                self.f_anomaly = 1
                print('Signpost._process_anomalies: f_anomaly is %d!' % (self.f_anomaly)) 

        if (DEBUG_PRINT):
            self.pp.pprint(self.anomalies)
        print('Signpost._process_anomalies:   Finished!')


class GDPDataProcessor(threading.Thread):
    def __init__(self, list_addr_dict, 
                 addr_gdprouter='gdp-01.eecs.berkeley.edu:8007',
                 b_print=False):
        '''Manages subscription to select Signpost logs within the GDP.
        
           list_addr_dict: list of dictionaries of the following form
                   {'id_signpost':, 'id_sensor':, 'addr':}'''
                   
        print('GDPDataProcessor: Initializing GDP Data Processor')
        super(GDPDataProcessor, self).__init__()
        gdp.gdp_init(addr_gdprouter)
        self.list_addr_dict = list_addr_dict
        self.initialize_gcl_handles()
        self.initialize_signposts()
        self._stop_event = threading.Event()
        self._stop_event.clear() # Unnecessary
        self.b_print = b_print
        self.start()
        
    def run(self):
        # Subscribe to all GDP threads
        for addr_gdp in self.list_addr_dict:
            if (addr_gdp['gcl_handle'] != None):
                addr_gdp['gcl_handle'].subscribe(0, 0, None)
                print('GDPDataProcessor: Subscribed to %s' % (addr_gdp['addr']))
            else:
                print('GDPDataProcessor: Skipped. No gcl_handle for %s' % (addr_gdp['addr']))
        
        # Run loop to process GDP events as they are received
        ind_rec = -3
        ind_handle = 0
        while not(self.stopped()):
            ## DEBUG
            #if (self.list_addr_dict[ind_handle]['gcl_handle'] != None):
            #    temp_data = self.list_addr_dict[ind_handle]['gcl_handle'].read(ind_rec)
            #    event = {'type':1, 'datum':temp_data, 'gcl_handle':self.list_addr_dict[ind_handle]['gcl_handle']}
            ## END DEBUG (remove indent below)
            event = gdp.GDP_GCL.get_next_event(None)
            if (self.b_print):
                print('GDPDataProcessor: Received %s' % (event))
            if (event['type'] == gdp.GDP_EVENT_DATA):
                gcl_name = event['gcl_handle'].getname().printable_name()
                for addr_gdp in self.list_addr_dict:
                    if ((addr_gdp['gcl_name'] != None) and (gcl_name == addr_gdp['gcl_name'].printable_name())):
                        addr_gdp['signpost'].update(msg_in=event['datum'], device=addr_gdp['id_sensor'])
                        break
            time.sleep(0.01)
            
            ## DEBUG
            #ind_handle = (ind_handle + 1) % len(list_addr_dict)
            #if (ind_handle == 0):
            #    ind_rec += 1
            #if (ind_rec > -1):
            #    break
            #time.sleep(0.5)
            ## END DEBUG
            
        # Clean up

    def stop(self):
        print('GDPDataProcessor: Stopping!')
        self._stop_event.set()

    def stopped(self):
        return self._stop_event.is_set()    
    
    def initialize_gcl_handles(self):
        print('GDPDataProcessor: Initializing GCL handles')
        for addr_gdp in self.list_addr_dict:
            print('GDPDataProcessor.initialize_gcl_handles: Initializing %s' % (addr_gdp['addr']))
            try:
                gcl_name = gdp.GDP_NAME(addr_gdp['addr'])
                addr_gdp['gcl_name'] = gcl_name
                addr_gdp['gcl_handle'] = gdp.GDP_GCL(gcl_name, gdp.GDP_MODE_RO)
                print('GDPDataProcessor.initialize_gcl_handles:   Success!')
            except:
                addr_gdp['gcl_name'] = None
                addr_gdp['gcl_handle'] = None
                print('GDPDataProcessor.initialize_gcl_handles:   FAIL')
        print('GDPDataProcessor:   Finished!')
            
    def initialize_signposts(self):
        print('GDPDataProcessor: Initializing Signpost objects')
        # Create all Signpost objects        
        list_id = []
        list_signpost_obj = []
        for id_signpost in set([addr_gdp['id_signpost'] for addr_gdp in self.list_addr_dict]):
            list_id.append(id_signpost)
            list_signpost_obj.append(Signpost(id_signpost))
        
        # Assign Signpost object to correct entry in dict
        for addr_gdp in self.list_addr_dict:
            ind_signpost_obj = list_id.index(addr_gdp['id_signpost'])
            addr_gdp['signpost'] = list_signpost_obj[ind_signpost_obj]
        print('GDPDataProcessor:   Finished!')
    
    def get_gps(self, id_signpost):
        ind_signpost = -1
        for ind, signpost in enumerate(self.list_addr_dict):
            if (signpost['id_signpost'] == id_signpost):
                ind_signpost = ind
                break
            
        loc = self.list_addr_dict[ind_signpost]['signpost'].loc
        return loc
        
    def check_trigger(self):
        list_signpost_anomalies = []
        for signpost in self.list_addr_dict:
            if ((signpost['signpost'].f_anomaly == 1) and (signpost['signpost'].loc != None)):
                #print(signpost)
                if not(signpost['id_signpost'] in [signpost_anomalies['id_signpost'] for signpost_anomalies in list_signpost_anomalies]):
                    list_signpost_anomalies.append(
                            {'id_signpost': signpost['id_signpost'], 
                             'gps': self.get_gps(signpost['id_signpost'])
                            }
                    )
                
        if (len(list_signpost_anomalies) > 0):
            return (True, list_signpost_anomalies)
        else:
            return (False, list_signpost_anomalies)
            

if __name__ == '__main__':
    pp = pprint.PrettyPrinter(indent=4)

    list_addr_base = [#'edu.berkeley.eecs.c098e5120003']#,
                      #'edu.berkeley.eecs.c098e5120011',
                      'edu.berkeley.eecs.c098e512000a']
                      #'edu.berkeley.eecs.c098e5120010']                  
    list_addr_sensors = ['signpost_gps.v0-0-1',
                         'signpost_energy.v0-0-1',
                         'signpost_radio.v0-0-1',
                         'signpost_audio_frequency.v0-0-1',
                         'anomalies.v0-0-1']                       
                         #'signpost_microwave_radar.v0-0-1',
                         #'signpost_ambient.v0-0-1',
                         #'signpost_ucsd_air_quality.v0-0-1']
    list_id_sensors = ['signpost_gps',
                       'signpost_energy',
                       'signpost_radio',
                       'signpost_audio_frequency',
                       'anomalies']
                       #'signpost_microwave_radar',
                       #'signpost_ambient',
                       #'signpost_ucsd_air_quality']
    list_addr_dict = []
    
    for addr_base in list_addr_base:
        for ind, addr_sensor in enumerate(list_addr_sensors):
            temp = {'id_signpost':addr_base, 
                    'id_sensor':list_id_sensors[ind], 
                    'addr':(addr_base + '.' + addr_sensor)}
            list_addr_dict.append(temp)
    
    gdp_processor = GDPDataProcessor(list_addr_dict)
    time.sleep(120)
    gdp_processor.stop()