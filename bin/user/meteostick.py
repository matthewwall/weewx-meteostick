#!/usr/bin/env python
# Meteostick driver for weewx
#
# Copyright 2016 Matthew Wall, Luc Heijst
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.
#
# See http://www.gnu.org/licenses/
#
# Thanks to Frank Bandle for testing during the development of this driver.

"""Meteostick is a USB device that receives radio transmissions from Davis
weather stations.

The meteostick has a preset radio frequency (RF) treshold value which is twice
the RF sensity value in dB.  Valid values for RF sensity range from 0 to 125.
Both positive and negative parameter values will be treated as the
same actual (negative) dB values.

The default RF sensitivity value is 90 (-90 dB).  Values between 95 and 125
tend to give too much noise and false readings (the higher value the more
noise).  Values lower than 50 likely result in no readings at all.
"""

from __future__ import with_statement
import math
import serial
import syslog
import time

import weewx
import weewx.drivers
import weewx.wxformulas
from weewx.crc16 import crc16

DRIVER_NAME = 'Meteostick'
DRIVER_VERSION = '0.31'

DEBUG_SERIAL = 0
DEBUG_RAIN = 0
DEBUG_PARSE = 0
DEBUG_RFS = 0


def loader(config_dict, _):
    return MeteostickDriver(**config_dict[DRIVER_NAME])

def confeditor_loader():
    return MeteostickConfEditor()

def configurator_loader(config_dict):
    return MeteostickConfigurator()


def logmsg(level, msg):
    syslog.syslog(level, 'meteostick: %s' % msg)

def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)


METEOSTICK_CHANNEL = 9 # fake channel for recording the receiver stats

class MeteostickDriver(weewx.drivers.AbstractDevice):
    NUM_CHAN = 10 # 8 channels, one fake channel (9), one unused channel (0)
    DEFAULT_PORT = '/dev/ttyUSB0'
    DEFAULT_BAUDRATE = 115200
    DEFAULT_FREQUENCY = 'EU'
    DEFAULT_RAIN_BUCKET_TYPE = 1
    DEFAULT_RF_SENSITIVITY = 90
    MAX_RF_SENSITIVITY = 125
    DEFAULT_SENSOR_MAP = {
        'pressure': 'pressure',
        'in_temp': 'inTemp',
        'wind_speed': 'windSpeed',
        'wind_dir': 'windDir',
        'temperature': 'outTemp',
        'humidity': 'outHumidity',
        'rain_count': 'rain',
        'solar_radiation': 'radiation',
        'uv': 'UV',
        'pct_good': 'rxCheckPercent',
        'solar_power': 'extraTemp3',
        'soil_temp_1': 'soilTemp1',
        'soil_temp_2': 'soilTemp2',
        'soil_temp_3': 'soilTemp3',
        'soil_temp_4': 'soilTemp4',
        'soil_moisture_1': 'soilMoist1',
        'soil_moisture_2': 'soilMoist2',
        'soil_moisture_3': 'soilMoist3',
        'soil_moisture_4': 'soilMoist4',
        'leaf_wetness_1': 'leafWet1',
        'leaf_wetness_2': 'leafWet2',
        'temp_1': 'extraTemp1',
        'temp_2': 'extraTemp2',
        'humid_1': 'extraHumid1',
        'humid_2': 'extraHumid2',
        'bat_iss': 'txBatteryStatus',
        'bat_anemometer': 'windBatteryStatus',
        'bat_leaf_soil': 'rainBatteryStatus',
        'bat_th_1': 'outTempBatteryStatus',
        'bat_th_2': 'inTempBatteryStatus'}

    def __init__(self, **stn_dict):
        loginf('driver version is %s' % DRIVER_VERSION)
        port = stn_dict.get('port', self.DEFAULT_PORT)
        baud = stn_dict.get('baudrate', self.DEFAULT_BAUDRATE)
        freq = stn_dict.get('transceiver_frequency', self.DEFAULT_FREQUENCY)
        rfs = int(stn_dict.get('rf_sensitivity', self.DEFAULT_RF_SENSITIVITY))
        self.rfs = abs(rfs)
        if self.rfs > self.MAX_RF_SENSITIVITY:
            self.rfs = self.DEFAULT_RF_SENSITIVITY
            loginf("invalid RF sensitivity %s, using %s" % (rfs, self.rfs))
        fmt = stn_dict.get('format', 'machine') # human, machine, raw
        if fmt.lower() not in ['machine', 'raw']:
            raise ValueError("unsupported format '%s'" % fmt)
        self.iss_channel = int(stn_dict.get('iss_channel', 1))
        self.anemometer_channel = int(stn_dict.get('anemometer_channel', 0))
        self.leaf_soil_channel = int(stn_dict.get('leaf_soil_channel', 0))
        self.temp_hum_1_channel = int(stn_dict.get('temp_hum_1_channel', 0))
        self.temp_hum_2_channel = int(stn_dict.get('temp_hum_2_channel', 0))
        xmit = Meteostick.ch_to_xmit(
            self.iss_channel, self.anemometer_channel, self.leaf_soil_channel,
            self.temp_hum_1_channel, self.temp_hum_2_channel)
        rain_bucket_type = int(stn_dict.get('rain_bucket_type',
                                            self.DEFAULT_RAIN_BUCKET_TYPE))
        self.rain_per_tip = 0.254 if rain_bucket_type == 0 else 0.2 # mm
        self.sensor_map = stn_dict.get('sensor_map', self.DEFAULT_SENSOR_MAP)
        self.max_tries = int(stn_dict.get('max_tries', 10))
        self.retry_wait = int(stn_dict.get('retry_wait', 10))
        self.last_rain_count = None

        global DEBUG_PARSE
        DEBUG_PARSE = int(stn_dict.get('debug_parse', DEBUG_PARSE))
        global DEBUG_SERIAL
        DEBUG_SERIAL = int(stn_dict.get('debug_serial', DEBUG_SERIAL))
        global DEBUG_RAIN
        DEBUG_RAIN = int(stn_dict.get('debug_rain', DEBUG_RAIN))
        global DEBUG_RFS
        DEBUG_RFS = int(stn_dict.get('debug_rf_sensitivity', DEBUG_RFS))
        if DEBUG_RFS:
            self._init_rf_stats()

        loginf('using serial port %s' % port)
        loginf('using baudrate %s' % baud)
        loginf('using frequency %s' % freq)
        loginf('using rf sensitivity %s (-%s dB)' % (rfs, self.rfs))
        loginf('using iss_channel %s' % self.iss_channel)
        loginf('using anemometer_channel %s' % self.anemometer_channel)
        loginf('using leaf_soil_channel %s' % self.leaf_soil_channel)
        loginf('using temp_hum_1_channel %s' % self.temp_hum_1_channel)
        loginf('using temp_hum_2_channel %s' % self.temp_hum_2_channel)
        loginf('using rain_bucket_type %s' % rain_bucket_type)
        loginf('using transmitters %02x' % xmit)
        loginf('sensor map is: %s' % self.sensor_map)

        self.station = Meteostick(port, baud, xmit, freq, self.rfs, fmt)
        self.station.open()
        self.station.configure()

    def closePort(self):
        if self.station is not None:
            self.station.close()
            self.station = None

    @property
    def hardware_name(self):
        return 'Meteostick'

    def genLoopPackets(self):
        while True:
            readings = self.station.get_readings_with_retry(self.max_tries,
                                                            self.retry_wait)
            data = self.station.parse_readings(
                readings, self.iss_channel,
                self.anemometer_channel, self.leaf_soil_channel,
                self.temp_hum_1_channel, self.temp_hum_2_channel)
            if data:
                if DEBUG_PARSE:
                    logdbg("data: %s" % data)
                packet = self._data_to_packet(data)
                if DEBUG_PARSE:
                    logdbg("packet: %s" % packet)
                if DEBUG_RFS:
                    self._update_rf_stats(data['channel'], data['rf_signal'])
                    now = int(time.time())
                    # report at 5 minute intervals
                    if now - self.rf_stats['ts'] > 60 and now % 300 < 30:
                        self._report_rf_stats()
                        self._init_rf_stats()
                yield packet

    def _data_to_packet(self, data):
        packet = {'dateTime': int(time.time() + 0.5),
                  'usUnits': weewx.METRICWX}
        # map sensor observations to database field names
        for k in data:
            if k in self.sensor_map:
                packet[self.sensor_map[k]] = data[k]
        # convert the rain count to a rain delta measure
        if 'rain' in packet:
            if self.last_rain_count is not None:
                rain_count = packet['rain'] - self.last_rain_count
            else:
                rain_count = 0
            # handle rain counter wrap around from 127 to 0
            if rain_count < 0:
                if DEBUG_RAIN:
                    logdbg("rain counter wraparound detected rain_count=%s" %
                           rain_count)
                rain_count += 128
            self.last_rain_count = packet['rain']
            packet['rain'] = float(rain_count) * self.rain_per_tip # mm
            if DEBUG_RAIN:
                logdbg("rain=%s rain_count=%s last_rain_count=%s" %
                       (packet['rain'], rain_count, self.last_rain_count))
        return packet

    def _init_rf_stats(self):
        self.rf_stats = {
            'min': [0] * self.NUM_CHAN, # rf sensitivity has negative values
            'max': [-125] * self.NUM_CHAN,
            'sum': [0] * self.NUM_CHAN,
            'cnt': [0] * self.NUM_CHAN,
            'last': [0] * self.NUM_CHAN,
            'avg': [0] * self.NUM_CHAN,
            'ts': int(time.time())}
        # unlike the rf sensitivity measures, pct_good is positive
        self.rf_stats['min'][METEOSTICK_CHANNEL] = 100

    def _update_rf_stats(self, ch, signal):
        self.rf_stats['min'][ch] = min(signal, self.rf_stats['min'][ch])
        self.rf_stats['max'][ch] = max(signal, self.rf_stats['max'][ch])
        self.rf_stats['sum'][ch] += signal
        self.rf_stats['cnt'][ch] += 1
        self.rf_stats['last'][ch] = signal
        for ch in range(1, self.NUM_CHAN): # skip unused channel 0
            if self.rf_stats['cnt'][ch] > 0:
                self.rf_stats['avg'][ch] = int(self.rf_stats['sum'][ch] / self.rf_stats['cnt'][ch])

    def _report_rf_stats(self):
        logdbg("RF summary: rf_sensitivity=%s (values in dB)" % self.rfs)
        logdbg("Station      max   min   avg  last  count")
        for x in [('iss', self.iss_channel),
                  ('wind', self.anemometer_channel),
                  ('leaf_soil', self.leaf_soil_channel),
                  ('temp_hum_1', self.temp_hum_1_channel),
                  ('temp_hum_2', self.temp_hum_2_channel),
                  ('pct_good', METEOSTICK_CHANNEL)]:
            self._report_channel(x[0], x[1])

    def _report_channel(self, label, ch):
        logdbg("%s %5d %5d %5d %5d %5d" % (label.ljust(12),
                                           self.rf_stats['max'][ch],
                                           self.rf_stats['min'][ch],
                                           self.rf_stats['avg'][ch],
                                           self.rf_stats['last'][ch],
                                           self.rf_stats['cnt'][ch]))


class Meteostick(object):
    def __init__(self, port, baudrate, transmitters, frequency, rfs,
                 output_format='machine'):
        self.output_format = output_format
        self.port = port
        self.baudrate = baudrate
        self.timeout = 3 # seconds
        self.transmitters = transmitters
        self.frequency = frequency
        self.rf_threshold = rfs * 2
        self.serial_port = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _, value, traceback):
        self.close()

    def open(self):
        if DEBUG_SERIAL:
            logdbg("open serial port %s" % self.port)
        self.serial_port = serial.Serial(self.port, self.baudrate,
                                         timeout=self.timeout)

    def close(self):
        if self.serial_port is not None:
            if DEBUG_SERIAL:
                logdbg("close serial port %s" % self.port)
            self.serial_port.close()
            self.serial_port = None

    def get_readings(self):
        buf = self.serial_port.readline()
        if DEBUG_SERIAL > 2 and len(buf) > 0:
            logdbg("station said: %s" %
                   ' '.join(["%0.2X" % ord(c) for c in buf]))
        return buf.strip()

    def get_readings_with_retry(self, max_tries=5, retry_wait=10):
        for ntries in range(0, max_tries):
            try:
                return self.get_readings()
            except serial.serialutil.SerialException, e:
                loginf("Failed attempt %d of %d to get readings: %s" %
                       (ntries + 1, max_tries, e))
                time.sleep(retry_wait)
        else:
            msg = "Max retries (%d) exceeded for readings" % max_tries
            logerr(msg)
            raise weewx.RetriesExceeded(msg)

    def parse_readings(self, raw, iss_ch=0, ls_ch=0, wind_ch=0, th1_ch=0, th2_ch=0):
        if not raw:
            return dict()
        if self.output_format == 'raw':
            return self.parse_raw(raw, iss_ch, wind_ch, ls_ch, th1_ch, th2_ch)
        return self.parse_machine(raw, iss_ch, wind_ch, ls_ch, th1_ch, th2_ch)

    @staticmethod
    def parse_machine(raw, iss_channel, wind_channel,
                      ls_channel, th1_channel, th2_channel):
        data = dict()
        try:
            parts = Meteostick.get_parts(raw)
            n = len(parts)
            if parts[0] == 'B':
                if n >= 3:
                    data['channel'] = 0
                    data['rf_signal'] = 0
                    data['in_temp'] = float(parts[1]) # C
                    data['pressure'] = float(parts[2]) # hPa
                    if n >= 4:
                        data['channel'] = METEOSTICK_CHANNEL
                        data['pct_good'] = float(parts[3].strip('%'))
                        data['rf_signal'] = data['pct_good']
                else:
                    logerr("B: not enough parts (%s) in '%s'" % (n, raw))
            elif parts[0] in 'WT':
                if n >= 5:
                    data['channel'] = int(parts[1])
                    data['rf_signal'] = float(parts[4])
                    bat = 1 if n >= 6 and parts[5] == 'L' else 0
                    if parts[0] == 'W':
                        if iss_channel != 0 and data['channel'] == iss_channel:
                            data['bat_iss'] = bat
                        else:
                            data['bat_anemometer'] = bat
                        data['wind_speed'] = float(parts[2]) # m/s
                        data['wind_dir'] = float(parts[3]) # degrees
                    elif parts[0] == 'T':
                        if th1_channel != 0 and data['channel'] == th1_channel:
                            data['bat_th_1'] = bat
                            data['temp_1'] = float(parts[2]) # C
                            data['humid_1'] = float(parts[3]) # %
                        elif th2_channel != 0 and data['channel'] == th2_channel:
                            data['bat_th_2'] = bat
                            data['temp_2'] = float(parts[2]) # C
                            data['humid_2'] = float(parts[3]) # %
                        else:
                            data['bat_iss'] = bat
                            data['temperature'] = float(parts[2]) # C
                            data['humidity'] = float(parts[3]) # %
                else:
                    logerr("WT: not enough parts (%s) in '%s'" % (n, raw))
            elif parts[0] in 'LMO':
                if n >= 5:
                    data['channel'] = int(parts[1])
                    data['rf_signal'] = float(parts[4])
                    data['bat_leaf_soil'] = 1 if n >= 6 and parts[5] == 'L' else 0
                    if parts[0] == 'L':
                        data['leaf_wetness_%s' % parts[2]] = float(parts[3]) # 0-15
                    elif parts[0] == 'M':
                        data['soil_moisture_%s' % parts[2]] = float(parts[3]) # cbar 0-200
                    elif parts[0] == 'O':
                        data['soil_temp_%s' % parts[2]] = float(parts[3])  # C
                else:
                    logerr("LMO: not enough parts (%s) in '%s'" % (n, raw))
            elif parts[0] in 'RSUP':
                if n >= 4:
                    data['channel'] = int(parts[1])
                    data['rf_signal'] = float(parts[3])
                    data['bat_iss'] = 1 if n >= 5 and parts[4] == 'L' else 0
                    if parts[0] == 'R':
                        rain_count = int(parts[2])
                        if 0 <= rain_count < 128:
                            data['rain_count'] = rain_count  # 0-127
                        else:
                            logerr("ignoring invalid rain %s on channel %s" %
                                   (rain_count, data['channel']))
                    elif parts[0] == 'S':
                        data['solar_radiation'] = float(parts[2])  # W/m^2
                    elif parts[0] == 'U':
                        data['uv'] = float(parts[2])
                    elif parts[0] == 'P':
                        data['solar_power'] = float(parts[2])  # 0-100
                else:
                    logerr("RSUP: not enough parts (%s) in '%s'" % (n, raw))
            elif parts[0] in '#':
                loginf("%s" % raw)
            else:
                logerr("unknown sensor identifier '%s' in '%s'" %
                       (parts[0], raw))
        except ValueError, e:
            logerr("parse failed for '%s': %s" % (raw, e))
            data = dict() # do not return partial data
        return data

    @staticmethod
    def parse_raw(raw, iss_channel,
                  wind_channel, ls_channel, th1_channel, th2_channel):
        data = dict()
        try:
            parts = Meteostick.get_parts(raw)
            n = len(parts)
            if parts[0] == 'B':
                if n >= 6:
                    data['channel'] = METEOSTICK_CHANNEL
                    data['pct_good'] = int(parts[5]) # number of good pkts
                    data['rf_signal'] = data['pct_good']
                    data['in_temp'] = float(parts[3]) / 10.0 # C
                    data['pressure'] = float(parts[4]) / 100.0 # hPa
                else:
                    logerr("B: not enough parts (%s) in '%s'" % (n, raw))
            elif parts[0] == 'I':
                raw_msg = [0] * 8
                for i in xrange(0, 8):
                    raw_msg[i] = chr(int(parts[i + 2], 16))
                Meteostick.check_crc(raw_msg)
                for i in xrange(0, 8):
                    raw_msg[i] = parts[i + 2]
                pkt = bytearray([int(i, base=16) for i in raw_msg])
                data['channel'] = (pkt[0] & 0xF) + 1
                data['rf_signal'] = int(parts[11])
                if data['channel'] == iss_channel:
                    # iss sensors
                    message_type = (pkt[0] >> 4 & 0xF)
                    if message_type == 2:
                        # supercap voltage (Vue only)
                        supercap_volt_raw = (pkt[3] * 4 + (pkt[4] & 0xC0) / 64)
                        logdbg("supercap_volt_raw: %04x" % supercap_volt_raw)
                        data['supercap_volt'] = supercap_volt_raw / 100.0
                    elif message_type == 4:
                        # uv
                        uv_raw = pkt[3] * 256 + pkt[4]
                        logdbg("uv_raw: %04x" % uv_raw)
                        if uv_raw != 0xffc5:
                            data['uv'] = uv_raw / 50.0
                    elif message_type == 5:
                        # rain rate
                        if pkt[4] & 0x40 == 0:
                            data['rain_rate'] = 720 / (((pkt[4] & 0x30) / 16 * 250) + pkt[3])
                            logdbg("light_rain: %s" % data['rain_rate'])
                        else:
                            data['rain_rate'] = 11520 / (((pkt[4] & 0x30) / 16 * 250) + pkt[3])
                            logdbg("heavy_rain: %s" % data['rain_rate'])
                    elif message_type == 6:
                        # solar radiation
                        sr_raw = pkt[3] * 256 + pkt[4]
                        logdbg("solar_radiation_raw: %04x" % sr_raw)
                        if solar_radiation_raw != 0xffc5:
                            data['solar_radiation'] = sr_raw * 1.757936
                    elif message_type == 7:
                        # solar cell output (Vue only)
                        sco_raw = 0 # FIXME
                        logdbg("solar_cell_output_raw: %s" % sco_raw)
                        data['solar_cell_output'] = sco_raw # FIXME
                    elif message_type == 8:
                        # temperature
                        temp_f_raw = pkt[3] * 256 + pkt[4]
                        logdbg("temp_f_raw: %04x" % temp_f_raw)
                        temp_f = temp_f_raw / 160.0
                        data['temperature'] = weewx.wxformulas.FtoC(temp_f) # C
                    elif message_type == 9:
                        # wind gust
                        wg_raw = 0 # FIXME
                        logdgb("wind gust: %s" % wg_raw)
                        data['wind_gust'] = wg_raw # FIXME
                    elif message_type == 0xA:
                        # humidity
                        humidity_raw = ((pkt[4] >> 4) << 8) + pkt[3]
                        logdbg("humidity_raw: %04x" % humidity_raw)
                        data['humidity'] = humidity_raw / 10.0
                    elif message_type == 0xE:
                        # rain
                        rain_count_raw = pkt[3] & 0x7F
                        logdbg("rain_count_raw: %04x" % rain_count_raw)
                        data['rain_count'] = rain_count_raw
                    else:
                        # unknown message type
                        logerr("unknown message type %01x" % message_type)
                elif data['channel'] == wind_channel:
                    # wind sensors
                    wind_speed_raw = pkt[1]
                    logdbg("wind_speed_raw: %04x" % wind_speed_raw)
                    data['wind_speed'] = wind_speed_raw * 0.44704 # mph to m/s
                    wind_dir_raw = pkt[2]
                    logdbg("wind_dir_raw: %04x" % wind_dir_raw)
                    data['wind_dir'] = 9 + wind_dir_raw * 342.0 / 255.0
                elif data['channel'] == ls_channel:
                    # leaf and soil sensors
                    data_type = pkt[0] >> 4
                    if data_type == 0xf:
                        data_subtype = pkt[1] & 0x3
                        if data_subtype == 1:
                            # soil temperature
                            data = Meteostick.decode_soil(pkt)
                        else:
                            sensor_num = ((pkt[1] & 0xe0) >> 5) + 1
                            l1_raw = (pkt[3] << 2) + (pkt[5] >> 6)
                            l2_raw = (pkt[2] << 2) + (pkt[4] >> 6)
                            logdbg("SOIL_LEAF_STATION: UNKNOWN data_subtype = %s" % data_subtype)
                            logdbg("sensor_num: %s, l1_raw: %04x, l2_raw: %04x" % (sensor_num, l1_raw, l2_raw))
            elif parts[0] in '#':
                loginf("%s" % raw)
            else:
                logerr("unknown sensor identifier '%s' in '%s'" %
                       (parts[0], raw))
        except ValueError, e:
            logerr("parse failed for '%s': %s" % (raw, e))
            data = dict() # do not return partial data
        return data

    @staticmethod
    def get_parts(raw):
        if DEBUG_PARSE:
            logdbg("readings: %s" % raw)
        parts = raw.split(' ')
        if DEBUG_PARSE > 2:
            logdbg("parts: %s (%s)" % (parts, len(parts)))
        if len(parts) < 2:
            raise ValueError("not enough parts in '%s'" % raw)
        return parts

    def reset(self):
        """Reset the device, leaving it in a state that we can talk to it."""

        # flush any previous data in the input buffer
        self.serial_port.flushInput()

        # Send a reset command
        self.serial_port.write('r\n')
        # Wait until we see the ? character
        ready = False
        response = ''
        while not ready:
            time.sleep(0.1)
            while self.serial_port.inWaiting() > 0:
                c = self.serial_port.read(1)
                if c == '?':
                    ready = True
                else:
                    response += c
        loginf("reset: %s" % response.split('\n')[0])
        if DEBUG_SERIAL > 2:
            logdbg("full response to reset: %s" % response)
        # Discard any serial input from the device
        time.sleep(0.2)
        self.serial_port.flushInput()
        return response

    @staticmethod
    def decode_soil(pkt):
        # decode the soil temperature then calculate the actual soil
        # temperature and the soil potential, using Davis' formulas.
        # when the sensor is not populated, soil_temp_raw and
        # soil_potential_raw are set to their max values (0x3ff).
        sensor_num = ((pkt[1] & 0xe0) >> 5) + 1
        soil_temp_raw = (pkt[3] << 2) + (pkt[5] >> 6)
        soil_potential_raw = (pkt[2] << 2) + (pkt[4] >> 6)
        soil_temp = None
        data = dict()
        if soil_temp_raw != 0x3ff:
            soil_temp = Meteostick._calculate_soil_temp(soil_temp_raw)
            data['soil_temp_%s' % sensor_num] = soil_temp
        if soil_potential_raw != 0x3ff:
            soil_potential = Meteostick._calculate_soil_potential(
                soil_potential_raw, soil_temp)
            data['soil_moisture_%s' % sensor_num] = soil_potential
        return data

    @staticmethod
    def _calculate_soil_temp(soil_temp_raw):
        # soil_temp_raw is in degrees C
        # R is in kohms
        # https://github.com/dekay/DavisRFM69/wiki/Leaf-Soil-Moisture-Temperature-Station
        logdbg('calculate soil temp from raw soil temp %s' % soil_temp_raw)
        A = 18.81099
        B = 0.0009988027
        R = A / (1.0 / soil_temp_raw - B) / 1000
        logdbg('thermistor resistance: %s kohm' % R)
        # Steinhart-Hart parameters.
        S1 = 0.002783573
        S2 = 0.0002509406
        try:
            return 1 / (S1 + S2 * math.log(R)) - 273
        except ValueError, e:
            logerr('soil temp failed for raw temp %s: %s' % (soil_temp_raw, e))
        return None

    DEFAULT_SOIL_TEMP = 24 # C

    @staticmethod
    def _calculate_soil_potential(soil_potential_raw, soil_temp):
        # Equations relating resistance to soil potential are from
        # http://www.kimberly.uidaho.edu/water/swm/Calibration_Watermark2.htm
        # The following units are used to make it easier to verify that the
        # equations here match those from the above link
        #
        # soil_temp is in degrees C
        # R is in kohms
        # potential is in kPa (equivalently to centibar)
        #
        # https://github.com/dekay/DavisRFM69/wiki/Leaf-Soil-Moisture-Temperature-Station
        logdbg('calculate soil potential from soil potential %s'
               ' and soil temp %s' % (soil_potential_raw, soil_temp))
        if soil_temp is None:
            soil_temp = Meteostick.DEFAULT_SOIL_TEMP

        A = 13.50903
        B = 0.001070697
        R = A / (1.0 / soil_potential_raw - B) / 1000
        logdbg('soil moisture sensor resistance: %s kohm' % R)
        if R <= 0:
            logerr('resistance of soil moisture sensor is negative for raw soil potential %s' % soil_potential_raw)
            return None
        if R < 1:
            potential = -20 * (R * (1 + 0.018 * (soil_temp - 24)) - 0.55)
        elif R < 8:
            potential = (-3.213 * R - 4.093) / (1 - 0.009733 * R - 0.01205 * soil_temp)
        else:
            potential = -2.246 - 5.239 * R * (1 + 0.018 * (soil_temp - 24)) \
                - 0.06756 * R ** 2 * (1 + 0.018 * (soil_temp - 24)) ** 2
        return potential

    @staticmethod
    def _check_crc(msg):
        if crc16(msg) != 0:
            raise ValueError("CRC error")

    def configure(self):
        """Configure the device to send data continuously."""
        if DEBUG_SERIAL:
            logdbg("set station to logger mode")

        # Put device into state that we can talk to it reliably
        self.reset()

        # Set rf threshold
        self.send_command('x' + str(self.rf_threshold))

        # Set device to listen to configured transmitters
        self.send_command('t' + str(self.transmitters))

        # Set to filter transmissions from anything other than transmitter 1
        self.send_command('f1')

        # Set device to produce machine readable data
        command = 'o1'
        if self.output_format == 'raw':
            command = 'o0'
        self.send_command('o1')

        # Set device to use the right frequency
        # Valid frequencies are US, EU and AU
        command = 'm0' # default to US
        if self.frequency == 'AU':
            command = 'm2'
        elif self.frequency == 'EU':
            command = 'm1'
        self.send_command(command)

        # From now on the device will produce lines with received data

    def send_command(self, cmd):
        self.serial_port.write(cmd + '\r')
        time.sleep(0.2)
        response = self.serial_port.read(self.serial_port.inWaiting())
        loginf("cmd: '%s': %s" % (cmd, response))
        self.serial_port.flushInput()

    @staticmethod
    def ch_to_xmit(iss_channel, anemometer_channel, leaf_soil_channel,
                   temp_hum_1_channel, temp_hum_2_channel):
        transmitters = 0
        transmitters += 1 << (iss_channel - 1)
        if anemometer_channel != 0:
            transmitters += 1 << (anemometer_channel - 1)
        if leaf_soil_channel != 0:
            transmitters += 1 << (leaf_soil_channel - 1)
        if temp_hum_1_channel != 0:
            transmitters += 1 << (temp_hum_1_channel - 1)
        if temp_hum_2_channel != 0:
            transmitters += 1 << (temp_hum_2_channel - 1)
        return transmitters


class MeteostickConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[Meteostick]
    # This section is for the Meteostick USB receiver.

    # The serial port to which the meteostick is attached, e.g., /dev/ttyS0
    port = /dev/ttyUSB0

    # Radio frequency to use between USB transceiver and console: US, EU or AU
    # US uses 915 MHz
    # EU uses 868.3 MHz
    # AU uses 915 MHz but has different frequency hopping values than US
    transceiver_frequency = EU

    # A channel has value 0-8 where 0 indicates not present
    # The channel of the Vantage Vue, Pro, or Pro2 ISS
    iss_channel = 1
    # Additional channels apply only to Vantage Pro or Pro2
    anemometer_channel = 0
    leaf_soil_channel = 0
    temp_hum_1_channel = 0
    temp_hum_2_channel = 0

    # Rain bucket type: 0 is 0.01 inch per tip, 1 is 0.2 mm per tip
    rain_bucket_type = 1

    # The driver to use
    driver = user.meteostick
"""

    def prompt_for_settings(self):
        settings = dict()
        print "Specify the serial port on which the meteostick is connected,"
        print "for example /dev/ttyUSB0 or /dev/ttyS0"
        settings['port'] = self._prompt('port', MeteostickDriver.DEFAULT_PORT)
        print "Specify the frequency between the station and the meteostick,"
        print "one of US (915 MHz), EU (868.3 MHz), or AU (915 MHz)"
        settings['transceiver_frequency'] = self._prompt('frequency', 'EU', ['US', 'EU', 'AU'])
        print "Specify the type of the rain bucket,"
        print "either 0 (0.01 inches per tip) or 1 (0.2 mm per tip)"
        settings['rain_bucket_type'] = self._prompt('rain_bucket_type', MeteostickDriver.DEFAULT_RAIN_BUCKET_TYPE)
        print "Specify the channel of the ISS (1-8)"
        settings['iss_channel'] = self._prompt('iss_channel', 1)
        print "Specify the channel of the Anemometer Transmitter Kit (0=none; 1-8)"
        settings['anemometer_channel'] = self._prompt('anemometer_channel', 0)
        print "Specify the channel of the Leaf & Soil station (0=none; 1-8)"
        settings['leaf_soil_channel'] = self._prompt('leaf_soil_channel', 0)
        print "Specify the channel of the first Temp/Humidity station (0=none; 1-8)"
        settings['temp_hum_1_channel'] = self._prompt('temp_hum_1_channel', 0)
        print "Specify the channel of the second Temp/Humidity station (0=none; 1-8)"
        settings['temp_hum_2_channel'] = self._prompt('temp_hum_2_channel', 0)
        return settings


class MeteostickConfigurator(weewx.drivers.AbstractConfigurator):
    def add_options(self, parser):
        super(MeteostickConfigurator, self).add_options(parser)
        parser.add_option(
            "--info", dest="info", action="store_true",
            help="display meteostick configuration")
        parser.add_option(
            "--show-options", dest="opts", action="store_true",
            help="display meteostic command options")
        parser.add_option(
            "--set-verbose", dest="verbose", metavar="X", type=int,
            help="set verbose: 0=off, 1=on; default off")
        parser.add_option(
            "--set-debug", dest="debug", metavar="X", type=int,
            help="set debug: 0=off, 1=on; default off")
        # bug in meteostick: according to docs, 0=high, 1=low
        parser.add_option(
            "--set-ledmode", dest="led", metavar="X", type=int,
            help="set led mode: 1=high 0=low; default low")
        parser.add_option(
            "--set-bandwidth", dest="bandwidth", metavar="X", type=int,
            help="set bandwidth: 0=narrow, 1=width; default narrow")
        parser.add_option(
            "--set-probe", dest="probe", metavar="X", type=int,
            help="set probe: 0=off, 1=on; default off")
        parser.add_option(
            "--set-repeater", dest="repeater", metavar="X", type=int,
            help="set repeater: 0-255; default 255")
        parser.add_option(
            "--set-channel", dest="channel", metavar="X", type=int,
            help="set channel: 0-255; default 255")
        parser.add_option(
            "--set-channel", dest="format", metavar="X", type=int,
            help="set format: 0=raw, 1=machine, 2=human")

    def do_options(self, options, parser, config_dict, prompt):
        driver = MeteostickDriver(**config_dict[DRIVER_NAME])
        info = driver.station.reset()
        if options.info:
            print info
        cfg = {
            'v': options.verbose,
            'd': options.debug,
            'l': options.led,
            'b': options.bandwidth,
            'p': options.probe,
            'r': options.repeater,
            'c': options.channel,
            'o': options.format}
        for opt in cfg:
            if cfg[opt]:
                cmd = opt + cfg[opt]
                print "set station parameter %s" % cmd
                driver.station.send_command(cmd)
        if options.opts:
            driver.station.send_command('?')
            print driver.station.get()
        driver.closePort()


# define a main entry point for basic testing of the station without weewx
# engine and service overhead.  invoke this as follows from the weewx root dir:
#
# PYTHONPATH=bin python bin/user/meteostick.py

if __name__ == '__main__':
    import optparse

    usage = """%prog [options] [--help]"""

    syslog.openlog('meteostick', syslog.LOG_PID | syslog.LOG_CONS)
    syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_DEBUG))
    parser = optparse.OptionParser(usage=usage)
    parser.add_option('--version', dest='version', action='store_true',
                      help='display driver version')
    parser.add_option('--port', dest='port', metavar='PORT',
                      help='serial port to which the station is connected',
                      default=MeteostickDriver.DEFAULT_PORT)
    parser.add_option('--baud', dest='baud', metavar='BAUDRATE',
                      help='serial port baud rate',
                      default=MeteostickDriver.DEFAULT_BAUDRATE)
    parser.add_option('--freq', dest='freq', metavar='FREQUENCY',
                      help='comm frequency, either US (915MHz) or EU (868MHz)',
                      default=MeteostickDriver.DEFAULT_FREQUENCY)
    parser.add_option('--rfs', dest='rfs', metavar='RF_SENSITIVITY',
                      help='RF sensitivity in dB',
                      default=MeteostickDriver.DEFAULT_RF_SENSITIVITY)
    parser.add_option('--iss-channel', dest='c_iss', metavar='ISS_CHANNEL',
                      help='channel for ISS', default=1)
    parser.add_option('--anemometer-channel', dest='c_a',
                      metavar='ANEMOMETER_CHANNEL',
                      help='channel for anemometer', default=0)
    parser.add_option('--leaf-soil-channel', dest='c_ls',
                      metavar='LEAF_SOIL_CHANNEL',
                      help='channel for leaf-soil', default=0)
    parser.add_option('--th1-channel', dest='c_th1', metavar='TH1_CHANNEL',
                      help='channel for T/H sensor 1', default=0)
    parser.add_option('--th2-channel', dest='c_th2', metavar='TH2_CHANNEL',
                      help='channel for T/H sensor 2', default=0)
    (opts, args) = parser.parse_args()

    if opts.version:
        print "meteostick driver version %s" % DRIVER_VERSION
        exit(0)

    xmitters = Meteostick.ch_to_xmit(
        int(opts.c_iss), int(opts.c_a), int(opts.c_ls),
        int(opts.c_th1), int(opts.c_th2))

    with Meteostick(opts.port, opts.baud, xmitters, opts.freq, opts.rfs) as s:
        while True:
            print time.time(), s.get_readings()
