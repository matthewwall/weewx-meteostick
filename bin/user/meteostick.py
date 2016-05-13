#!/usr/bin/env python
# Meteostick driver for weewx
#
# Copyright 2016 Matthew Wall, Luc Heijst
#
# Thanks to Frank Bandle for testing during the development of this driver.
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

"""Meteostick is a USB device that receives radio transmissions from Davis
weather stations.

The meteostick has a preset radio frequency (RF) treshold value which is twice
the RF sensity value in dB.  Valid values for RF sensity range from 0 to 125.
Both positive and negative parameter values will be treated as the
same actual (negative) dB values.

The default RF sensitivity value is 90 (-90 dB).  Values between 95 and 125
tend to give too much noise and false readings (the higher value the more
noise).  Values lower than 50 likely result in no readings at all.

The meteostick outputs data in one of 3 formats: human-readable, machine, and
raw.  The machine format is, in fact, human-readable as well.  This driver
supports the machine and raw formats.  The raw format provides more data and
seems to result in higher quality readings, so it is the default.
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
DRIVER_VERSION = '0.36'

MPH_TO_MPS = 0.44704 # mph to m/s

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


def dbg_serial(verbosity, msg):
    if DEBUG_SERIAL >= verbosity:
        logdbg(msg)

def dbg_parse(verbosity, msg):
    if DEBUG_PARSE >= verbosity:
        logdbg(msg)


# default temperature for soil moisture and leaf wetness sensors that
# do not have a temperature sensor.
DEFAULT_SOIL_TEMP = 24 # C

raw_moist_table = [   0,  118,  282,  386,  476,  541,  585,  621,  643,  664,  687,  707, 1022]
temp_corr_table = [-2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5, -2.5]

def calculate_leaf_soil_temp(leaf_soil_temp_raw):
    """ Decode the ls temperature then calculate the actual ls temperature
    and the soil potential, using Davis' formulas.
    When the sensor is not populated, leaf_soil_temp_raw and
    soil_potential_raw are set to their max values (0x3ff).
    see: https://github.com/cmatteri/CC1101-Weather-Receiver/wiki/Soil-Moisture-Station-Protocol
    :param leaf_soil_temp_raw: temperature in degrees C
    """

    # Convert leaf_soil_temp_raw to a resistance (R) in kiloOhms
    A = 18.81099
    B = 0.0009988027
    R = A / (1.0 / leaf_soil_temp_raw - B) / 1000 # k ohms

    # Steinhart-Hart parameters
    S1 = 0.002783573
    S2 = 0.0002509406
    try:
        leaf_soil_temp = 1 / (S1 + S2 * math.log(R)) - 273
        dbg_parse(2, 'leaf_soil_temp thermistor resistance: %s kohm, leaf_soil_temp: %s' % (R, leaf_soil_temp))
        return leaf_soil_temp
    except ValueError, e:
        logerr('soil/leaf temp failed for raw temp %s: %s' %
               (leaf_soil_temp_raw, e))
    return None

def calculate_soil_moisture(soil_potential_raw, soil_temp, leaf_soil_temp_raw):
    # The calculation of the soil potential in cb is based upon empirical
    # values in two lookup tables. Each step in the table represents a value
    # of 20 cb.

    if soil_temp is None:
        soil_temp = DEFAULT_SOIL_TEMP

    potential = 0.0  # preset soil moisture
    raw_corr = 0.0  # preset raw correction
    sp_raw_corr = 0.0  # preset raw correction

    # we correct the soil moisture raw value for the temperature
    # We have added a table with temperature correction factors per section
    # First lookup the section for the soil_potential_raw value
    for x in range(0, 13):
        if soil_potential_raw < raw_moist_table[x]:
            # Use the corresponding temperature correction factor
            temp_corr_factor = temp_corr_table[x-1]  # raw correction per degree Celsius
            # Determine the temperature correction for the soil moist raw value
            raw_corr = (DEFAULT_SOIL_TEMP - soil_temp) * temp_corr_factor
            # Determine the corrected soil potential raw value
            sp_raw_corr = soil_potential_raw + raw_corr
            break

    # Now lookup sp_raw_corr value in table
    for x in range(0, 13):
        if sp_raw_corr < raw_moist_table[x]:
            # determine the soil moisture value in cb
            potential = (20.0 * (x-1)) \
                        - (20.0 * (raw_moist_table[x] - sp_raw_corr) / (raw_moist_table[x] - raw_moist_table[x-1]))
            dbg_parse(1, "soil_potential_raw: %s sp_raw_corr: %s x: %s sm: %s"
                   % (soil_potential_raw, sp_raw_corr, x, potential))
            break

    # Clamp potentional between 0 and 200
    potential = max(0, potential)
    potential = min(200, potential)
    return potential, raw_corr

def calculate_leaf_wetness(leaf_potential_raw, leaf_temp):
    """
    :param leaf_potential_raw:
    :param leaf_temp: temperature in degrees C
    potential is in Davis' leaf-wet units (1-15)
    """

    if leaf_temp is None:
        leaf_temp = DEFAULT_SOIL_TEMP

    if leaf_potential_raw < 847:
        # 0 - 846
        potential = 15
    elif leaf_potential_raw < 1010:
        # 847 - 1009
        # formula based upon regression formula
        # in 1: leaf_potential_raw (used range: 847 - 1010)
        # in 2: leaf_temp (c)
        # out:  potential (Vantage loop data)
        # lh TODO regression parameters are not accurate enough
        potential = 106.179014986425 -0.0977158641686967 * leaf_potential_raw -0.311507255127535 * leaf_temp
    else:
        # 1010 -1022
        potential = 0

    dbg_parse(1, "leaf_wetness_raw: %s, leaf_wetness_temp: %s, leaf_wetness: %s"
           % (leaf_potential_raw, leaf_temp, potential))

    # Clamp potentional between 0 and 15
    potential = max(0, potential)
    potential = min(15, potential)
    return potential


RAW_CHANNEL = 0 # unused channel for the receiver stats in raw format
MACHINE_CHANNEL = 9 # fake channel for the receiver stats in machine format

class MeteostickDriver(weewx.drivers.AbstractDevice):
    NUM_CHAN = 10 # 8 channels, one fake channel (9), one unused channel (0)
    DEFAULT_RAIN_BUCKET_TYPE = 1
    DEFAULT_SENSOR_MAP = {
        'pressure': 'pressure',
        'in_temp': 'inTemp',
        'wind_speed': 'windSpeed',
        'wind_dir': 'windDir',
        'temperature': 'outTemp',
        'humidity': 'outHumidity',
        'rain_count': 'rain',
        'rain_rate': 'rainRate', # FIXME: remove rain_rate after testing
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
        'leaf_temp_1': 'leafTemp1',
        'leaf_temp_2': 'leafTemp2',
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

        global DEBUG_PARSE
        DEBUG_PARSE = int(stn_dict.get('debug_parse', DEBUG_PARSE))
        global DEBUG_SERIAL
        DEBUG_SERIAL = int(stn_dict.get('debug_serial', DEBUG_SERIAL))
        global DEBUG_RAIN
        DEBUG_RAIN = int(stn_dict.get('debug_rain', DEBUG_RAIN))
        global DEBUG_RFS
        DEBUG_RFS = int(stn_dict.get('debug_rf_sensitivity', DEBUG_RFS))

        bucket_type = int(stn_dict.get('rain_bucket_type',
                                       self.DEFAULT_RAIN_BUCKET_TYPE))
        if bucket_type not in [0, 1]:
            raise ValueError("unsupported rain bucket type %s" % bucket_type)
        self.rain_per_tip = 0.254 if bucket_type == 0 else 0.2 # mm
        loginf('using rain_bucket_type %s' % bucket_type)
        self.sensor_map = stn_dict.get('sensor_map', self.DEFAULT_SENSOR_MAP)
        loginf('sensor map is: %s' % self.sensor_map)
        self.max_tries = int(stn_dict.get('max_tries', 10))
        self.retry_wait = int(stn_dict.get('retry_wait', 10))
        self.last_rain_count = None
        self._init_rf_stats()

        self.station = Meteostick(**stn_dict)
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
            data = self.station.parse_readings(readings, self.rain_per_tip)
            if data:
                self._update_rf_stats(data['channel'], data['rf_signal'],
                                      data['rf_missed'])
                now = int(time.time())
                if now - self.rf_stats['ts'] > 50 and now % 300 > 290:
                    # Flush (and report) the rf stats at 5 min intervals.
                    # Typically this will be about 10 seconds before a new
                    # archive interval (assuming the signals come each 2-3
                    # seconds) to let the calculated pctgood be stored in the
                    # right period.
                    self._update_rf_summaries()  # calculate rf summaries
                    data['pct_good'] = self.rf_stats['pctgood'][self.station.channels['iss']]
                    logdbg("data['pct_good']: %s" % data['pct_good'])
                    if DEBUG_RFS:
                        self._report_rf_stats()
                    self._init_rf_stats()  # flush rf statistics
                dbg_parse(2, "data: %s" % data)
                packet = self._data_to_packet(data)
                dbg_parse(2, "packet: %s" % packet)
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
                loginf("rain counter wraparound detected rain_count=%s" %
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
            'missed': [0] * self.NUM_CHAN,
            'pctgood': [None] * self.NUM_CHAN,
            'ts': int(time.time())}
        # unlike the rf sensitivity measures, pct_good is positive
        self.rf_stats['min'][MACHINE_CHANNEL] = 100

    def _update_rf_stats(self, ch, signal, missed):
        # update the rf statistics
        self.rf_stats['min'][ch] = min(signal, self.rf_stats['min'][ch])
        self.rf_stats['max'][ch] = max(signal, self.rf_stats['max'][ch])
        self.rf_stats['sum'][ch] += signal
        self.rf_stats['cnt'][ch] += 1
        self.rf_stats['last'][ch] = signal
        self.rf_stats['missed'][ch] += missed

    def _update_rf_summaries(self):
        # Update the summary stats, skip channels that do not matter.
        #
        # The pctgood is a measure of rf quality.  The way it is calculated
        # depends on the output format.
        #
        # For machine, pct_good is (can be) part of the 'B' messages.  This
        # value will be saved in the rf_stats of MACHINE_CHANNEL, typically
        # once per minute.  When the summaries are calculated, the average of
        # the pct_good values will be stored in the pctgood value of
        # iss_channel.  The values of pctgood for the other active channels
        # remain unchanged (i.e., None)
        #
        # For raw format, the values of pctgood will be calculated per active
        # channel when the summaries are calculated, based upon the number of
        # received good packets and number of missed packets.  The min stat
        # for MACHINE_CHANNEL will always have the (initial) value of 100
        # when the format is raw.

        for ch in range(1, self.NUM_CHAN): # skip channel 0 (it is not used)
            if self.rf_stats['cnt'][ch] > 0:
                self.rf_stats['avg'][ch] = int(self.rf_stats['sum'][ch] / self.rf_stats['cnt'][ch])
        if self.station.output_format == 'raw':
            # raw format
            for ch in range(1, self.NUM_CHAN - 1): # no ch 0 or MACHINE_CHANNEL
                if self.rf_stats['cnt'][ch] > 0:
                    self.rf_stats['pctgood'][ch] = \
                        int(100.0 * self.rf_stats['cnt'][ch] /
                            (self.rf_stats['cnt'][ch] + self.rf_stats['missed'][ch]))
        else:
            # machine format
            self.rf_stats['pctgood'][self.station.channels['iss']] = self.rf_stats['avg'][MACHINE_CHANNEL]

    def _report_rf_stats(self):
        raw_format = self.rf_stats['min'][MACHINE_CHANNEL] == 100
        logdbg("RF summary: rf_sensitivity=%s (values in dB)" %
               self.station.rfs)
        logdbg("Station           max   min   avg   last  count [missed] [good]")
        for x in [('iss', self.station.channels['iss']),
                  ('wind', self.station.channels['anemometer']),
                  ('leaf_soil', self.station.channels['leaf_soil']),
                  ('temp_hum_1', self.station.channels['temp_hum_1']),
                  ('temp_hum_2', self.station.channels['temp_hum_2']),
                  ('pct_good', MACHINE_CHANNEL)]:
            if x[1] != 0:
                if not(raw_format and x[1] == MACHINE_CHANNEL):
                    self._report_channel(x[0], x[1], raw_format)

    def _report_channel(self, label, ch, raw_format):
        if raw_format:
            logdbg("%s %5d %5d %5d %5d %5d %7d      %s" %
                   (label.ljust(15),
                    self.rf_stats['max'][ch],
                    self.rf_stats['min'][ch],
                    self.rf_stats['avg'][ch],
                    self.rf_stats['last'][ch],
                    self.rf_stats['cnt'][ch],
                    self.rf_stats['missed'][ch],
                    self.rf_stats['pctgood'][ch]))
        else:
            logdbg("%s %5d %5d %5d %5d %5d" %
                   (label.ljust(15),
                    self.rf_stats['max'][ch],
                    self.rf_stats['min'][ch],
                    self.rf_stats['avg'][ch],
                    self.rf_stats['last'][ch],
                    self.rf_stats['cnt'][ch]))


class Meteostick(object):
    DEFAULT_PORT = '/dev/ttyUSB0'
    DEFAULT_BAUDRATE = 115200
    DEFAULT_FREQUENCY = 'EU'
    DEFAULT_RF_SENSITIVITY = 90
    MAX_RF_SENSITIVITY = 125

    def __init__(self, **cfg):
        self.port = cfg.get('port', self.DEFAULT_PORT)
        loginf('using serial port %s' % self.port)

        self.baudrate = cfg.get('baudrate', self.DEFAULT_BAUDRATE)
        loginf('using baudrate %s' % self.baudrate)

        freq = cfg.get('transceiver_frequency', self.DEFAULT_FREQUENCY)
        if freq not in ['EU', 'US', 'AU']:
            raise ValueError("invalid frequency %s" % freq)
        self.frequency = freq
        loginf('using frequency %s' % self.frequency)

        rfs = int(cfg.get('rf_sensitivity', self.DEFAULT_RF_SENSITIVITY))
        absrfs = abs(rfs)
        if absrfs > self.MAX_RF_SENSITIVITY:
            raise ValueError("invalid RF sensitivity %s" % rfs)
        self.rfs = absrfs
        self.rf_threshold = absrfs * 2
        loginf('using rf sensitivity %s (-%s dB)' % (rfs, absrfs))

        fmt = cfg.get('format', 'raw')
        if fmt.lower() not in ['machine', 'raw']:
            raise ValueError("unsupported format '%s'" % fmt)
        self.output_format = fmt.lower()
        loginf('using output_format %s' % fmt)
        channels = dict()
        channels['iss'] = int(cfg.get('iss_channel', 1))
        channels['anemometer'] = int(cfg.get('anemometer_channel', 0))
        channels['leaf_soil'] = int(cfg.get('leaf_soil_channel', 0))
        channels['temp_hum_1'] = int(cfg.get('temp_hum_1_channel', 0))
        channels['temp_hum_2'] = int(cfg.get('temp_hum_2_channel', 0))
        if channels['anemometer'] == 0:
            channels['wind_channel'] = channels['iss']
        else: 
            channels['wind_channel'] = channels['anemometer']
        self.channels = channels
        loginf('using iss_channel %s' % channels['iss'])
        loginf('using anemometer_channel %s' % channels['anemometer'])
        loginf('using leaf_soil_channel %s' % channels['leaf_soil'])
        loginf('using temp_hum_1_channel %s' % channels['temp_hum_1'])
        loginf('using temp_hum_2_channel %s' % channels['temp_hum_2'])

        self.transmitters = Meteostick.ch_to_xmit(
            channels['iss'], channels['anemometer'], channels['leaf_soil'],
            channels['temp_hum_1'], channels['temp_hum_2'])
        loginf('using transmitters %02x' % self.transmitters)

        self.timeout = 3 # seconds
        self.serial_port = None

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

    @staticmethod
    def _check_crc(msg):
        if crc16(msg) != 0:
            raise ValueError("CRC error")

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _, value, traceback):
        self.close()

    def open(self):
        dbg_serial(1, "open serial port %s" % self.port)
        self.serial_port = serial.Serial(self.port, self.baudrate,
                                         timeout=self.timeout)

    def close(self):
        if self.serial_port is not None:
            dbg_serial(1, "close serial port %s" % self.port)
            self.serial_port.close()
            self.serial_port = None

    def get_readings(self):
        buf = self.serial_port.readline()
        if len(buf) > 0:
            dbg_serial(2, "station said: %s" %
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
        dbg_serial(2, "full response to reset: %s" % response)
        # Discard any serial input from the device
        time.sleep(0.2)
        self.serial_port.flushInput()
        return response

    def configure(self):
        """Configure the device to send data continuously."""
        loginf("configure meteostick to logger mode")

        # Put device into state that we can talk to it reliably
        self.reset()

        # Show default settings (they might change with a new firmware version)
        self.send_command('?')

        # Set RF threshold
        self.send_command('x' + str(self.rf_threshold))

        # Listen to configured transmitters
        self.send_command('t' + str(self.transmitters))

        # Filter transmissions from anything other than configured transmitters
        self.send_command('f1')

        # Set device to produce machine readable data
        command = 'o1'
        if self.output_format == 'raw':
            # Set device to produce raw data
            command = 'o0'
        self.send_command(command)

        # Set the frequency. Valid frequencies are US, EU and AU
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
        dbg_serial(1, "cmd: '%s': %s" % (cmd, response))
        self.serial_port.flushInput()

    @staticmethod
    def get_parts(raw):
        dbg_parse(1, "readings: %s" % raw)
        parts = raw.split(' ')
        dbg_parse(3, "parts: %s (%s)" % (parts, len(parts)))
        if len(parts) < 2:
            raise ValueError("not enough parts in '%s'" % raw)
        return parts

    def parse_readings(self, raw, rain_per_tip):
        data = dict()
        if not raw:
            return data
        try:
            if self.output_format == 'raw':
                data = self.parse_raw(raw,
                                      self.channels['iss'],
                                      self.channels['anemometer'],
                                      self.channels['leaf_soil'],
                                      self.channels['temp_hum_1'],
                                      self.channels['temp_hum_2'],
                                      rain_per_tip)
            else:
                data = self.parse_machine(raw,
                                          self.channels['iss'],
                                          self.channels['temp_hum_1'],
                                          self.channels['temp_hum_2'])
        except ValueError, e:
            logerr("parse failed for '%s': %s" % (raw, e))
            data = dict() # do not return partial data
        return data

    @staticmethod
    def parse_machine(raw, iss_ch, th1_ch, th2_ch):
        # parse the 'machine' format
        #
        # FIXME: put sample output here
        data = dict()
        parts = Meteostick.get_parts(raw)
        n = len(parts)
        data['channel'] = 0  # preset not available
        data['rf_signal'] = 0  # preset not available
        data['rf_missed'] = 0  # preset not available
        if parts[0] == 'B':
            # message examples:
            # B 35.2 1023.57
            # B 35.2 1023.57 65%
            if n >= 3:
                data['in_temp'] = float(parts[1]) # C
                data['pressure'] = float(parts[2]) # hPa
                if n >= 4:
                    data['channel'] = MACHINE_CHANNEL
                    data['rf_signal'] = float(parts[3].strip('%'))
            else:
                logerr("B: not enough parts (%s) in '%s'" % (n, raw))
        elif parts[0] in 'WT':
            # message examples:
            # W 1 0.44 233 -68
            # W 3 0.00 261 -53 L
            # T 1 19.6 41 -66
            # T 3 20.8 36 -53 L
            if n >= 5:
                data['channel'] = int(parts[1])
                data['rf_signal'] = float(parts[4])
                bat = 1 if n >= 6 and parts[5] == 'L' else 0
                if parts[0] == 'W':
                    if iss_ch != 0 and data['channel'] == iss_ch:
                        data['bat_iss'] = bat
                    else:
                        data['bat_anemometer'] = bat
                    data['wind_speed'] = float(parts[2]) # m/s
                    data['wind_dir'] = float(parts[3]) # degrees
                elif parts[0] == 'T':
                    if th1_ch != 0 and data['channel'] == th1_ch:
                        data['bat_th_1'] = bat
                        data['temp_1'] = float(parts[2]) # C
                        data['humid_1'] = float(parts[3]) # %
                    elif th2_ch != 0 and data['channel'] == th2_ch:
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
            # message examples:
            # L 7 2 0 -52
            # M 7 1 49 -51
            # O 7 1 22.3 -51
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
            # message examples:
            # R 2 128 -67
            # S 621.2 -68
            # U 3 0.4 -58 L
            # P 1 21.1 -68
            if n >= 4:
                data['channel'] = int(parts[1])
                data['rf_signal'] = float(parts[3])
                data['bat_iss'] = 1 if n >= 5 and parts[4] == 'L' else 0
                if parts[0] == 'R':
                    rain_count = int(parts[2])
                    if 0 <= rain_count < 128:
                        data['rain_count'] = rain_count  # 0-127
                    else:
                        loginf("ignoring invalid rain %s on channel %s" %
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
            logerr("unknown sensor identifier '%s' in '%s'" % (parts[0], raw))
        return data

    @staticmethod
    def parse_raw(raw, iss_ch, wind_ch, ls_ch, th1_ch, th2_ch, rain_per_tip):
        data = dict()
        parts = Meteostick.get_parts(raw)
        n = len(parts)
        if parts[0] == 'B':
            # message example:
            # B 29530 338141 366 101094 60 37
            data['channel'] = RAW_CHANNEL # rf_signal data will not be used
            data['rf_signal'] = 0  # not available
            data['rf_missed'] = 0  # not available
            if n >= 6:
                data['in_temp'] = float(parts[3]) / 10.0 # C
                data['pressure'] = float(parts[4]) / 100.0 # hPa
            else:
                logerr("B: not enough parts (%s) in '%s'" % (n, raw))
        elif parts[0] == 'I':
            raw_msg = [0] * 8
            for i in xrange(0, 8):
                raw_msg[i] = chr(int(parts[i + 2], 16))
            Meteostick._check_crc(raw_msg)
            for i in xrange(0, 8):
                raw_msg[i] = parts[i + 2]
            pkt = bytearray([int(i, base=16) for i in raw_msg])
            data['channel'] = (pkt[0] & 0x7) + 1
            battery_low = (pkt[0] >> 3) & 0x1
            data['rf_signal'] = int(parts[11])
            time_since_last = int(parts[12])
            data['rf_missed'] = ((time_since_last + 1250000) // 2500000) - 1

            if data['channel'] == iss_ch or data['channel'] == wind_ch:
                if data['channel'] == iss_ch:
                    data['bat_iss'] = battery_low
                else:
                    data['bat_anemometer'] = battery_low
                # Each data packet of iss or anemometer contains wind info,
                # but it is only valid when received from the channel with
                # the anemometer connected
                # message examples:
                # I 101 51 6 B2 FF 73 0 76 61  -69 2624964 59
                # I 101 E0 0 0 4E 5 0 72 61  -68 2562440 68 (no sensor)
                wind_speed_raw = pkt[1]
                wind_dir_raw = pkt[2]
                if not(wind_speed_raw == 0 and wind_dir_raw == 0):
                    """ The elder Vantage Pro and Pro2 stations measured
                    the wind direction with a potentiometer. This type has
                    a fairly big dead band around the North. The Vantage
                    Vue station uses a hall effect device to measure the
                    wind direction. This type has a much smaller dead band,
                    so there are two different formulas for calculating
                    the wind direction. To be able to select the right
                    formula the Vantage type must be known. The more recent
                    Vantage pro2 wind meters use the same sensor technique
                    as the Vantage Vue. It is not known how Davis handles
                    the values of the new wind meters in its protocol.

                    For now we use the traditional 'pro' formula for all
                    wind directions.
                    """
                    dbg_parse(2, "wind_speed_raw: %03x, wind_dir_raw: 0x%03x" %
                              (wind_speed_raw, wind_dir_raw))
                    data['wind_speed'] = wind_speed_raw * MPH_TO_MPS
                    W_S = wind_speed_raw # mph
                    # Vantage Pro and Pro2
                    wind_dir_pro = 9 + wind_dir_raw * 342.0 / 255.0
                    # Vantage Vue (maybe also for newer Pro 2)
                    wind_dir_vue = wind_dir_raw * 1.40625 + 0.3
                    data['wind_dir'] = wind_dir_pro
                    dbg_parse(1, "WS %s WD %s WD_vue %s" %
                              (W_S, wind_dir_pro, wind_dir_vue))

                # data from both iss sensors and extra sensors on
                # Anemometer Transport Kit
                message_type = (pkt[0] >> 4 & 0xF)
                if message_type == 2:
                    # supercap voltage (Vue only) max: 0x3FF (1023)
                    # message examples:
                    # TODO
                    # TODO (no sensor)
                    supercap_volt_raw = ((pkt[3] << 2) + (pkt[4] >> 6)) & 0x3FF
                    if supercap_volt_raw != 0x3FF:
                        data['supercap_volt'] = supercap_volt_raw / 100.0
                        dbg_parse(2, "supercap_volt_raw: 0x%03x, value: %s" %
                                  (supercap_volt_raw, data['supercap_volt']))
                elif message_type == 3:
                    # unknown message type
                    # message examples:
                    # TODO
                    # TODO (no sensor)
                    dbg_parse(1, "unknown message with type=0x03; "
                              "pkt[3]: 0x%02x, pkt[4]: 0x%02x, pkt[5]: 0x%02x"
                              % (pkt[3], pkt[4], pkt[5]))
                elif message_type == 4:
                    # uv max: 0xFFC0 (65472)
                    # message examples:
                    # TODO
                    # TODO (no sensor)
                    uv_raw = (pkt[3] * 256 + pkt[4]) & 0xFFC0
                    if pkt[3] != 0xFF:
                        data['uv'] = uv_raw / 50.0
                        dbg_parse(2, "uv_raw: %04x, value: %s" %
                                  (uv_raw, data['uv']))
                elif message_type == 5:
                    # rain rate
                    # message examples:
                    # I 104 50 0 0 FF 75 0 48 5B  -77 2562452 140 (no rain)
                    # I 101 50 0 0 FE 75 0 7F 6B  -66 2562464 68 (light_rain)
                    # I 100 50 0 0 1B 15 0 3F 80  -67 2562448 -95 (heavy_rain)
                    # I 102 51 0 DB FF 73 0 11 41  -65 5249944 202 (no sensor)
                    """ The published rain_rate formulas differ from each
                    other. For both light and heavy rain we like to know a
                    'time between tips' in s. The rain_rate then would be:
                    3600 [s/h] / time_between_tips [s] * 0.2 [mm] = xxx [mm/h]
                    """
                    time_between_tips_raw = ((pkt[4] & 0x30) << 4) + pkt[3]  # typical: 64-1022
                    dbg_parse(2, "time_between_tips_raw: %03x (%s)" %
                              (time_between_tips_raw, time_between_tips_raw))
                    if time_between_tips_raw == 0x3FF:
                        # no rain
                        data['rain_rate'] = 0
                        dbg_parse(2, "no_rain: %s mm/h" % data['rain_rate'])
                    else:
                        if pkt[4] & 0x40 == 0:
                            # heavy rain. typical value:
                            # 64/16 - 1020/16 = 4 - 63.8 (180.0 - 11.1 mm/h)
                            time_between_tips = time_between_tips_raw / 16
                            data['rain_rate'] = 3600 / time_between_tips * rain_per_tip # mm/h
                            dbg_parse(2, "heavy_rain: %s mm/h, time_between_tips: %s s" %
                                      (data['rain_rate'], time_between_tips))
                        else:
                            # light rain. typical value:
                            # 64 - 1022 (11.1 - 0.8 mm/h)
                            time_between_tips = time_between_tips_raw
                            data['rain_rate'] = 3600 / time_between_tips * rain_per_tip # mm/h
                            dbg_parse(2, "heavy_rain: %s mm/h, time_between_tips: %s s" %
                                      (data['rain_rate'], time_between_tips))
                elif message_type == 6:
                    # solar radiation
                    # examples
                    # I 104 61 0 DB 0 43 0 F4 3B  -66 2624972 121
                    # I 104 60 0 0 FF C5 0 79 DA  -77 2562444 137 (no sensor)
                    sr_raw = ((pkt[3] << 2) + (pkt[4] >> 6)) & 0x3FF
                    if sr_raw != 0x3FF:
                        data['solar_radiation'] = sr_raw * 1.757936
                        dbg_parse(2, "solar_radiation_raw: 0x%04x, value: %s" % (sr_raw, data['solar_radiation']))
                elif message_type == 7:
                    # solar cell output (Vue only)
                    sco_raw = ((pkt[3] << 2) + (pkt[4] >> 6)) & 0x3FF
                    if sco_raw != 0xFFC0:
                        data['solar_cell_output'] = sco_raw
                        dbg_parse(2, "sco_raw: 0x%03x, value: %s" % (sco_raw, data['solar_cell_output']))
                elif message_type == 8:
                    # temperature
                    # message examples:
                    # I 103 80 0 0 33 8D 0 25 11  -78 2562444 -25
                    # I 104 81 0 DB FF C3 0 AB F8  -66 2624980 125 (no sensor)
                    temp_f_raw = (pkt[3] << 4) + (pkt[4] >> 4)
                    if temp_f_raw != 0xFFC:
                        temp_f = temp_f_raw / 10.0
                        data['temperature'] = weewx.wxformulas.FtoC(temp_f) # C
                        dbg_parse(2, "temp_f_raw: 0x%03x, temp_f: %s, temp_c: %s" % (temp_f_raw, temp_f, data['temperature']))
                elif message_type == 9:
                    # 10-min average wind gust
                    # message examples:
                    # I 102 91 0 DB 0 3 E 89 85  -66 2624972 204
                    # I 102 90 0 0 0 5 0 31 51  -75 2562456 223 (no sensor)
                    gust_raw = pkt[3]  # mph
                    gust_index_raw = pkt[5] >> 4
                    if not(gust_raw == 0 and gust_index_raw == 0):
                        dbg_parse(1, "W10 %s gust_index_raw %s" % (gust_raw, gust_index_raw))
                    # don't store the 10-min gust data
                elif message_type == 0xA:
                    # humidity
                    # message examples:
                    # I 101 A0 0 0 C9 3D 0 2A 87  -76 2562432 54
                    # I 100 A1 0 DB 0 3 0 47 C7  -67 5249932 -130 (no sensor)
                    humidity_raw = ((pkt[4] >> 4) << 8) + pkt[3]
                    if humidity_raw != 0:
                        data['humidity'] = humidity_raw / 10.0
                        dbg_parse(2, "humidity_raw: 0x%03x, value: %s" % (humidity_raw, data['humidity']))
                elif message_type == 0xC:
                    # unknown ATK message
                    # example:
                    # I 101 C1 0 DB 0 3 0 18 DF  -66 2624964 40
                    unknown_atk_raw = ((pkt[3] << 2) + (pkt[4] >> 6)) & 0x3FF
                    if unknown_atk_raw != 0xFFC0:
                        dbg_parse(2, "unknown_atk_raw: 0x%03x, value: %s" % (unknown_atk_raw, unknown_atk_raw))
                elif message_type == 0xE:
                    # rain
                    # message examples:
                    # I 103 E0 0 0 5 5 0 9F 3D  -78 2562416 -28
                    # I 101 E1 0 DB 80 3 0 16 8D  -67 5249956 37 (no sensor)
                    rain_count_raw = pkt[3]
                    """We have seen rain counters wrap around at 127 and
                    others wrap around at 255.  When we filter the highest
                    bit, both counter types will wrap at 127.
                    """
                    if rain_count_raw != 0x80:
                        rain_count = rain_count_raw & 0x7F  # skip high bit
                        data['rain_count'] = rain_count
                        dbg_parse(2, "rain_count_raw: 0x%02x, value: %s" % (rain_count_raw, rain_count))
                else:
                    # unknown message type
                    logerr("unknown message type 0x%01x" % message_type)

            elif data['channel'] == ls_ch:
                # leaf and soil station
                data_type = pkt[0] >> 4
                if data_type == 0xF:
                    data_subtype = pkt[1] & 0x3
                    sensor_num = ((pkt[1] & 0xe0) >> 5) + 1
                    leaf_soil_temp = DEFAULT_SOIL_TEMP
                    leaf_soil_temp_raw = ((pkt[3] << 2) + (pkt[5] >> 6)) & 0x3FF
                    leaf_soil_potential_raw = ((pkt[2] << 2) + (pkt[4] >> 6)) & 0x3FF

                    if data_subtype == 1:
                        # soil moisture
                        # message examples:
                        # I 102 F2 9 1A 55 C0 0 62 E6  -51 2687524 207
                        # I 104 F2 29 FF FF C0 C0 F1 EC  -52 2687408 124 (no sensor)
                        if leaf_soil_temp_raw != 0x3FF:
                            leaf_soil_temp = calculate_leaf_soil_temp(leaf_soil_temp_raw)
                            dbg_parse(3, "soil_temp_%s raw: 0x%03x (%s)" %
                                     (sensor_num, leaf_soil_temp, leaf_soil_temp))
                            data['soil_temp_%s' % sensor_num] = leaf_soil_temp  # C
                        if leaf_soil_potential_raw != 0x3FF:
                            # soil moisture
                            soil_moisture, raw_corr = calculate_soil_moisture(
                                leaf_soil_potential_raw, leaf_soil_temp, leaf_soil_temp_raw)
                            data['soil_moisture_%s' % sensor_num] = soil_moisture
                            #lh TODO put raw values in temporary tag soil_moisture_2 to view the graph
                            data['soil_moisture_2'] =  leaf_soil_potential_raw
                            #lh TODO put raw correction in temporary tag soil_moisture_3 to view the graph
                            data['soil_moisture_3'] =  raw_corr
                            # Sample data for regression
                            dbg_parse(1, "S_M_R %s R_C %s S_T_R %s S_T %s S_M %s" % (leaf_soil_potential_raw, raw_corr, leaf_soil_temp_raw, leaf_soil_temp, soil_moisture))

                    elif data_subtype == 2:
                        # leaf wetness
                        # message examples:
                        # I 100 F2 A D4 55 80 0 90 6  -53 2687516 -121
                        # I 101 F2 2A 0 FF 40 C0 4F 5  -52 2687404 43 (no sensor, variant 1)
                        # I 100 F2 2A 0 FF 80 C0 59 51  -53 2687416 -124 (no sensor, variant 2)
                        if leaf_soil_temp_raw != 0x3FF:
                            leaf_soil_temp = calculate_leaf_soil_temp(leaf_soil_temp_raw)
                            dbg_parse(3, "leaf_temp_%s raw: 0x%03x (%s)" % (sensor_num, leaf_soil_temp, leaf_soil_temp))
                            data['leaf_temp_%s' % sensor_num] = leaf_soil_temp # C
                        if not(leaf_soil_potential_raw == 0x002 or leaf_soil_potential_raw == 0x001) :  # (00 80 and 00 40)
                            leaf_wetness = calculate_leaf_wetness(
                                leaf_soil_potential_raw, leaf_soil_temp)
                            data['leaf_wetness_%s' % sensor_num] = leaf_wetness
                            #lh TODO put raw values in temporary tag soil_moisture_4 to view the graph
                            data['soil_moisture_4'] =  leaf_soil_potential_raw
                            # Sample data for regression
                            dbg_parse(1, "L_W_R %s L_T_R %s L_T %s L_W %s" % (leaf_soil_potential_raw, leaf_soil_temp_raw, leaf_soil_temp, leaf_wetness))

                    else:
                        logerr("unknown subtype '%s' in '%s'" % (data_subtype, raw))

            elif data['channel'] == th1_ch or data['channel'] == th2_ch:
                # themro/hygro station
                        # message examples:
                        # TODO
                        # TODO (no sensor)
                if data['channel'] == th1_ch:
                    data['bat_th_1'] = battery_low
                else:
                    data['bat_th_2'] = battery_low
                dbg_parse(2, "data from thermo/hygro channel: %s, raw message: %s" % (data['channel'], raw))
                # TODO find out message protocol thermo/hygro station

            else:
                logerr("unknown station with channel: %s, raw message: %s" % (data['channel'], raw))
        elif parts[0] in '#':
            loginf("%s" % raw)
        else:
            logerr("unknown sensor identifier '%s' in '%s'" % (parts[0], raw))
        return data


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
        settings['port'] = self._prompt('port', Meteostick.DEFAULT_PORT)
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
                      default=Meteostick.DEFAULT_PORT)
    parser.add_option('--baud', dest='baud', metavar='BAUDRATE',
                      help='serial port baud rate',
                      default=Meteostick.DEFAULT_BAUDRATE)
    parser.add_option('--freq', dest='freq', metavar='FREQUENCY',
                      help='comm frequency, either US (915MHz) or EU (868MHz)',
                      default=Meteostick.DEFAULT_FREQUENCY)
    parser.add_option('--rfs', dest='rfs', metavar='RF_SENSITIVITY',
                      help='RF sensitivity in dB',
                      default=Meteostick.DEFAULT_RF_SENSITIVITY)
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

    with Meteostick(port=opts.port, baudrate=opts.baud,
                    transceiver_frequency=opts.freq,
                    iss_channel=int(opts.c_iss),
                    anemometer_channel=int(opts.c_a),
                    leaf_soil_channel=int(opts.c_ls),
                    temp_hum_1_channel=int(opts.c_th1),
                    temp_hum_2_channel=int(opts.c_th2),
                    rf_sensitivity=int(opts.rfs)) as s:
        while True:
            print time.time(), s.get_readings()
