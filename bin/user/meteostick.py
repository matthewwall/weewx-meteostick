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
import string
import syslog
import time

import weewx
import weewx.drivers
import weewx.engine
import weewx.wxformulas
from weewx.crc16 import crc16

DRIVER_NAME = 'Meteostick'
DRIVER_VERSION = '0.40lh'

MPH_TO_MPS = 0.44704 # mph to m/s

DEBUG_SERIAL = 0
DEBUG_RAIN = 0
DEBUG_PARSE = 0
DEBUG_RFS = 0

def loader(config_dict, engine):
    return MeteostickService(engine, config_dict)

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
# Also used to normalize raw values for a stanard temperature.
DEFAULT_SOIL_TEMP = 24 # C

RAW = 0  # indices of table with raw values
POT = 1  # indices of table with potentials

# Lookup table for soil_moisture_raw values to get a soil_moisture value based upon a linear formula
# Correction factor = 0.009
SM_MAP = {RAW: ( 99.2, 140.1, 218.7, 226.9, 266.8, 391.7, 475.6, 538.2, 596.1, 673.7, 720.1),
          POT: (  0.0,   1.0,   9.0,  10.0,  15.0,  35.0,  55.0,  75.0, 100.0, 150.0, 200.0)}

# Lookup table for leaf_wetness_raw values to get a leaf_wetness value based upon a linear formula
# Correction factor = 0.0
LW_MAP = {RAW: (857.0, 864.0, 895.0, 911.0, 940.0, 952.0, 991.0, 1013.0),
          POT: ( 15.0,  14.0,   5.0,   4.0,   3.0,   2.0,   1.0,    0.0)}


def calculate_leaf_soil_temp(leaf_soil_temp_raw):
    """ Decode the raw leaf-soil temperature, then calculate the actual leaf-soil
    temperature and the leaf_soil potential, using Davis' formulas.
    When the sensor is not populated, leaf_soil_temp_raw and
    leaf_soil_potential_raw are set to their max values (0x3ff).
    see: https://github.com/cmatteri/CC1101-Weather-Receiver/wiki/Soil-Moisture-Station-Protocol
    :param leaf_soil_temp_raw: raw value of sensor temp of both leaf wetness and soil moisture sensors.
    """

    # Convert leaf_soil_temp_raw to a resistance (R) in kiloOhms
    a = 18.81099
    b = 0.0009988027
    r = a / (1.0 / leaf_soil_temp_raw - b) / 1000 # k ohms

    # Steinhart-Hart parameters
    s1 = 0.002783573
    s2 = 0.0002509406
    try:
        leaf_soil_temp = 1 / (s1 + s2 * math.log(r)) - 273
        dbg_parse(3, 'r (k ohm) %s leaf_soil_temp_raw %s leaf_soil_temp %s' %
                  (r, leaf_soil_temp_raw, leaf_soil_temp))
        return leaf_soil_temp
    except ValueError, e:
        logerr('leaf_soil_temp failed for leaf_soil_temp_raw %s r (k ohm) %s error: %s' %
               (leaf_soil_temp_raw, r, e))
    return DEFAULT_SOIL_TEMP

def lookup_potential(sensor_name, norm_fact, sensor_raw, sensor_temp, lookup):
    """Look up potential based upon a normalized raw value (i.e. temp corrected for DEFAULT_SOIL_TEMP)
    and a linear function between two points in the lookup table.
    :param lookup: a table with both sensor_raw_norm values and corresponding potential values;
    the table is composed for a specific norm-factor.
    :param sensor_temp: sensor temp in C
    :param sensor_raw: sensor raw potential value
    :param norm_fact: temp correction factor for normalizing sensor-raw values
    :param sensor_name: string used in debug messages
    """

    # normalize raw value for standard temperature (DEFAULT_SOIL_TEMP)
    sensor_raw_norm = sensor_raw * (1 + norm_fact * (sensor_temp - DEFAULT_SOIL_TEMP))

    numcols = len(lookup[RAW])
    if sensor_raw_norm >= lookup[RAW][numcols-1]:
        potential = lookup[POT][numcols-1] # preset potential to last value
        dbg_parse(2, "%s: temp=%s fact=%s raw=%s norm=%s potential=%s >= RAW=%s" %
                  (sensor_name, sensor_temp, norm_fact, sensor_raw, sensor_raw_norm, potential,
                   lookup[RAW][numcols-1]))
    else:
        potential = lookup[POT][0] # preset potential to first value
        # lookup sensor_raw_norm value in table
        for x in range(0, numcols):
            if sensor_raw_norm < lookup[RAW][x]:
                if x == 0:
                    # 'pre zero' phase; potential = first value
                    dbg_parse(2, "%s: temp=%s fact=%s raw=%s norm=%s potential=%s < RAW=%s" %
                              (sensor_name, sensor_temp, norm_fact, sensor_raw, sensor_raw_norm, potential,
                               lookup[RAW][0]))
                    break
                else:
                    # determine the potential value
                    potential_per_raw =  (lookup[POT][x] - lookup[POT][x - 1]) / (lookup[RAW][x] - lookup[RAW][x - 1])
                    potential_offset = (sensor_raw_norm - lookup[RAW][x - 1]) * potential_per_raw
                    potential = lookup[POT][x - 1] + potential_offset
                    dbg_parse(2, "%s: temp=%s fact=%s raw=%s norm=%s potential=%s RAW=%s to %s POT=%s to %s " %
                              (sensor_name, sensor_temp, norm_fact, sensor_raw, sensor_raw_norm, potential,
                               lookup[RAW][x - 1], lookup[RAW][x], lookup[POT][x - 1], lookup[POT][x]))
                    break
    return potential


RAW_CHANNEL = 0  # unused channel for the receiver stats in raw format
MACHINE_CHANNEL = 9  # fake channel for the receiver stats in machine format


class MeteostickDriver(weewx.drivers.AbstractDevice):
    NUM_CHAN = 10 # 8 channels, one fake channel (9), one unused channel (0)
    DEFAULT_RAIN_BUCKET_TYPE = 1
    DEFAULT_SENSOR_MAP = {
        'pressure': 'pressure',
        'in_temp': 'inTemp',  # temperature inside meteostick
        'wind_speed': 'windSpeed',
        'wind_dir': 'windDir',
        'temperature': 'outTemp',
        'humidity': 'outHumidity',
        'rain_count': 'rain',
        # When field rainRate is used, specify in section [StdWXCalculate]
        # rainRate = hardware; option 'prefer_hardware' won't work with partial loop packets
        # leaving field rainRate out is the same as option 'software'
        # 'rain_rate': 'rainRate',
        'solar_radiation': 'radiation',
        'uv': 'UV',
        'pct_good': 'rxCheckPercent',
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
        'bat_th_2': 'inTempBatteryStatus',
        'solar_power': 'referenceVoltage',
        'supercap_volt': 'supplyVoltage'}

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
        self.station.reset()
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
                dbg_parse(2, "data: %s" % data)
                packet = self._data_to_packet(data)
                if packet is not None:
                    dbg_parse(2, "packet: %s" % packet)
                    yield packet

    def _data_to_packet(self, data):
        packet = {'dateTime': int(time.time() + 0.5),
                  'usUnits': weewx.METRICWX}
        # map sensor observations to database field names
        for k in data:
            if k in self.sensor_map:
                packet[self.sensor_map[k]] = data[k]
        if len(packet) <= 3:
            # No data found for loop packet, only the initial dateTime and
            # usUnits and the batteryStatus of the non-present sensor
            dbg_parse(2, "skip loop packet for data packet: %s" % data)
            return None
        # convert the rain count to a rain delta measure
        if 'rain_count' in data:
            if self.last_rain_count is not None:
                rain_count = data['rain_count'] - self.last_rain_count
            else:
                rain_count = 0
            # handle rain counter wrap around from 127 to 0
            if rain_count < 0:
                loginf("rain counter wraparound detected rain_count=%s" %
                       rain_count)
                rain_count += 128
            self.last_rain_count = data['rain_count']
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
                        int(0.5 + 100.0 * self.rf_stats['cnt'][ch] /
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


class MeteostickService(MeteostickDriver, weewx.engine.StdService):
    """Weewx service for the Vantage weather stations read by a meteostick."""

    def __init__(self, engine, config_dict):
        MeteostickDriver.__init__(self, **config_dict[DRIVER_NAME])
        weewx.engine.StdService.__init__(self, engine, config_dict)

        self.bind(weewx.NEW_ARCHIVE_RECORD, self.new_archive_record)

    def new_archive_record(self, event):
        self._update_rf_summaries()  # calculate rf summaries
        event.record['rxCheckPercent'] = self.rf_stats['pctgood'][self.station.channels['iss']]
        if DEBUG_RFS:
            self._report_rf_stats()
        else:
            logdbg("data['rxCheckPercent']: %s" % event.record['rxCheckPercent'])
        self._init_rf_stats()  # flush rf statistics


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

    def reset(self, max_wait=30):
        """Reset the device, leaving it in a state that we can talk to it."""
        loginf("establish communication with the meteostick")

        # flush any previous data in the input buffer
        self.serial_port.flushInput()

        # Send a reset command
        self.serial_port.write('r\n')
        # Wait until we see the ? character
        start_ts = time.time()
        ready = False
        response = ''
        while not ready:
            time.sleep(0.1)
            while self.serial_port.inWaiting() > 0:
                c = self.serial_port.read(1)
                if c == '?':
                    ready = True
                elif c in string.printable:
                    response += c
            if time.time() - start_ts > max_wait:
                raise weewx.WakeupError("No 'ready' response from meteostick after %s seconds" % max_wait)
        loginf("reset: %s" % response.split('\n')[0])
        dbg_serial(2, "full response to reset: %s" % response)
        # Discard any serial input from the device
        time.sleep(0.2)
        self.serial_port.flushInput()
        return response

    def configure(self):
        """Configure the device to send data continuously."""
        loginf("configure meteostick to logger mode")

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
        dbg_parse(2, "readings: %s" % raw)
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
        # message example:
        # W 3 0.00 261 -53 L
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
                    if data['wind_speed'] < 0.0001:
                        data['wind_dir'] = None
                    else:
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
                    # bug: the zero values of not connected leaf wetness sensors are also given
                    data['leaf_wetness_%s' % parts[2]] = float(parts[3]) # 0-15
                    dbg_parse(2, "leaf_wetness_%s=%s" % (parts[2], float(parts[3])))
                elif parts[0] == 'M':
                    data['soil_moisture_%s' % parts[2]] = float(parts[3]) # cbar 0-200
                    dbg_parse(2, "soil_moisture_%s=%s" % (parts[2], float(parts[3])))
                elif parts[0] == 'O':
                    data['soil_temp_%s' % parts[2]] = float(parts[3])  # C
                    # When connected to ports 1 and 2 the temp sensor is also used
                    # for the leaf wetness temperatures
                    if int(parts[2]) <= 2:
                        data['leaf_temp_%s' % parts[2]] = float(parts[3])  # C
                        dbg_parse(2, "soil- and leaf_temp_%s=%s" % (parts[2], float(parts[3])))
                    else:
                        dbg_parse(2, "soil_temp_%s=%s" % (parts[2], float(parts[3])))
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
                    elif rain_count > 128:
                        # rain_count == 128 is no sensor
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
            # raw Davis sensor message in 8 byte format incl header and
            # additional info
            # message example:
            #       ---- raw message ----  rfs ts_last
            # I 102 51 0 DB FF 73 0 11 41  -65 5249944 202
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
            if data['rf_missed'] > 0:
                dbg_parse(3, "channel %s missed %s" %
                          (data['channel'], data['rf_missed']))

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
                    formula the Vantage type must be known.
                    For now we use the traditional 'pro' formula for all
                    wind directions.
                    """
                    dbg_parse(2, "wind_speed_raw=%03x wind_dir_raw=0x%03x" %
                              (wind_speed_raw, wind_dir_raw))
                    data['wind_speed'] = wind_speed_raw * MPH_TO_MPS
                    ws = wind_speed_raw # mph
                    # Vantage Pro and Pro2
                    wind_dir_pro = 9.0 + wind_dir_raw * 342.0 / 255.0
                    # Vantage Vue (maybe also for newer Pro 2)
                    wind_dir_vue = wind_dir_raw * 1.40625 + 0.3
                    data['wind_dir'] = wind_dir_pro
                    dbg_parse(2, "WS=%s WD=%s WD_vue=%s" %
                              (ws, wind_dir_pro, wind_dir_vue))

                # data from both iss sensors and extra sensors on
                # Anemometer Transport Kit
                message_type = (pkt[0] >> 4 & 0xF)
                if message_type == 2:
                    # supercap voltage (Vue only) max: 0x3FF (1023)
                    # message example:
                    # I 103 20 4 C3 D4 C1 81 89 EE  -77 2562520 -70
                    """When the raw values are divided by 300 the maximum
                    voltage of the super capacitor will be about 2.8 V. This
                    is close to its maximum operating voltage of 2.7 V
                    """
                    supercap_volt_raw = ((pkt[3] << 2) + (pkt[4] >> 6)) & 0x3FF
                    if supercap_volt_raw != 0x3FF:
                        data['supercap_volt'] = supercap_volt_raw / 300.0
                        dbg_parse(2, "supercap_volt_raw=0x%03x value=%s" %
                                  (supercap_volt_raw, data['supercap_volt']))
                elif message_type == 3:
                    # unknown message type
                    # message examples:
                    # TODO
                    # TODO (no sensor)
                    dbg_parse(1, "unknown message with type=0x03; "
                              "pkt[3]=0x%02x pkt[4]=0x%02x pkt[5]=0x%02x"
                              % (pkt[3], pkt[4], pkt[5]))
                elif message_type == 4:
                    # uv
                    # message examples:
                    # I 103 40 00 00 12 45 00 B5 2A  -78 2562444 -24
                    # I 103 41 0 DE FF C3 0 A9 8D  -65 2624976 -38 (no sensor)
                    uv_raw = ((pkt[3] << 2) + (pkt[4] >> 6)) & 0x3FF
                    if uv_raw != 0x3FF:
                        data['uv'] = uv_raw / 50.0
                        dbg_parse(2, "uv_raw=%04x value=%s" %
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
                    dbg_parse(2, "time_between_tips_raw=%03x (%s)" %
                              (time_between_tips_raw, time_between_tips_raw))
                    sensor_present = (pkt[4] & 0x7) == 0x5
                    if sensor_present:
                        if time_between_tips_raw == 0x3FF:
                            # no rain
                            data['rain_rate'] = 0
                            dbg_parse(3, "no_rain=%s mm/h" % data['rain_rate'])
                        else:
                            if pkt[4] & 0x40 == 0:
                                # heavy rain. typical value:
                                # 64/16 - 1020/16 = 4 - 63.8 (180.0 - 11.1 mm/h)
                                time_between_tips = time_between_tips_raw / 16.0  # convert to a real
                                data['rain_rate'] = 3600.0 / time_between_tips * rain_per_tip # mm/h
                                dbg_parse(2, "heavy_rain=%s mm/h, time_between_tips=%s s" %
                                          (data['rain_rate'], time_between_tips))
                            else:
                                # light rain. typical value:
                                # 64 - 1022 (11.1 - 0.8 mm/h)
                                time_between_tips = time_between_tips_raw * 1.0  # convert to a real
                                data['rain_rate'] = 3600.0 / time_between_tips * rain_per_tip # mm/h
                                dbg_parse(2, "light_rain=%s mm/h, time_between_tips=%s s" %
                                          (data['rain_rate'], time_between_tips))
                elif message_type == 6:
                    # solar radiation
                    # message examples
                    # I 104 61 0 DB 0 43 0 F4 3B  -66 2624972 121
                    # I 104 60 0 0 FF C5 0 79 DA  -77 2562444 137 (no sensor)
                    sr_raw = ((pkt[3] << 2) + (pkt[4] >> 6)) & 0x3FF
                    if sr_raw != 0x3FF:
                        data['solar_radiation'] = sr_raw * 1.757936
                        dbg_parse(2, "solar_radiation_raw=0x%04x value=%s"
                                  % (sr_raw, data['solar_radiation']))
                elif message_type == 7:
                    # solar cell output / solar power (Vue only)
                    # message example:
                    # I 102 70 1 F5 CE 43 86 58 E2  -77 2562532 173
                    """When the raw values are divided by 300 the voltage comes
                    in the range of 2.8-3.3 V measured by the machine readable
                    format
                    """
                    solar_power_raw = ((pkt[3] << 2) + (pkt[4] >> 6)) & 0x3FF
                    if solar_power_raw != 0x3FF:
                        data['solar_power'] = solar_power_raw / 300.0
                        dbg_parse(2, "solar_power_raw=0x%03x solar_power=%s"
                                  % (solar_power_raw, data['solar_power']))
                elif message_type == 8:
                    # outside temperature
                    # message examples:
                    # I 103 80 0 0 33 8D 0 25 11  -78 2562444 -25
                    # I 104 81 0 DB FF C3 0 AB F8  -66 2624980 125 (no sensor)
                    temp_f_raw = (pkt[3] << 4) + (pkt[4] >> 4)
                    if temp_f_raw != 0xFFC:
                        temp_f = temp_f_raw / 10.0
                        data['temperature'] = weewx.wxformulas.FtoC(temp_f) # C
                        dbg_parse(2, "temp_f_raw=0x%03x temp_f=%s temp_c=%s"
                                  % (temp_f_raw, temp_f, data['temperature']))
                elif message_type == 9:
                    # 10-min average wind gust
                    # message examples:
                    # I 102 91 0 DB 0 3 E 89 85  -66 2624972 204
                    # I 102 90 0 0 0 5 0 31 51  -75 2562456 223 (no sensor)
                    gust_raw = pkt[3]  # mph
                    gust_index_raw = pkt[5] >> 4
                    if not(gust_raw == 0 and gust_index_raw == 0):
                        dbg_parse(2, "W10=%s gust_index_raw=%s" %
                                  (gust_raw, gust_index_raw))
                        # don't store the 10-min gust data because there is no
                        # field for it reserved in the standard wview schema
                elif message_type == 0xA:
                    # humidity
                    # message examples:
                    # I 101 A0 0 0 C9 3D 0 2A 87  -76 2562432 54
                    # I 100 A1 0 DB 0 3 0 47 C7  -67 5249932 -130 (no sensor)
                    humidity_raw = ((pkt[4] >> 4) << 8) + pkt[3]
                    if humidity_raw != 0:
                        data['humidity'] = humidity_raw / 10.0
                        dbg_parse(2, "humidity_raw=0x%03x value=%s" %
                                  (humidity_raw, data['humidity']))
                elif message_type == 0xC:
                    # unknown ATK message
                    # message example:
                    # I 101 C1 4 D0 0 1 0 E9 A4  -69 2624968 56
                    # As we have seen after one day of received data
                    # pkt[3] and pkt[5] are always zero; pckt[4] has values 0-3
                    dbg_parse(3, "unknown_atk pkt[3]=0x%02x pkt[4]=0x%02x pkt[5]=0x%02x" %
                              (pkt[3], pkt[4], pkt[5]))
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
                        dbg_parse(2, "rain_count_raw=0x%02x value=%s" %
                                  (rain_count_raw, rain_count))
                else:
                    # unknown message type
                    logerr("unknown message type 0x%01x" % message_type)

            elif data['channel'] == ls_ch:
                # leaf and soil station
                data['bat_leaf_soil'] = battery_low
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
                        if pkt[3] != 0xFF:
                            # soil moisture temperature
                            leaf_soil_temp = calculate_leaf_soil_temp(leaf_soil_temp_raw)
                            dbg_parse(2, "soil_temp_%s_raw=0x%03x (%s)" %
                                      (sensor_num, leaf_soil_temp, leaf_soil_temp))
                            data['soil_temp_%s' % sensor_num] = leaf_soil_temp # C
                        if pkt[2] != 0xFF:
                            # soil moisture potential
                            # Lookup soil moisture potential in SM_MAP (correction factor = 0.009)
                            norm_fact = 0.009  # Normalize potential_raw for standard temp
                            soil_moisture = lookup_potential("soil_moisture", norm_fact,
                                                             leaf_soil_potential_raw, leaf_soil_temp, SM_MAP)
                            data['soil_moisture_%s' % sensor_num] = soil_moisture

                    elif data_subtype == 2:
                        # leaf wetness
                        # message examples:
                        # I 100 F2 A D4 55 80 0 90 6  -53 2687516 -121
                        # I 101 F2 2A 0 FF 40 C0 4F 5  -52 2687404 43 (no sensor)
                        if pkt[3] != 0xFF:
                            # leaf wetness temperature
                            leaf_soil_temp = calculate_leaf_soil_temp(leaf_soil_temp_raw)
                            dbg_parse(2, "leaf_temp_%s_raw=0x%03x (%s)" %
                                      (sensor_num, leaf_soil_temp, leaf_soil_temp))
                            data['leaf_temp_%s' % sensor_num] = leaf_soil_temp # C
                        if pkt[2] != 0:
                            # leaf wetness potential
                            # Lookup leaf wetness potential in LW_MAP (correction factor = 0.0)
                            norm_fact = 0.0  # Don't normalize potential_raw for standard temp
                            leaf_wetness = lookup_potential("leaf_wetness", norm_fact,
                                                            leaf_soil_potential_raw, leaf_soil_temp, LW_MAP)
                            data['leaf_wetness_%s' % sensor_num] = leaf_wetness

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
                # TODO find out message protocol of thermo/hygro station

            else:
                logerr("unknown station with channel: %s, raw message: %s" %
                       (data['channel'], raw))
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
