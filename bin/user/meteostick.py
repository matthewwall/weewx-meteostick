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

from __future__ import with_statement
import serial
import syslog
import time

import weewx
import weewx.drivers

DRIVER_NAME = 'Meteostick'
DRIVER_VERSION = '0.10'

DEBUG_SERIAL = 0
DEBUG_RAIN = 0
DEBUG_PARSE = 0


def loader(config_dict, _):
    return MeteostickDriver(**config_dict[DRIVER_NAME])


def confeditor_loader():
    return MeteostickConfEditor()


def logmsg(level, msg):
    syslog.syslog(level, 'meteostick: %s' % msg)


def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)


def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)


def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)


class MeteostickDriver(weewx.drivers.AbstractDevice):
    DEFAULT_PORT = '/dev/ttyUSB0'
    DEFAULT_BAUDRATE = 115200
    DEFAULT_FREQUENCY = 'EU'
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
        'rf_signal': 'rxCheckPercent',
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
        'bat_soil_leaf': 'rainBatteryStatus',
        'bat_th_1': 'outTempBatteryStatus',
        'bat_th_2': 'inTempBatteryStatus'}

    def __init__(self, **stn_dict):
        loginf('driver version is %s' % DRIVER_VERSION)
        port = stn_dict.get('port', self.DEFAULT_PORT)
        baudrate = stn_dict.get('baudrate', self.DEFAULT_BAUDRATE)
        freq = stn_dict.get('transceiver_frequency', self.DEFAULT_FREQUENCY)
        self.iss_channel = int(stn_dict.get('iss_channel', 1))
        anemometer_channel = int(stn_dict.get('anemometer_channel', 0))
        leaf_soil_channel = int(stn_dict.get('leaf_soil_channel', 0))
        self.temp_hum_1_channel = int(stn_dict.get('temp_hum_1_channel', 0))
        self.temp_hum_2_channel = int(stn_dict.get('temp_hum_2_channel', 0))
        transmitters = Meteostick.ch_to_xmit(
            self.iss_channel, anemometer_channel, leaf_soil_channel,
            self.temp_hum_1_channel, self.temp_hum_2_channel)
        rain_bucket_type = int(stn_dict.get('rain_bucket_type', 0))
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

        loginf('using serial port %s' % port)
        loginf('using baudrate %s' % baudrate)
        loginf('using frequency %s' % freq)
        loginf('using iss_channel %s' % self.iss_channel)
        loginf('using anemometer_channel %s' % anemometer_channel)
        loginf('using leaf_soil_channel %s' % leaf_soil_channel)
        loginf('using temp_hum_1_channel %s' % self.temp_hum_1_channel)
        loginf('using temp_hum_2_channel %s' % self.temp_hum_2_channel)
        loginf('using rain_bucket_type %s' % rain_bucket_type)
        loginf('using transmitters %02x' % transmitters)
        loginf('sensor map is: %s' % self.sensor_map)

        self.station = Meteostick(port, baudrate, transmitters, freq)
        self.station.open()

    def closePort(self):
        if self.station is not None:
            self.station.close()
            self.station = None

    @property
    def hardware_name(self):
        return 'Meteostick'

    def genLoopPackets(self):
        self.station.configure()

        while True:
            readings = self.station.get_readings_with_retry(self.max_tries,
                                                            self.retry_wait)
            if len(readings) > 0:
                if DEBUG_PARSE:
                    logdbg("readings: %s" % readings)
                data = Meteostick.parse_readings(
                    readings, self.iss_channel,
                    self.temp_hum_1_channel, self.temp_hum_2_channel)
                if data:
                    if DEBUG_PARSE:
                        logdbg("data: %s" % data)
                    packet = self._data_to_packet(data)
                    if DEBUG_PARSE:
                        logdbg("packet: %s" % packet)
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
                rain_count += 128
            self.last_rain_count = packet['rain']
            packet['rain'] = rain_count * self.rain_per_tip # mm
            if DEBUG_RAIN:
                logdbg("rain=%s rain_count=%s last_rain_count=%s" %
                       (packet['rain'], rain_count, self.last_rain_count))
        return packet


class Meteostick(object):
    def __init__(self, port, baudrate, transmitters, frequency):
        self.port = port
        self.baudrate = baudrate
        self.transmitters = transmitters
        self.frequency = frequency
        self.timeout = 3 # seconds
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
        buf = buf.strip()
        return buf

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

    @staticmethod
    def parse_readings(raw, iss_channel=0, th1_channel=0, th2_channel=0):
        data = dict()
        if not raw:
            return data
        parts = raw.split(' ')
        n = len(parts)
        if DEBUG_PARSE > 2:
            logdbg("parts: %s (%s)" % (parts, n))
        try:
            if parts[0] == 'B':
                if n >= 3:
                    data['in_temp'] = float(parts[1]) # C
                    data['pressure'] = float(parts[2]) # hPa
                else:
                    loginf("B: not enough parts (%s) in '%s'" % (n, raw))
            elif parts[0] in 'WT':
                if n >= 5:
                    data['rf_signal'] = float(parts[4])
                    bat = 1 if n >= 6 and parts[5] == 'L' else 0
                    if parts[0] == 'W':
                        if iss_channel != 0 and int(parts[1]) == iss_channel:
                            data['bat_iss'] = bat
                        else:
                            data['bat_anemometer'] = bat
                        data['wind_speed'] = float(parts[2]) # m/s
                        data['wind_dir'] = float(parts[3]) # degrees
                    elif parts[0] == 'T':
                        if th1_channel != 0 and int(parts[1]) == th1_channel:
                            data['bat_th_1'] = bat
                            data['temp_1'] = float(parts[2]) # C
                            data['humid_1'] = float(parts[3]) # %
                        elif th2_channel != 0 and int(parts[1]) == th2_channel:
                            data['bat_th_2'] = bat
                            data['temp_2'] = float(parts[2]) # C
                            data['humid_2'] = float(parts[3]) # %
                        else:
                            data['bat_iss'] = bat
                            data['temperature'] = float(parts[2]) # C
                            data['humidity'] = float(parts[3]) # %
                else:
                    loginf("WT: not enough parts (%s) in '%s'" % (n, raw))
            elif parts[0] in 'LMO':
                if n >= 5:
                    data['rf_signal'] = float(parts[4])
                    data['bat_soil_leaf'] = 1 if n >= 6 and parts[5] == 'L' else 0
                    if parts[0] == 'L':
                        data['leaf_wetness_%s' % parts[2]] = float(parts[3]) # 0-15
                    elif parts[0] == 'M':
                        data['soil_moisture_%s' % parts[2]] = float(parts[3]) # cbar 0-200
                    elif parts[0] == 'O':
                        data['soil_temp_%s' % parts[2]] = float(parts[3])  # C
                else:
                    loginf("LMO: not enough parts (%s) in '%s'" % (n, raw))
            elif parts[0] in 'RSUP':
                if n >= 4:
                    data['rf_signal'] = float(parts[3])
                    data['bat_iss'] = 1 if n >= 5 and parts[4] == 'L' else 0
                    if parts[0] == 'R':
                        data['rain_count'] = float(parts[2])  # 0-255
                    elif parts[0] == 'S':
                        data['solar_radiation'] = float(parts[2])  # W/m^2
                    elif parts[0] == 'U':
                        data['uv'] = float(parts[2])
                    elif parts[0] == 'P':
                        data['solar_power'] = float(parts[2])  # 0-100
                else:
                    loginf("RSUP: not enough parts (%s) in '%s'" % (n, raw))
            elif parts[0] in '#':
                loginf("%s" % raw)
            else:
                logerr("unknown sensor identifier '%s' in '%s'" %
                       (parts[0], raw))
        except ValueError, e:
            logerr("parse failed for '%s': %s" % (raw, e))
        return data

    def configure(self):
        # in logger mode, station sends records continuously
        if DEBUG_SERIAL:
            logdbg("set station to logger mode")

        # Send a reset command
        command = 'r\n'
        self.serial_port.write(command)
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
        loginf("cmd: '%s': %s" % (command, response.split('\n')[0]))
        if DEBUG_SERIAL > 2:
            logdbg("full response to reset: %s" % response)
        # Discard any serial input from the device
        time.sleep(0.2)
        self.serial_port.flushInput()

        # Set device to listen to configured transmitters
        self.send_command('t' + str(self.transmitters) + '\r')

        # Set to filter transmissions from anything other than transmitter 1
        self.send_command('f1\r')

        # Set device to produce machine readable data
        self.send_command('o1\r')

        # Set device to use the right frequency
        command = 'm0\r' if self.frequency == 'US' else 'm1\r'
        self.send_command(command)

        # From now on the device will produce lines with received data

    def send_command(self, cmd):
        self.serial_port.write(cmd)
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

    # Radio frequency to use between USB transceiver and console: US or EU
    # US uses 915 MHz, EU uses 868.3 MHz
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
    rain_bucket_type = 0

    # The driver to use
    driver = user.meteostick

"""

    def prompt_for_settings(self):
        settings = dict()
        print "Specify the serial port on which the meteostick is connected,"
        print "for example /dev/ttyUSB0 or /dev/ttyS0"
        settings['port'] = self._prompt('port', MeteostickDriver.DEFAULT_PORT)
        print "Specify the frequency between the station and the meteostick,"
        print "either US (915 MHz) or EU (868.3 MHz)"
        settings['transceiver_frequency'] = self._prompt('frequency', 'EU', ['US', 'EU'])
        print "Specify the type of the rain bucket,"
        print "either 0 (0.01 inches per tip) or 1 (0.2 mm per tip)"
        settings['rain_bucket_type'] = self._prompt('rain_bucket_type', 1)
        print "Specify the channel of the ISS (1-8)"
        settings['iss_channel'] = self._prompt('iss_channel', 1)
        print "Specify the channel of the Anemometer Transmitter Kit if any (0=none; 1-8)"
        settings['anemometer_channel'] = self._prompt('anemometer_channel', 0)
        print "Specify the channel of the Leaf & Soil station if any (0=none; 1-8)"
        settings['leaf_soil_channel'] = self._prompt('leaf_soil_channel', 0)
        print "Specify the channel of the first Temp/Humidity station if any (0=none; 1-8)"
        settings['temp_hum_1_channel'] = self._prompt('temp_hum_1_channel', 0)
        print "Specify the channel of the second Temp/Humidity station if any (0=none; 1-8)"
        settings['temp_hum_2_channel'] = self._prompt('temp_hum_2_channel', 0)
        return settings


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
    parser.add_option('--baud', dest='baudrate', metavar='BAUDRATE',
                      help='serial port baud rate',
                      default=MeteostickDriver.DEFAULT_BAUDRATE)
    parser.add_option('--freq', dest='frequency', metavar='FREQUENCY',
                      help='comm frequency, either US (915MHz) or EU (868MHz)',
                      default=MeteostickDriver.DEFAULT_FREQUENCY)
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
    (options, args) = parser.parse_args()

    if options.version:
        print "meteostick driver version %s" % DRIVER_VERSION
        exit(0)

    xmitters = Meteostick.ch_to_xmit(
        int(options.c_iss), int(options.c_a), int(options.c_ls),
        int(options.c_th1), int(options.c_th2))

    with Meteostick(options.port, options.baudrate, xmitters,
                    options.frequency) as s:
        while True:
            print time.time(), s.get_readings()
