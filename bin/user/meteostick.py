#!/usr/bin/env python
# Copyright 2016 Matthew Wall, Luc Heijst

# TODO: fix error Unresolfed reference 'DEFAULT_PORT' in parser.add_option('--port'

from __future__ import with_statement
import serial
import syslog
import time

import weewx
import weewx.restx
import weewx.units
import weewx.wxformulas
import weewx.drivers

DRIVER_NAME = 'Meteostick'
DRIVER_VERSION = '0.6'

DEBUG_SERIAL = 0


def loader(config_dict, _):
    return MeteostickDriver(**config_dict[DRIVER_NAME])


def confeditor_loader():
    return MeteostickConfEditor()


def logmsg(level, msg):
    syslog.syslog(level, 'meteostick: %s' % msg)


def logdbg(msg):
    # lh temporary changed: logmsg(syslog.LOG_DEBUG, msg)
    logmsg(syslog.LOG_INFO, msg)


def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)


def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)


class MeteostickDriver(weewx.drivers.AbstractDevice):
    DEFAULT_PORT = '/dev/ttyUSB0'
    DEFAULT_BAUDRATE = 115200
    DEFAULT_TRANSMITTERS = 1
    DEFAULT_MAP = {
        'pressure': 'pressure',
        'in_temp': 'inTemp',
        'wind_speed': 'windSpeed',
        'wind_dir': 'windDir',
        'temperature': 'outTemp',
        'humidity': 'outHumidity',
        'rain_counter': 'rain',
        'solar_radiation': 'radiation',
        'uv': 'UV',
        'battery': 'txBatteryStatus',
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
        'extra_temp_1': 'extraTemp1',
        'extra_temp_2': 'extraTemp2',
        'extra_humid_1': 'extraHumid1',
        'extra_humid_2': 'extraHumid2'
    }

    def __init__(self, **stn_dict):
        loginf('driver version is %s' % DRIVER_VERSION)
        self.port = stn_dict.get('port', self.DEFAULT_PORT)
        self.baudrate = stn_dict.get('baudrate', self.DEFAULT_BAUDRATE)
        self.frequency = stn_dict.get('transceiver_frequency', 'EU')
        self.transmitters = 0
        self.iss_channel = int(stn_dict.get('iss_channel', 1))
        self.transmitters += 1 << (self.iss_channel - 1)
        self.anemometer_channel = int(stn_dict.get('anemometer_channel', 0))
        if self.anemometer_channel != 0:
            self.transmitters += 1 << (self.anemometer_channel - 1)
        self.leaf_soil_channel = int(stn_dict.get('leaf_soil_channel', 0))
        if self.leaf_soil_channel != 0:
            self.transmitters += 1 << (self.leaf_soil_channel - 1)
        self.temp_hum_1_channel = int(stn_dict.get('temp_hum_1_channel', 0))
        if self.temp_hum_1_channel != 0:
            self.transmitters += 1 << (self.temp_hum_1_channel - 1)
        self.temp_hum_2_channel = int(stn_dict.get('temp_hum_2_channel', 0))
        if self.temp_hum_2_channel != 0:
            self.transmitters += 1 << (self.temp_hum_2_channel - 1)
        self.rain_bucket_type = int(stn_dict.get('rain_bucket_type', 0))
        self.obs_map = stn_dict.get('map', self.DEFAULT_MAP)
        if self.rain_bucket_type == 0:
            self.rain_per_tip = 0.254  # mm (0.01 inch)
        else:
            self.rain_per_tip = 0.2  # mm

        self.max_tries = int(stn_dict.get('max_tries', 10))
        self.retry_wait = int(stn_dict.get('retry_wait', 10))
        self.last_rain_counter = None

        global DEBUG_SERIAL
        DEBUG_SERIAL = int(stn_dict.get('debug_serial', DEBUG_SERIAL))

        loginf('using serial port %s' % self.port)
        loginf('using baudrate %s' % self.baudrate)
        loginf('using frequency %s' % self.frequency)
        loginf('using iss_channel %s' % self.iss_channel)
        loginf('using anemometer_channel %s' % self.anemometer_channel)
        loginf('using leaf_soil_channel %s' % self.leaf_soil_channel)
        loginf('using temp_hum_1_channel %s' % self.temp_hum_1_channel)
        loginf('using temp_hum_2_channel %s' % self.temp_hum_2_channel)
        loginf('using rain_bucket_type %s' % self.rain_bucket_type)
        loginf('using transmitters %02x' % self.transmitters)

        self.station = Meteostick(self.port, self.baudrate, self.transmitters, self.frequency, self.temp_hum_1_channel,
                                  self.temp_hum_2_channel)
        self.station.open()

    def closePort(self):
        if self.station is not None:
            self.station.close()
            self.station = None

    @property
    def hardware_name(self):
        return 'Meteostick'

    def genLoopPackets(self):
        self.station.set_logger_mode()

        while True:
            readings = self.station.get_readings_with_retry(self.max_tries, self.retry_wait)
            if len(readings) > 0:
                if DEBUG_SERIAL > 0:
                    logdbg("readings = %s" % readings)
                data = Meteostick.parse_readings(readings, self.station.get_temp_hum_1_sensor(),
                                                 self.station.get_temp_hum_2_sensor())
                if data:
                    if DEBUG_SERIAL > 1:
                        logdbg("data = %s" % data)
                    packet = self._data_to_packet(data)
                    if DEBUG_SERIAL > 0:
                        logdbg("packet = %s" % packet)
                    yield packet

    def _data_to_packet(self, data):
        packet = {'dateTime': int(time.time() + 0.5),
                  'usUnits': weewx.METRICWX}
        if DEBUG_SERIAL > 2:
            logdbg("self.obs_map = %s, data = %s" % (self.obs_map, data))
        for k in data:
            if DEBUG_SERIAL > 2:
                logdbg("k = %s, data[k] = %s" % (k, data[k]))
            if k in self.obs_map:
                packet[self.obs_map[k]] = data[k]
                if self.obs_map[k] == 'rain':
                    if self.last_rain_counter is not None:
                        rain_count = packet['rain'] - self.last_rain_counter
                    else:
                        rain_count = 0
                    # Take care for the rain counter wrap around from 255 to 0
                    if rain_count < 0:
                        rain_count += 256
                    self.last_rain_counter = packet['rain']
                    packet['rain'] = rain_count * self.rain_per_tip  # rain in mm
                    if DEBUG_SERIAL > 2:
                        logdbg("last_rain_counter = %s, packet['rain'] = %s" % (self.last_rain_counter, packet['rain']))
            else:
                if DEBUG_SERIAL > 0:
                    loginf("self.obs_map has no key %s" % k)
        return packet


class Meteostick(object):
    def __init__(self, port, baudrate, transmitters, frequency, temp_hum_1_channel, temp_hum_2_channel):
        self.port = port
        self.baudrate = baudrate
        self.transmitters = transmitters
        self.frequency = frequency
        self.temp_hum_1_channel = temp_hum_1_channel
        self.temp_hum_2_channel = temp_hum_2_channel
        self.timeout = 3  # seconds
        self.serial_port = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _, value, traceback):
        self.close()

    def open(self):
        logdbg("open serial port %s" % self.port)
        self.serial_port = serial.Serial(self.port, self.baudrate, timeout=self.timeout)

    def close(self):
        if self.serial_port is not None:
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

    def get_temp_hum_1_sensor(self):
        return self.temp_hum_1_channel

    def get_temp_hum_2_sensor(self):
        return self.temp_hum_2_channel

    @staticmethod
    def parse_readings(raw, temp_hum_1_channel, temp_hum_2_channel):
        parts = raw.split(' ')
        number_of_parts = len(parts)
        if number_of_parts > 1:
            if DEBUG_SERIAL > 2:
                logdbg("line '%s' has %s parts" % (raw, number_of_parts))
            data = dict()
            if DEBUG_SERIAL > 2:
                logdbg("parts = %s" % parts)
            if parts[0] == 'B':
                if number_of_parts >= 3:
                    data['in_temp'] = float(parts[1])  # C
                    data['pressure'] = float(parts[2])  # hPa
                else:
                    loginf("line '%s' has too few parts (%s)" % (raw, number_of_parts))
            elif parts[0] in 'WTLMO':
                if number_of_parts >= 5:
                    data['rf_signal'] = float(parts[4])
                    if number_of_parts == 5:
                        data['battery'] = 0
                    else:
                        data['battery'] = 1 if parts[5] == 'L' else 0
                    if parts[0] == 'W':
                        data['wind_speed'] = float(parts[2])  # m/s
                        data['wind_dir'] = float(parts[3])  # degrees
                    elif parts[0] == 'T':
                        if int(parts[1]) == temp_hum_1_channel:
                            data['extra_temp_1'] = float(parts[2])  # C
                            data['extra_humid_1'] = float(parts[3])  # %
                        elif int(parts[1]) == temp_hum_2_channel:
                            data['extra_temp_2'] = float(parts[2])  # C
                            data['extra_humid_2'] = float(parts[3])  # %
                        else:
                            data['temperature'] = float(parts[2])  # C
                            data['humidity'] = float(parts[3])  # %
                    elif parts[0] == 'L':
                        data['leaf_wetness_%s' % parts[2]] = float(parts[3])  # 0-15
                    elif parts[0] == 'M':
                        data['soil_moisture_%s' % parts[2]] = float(parts[3])  # cbar 0-200
                    elif parts[0] == 'O':
                        data['soil_temp_%s' % parts[2]] = float(parts[3])  # C
                else:
                    loginf("line '%s' has too few parts (%s)" % (raw, number_of_parts))
            elif parts[0] in 'RSUP':
                if number_of_parts >= 4:
                    data['rf_signal'] = float(parts[3])
                    if number_of_parts == 4:
                        data['battery'] = 0
                    else:
                        data['battery'] = 1 if parts[4] == 'L' else 0
                    if parts[0] == 'R':
                        data['rain_counter'] = float(parts[2])  # 0-255
                    elif parts[0] == 'S':
                        data['solar_radiation'] = float(parts[2])  # W/m^2
                    elif parts[0] == 'U':
                        data['uv'] = float(parts[2])
                    elif parts[0] == 'P':
                        data['solar_power'] = float(parts[2])  # 0-100
                else:
                    loginf("line '%s' has too few parts (%s)" % (raw, number_of_parts))
            elif parts[0] in '#':
                loginf("info message: %s" % raw)
            else:
                loginf("unknown sensor identifier '%s' in line '%s'" % (parts[0], raw))
            return data

    def set_logger_mode(self):
        # in logger mode, station sends logger mode records continuously
        if DEBUG_SERIAL > 1:
            logdbg("set station to logger mode")
        command = 'r\n'
        self.serial_port.write(command)  # This is the reset command
        # Wait until we see the ? character
        ready = False
        response = ""
        while not ready:
            time.sleep(0.1)
            while self.serial_port.inWaiting() > 0:
                response = self.serial_port.read(1)
                if response == '?':
                    ready = True
            response += response
        if DEBUG_SERIAL > 2:
            logdbg("Logger reacted on command '%s' with: %s" % (command, response))
        time.sleep(0.2)
        self.serial_port.flushInput()

        # Set device to listen to configured transmitters
        # Then discard any serial input from the device
        command = 't' + self.transmitters + '\r'
        self.serial_port.write(command)
        time.sleep(0.2)
        response = self.serial_port.read(self.serial_port.inWaiting())
        if DEBUG_SERIAL > 2:
            logdbg("Logger reacted on '%s' with; %s" % (command, response))
        self.serial_port.flushInput()

        # Set device to filter out transmissions from anything other than transmitter 1
        # Then discard any serial input from the device
        command = 'f1\r'
        self.serial_port.write(command)
        time.sleep(0.2)
        response = self.serial_port.read(self.serial_port.inWaiting())
        if DEBUG_SERIAL > 2:
            logdbg("Logger reacted on '%s' with; %s" % (command, response))
        self.serial_port.flushInput()

        # Set device to produce machine readable data
        # Then discard any serial input from the device
        command = 'o1\r'
        self.serial_port.write(command)
        time.sleep(0.2)
        response = self.serial_port.read(self.serial_port.inWaiting())
        if DEBUG_SERIAL > 2:
            logdbg("Logger reacted on '%s' with; %s" % (command, response))
        self.serial_port.flushInput()

        # Set device to use the right frequency
        # Then discard any serial input from the device
        if self.frequency == 'US':
            # Set device to listen on US frequencies (915 MHz)
            command = 'm0\r'
        else:
            # Set device to listen on european frequencies (868 MHz)
            command = 'm1\r'
        self.serial_port.write(command)
        time.sleep(0.2)
        response = self.serial_port.read(self.serial_port.inWaiting())
        if DEBUG_SERIAL > 2:
            logdbg("Logger reacted on '%s' with; %s" % (command, response))
        self.serial_port.flushInput()
        # From now on the device will produce lines with received data
        # Ignore data of first line (may not be complete)


class MeteostickConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[Meteostick]
    # This section is for the Meteostick USB receiver.

    # A serial port must be specified:
    port = /dev/ttyUSB0

    # Radio frequency to use between USB transceiver and console: US or EU
    # US uses 915 MHz, EU uses 868.3 MHz.  Default is EU.
    transceiver_frequency = EU

    # Used channels: 0=not present, 1-8)
    # The channel of the Vantage Vue ISS or Vantage Pro or Pro2 ISS
    iss_channel = 1
    # The values below only apply for Vantage Pro or Pro2
    anemometer_channel = 0
    leaf_soil_channel = 0
    temp_hum_1_channel = 0
    temp_hum_2_channel = 0
    # rain bucket type (0: 0.01 inch, 1: 0.2 mm)
    rain_bucket_type = 0

    # Print debug messages
    #  0=no logging; 1=minimum logging; 2=normal logging; 3=detailed logging
    debug_serial = 2

    # The driver to use:
    driver = user.meteostick

    # The mapping between the read values and the weewx database fields
    [[map]]
        pressure        =  pressure
        in_temp         =  inTemp
        wind_speed      =  windSpeed
        wind_dir        =  windDir
        temperature     =  outTemp
        humidity        =  outHumidity
        rain_counter    =  rain
        solar_radiation =  radiation
        uv              =  UV
        battery         =  txBatteryStatus
        rf_signal       =  rxCheckPercent
        solar_power     =  extraTemp3
        soil_temp_1     =  soilTemp1
        soil_temp_2     =  soilTemp2
        soil_temp_3     =  soilTemp3
        soil_temp_4     =  soilTemp4
        soil_moisture_1 =  soilMoist1
        soil_moisture_2 =  soilMoist2
        soil_moisture_3 =  soilMoist3
        soil_moisture_4 =  soilMoist4
        leaf_wetness_1  =  leafWet1
        leaf_wetness_2  =  leafWet2
        extra_temp_1    =  extraTemp1
        extra_temp_2    =  extraTemp2
        extra_humid_1   =  extraHumid1
        extra_humid_2   =  extraHumid2
"""

    def prompt_for_settings(self):
        settings = dict()
        print "Specify the serial port on which the meteostick is connected, for"
        print "example /dev/ttyUSB0 or /dev/ttyS0"
        settings['port'] = self._prompt('port', MeteostickDriver.DEFAULT_PORT)
        print "Specify the frequency used between the station and the"
        print "meteostick, either 'US' (915 MHz) or 'EU' (868.3 MHz)"
        settings['transceiver_frequency'] = self._prompt('frequency', 'EU', ['US', 'EU'])
        print "Specify the type of the rain bucket, either 0: '0.01 inches' or 1: '0.2 mm'"
        settings['rain_bucket_type'] = self._prompt('rain-bucket-type', 1)
        print "Specify the channel of the ISS (1-8)"
        settings['iss_channel'] = self._prompt('ISS-channel', 1)
        print "Specify the channel of the Anemometer Transmitter Kit if any (0=none; 1-8)"
        settings['anemometer_channel'] = self._prompt('Anemometer-channel', 0)
        print "Specify the channel of the Leaf & Soil station if any (0=none; 1-8)"
        settings['leaf_soil_channel'] = self._prompt('Leaf-soil-channel', 0)
        print "Specify the channel of the first extra Temp/Humidity station if any (0=none; 1-8)"
        settings['temp_hum_1_channel'] = self._prompt('Temp-hum-1-channel', 0)
        print "Specify the channel of the second extra Temp/Humidity station if any (0=none; 1-8)"
        settings['temp_hum_2_channel'] = self._prompt('Temp-hum-2-channel', 0)
        return settings


# define a main entry point for basic testing of the station without weewx
# engine and service overhead.  invoke this as follows from the weewx root dir:
#
# PYTHONPATH=bin python bin/user.meteostick.py

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
    (options, args) = parser.parse_args()

    if options.version:
        print "meteostick driver version %s" % DRIVER_VERSION
        exit(0)

    with Meteostick(options.port, options.baudrate, options.transmitters, options.frequency,
                    options.temp_hum_1_channel, options.temp_hum_2_channel) as s:
        while True:
            print time.time(), s.get_readings()
