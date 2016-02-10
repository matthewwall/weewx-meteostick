#!/usr/bin/env python
# Copyright 2016 Matthew Wall, Luc Heijst

# TODO: implement configurator to set params such as t, f, o, m, r
# TODO: figure out how to automatically configure mapping, or at least
#       help a bit in the configurator

from __future__ import with_statement
import serial
import syslog
import time

import weewx.drivers
from weeutil.weeutil import timestamp_to_string

DRIVER_NAME = 'Meteostick'
DRIVER_VERSION = '0.1'

DEBUG_SERIAL = 0

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
    DEFAULT_PORT = '/dev/ttyS0'
    DEFAULT_MAP = {
        'pressure': 'pressure',
        'temperature': 'inTemp',
        'wind_speed_0': 'windSpeed',
        'wind_dir_0': 'windDir',
        'temperature_0': 'outTemp',
        'humidity_0': 'outHumidity',
        'rain_counter_0': 'rain_total',
        'solar_radiation_0': 'radiation',
        'uv_0': 'UV'
    }

    def __init__(self, **stn_dict):
        loginf('driver version is %s' % DRIVER_VERSION)
        self.port = stn_dict.get('port', self.DEFAULT_PORT)
        self.obs_map = stn_dict.get('map', self.DEFAULT_MAP)
        self.rain_per_tip = float(stn_dict.get('rain_per_tip', 0.2)) # mm
        self.max_tries = int(stn_dict.get('max_tries', 10))
        self.retry_wait = int(stn_dict.get('retry_wait', 10))
        self.last_rain = None

        global DEBUG_SERIAL
        DEBUG_SERIAL = int(stn_dict.get('debug_serial', DEBUG_SERIAL))

        loginf('using serial port %s' % self.port)
        self.station = Meteostick(self.port)
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
            readings = self.station.get_readings_with_retry(self.max_tries,
                                                            self.retry_wait)
            data = Meteostick.parse_readings(readings)
            if data:
                packet = self._data_to_packet(data)
                yield packet

    def _data_to_packet(self, data):
        packet = {'dateTime': int(time.time() + 0.5),
                  'usUnits': weewx.METRICWX}
        for k in self.obs_map:
            packet[self.obs_map[k]] = data[k]
            if self.obs_map[k] == 'rain_total':
                packet[self.obs_map[k]] *= self.rain_per_tip
        if 'rain_total' in packet:
            if self.last_rain is not None:
                packet['rain'] = packet['rain_total'] - self.last_rain
            else:
                packet['rain'] = None
            self.last_rain = packet['rain_total']
        return packet


class Meteostick(object):
    def __init__(self, port):
        self.port = port
        self.baudrate = 2400
        self.timeout = 3 # seconds
        self.serial_port = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, _, value, traceback):
        self.close()

    def open(self):
        logdbg("open serial port %s" % self.port)
        self.serial_port = serial.Serial(self.port, self.baudrate,
                                         timeout=self.timeout)

    def close(self):
        if self.serial_port is not None:
            logdbg("close serial port %s" % self.port)
            self.serial_port.close()
            self.serial_port = None

    def get_readings(self):
        buf = self.serial_port.readline()
        if DEBUG_SERIAL:
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
    def parse_readings(raw):
        parts = raw.split(' ')
        data = dict()
        if parts[0] == 'B':
            data['temperature'] = parts[2] # C
            data['pressure'] = parts[3] # hPa
            data['rf_ok'] = parts[4]
            data['rf_fail'] = parts[5]
        elif parts[0] in 'WTRSULMOP':
            data['rf_signal_%s' % parts[1]] = parts[4]
            data['battery_%s' % parts[1]] = 1 if parts[5] == 'L' else 0
            if parts[0] == 'W':
                data['wind_speed_%s' % parts[1]] = parts[2] # m/s
                data['wind_dir_%s' % parts[1]] = parts[3] # degrees
            elif parts[0] == 'T':
                data['temperature_%s' % parts[1]] = parts[2] # C
                data['humidity_%s' % parts[1]] = parts[3] # %
            elif parts[0] == 'R':
                data['rain_counter_%s' % parts[1]] = parts[2] # 0-255
            elif parts[0] == 'S':
                data['solar_radiation_%s' % parts[1]] = parts[2] # W/m^2
            elif parts[0] == 'U':
                data['uv_%s' % parts[1]] = parts[2]
            elif parts[0] == 'L':
                data['leaf_wetness_%s_%s' % (parts[2], parts[1])] = parts[3] # 0-15
            elif parts[0] == 'M':
                data['soil_moisture_%s_%s' % (parts[2], parts[1])] = parts[3] # cbar 0-200
            elif parts[0] == 'O':
                data['temperature_%s_%s' % (parts[2], parts[1])] = parts[3] # C
            elif parts[0] == 'P':
                data['solar_power_%s' % parts[1]] = parts[2] # 0-100
        else:
            logerr("unknown sensor identifier '%s'" % parts[0])
        return data


class MeteostickConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[Meteostick]
    # This section is for the Meteostick

    # Serial port such as /dev/ttyS0, /dev/ttyUSB0, or /dev/cuaU0
    port = %s

    # The driver to use
    driver = user.meteostick

    # Mapping between sensor readings and database schema
    [[map]]
%s
""" % (MeteostickDriver.DEFAULT_PORT, "\n".join(["        %s = %s" % (x, MeteostickDriver.DEFAULT_MAP[x]) for x in MeteostickDriver.DEFAULT_MAP]))

    def prompt_for_settings(self):
        print "Specify the serial port on which the station is connected, for"
        print "example /dev/ttyUSB0 or /dev/ttyS0."
        port = self._prompt('port', MeteostickDriver.DEFAULT_PORT)
        return {'port': port}


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
                      default=Meteostick.DEFAULT_PORT)
    (options, args) = parser.parse_args()

    if options.version:
        print "meteostick driver version %s" % DRIVER_VERSION
        exit(0)

    with Meteostick(options.port) as s:
        while True:
            print time.time(), s.get_readings()
