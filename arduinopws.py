#!/usr/bin/env python
#
# Copyright 2016 Radek Kaczorek <rkaczorek AT gmail DOT com>
# Based on Ultimeter driver
#
# Credit to and contributions to original Ultimeter driver:
#  Matthew Wall
#  Nate Bargmann <n0nb@n0nb.us>
#  Jay Nugent (WB8TKL) and KRK6 for weather-2.kr6k-V2.1
#  Steve (sesykes71) for testing the first implementations of this driver
#  Garret Power for improved decoding and proper handling of negative values
#  Chris Thompstone for testing the fast-read implementation
#
# See the file LICENSE.txt for your rights.

"""Driver for Arduino Personal Weather Station using:
- Arduino Uno
- Sparkfun Weather Shield
- XM-15 bluetooth module connected to Arduino HW serial
- Arduino sketch hosted on github (https://github.com/rkaczorek/arduino-pws)

The driver communicates to Sparkfun Weather Shield over bluetooth serial port
Port parameters are 9600, 8N1, with no flow control.
"""

from __future__ import with_statement
import serial
import syslog
import time

import weewx.drivers
from weewx.units import INHG_PER_MBAR, MILE_PER_KM
from weeutil.weeutil import timestamp_to_string

DRIVER_NAME = 'ArduinoPWS'
DRIVER_VERSION = '0.1'

DEBUG_SERIAL = 1

def loader(config_dict, _):
    return ArduinoPWS(**config_dict[DRIVER_NAME])

def confeditor_loader():
    return ArduinoPWSConfEditor()


DEFAULT_PORT = '/dev/rfcomm0'

def logmsg(level, msg):
    syslog.syslog(level, 'ArduinoPWS: %s' % msg)

def logdbg(msg):
    logmsg(syslog.LOG_DEBUG, msg)

def loginf(msg):
    logmsg(syslog.LOG_INFO, msg)

def logerr(msg):
    logmsg(syslog.LOG_ERR, msg)


class ArduinoPWS(weewx.drivers.AbstractDevice):
    def __init__(self, **stn_dict):
        self.model = stn_dict.get('model', 'ArduinoPWS')
        self.port = stn_dict.get('port', DEFAULT_PORT)
        self.max_tries = int(stn_dict.get('max_tries', 10))
        self.retry_wait = int(stn_dict.get('retry_wait', 10))
        self.last_rain = None

        global DEBUG_SERIAL
        DEBUG_SERIAL = int(stn_dict.get('debug_serial', DEBUG_SERIAL))

        loginf('driver version is %s' % DRIVER_VERSION)
        loginf('using serial port %s' % self.port)
        self.station = Station(self.port)
        self.station.open()

    def closePort(self):
        if self.station is not None:
            self.station.close()
            self.station = None

    @property
    def hardware_name(self):
        return self.model

    def genLoopPackets(self):
        while True:
            packet = {'dateTime': int(time.time() + 0.5),
                      'usUnits': weewx.METRICWX}
            readings = self.station.get_readings_with_retry(self.max_tries, self.retry_wait)
            data = Station.parse_readings(readings)
            packet.update(data)
            yield packet

class Station(object):
    def __init__(self, port):
        self.port = port
        self.baudrate = 9600
        self.timeout = 10 # seconds
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

#    def get_time(self):
#        try:
#            buf = self.get_readings_with_retry()
#            data = Station.parse_readings(buf)
#            d = data['day_of_year']  # seems to start at 0
#            m = data['minute_of_day']  # 0 is midnight before start of day
#            tt = time.localtime()
#            y = tt.tm_year
#            s = tt.tm_sec
#            ts = time.mktime((y,1,1,0,0,s,0,0,-1)) + d * 86400 + m * 60
#            logdbg("station time: day:%s min:%s (%s)" %
#                   (d, m, timestamp_to_string(ts)))
#            return ts
#        except (serial.serialutil.SerialException, weewx.WeeWxIOError), e:
#            logerr("get_time failed: %s" % e)
#        return int(time.time())

    def get_readings(self):
        buf = self.serial_port.readline()
        if DEBUG_SERIAL:
            logdbg("station said: %s" %
                   ' '.join(["%0.2X" % ord(c) for c in buf]))
        buf = buf.strip()
        return buf

    def validate_string(self, buf):
        if len(buf) > 245:
            raise weewx.WeeWxIOError("Unexpected buffer length %d" % len(buf))
        if buf.split(",")[0] != '$':
            raise weewx.WeeWxIOError("Unexpected header bytes '%s'" % buf.split(",")[0])
        return buf

    def get_readings_with_retry(self, max_tries=5, retry_wait=10):
        for ntries in range(0, max_tries):
            try:
                buf = self.get_readings()
                self.validate_string(buf)
                return buf
            except (serial.serialutil.SerialException, weewx.WeeWxIOError), e:
                loginf("Failed attempt %d of %d to get readings: %s" %
                       (ntries + 1, max_tries, e))
                time.sleep(retry_wait)
        else:
            msg = "Max retries (%d) exceeded for readings" % max_tries
            logerr(msg)
            raise weewx.RetriesExceeded(msg)

    @staticmethod
    def parse_readings(raw):
        """Arduino Personal Weather Station emit data in the following format:

        $,winddir=-1,windspeedmps=0.0,humidity=48.5,tempc=20.3,rainmm=0.00,dailyrainmm=0.00,pressure=1024.33,dewptc=9.05,light_lvl=0.00,
        latitude=0.000000,longitude=0.000000,altitude=0.00,sats=0,date=28/12/2016,time=20:27:26,batt_lvl=4.25,#

        Each line has 240 characters - 1 header byte, x data bytes, 1 trailer byte and a carriage return and line feed (new line)

        winddir=338			-	wind direction (0-255)
        windspeedmps=0.0	-	wind speed (0.1 m/s)
        humidity=48.5		-	humidity (0.1 %)
        tempc=20.3			-	outdoor temperature (0.1 C)
        rainmm=0.00			-	rain (0.01 mm per hour)
        dailyrainmm=0.00	-	daily rain (0.01 mm per day)
        pressure=1024.33	-	pressure (0.1 hPa)
        dewptc=9.05			-	dew point (0.01 C)
        light_lvl=0.00		-	light level (??)
        latitude=0.000000	-	GPS latitude (0.000001 deg)
        longitude=0.000000	-	GPS longitude (0.000001 deg)
        altitude=0.00		-	GPS altitude (0.01 m)
        sats=0				-	GPS number of visible sats (0-n)
        date=28/12/2016		-	date (DD/MM/YYYY)
        time=20:27:26		-	time (HH:MM:SS)
        batt_lvl=4.25		-	baterry level (0.01 V)
        """
        data = dict()
        data['windDir'] = Station._decode(raw, 1)  # compass deg
        data['windSpeed'] = Station._decode(raw, 2)  # m/s
        data['outHumidity'] = Station._decode(raw, 3)  # percent
        data['outTemp'] = Station._decode(raw, 4)  # degree_C
        data['hourRain'] = Station._decode(raw, 5)  # mm/h
        data['dayRain'] = Station._decode(raw, 6)  # mm/h
        data['pressure'] = Station._decode(raw, 7)  # mbar
        data['dewpoint'] = Station._decode(raw, 8)  # degree_C
        data['radiation'] = Station._decode(raw, 9)  # W/m2
        data['gpsSats'] = Station._decode(raw, 13)  # none

        # don't collect GPS data if no satellites available
        if data['gpsSats'] > 0:
          data['gpsLatitude'] = Station._decode(raw, 10)  # degree
          data['gpsLongitude'] = Station._decode(raw, 11)  # degree
          data['gpsAltitude'] = Station._decode(raw, 12)  # meters
          # let's make a date ;-)
          DateTime = Station._decode(raw, 14) + ' ' + Station._decode(raw, 15)
          DateTimeFormat = '%d/%m/%Y %H:%M:%S'
          try:
            td = int(time.mktime(time.strptime(DateTime, DateTimeFormat)))  # unix epoch
            data['gpsDateTime'] = td
          except ValueError:
            logdbg("No GPS time data available")

        data['supplyVoltage'] = Station._decode(raw, 16)  # volt

        return data

    @staticmethod
    def _decode(s, index):
        v = None
        try:
            sensors = s.split(",")
            if index <= len(sensors) - 1 and len(sensors[index].split("=")) == 2:
               sensor = sensors[index].split("=")[1] # return only value at given index
               if index == 14 or index == 15:
                 v = sensor
               else:
                 v = float(sensor)
        except ValueError, e:
            logdbg("decode failed for '%s': %s" % (s, e))
        return v


class ArduinoPWSConfEditor(weewx.drivers.AbstractConfEditor):
    @property
    def default_stanza(self):
        return """
[ArduinoPWS]
    # This section is for the Arduino Personal Weather Station.

    # Serial port such as /dev/rfcomm0 (default) or /dev/ttyUSB0
    port = /dev/rfcomm0

    # The driver to use:
    driver = weewx.drivers.arduinopws
"""

    def prompt_for_settings(self):
        print "Specify the serial port on which the station is connected, for"
        print "example /dev/rfcomm0 or /dev/ttyUSB0"
        port = self._prompt('port', '/dev/rfcomm0')
        return {'port': port}


# define a main entry point for basic testing of the station without weewx
# engine and service overhead.  invoke this as follows from the weewx root dir:
# PYTHONPATH=bin python bin/weewx/drivers/arduinopws.py

if __name__ == '__main__':
    import optparse

    usage = """%prog [options] [--help]"""

    syslog.openlog('arduinopws', syslog.LOG_PID | syslog.LOG_CONS)
    syslog.setlogmask(syslog.LOG_UPTO(syslog.LOG_DEBUG))
    parser = optparse.OptionParser(usage=usage)
    parser.add_option('--version', dest='version', action='store_true',
                      help='display driver version')
    parser.add_option('--port', dest='port', metavar='PORT',
                      help='serial port to which the station is connected',
                      default=DEFAULT_PORT)
    (options, args) = parser.parse_args()

    if options.version:
        print "arduinopws driver version %s" % DRIVER_VERSION
        exit(0)

    with Station(options.port) as s:
        while True:
            print time.time(), s.get_readings()
