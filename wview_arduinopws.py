#
#    Copyright (c) 2016 Radek Kaczorek <rkaczorek@gmail.com>
#
#    See the file LICENSE.txt for your full rights.
#
"""The wview schema, which is also used by weewx."""

schema = [('dateTime',             'INTEGER NOT NULL UNIQUE PRIMARY KEY'),
          ('usUnits',              'INTEGER NOT NULL'),
          ('interval',             'INTEGER NOT NULL'),
          ('barometer',            'REAL'),
          ('pressure',             'REAL'),
          ('altimeter',            'REAL'),
          ('outTemp',              'REAL'),
          ('outHumidity',          'REAL'),
          ('windSpeed',            'REAL'),
          ('windDir',              'REAL'),
          ('windGust',             'REAL'),
          ('windGustDir',          'REAL'),
          ('rain',                 'REAL'),
          ('rainRate',             'REAL'),
          ('dewpoint',             'REAL'),
          ('windchill',            'REAL'),
          ('heatindex',            'REAL'),
          ('ET',                   'REAL'),
          ('radiation',            'REAL'),
          ('supplyVoltage',        'REAL'),
          ('gpsLatitude',          'REAL'),
          ('gpsLongitude',         'REAL'),
          ('gpsAltitude',          'REAL'),
          ('gpsSats',              'REAL'),
          ('gpsDateTime',          'REAL')]
