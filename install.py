# installer for meteostick driver
# Copyright 2016 Matthew Wall

from setup import ExtensionInstaller

def loader():
    return MeteostickInstaller()

class MeteostickInstaller(ExtensionInstaller):
    def __init__(self):
        super(MeteostickInstaller, self).__init__(
            version="0.6",
            name='meteostick',
            description='Collect data from meteostick via serial port',
            author="Matthew Wall",
            author_email="mwall@users.sourceforge.net",
            config={
                'Station': {
                    'station_type': ''},
                'Meteostick': {
                    'driver': 'user.meteostick',
                    'port': '/dev/ttyUSB0',
                    'transceiver_frequency': 'EU',
                    'rain_bucket_type': '1',
                    'iss_channel': '1',
                    'anemometer_channel': '0',
                    'leaf_soil_channel': '0',
                    'temp_hum_1_channel': '0',
                    'temp_hum_2_channel': '0'}},
            files=[('bin/user', ['bin/user/meteostick.py'])]
            )
