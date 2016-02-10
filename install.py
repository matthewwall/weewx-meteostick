# installer for meteostick driver
# Copyright 2016 Matthew Wall

from setup import ExtensionInstaller

def loader():
    return MeteostickInstaller()

class MeteostickInstaller(ExtensionInstaller):
    def __init__(self):
        super(MeteostickInstaller, self).__init__(
            version="0.1",
            name='meteostick',
            description='Collect data from meteostick via serial port',
            author="Matthew Wall",
            author_email="mwall@users.sourceforge.net",
            config={
                'Station': {
                    'station_type': ''},
                'Meteostick': {
                    'driver': 'user.meteostick',
                    'port': '/dev/ttyUSB0'}},
            files=[('bin/user', ['bin/user/meteostick.py'])]
            )
