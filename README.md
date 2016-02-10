# weewx-meteostick

Read data from a meteostick via serial connection.

The meteostick must be in 'Computed Values' mode.  Send it the o1 command to
put it in this mode.  See the Meteostick Manual for instructions.

How to install this driver:

1) download the extension

wget https://github.com/matthewwall/weewx-meteostick/archive/master.zip

2) install the extension

sudo wee_extension --install weewx-meteostick-master.zip

3) configure weewx to use the driver

sudo wee_config

4) start weewx

sudo /etc/init.d/weewx start
