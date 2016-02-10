# weewx-meteostick

Read data from a meteostick via serial connection.

1) download the extension

wget https://github.com/matthewwall/weewx-meteostick/archive/master.zip

2) install the extension

sudo wee_extension --install weewx-meteostick-master.zip

3) configure weewx to use the driver

sudo wee_config

4) start weewx

sudo /etc/init.d/weewx start
