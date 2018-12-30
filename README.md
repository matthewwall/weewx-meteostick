# weewx-meteostick

Read data from a meteostick via serial connection.


Installation

0) Install weewx, select Simulator as the weather station

    http://weewx.com/docs/usersguide.htm

1) Download the driver

    wget -O weewx-meteostick.zip https://github.com/matthewwall/weewx-meteostick/archive/master.zip

2) Install the driver

    sudo wee_extension --install weewx-meteostick.zip

3) Configure the driver

    sudo wee_config --reconfigure

4) Start weewx

    sudo /etc/init.d/weewx start
