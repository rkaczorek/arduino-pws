# What is Arduino Personal Weather Station
ArduinoPWS is a full featured personal weather station build on:
- Arduino Uno / Sparkfun RedBoard
- SpurkFun Weather Shield with GP-645T GPS module
- Weewx weather server

ArduinPWS provides valuable information about weather conditions such as temperature, humidity, dewpoint, pressure, wind speed, wind direction, rain rate, solar radiation, GPS position and time.
Additionally it can provide some basic information about rise, transit and set of the Sun, Moon and all solar planets. A real astronomy geek will appreciate information about current Polaris Hour Angle.

# How to start?
Running ArduinoPWS is quite simple but requires some linux skills.
1. First put your parts together and ttach your weather shield to the main controller.

2. When your're done with it upload ArduinoPWS.ino sketch to your Arduino Uno / SparkFun RedBoard.
To test weather hardware works as expected, check serial monitor and make sure that your sensors return valid data.
You should see something like this on your serial console.
```
$,winddir=-1,windspeedmps=0.0,humidity=48.5,tempc=20.3,rainmm=0.00,dailyrainmm=0.00,pressure=1024.33,dewptc=9.05,light_lvl=0.00,latitude=0.000000,longitude=0.000000,altitude=0.00,sats=0,date=28/12/2016,time=20:27:26,batt_lvl=4.25,#
```

3. Then connect your Arduino to your weather server running weewx.
You can use USB cable, Bluetooth adapter attached to the hardware serial on your Arduino or anything that forwards serial data to your weather server.
Choose whatever suits you. However if you use anything else than USB cable, make sure to disconnect it before reprogramming your Arduino in the future.
Otherwise you will not be able to upload a sketch to your Arduino.

4. Verify weather serial connection between your weather server and Arduino is operational. You can to this by running:
```
miniterm.py /dev/your_serial_device
```
If everything is ok you should see the results same to running serial monitor (see point 2 above)

5. Install weewx on your weather server.

6. Copy arduinopws.py to your weewx drivers directory (e.g. /usr/share/weewx/weewx/drivers/)

7. Copy extensions.py to your weewx extensions directory (e.g. /usr/share/weewx/user/)

8. Copy wview_with_gps.py to your weewx schemas directory (e.g. /usr/share/weewx/schemas/)

9. Copy and customize weewx configuration file weewx.conf to its destination (e.g. /etc/weewx/)

10. Copy whole ArduinoPWS directory to weewx skin directory (e.g. /etc/weewx/skins/)

11. Start your weewx service by running
```
service weewx start
```

12. If you want to access your weather information with your browser install Apache web server by running
```
sudo apt-get install apache2
```

13. Access your weather information by pointing your browser to the weather server http://YOUR_IP_OR_DOMAIN/weewx

14. Enjoy!

