# What is Arduino Personal Weather Station
ArduinoPWS is a full featured personal weather station build on:
- Arduino Uno / Sparkfun RedBoard
- SpurkFun Weather Shield with GP-645T GPS module
- Weewx weather server

ArduinPWS provides valuable information about weather conditions such as temperature, humidity, dewpoint, pressure, wind speed, wind direction, rain rate, solar radiation, GPS position and time.
Additionally it can provide some basic information about rise, transit and set of the Sun, Moon and all solar planets. A real astronomy geek will appreciate information about current Moon Phase and Polaris Hour Angle.

# How to start?
Running ArduinoPWS is quite simple but requires some linux skills.
1. First put your parts together and attach your weather shield to the main controller.

2. When your're done upload ArduinoPWS.ino sketch to your Arduino Uno / SparkFun RedBoard.
To test weather hardware works as expected, check serial monitor and make sure that your sensors return valid data.
You should see something like this on your serial console.
```
$,WindDir=338,WindSpeed=0.0,Humidity=46.5,Temp=29.7,Rain=0.00,Pressure=1003.94,DewPoint=17.04,Light=1.43,Latitude=0.000000,Longitude=0.000000,Altitude=0.00,Satellites=0,FixDate=00/00/2000,FixTime=00:00:00,Battery=3.94,#
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

8. Copy wview_arduinopws.py to your weewx schemas directory (e.g. /usr/share/weewx/schemas/)

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

