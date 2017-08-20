/*
  Arduino Weather Station
  By: Radek Kaczorek, December 19th, 2016
  Based on example by: Nathan Seidle, SparkFun Electronics, November 16th, 2013
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Much of this is based on Mike Grusin's USB Weather Board code: https://www.sparkfun.com/products/10586

  This code reads all the various sensors (wind speed, direction, rain gauge, humidty, pressure, light, batt_lvl, gps latitude, gps longitude, gps altitude, sat no, gps date, gps time)
  and reports it over the serial com port.

  This code assumes the GP-635T GPS module is attached
*/

#include <Wire.h> //I2C needed for sensors
#include "MPL3115A2.h" //Pressure sensor
#include "HTU21D.h" //Humidity sensor
#include <SoftwareSerial.h> //Needed for GPS and WiFi
#include <TinyGPS++.h> //GPS parsing
#include <math.h>

// GPS
TinyGPSPlus gps;
static const int RXPin = 5, TXPin = 4; //GPS is attached to pin 4(TX from GPS) and pin 5(RX into GPS)
SoftwareSerial gpsSerial(RXPin, TXPin);

MPL3115A2 myPressure; //Create an instance of the pressure sensor
HTU21D myHumidity; //Create an instance of the humidity sensor

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// digital I/O pins
const byte WSPEED = 3;
const byte RAIN = 2;
const byte STAT1 = 8; // Green status LED - weather data
const byte STAT2 = 7; // Blue status LED - GPS fix
const byte GPS_PWRCTL = 6; //Pulling this pin low puts GPS to sleep but maintains RTC and RAM

// analog I/O pins
const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1;
const byte BATT = A2;
const byte WDIR = A0;

// update time in seconds
const byte UPDATE = 30;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSecond = 0; //The millis counter to see when a second rolls by
byte seconds = 0;

long lastWindCheck = 0;

// volatiles are subject to modification by IRQs
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;
volatile float rainFall = 0;
volatile unsigned long raintime, rainlast, raininterval;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

  if (raininterval > 10) // ignore switch-bounce glitches less than 10ms after initial edge
  {
    //Each dump is 0.011" / 0.2794 mm of water
    rainFall += 0.2794; //Increase amount of rain
    rainlast = raintime; // set up for next event
  }
}

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second = 1.492MPH * 1609.34/3600 = 0.666982022 m/s for each click per second
  }
}

void setup()
{
  Serial.begin(115200);
  gpsSerial.begin(9600);

  pinMode(STAT1, OUTPUT); //Status LED Green - Weather data refresh
  pinMode(STAT2, OUTPUT); //Status LED Blue - GPS fix

  pinMode(GPS_PWRCTL, OUTPUT);
  digitalWrite(GPS_PWRCTL, HIGH); //Pulling this pin low puts GPS to sleep but maintains RTC and RAM

  pinMode(WSPEED, INPUT_PULLUP); // input from windspeed sensor
  pinMode(RAIN, INPUT_PULLUP); // input from rain gauge sensor

  pinMode(REFERENCE_3V3, INPUT);
  pinMode(LIGHT, INPUT);

  //Configure the pressure sensor
  myPressure.begin(); // Get sensor online
  myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
  myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
  myPressure.enableEventFlags(); // Enable all three pressure and temp event flags

  //Configure the humidity sensor
  myHumidity.begin();

  lastSecond = millis();

  // attach external interrupt pins to IRQ functions
  attachInterrupt(0, rainIRQ, FALLING);
  attachInterrupt(1, wspeedIRQ, FALLING);

  // turn on interrupts
  interrupts();
  
  Serial.println("Arduino Personal Weather Station");
}

void loop()
{
  //Keep track of which minute it is
  if (millis() - lastSecond >= 1000)
  {
    digitalWrite(STAT1, HIGH); //Green stat LED

    if (get_battery_level() > 3.5 && gps.location.age() > 3600000) //Wake up GPS from sleep every hour if battery level is OK
    {
      digitalWrite(GPS_PWRCTL, HIGH);
    }

    if (gps.location.isUpdated())
    {
      digitalWrite(STAT2, LOW); //Turn off Blue stat LED if GPS is fixed
      digitalWrite(GPS_PWRCTL, LOW); //Put GPS to sleep when fixed and wake up every hour
    } else {
      digitalWrite(STAT2, HIGH); //Turn on Blue stat LED if GPS is looking for fix
    }

    lastSecond += 1000;

    //Get all sensors readings every 10 seconds
    if (++seconds == UPDATE)
    {
      seconds = 0;
      getSensors();
      if (digitalRead(GPS_PWRCTL))
         getGPS(1000); //Wait 1 second, and gather GPS data
    }

    // Blink every loop
    digitalWrite(STAT1, LOW); //Turn off Green stat LED
    digitalWrite(STAT2, LOW); //Turn off Blue stat LED

  }
}

//While we delay for a given amount of time, gather GPS data
static void getGPS(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}


//Calculates each of the variables
void getSensors()
{
  int winddir = 0; // [0-360 instantaneous wind direction]
  float windspeedmps = 0; // [m/s instantaneous wind speed]
  float humidity = 0; // [%]
  float tempc = 0; // [temperature C]
  float pressure = 0;
  float rainmm = 0; // rain mm from last report
  float dewptc; // [dewpoint C]
  float batt_lvl = 11.8; //[analog value from 0 to 1023]
  float light_lvl = 455; //[analog value from 0 to 1023]

  //Calc winddir
  winddir = get_wind_direction();

  //Calc windspeed
  windspeedmps = get_wind_speed();

  //Calc humidity
  humidity = myHumidity.readHumidity();

  //Calc temp from pressure sensor
  tempc = myPressure.readTemp();

  //Calc rainmm and reset rainFall
  rainmm = rainFall;
  rainFall = 0;

  //Calc pressure
  pressure = myPressure.readPressure();
  pressure /= 100;

  //Calc dewptc
  dewptc = pow(humidity/100,0.125) * (112 + (0.9 * tempc)) + (0.1 * tempc) - 112;
  
  //Calc light level
  light_lvl = get_light_level();

  //Calc battery level
  batt_lvl = get_battery_level();

  // Print to serial
  Serial.print("$,WindDir=");
  Serial.print(winddir);
  Serial.print(",WindSpeed=");
  Serial.print(windspeedmps, 1);
  Serial.print(",Humidity=");
  Serial.print(humidity, 1);
  Serial.print(",Temp=");
  Serial.print(tempc, 1);
  Serial.print(",Rain=");
  Serial.print(rainmm, 4);
  Serial.print(",Pressure=");
  Serial.print(pressure, 2);
  Serial.print(",DewPoint=");
  Serial.print(dewptc, 2);
  Serial.print(",Light=");
  Serial.print(light_lvl, 2);
  Serial.print(",Latitude=");
  Serial.print(gps.location.lat(), 6);
  Serial.print(",Longitude=");
  Serial.print(gps.location.lng(), 6);
  Serial.print(",Altitude=");
  Serial.print(gps.altitude.meters());
  Serial.print(",Satellites=");
  Serial.print(gps.satellites.value());

  char sz[32];
  Serial.print(",FixDate=");
  sprintf(sz, "%02d/%02d/%02d", gps.date.day(), gps.date.month(), gps.date.year());
  Serial.print(sz);

  Serial.print(",FixTime=");
  sprintf(sz, "%02d:%02d:%02d", gps.time.hour(), gps.time.minute(), gps.time.second());
  Serial.print(sz);

  Serial.print(",Battery=");
  Serial.print(batt_lvl, 2);

  Serial.print(",");
  Serial.println("#");
}

//Returns the voltage of the light sensor based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
float get_light_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float lightSensor = analogRead(LIGHT);
  operatingVoltage = 3.3 / operatingVoltage; //The reference voltage is 3.3V
  lightSensor = operatingVoltage * lightSensor;
  return (lightSensor);
}

//Returns the voltage of the raw pin based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
//Battery level is connected to the RAW pin on Arduino and is fed through two 5% resistors:
//3.9K on the high side (R1), and 1K on the low side (R2)
float get_battery_level()
{
  float operatingVoltage = analogRead(REFERENCE_3V3);
  float rawVoltage = analogRead(BATT);
  operatingVoltage = 3.30 / operatingVoltage; //The reference voltage is 3.3V
  rawVoltage = operatingVoltage * rawVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin
  
  //voltage divider on weather shield
  // rawVoltage *= 4.90; //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage

  // voltage divider MOD
  // 1) On Weather Shield: cut the track from original voltage divider to A2 pin (bootom layer)
  // 2) Solar Charger Shield: connect voltage divider to A2 pin (top layer - R6 soldering field from R5 resistor)
  
  //voltage divider on solar charger shield
  rawVoltage *= 1.74; // multiple BATT voltage by the voltage divider to get actual system voltage  
  
  return (rawVoltage);
}

//Returns the instataneous wind speed
float get_wind_speed()
{
  float deltaTime = millis() - lastWindCheck;
  deltaTime /= 1000.0; //Convert milliseconds to seconds
  float windSpeed = (float)windClicks / deltaTime;
  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();
  windSpeed *= (1.492 * 1609.34 / 3600.0); //1.492 * 1609.34 / 3600 = 0.666982022 m/s
  return (windSpeed);
}

//Read the wind direction sensor, return heading in degrees
int get_wind_direction()
{
  unsigned int adc;
  adc = analogRead(WDIR); // get the current reading from the sensor
  
  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.
  if (adc < 380) return (113);
  if (adc < 393) return (68);
  if (adc < 414) return (90);
  if (adc < 456) return (158);
  if (adc < 508) return (135);
  if (adc < 551) return (203);
  if (adc < 615) return (180);
  if (adc < 680) return (23);
  if (adc < 746) return (45);
  if (adc < 801) return (248);
  if (adc < 833) return (225);
  if (adc < 878) return (338);
  if (adc < 913) return (0);
  if (adc < 940) return (293);
  if (adc < 967) return (315);
  if (adc < 990) return (270);
  return (-1); // error, disconnected?
}

