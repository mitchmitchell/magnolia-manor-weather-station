/******************************************************************************
  SparkFun_Photon_Weather_Basic_Soil_Meters.ino
  SparkFun Photon Weather Shield basic example with soil moisture and temp
  and weather meter readings including wind speed, wind direction and rain.
  Joel Bartlett @ SparkFun Electronics
  Original Creation Date: May 18, 2015

  Based on the Wimp Weather Station sketch by: Nathan Seidle
  https://github.com/sparkfun/Wimp_Weather_Station

  This sketch prints the temperature, humidity, barometric pressure, altitude,
  soil moisture, and soil temperature to the Seril port. This sketch also
  incorporates the Weather Meters avaialbe from SparkFun (SEN-08942), which allow
  you to measure Wind Speed, Wind Direction, and Rainfall. Upload this sketch
  after attaching a soil moisture and or soil temperature sensor and Wetaher
  Meters to test your connections.

  Hardware Connections:
	This sketch was written specifically for the Photon Weather Shield,
	which connects the HTU21D and MPL3115A2 to the I2C bus by default.
  If you have an HTU21D and/or an MPL3115A2 breakout,	use the following
  hardware setup:

    HTU21D -------------------- Photon
        (-) ------------------- GND
        (+) ------------------- 3.3V (VCC)
         CL ------------------- D1/SCL
         DA ------------------- D0/SDA

    MPL3115A2 ----------------- Photon
        GND ------------------- GND
        VCC ------------------- 3.3V (VCC)
        SCL ------------------- D1/SCL
        SDA ------------------- D0/SDA

    Soil Moisture Sensor ------ Photon
        GND ------------------- GND
        VCC ------------------- D5
        SIG ------------------- A1

    DS18B20 Temp Sensor ------- Photon
        VCC (Red) ------------- 3.3V (VCC)
        GND (Black) ----------- GND
        SIG (White) ----------- D4

    CCS811 Air Quality Sensor - Photon
       3.3V ------------------- 3.3V (VCC)
        GND ------------------- GND
        SCL ------------------- D1/SCL
        SDA ------------------- D0/SDA
        RST ------------------- ???
        INT ------------------- ???
        WAK ------------------- ???
        SEN-00250 (NTCLE100E3103JB0) between NTC terminals

    APDS-9301 Lux Sensor ------ Photon
       3.3V ------------------- 3.3V (VCC)
        GND ------------------- GND
        SCL ------------------- D1/SCL
        SDA ------------------- D0/SDA
        INT ------------------- ???

    Wind Speed Sensor --------- Photon
        VCC ------------------- 3.3V (VCC)
        GND ------------------- GND
        SIG ------------------- D3

    Wind Direction Sensor ----- Photon
        VCC ------------------- 3.3V (VCC)
        GND ------------------- GND
        SIG ------------------- A0

    Rain Sensor --------------- Photon
        VCC ------------------- 3.3V (VCC)
        GND ------------------- GND
        SIG ------------------- D2

        Direction	Resistance	Voltage	   ADC
        (Degrees)	(ohms)	(v=5v, R=10k)	Value
        0          	33000   	2.91	3610-3615
        22.5	       6570   	2.13	2640-2647
        45	         8200   	2.26	2800-2810
        67.5	        891   	1.26	1563-1567
        90	         1000   	1.29	1595-1600
        112.5	        688   	1.20	1491-1494
        135	         2200   	1.56	1932-1936
        157.5        1410   	1.39	1720-1724
        180	         3900   	1.84	2279-2285
        202.5	       3140   	1.73	2139-2144
        225	        16000   	2.62	3245-3250
        247.5       14120   	2.56	3169-3174
        270        120000   	3.18	3940-3945
        292.5       42120   	2.98	3699-3705
        315         64900   	3.08	3825-3829
        337.5       21880   	2.76	3419-3427


  Development environment specifics:
  	IDE: Particle Dev
  	Hardware Platform: Particle Photon

  This code is beerware; if you see me (or any other SparkFun
  employee) at the local, and you've found our code helpful,
  please buy us a round!
  Distributed as-is; no warranty is given.
*******************************************************************************/
#include "Particle.h"
#include "SparkFun_Photon_Weather_Shield_Library.h"
#include "OneWire.h"
#include "Wire.h"
#include "spark-dallas-temperature.h"
#include "Sparkfun_APDS9301_Library.h"
#include "SparkFunCCS811.h"
#include "Tinker.h"

// System threaded mode is not required here, but it's a good idea with 0.5.0 and later.
// https://docs.particle.io/reference/firmware/electron/#system-thread
//SYSTEM_THREAD(ENABLED);

//#define WAIT_FOR_OPERATOR true
#define SERIAL_PORT Serial1
//#define EXTENDED_SERIAL_DEBUG true
//#define ALTITUDE_MODE true
//#define BASELINE_CCS811 true
#define USE_NTC_TEMPERATURE true

#define CCS811_ADDR 0x5B //Default I2C Address
//#define CCS811_ADDR 0x5A //Alternate I2C Address

#define ONE_WIRE_BUS D4
#define TEMPERATURE_PRECISION 11

#define SOIL_MOIST A1
#define SOIL_MOIST_POWER D5

//#define APDS9301_USE_INTERRUPT true
#define ADPS9301_INTERRUPT_PIN 16 // We'll connect the INT pin from our sensor to the
                  // INT0 interrupt pin on the Arduino.





//OneWire and DallasTemperature libraries are needed for DS18B20 Temp sensor
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

APDS9301 apds;

#ifdef APDS9301_USE_INTERRUPT
volatile bool lightIntHappened = false; // flag set in the interrupt to let the
                                        //  mainline code know that an interrupt occurred.
#endif

//Create Instance of HTU21D or SI7021 temp and humidity sensor and MPL3115A2 barrometric sensor
Weather sensor;

CCS811 myCCS811(CCS811_ADDR);


int WDIR = A0;
int RAIN = D2;
int WSPEED = D3;

//Run I2C Scanner to get address of DS18B20(s)
//(found in the Firmware folder in the Photon Weather Shield Repo)
/***********REPLACE THIS ADDRESS WITH YOUR ADDRESS*************/
DeviceAddress inSoilThermometer = {0x28, 0x09, 0x1F, 0x62, 0x09, 0x00, 0x00, 0x82}; //Waterproof temp sensor address
/***********REPLACE THIS ADDRESS WITH YOUR ADDRESS*************/

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSecond; //The millis counter to see when a second rolls by
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data

//We need to keep track of the following variables:
//Wind speed/dir each update (no storage)
//Wind gust/dir over the day (no storage)
//Wind speed/dir, avg over 2 minutes (store 1 per second)
//Wind gust/dir over last 10 minutes (store 1 per minute)
//Rain over the past hour (store 1 per minute)
//Total rain over date (store one per day)

byte windspdavg[120]; //120 bytes to keep track of 2 minute average
int winddiravg[120]; //120 ints to keep track of 2 minute average
float windgust_10m[10]; //10 floats to keep track of 10 minute max
int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
volatile float rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain

//These are all the weather values that wunderground expects:
int winddir = 0; // [0-360 instantaneous wind direction]
double windspeedmph = 0; // [mph instantaneous wind speed]
double windgustmph = 0; // [mph current wind gust, using software specific time period]
int windgustdir = 0; // [0-360 using software specific time period]
double windspdmph_avg2m = 0; // [mph 2 minute average wind speed mph]
int winddir_avg2m = 0; // [0-360 2 minute average wind direction]
double windgustmph_10m = 0; // [mph past 10 minutes wind gust mph ]
int windgustdir_10m = 0; // [0-360 past 10 minutes wind gust direction]
double rainin = 0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
long lastWindCheck = 0;
volatile double dailyrainin = 0; // [rain inches so far today in local time]
double dailyrainin_nv = 0; // [rain inches so far today in local time]

double humidity = 0;
double tempf = 0;
double tempc = 0;
double InTempC = 0;         //original temperature in C from DS18B20
double soiltempf = 0;       //converted temperature in F from DS18B20

#ifdef ALTITUDE_MODE
double altf = 0;
#else
double pascals = 0;
#endif

double baroTemp = 0;
int soilMoisture = 0;
double LuminousFlux = 0;
int airTVOC = 0;
int airCO2 = 0;

int count = 0;

long lastPublish = 0;
long lastPublish2 = 0;
long delayPublish = 60000;
long delayPublish2 = 2000;
bool success = false;

#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
unsigned long lastSync = millis();

// volatiles are subject to modification by IRQs
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;
volatile unsigned long raintime, rainlast, raininterval, rain;

void update18B20Temp(DeviceAddress deviceAddress, double &tempC);//predeclare to compile
void printDriverError( CCS811Core::status errorCode );//predeclare to compile

//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
  raintime = millis(); // grab current time
  raininterval = raintime - rainlast; // calculate interval between this and last event

    if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
  {
    dailyrainin += 0.011; //Each dump is 0.011" of water
    rainHour[minutes] += 0.011; //Increase this minute's amount of rain

    rainlast = raintime; // set up for next event
  }
}

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
  if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
  {
    lastWindIRQ = millis(); //Grab the current time
    windClicks++; //There is 1.492MPH for each click per second.
  }
}

#ifdef APDS9301_USE_INTERRUPT
void lightIRQ()
{
  lightIntHappened = true;
}
#endif

//---------------------------------------------------------------
void setup()
{

#ifdef BASELINE_CCS811
    SetupBaselineOperator();
#else

    SERIAL_PORT.begin(9600);   // open serial over dedicated serial pins
    #ifdef WAIT_FOR_OPERATOR
        // Make sure your Serial Terminal app is closed before powering your device
        // Now open your Serial Terminal, and hit any key to continue!
        SERIAL_PORT.println("Press any key to begin");
        //This line pauses the Serial port until a key is pressed
        while(!SERIAL_PORT.available()) Particle.process();
    #endif
    // reset the system after 60 seconds if the application is unresponsive
//    ApplicationWatchdog wd(60000, System.reset);

    // DS18B20 initialization
    sensors.begin();
    sensors.setResolution(inSoilThermometer, TEMPERATURE_PRECISION);
    Wire.begin();

    // APDS9301 sensor setup.
    apds.begin(0x39);  // We're assuming you haven't changed the I2C
                       //  address from the default by soldering the
                       //  jumper on the back of the board.
    apds.setGain(APDS9301::LOW_GAIN); // Set the gain to low. Strictly
                       //  speaking, this isn't necessary, as the gain
                       //  defaults to low.
    apds.setIntegrationTime(APDS9301::INT_TIME_13_7_MS); // Set the
                       //  integration time to the shortest interval.
                       //  Again, not strictly necessary, as this is
                       //  the default.
#ifdef APDS9301_USE_INTERRUPT
    apds.setLowThreshold(0); // Sets the low threshold to 0, effectively
                       //  disabling the low side interrupt.
    apds.setHighThreshold(50); // Sets the high threshold to 500. This
                       //  is an arbitrary number I pulled out of thin
                       //  air for purposes of the example. When the CH0
                       //  reading exceeds this level, an interrupt will
                       //  be issued on the INT pin.
    apds.setCyclesForInterrupt(1); // A single reading in the threshold
                       //  range will cause an interrupt to trigger.
    apds.enableInterrupt(APDS9301::INT_ON); // Enable the interrupt.
    apds.clearIntFlag();

    // Interrupt setup
    pinMode(ADPS9301_INTERRUPT_PIN, INPUT_PULLUP); // This pin must be a pullup or have
                       //  a pullup resistor on it as the interrupt is a
                       //  negative going open-collector type output.
    attachInterrupt(digitalPinToInterrupt(ADPS9301_INTERRUPT_PIN), lightIRQ, FALLING);
#endif


    pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
    pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor

    pinMode(SOIL_MOIST_POWER, OUTPUT);//power control for soil moisture
    digitalWrite(SOIL_MOIST_POWER, LOW);//Leave off by defualt



    //This begins the CCS811 sensor and prints error status of .begin()
    CCS811Core::status returnCode = myCCS811.begin();
#ifndef EXTENDED_SERIAL_DEBUG
    if (returnCode != CCS811Core::SENSOR_SUCCESS) {  // suppress extended debug print if returnCode is SENSOR_SUCCESS
#endif
      SERIAL_PORT.print("myCCS811.begin exited with: ");
      printDriverError( returnCode );
      SERIAL_PORT.println();
#ifndef EXTENDED_SERIAL_DEBUG
    }
#endif
    //Initialize the I2C sensors and ping them
    sensor.begin();

    /*You can only receive acurate barrometric readings or acurate altitiude
    readings at a given time, not both at the same time. The following two lines
    tell the sensor what mode to use. You could easily write a function that
    takes a reading in one made and then switches to the other mode to grab that
    reading, resulting in data that contains both acurate altitude and barrometric
    readings. For this example, we will only be using the barometer mode. Be sure
    to only uncomment one line at a time. */
    sensor.setModeBarometer();//Set to Barometer Mode
    //baro.setModeAltimeter();//Set to altimeter Mode

    //These are additional MPL3115A2 functions the MUST be called for the sensor to work.
    sensor.setOversampleRate(7); // Set Oversample rate
    //Call with a rate from 0 to 7. See page 33 for table of ratios.
    //Sets the over sample rate. Datasheet calls for 128 but you can set it
    //from 1 to 128 samples. The higher the oversample rate the greater
    //the time between data samples.

    sensor.enableEventFlags(); //Necessary register calls to enble temp, baro ansd alt

    seconds = 0;
    lastSecond = millis();

    // attach external interrupt pins to IRQ functions
    attachInterrupt(RAIN, rainIRQ, FALLING);
    attachInterrupt(WSPEED, wspeedIRQ, FALLING);

    // turn on interrupts
    interrupts();


    Particle.function("pubDelayWTHR",publishDelayWTHR);
    Particle.function("pubDelayWIND",publishDelayWIND);
    Particle.function("resetDevice", resetDevice);

    // Register all the Tinker functions
    Particle.function("digitalread", tinkerDigitalRead);
    Particle.function("digitalwrite", tinkerDigitalWrite);
    Particle.function("analogread", tinkerAnalogRead);
    Particle.function("analogwrite", tinkerAnalogWrite);

    // reset the AirTVOC and airCO2 sensor without rebooting photon
    Particle.function("resetAirSensor", resetAirSensor);

    Particle.variable("winddir",winddir); // [0-360 instantaneous wind direction]
    Particle.variable("windspeedmph",windspeedmph); // [mph instantaneous wind speed]
    Particle.variable("windgustmph",windgustmph); // [mph current wind gust, using software specific time period]
    Particle.variable("windgustdir",windgustdir); // [0-360 using software specific time period]
    Particle.variable("winds_avg2m",windspdmph_avg2m); // [mph 2 minute average wind speed mph]
    Particle.variable("windd_avg2m",winddir_avg2m); // [0-360 2 minute average wind direction]
    Particle.variable("windgmph_10m",windgustmph_10m); // [mph past 10 minutes wind gust mph ]
    Particle.variable("windgdir_10m",windgustdir_10m); // [0-360 past 10 minutes wind gust direction]
    Particle.variable("rainin",rainin); // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
    Particle.variable("dailyrainin",dailyrainin_nv); // [rain inches so far today in local time]
    Particle.variable("humidity",humidity);
    Particle.variable("tempf",tempf);
    Particle.variable("soiltempf",soiltempf);//converted temperature in F from DS18B20

    #ifdef ALTITUDE_MODE
    Particle.variable("altf",altf);
    #else
    Particle.variable("pascals",pascals);
    #endif

    Particle.variable("baroTemp",baroTemp);
    Particle.variable("soilMoisture",soilMoisture);
    Particle.variable("LumensFlux",LuminousFlux);
    Particle.variable("airTVOC",airTVOC);
    Particle.variable("airCO2",airCO2);
#endif

}
//---------------------------------------------------------------
void loop()
{
#ifdef BASELINE_CCS811
  BaselineOperator();
#else
  dailyrainin_nv = dailyrainin; // particle.variable cannot seem to handle a casted volatile variable
  //Keep track of which minute it is
  if(millis() - lastSecond >= 1000)
  {

    lastSecond += 1000;

    //Get readings from all sensors
    getWeather();

#ifdef APDS9301_USE_INTERRUPT
    apds.clearIntFlag();
#endif

    LuminousFlux = apds.readLuxLevel();

    if (myCCS811.dataAvailable())
    {
      myCCS811.readAlgorithmResults();

      airTVOC = myCCS811.getTVOC();
      airCO2 = myCCS811.getCO2();

#ifdef USE_NTC_TEMPERATURE
      //.readNTC() causes the CCS811 library to gather ADC data and save value
      myCCS811.readNTC();
#ifdef EXTENDED_SERIAL_DEBUG
      SERIAL_PORT.print(" NTC Measured resistance : ");
#endif
      //After .readNTC() is called, .getResistance() can be called to actually
      //get the resistor value.  This is not needed to get the temperature,
      //but can be useful information for debugging.
      //
      //Use the resistance value for custom thermistors, and calculate the
      //temperature yourself.
#ifdef EXTENDED_SERIAL_DEBUG
      SERIAL_PORT.print( myCCS811.getResistance() );
      SERIAL_PORT.println(" ohms");
#endif
      //After .readNTC() is called, .getTemperature() can be called to get
      //a temperature value providing that part SEN-00250 is used in the
      //NTC terminals. (NTCLE100E3103JB0)
#ifdef EXTENDED_SERIAL_DEBUG
      SERIAL_PORT.print(" NTC Converted temperature : ");
#endif
      float readTemperature = myCCS811.getTemperature();
#ifdef EXTENDED_SERIAL_DEBUG
      SERIAL_PORT.print( readTemperature, 2);
      SERIAL_PORT.println(" deg C");
#endif

      //Pass the temperature back into the CCS811 to compensate
      myCCS811.setEnvironmentalData((float)humidity, readTemperature);
#else
      //This sends the temperature data to the CCS811
      myCCS811.setEnvironmentalData((float)humidity, (float)tempc);
#endif
    }
    else if (myCCS811.checkForStatusError())
    {
      printSensorError();
    }

    //Rather than use a delay, keeping track of a counter allows the photon to
    // still take readings and do work in between printing out data.
    count++;
    //alter this number to change the amount of time between each reading
    if(count == 5)
    {
       printInfo();
       count = 0;
    }
    // This math looks at the current time vs the last time a publish happened
    if(millis() - lastPublish > delayPublish) //Publishes every 60000 milliseconds, or 60 seconds
    {
      // Record when you published
      lastPublish = millis();

      // Choose which values you actually want to publish- remember, if you're
      // publishing more than once per second on average, you'll be throttled!
#ifdef ALTITUDE_MODE
      success = Particle.publish("MMWEATHER", String::format("{\"humidity\":%f,\"tempF\":%f,\"altf\":%f,\"baroTempF\":%f,\"SoilTempF\":%f,\"SoilMoisture\":%d,\"LuminousFlux\":%f,\"AirTVOC\":%d,\"airCO2\":%d}",humidity,tempf,altf,baroTemp,soiltempf,soilMoisture,LuminousFlux > 65534 ? 65534 : LuminousFlux,airTVOC, airCO2), 60, PRIVATE);
#else
      success = Particle.publish("MMWEATHER", String::format("{\"humidity\":%f,\"tempF\":%f,\"pressurehPa\":%f,\"pressureHg\":%f,\"baroTempF\":%f,\"SoilTempF\":%f,\"SoilMoisture\":%d,\"LuminousFlux\":%f,\"AirTVOC\":%d,\"airCO2\":%d}",humidity,tempf,(pascals/100),((pascals/100) * 0.0295300),baroTemp,soiltempf,soilMoisture,LuminousFlux > 65534 ? 65534 : LuminousFlux,airTVOC, airCO2), 60, PRIVATE);
#endif
      if (!success) {
        // get here if event publish did not work
        SERIAL_PORT.println("MMWEATHER Publish Failed");
#ifdef EXTENDED_SERIAL_DEBUG
      } else {
        SERIAL_PORT.println("MMWEATHER Publish Succeeded");
#endif
      }
    }

    if(millis() - lastPublish2 > delayPublish2) //Publishes every 60000 milliseconds, or 60 seconds
    {
      // Record when you published
      lastPublish2 = millis();

      // Choose which values you actually want to publish- remember, if you're
      // publishing more than once per second on average, you'll be throttled!
      success = Particle.publish("MMWIND", String::format("{\"winddir\":%d,\"windspeedmph\":%f,\"windgustmph\":%f,\"windgustdir\":%d,\"windspdmph_avg2m\":%f,\"winddir_avg2m\":%d,\"windgustmph_10m\":%f,\"windgustdir_10m\":%d,\"rainin\":%f,\"dailyrainin\":%f}", winddir,windspeedmph,windgustmph,windgustdir,windspdmph_avg2m,winddir_avg2m,windgustmph_10m,windgustdir_10m,rainin,dailyrainin), 60, PRIVATE);

      if (!success) {
        // get here if event publish did not work
        SERIAL_PORT.println("MMWIND Publish Failed");
#ifdef EXTENDED_SERIAL_DEBUG
      } else {
        SERIAL_PORT.println("MMWIND Publish Succeeded");
#endif
      }
    }
  }
#endif

  if (millis() - lastSync > ONE_DAY_MILLIS) {
    //Reset the largest windgust today
    windgustmph = 0;
    windgustdir = 0;
    dailyrainin = 0; // we meed to figure out how to do this right!!

    // Request time synchronization from the Particle Cloud
    Particle.syncTime();
    waitUntil(Particle.syncTimeDone);
    lastSync = millis();

  }

}

int resetAirSensor(String command)
{
  //This begins the CCS811 sensor and prints error status of .begin()
  CCS811Core::status returnCode = myCCS811.begin();
#ifndef EXTENDED_SERIAL_DEBUG
  if (returnCode != CCS811Core::SENSOR_SUCCESS) {  // suppress extended debug print if returnCode is SENSOR_SUCCESS
#endif
    SERIAL_PORT.print("myCCS811.begin exited with: ");
    printDriverError( returnCode );
    SERIAL_PORT.println();
#ifndef EXTENDED_SERIAL_DEBUG
  }
#endif
    return returnCode;
}

// Allows you to remotely change how often the device is publishing to the cloud
// Change the default at the top of the code.
int publishDelayWTHR(String command) {
    delayPublish = atoi(command) * 1000;
    return delayPublish;
}
int publishDelayWIND(String command) {
    delayPublish2 = atoi(command) * 1000;
    return delayPublish2;
}
// remote reboot of photon
int resetDevice(String command) {
    System.reset();
}
//---------------------------------------------------------------
void printInfo()
{
  //This function prints the weather data out to the default Serial Port
      SERIAL_PORT.print("Wind_Dir:");
      switch (winddir)
      {
        case 0:
          SERIAL_PORT.print("North");
          break;
        case 225:
          SERIAL_PORT.print("NNE");
          break;
        case 450:
          SERIAL_PORT.print("NE");
          break;
        case 675:
          SERIAL_PORT.print("ENE");
          break;
        case 900:
          SERIAL_PORT.print("East");
          break;
        case 1125:
          SERIAL_PORT.print("ESE");
          break;
        case 1350:
          SERIAL_PORT.print("SE");
          break;
        case 1575:
          SERIAL_PORT.print("SSE");
          break;
        case 1800:
          SERIAL_PORT.print("South");
          break;
        case 2025:
          SERIAL_PORT.print("SSW");
          break;
        case 2250:
          SERIAL_PORT.print("SW");
          break;
        case 2475:
          SERIAL_PORT.print("WSW");
          break;
        case 2700:
          SERIAL_PORT.print("West");
          break;
        case 2925:
          SERIAL_PORT.print("WNW");
          break;
        case 3150:
          SERIAL_PORT.print("NW");
          break;
        case 3375:
          SERIAL_PORT.print("NNW");
          break;
        default:
          SERIAL_PORT.print("No Wind");
          // if nothing else matches, do the
          // default (which is optional)
      }

      SERIAL_PORT.print(" Wind_Speed:");
      SERIAL_PORT.print(windspeedmph, 1);
      SERIAL_PORT.print("mph, ");

      SERIAL_PORT.print("Rain:");
      SERIAL_PORT.print(rainin, 2);
      SERIAL_PORT.print("in., ");

      SERIAL_PORT.print("Daily Rain:");
      SERIAL_PORT.print(dailyrainin, 2);
      SERIAL_PORT.print("in., ");

      SERIAL_PORT.print("Temp:");
      SERIAL_PORT.print(tempf);
      SERIAL_PORT.print("F, ");

      SERIAL_PORT.print("Humidity:");
      SERIAL_PORT.print(humidity);
      SERIAL_PORT.print("%%, ");

      SERIAL_PORT.print("Baro_Temp:");
      SERIAL_PORT.print(baroTemp);
      SERIAL_PORT.print("F, ");

      #ifdef ALTITUDE_MODE
      //If in altitude mode, print with these lines
      SERIAL_PORT.print("Altitude:");
      SERIAL_PORT.print(altf);
      SERIAL_PORT.println("ft.");
      #else
      //The MPL3115A2 outputs the pressure in Pascals. However, most weather stations
      //report pressure in hectopascals or millibars. Divide by 100 to get a reading
      //more closely resembling what online weather reports may say in hPa or mb.
      //Another common unit for pressure is Inches of Mercury (in.Hg). To convert
      //from mb to in.Hg, use the following formula. P(inHg) = 0.0295300 * P(mb)
      //More info on conversion can be found here:
      //www.srh.noaa.gov/images/epz/wxcalc/pressureConversion.pdf
      SERIAL_PORT.print("Pressure:");
      SERIAL_PORT.print(pascals/100);
      SERIAL_PORT.print("hPa, ");
      #endif


      SERIAL_PORT.print("Soil_Temp:");
      SERIAL_PORT.print(soiltempf);
      SERIAL_PORT.print("F, ");

      SERIAL_PORT.print("Soil_Mositure:");
      SERIAL_PORT.print(soilMoisture);
      SERIAL_PORT.print(" ");

      //Mositure Content is expressed as an analog
      //value, which can range from 0 (completely dry) to the value of the
      //materials' porosity at saturation. The sensor tends to max out between
      //3000 and 3500.

      SERIAL_PORT.print("Luminous_Flux:");
      SERIAL_PORT.print(LuminousFlux,6);
      SERIAL_PORT.print(" ");

      SERIAL_PORT.print("CO2:");
      SERIAL_PORT.print(airCO2);
      SERIAL_PORT.print(" ");

      SERIAL_PORT.print("tVOC:");
      SERIAL_PORT.print(airTVOC);
      SERIAL_PORT.print(" ");

      printRunTime();
      SERIAL_PORT.println();

#ifdef APDS9301_USE_INTERRUPT
      if (lightIntHappened)
      {
        SERIAL_PORT.println("Interrupt");
        lightIntHappened = false;
      }
#endif
}
//---------------------------------------------------------------
void getSoilTemp()
{
    //get temp from DS18B20
    sensors.requestTemperatures();
    update18B20Temp(inSoilThermometer, InTempC);
    //Every so often there is an error that throws a -127.00, this compensates
    if(InTempC < -100)
      soiltempf = soiltempf;//push last value so data isn't out of scope
    else
      soiltempf = (InTempC * 9)/5 + 32;//else grab the newest, good data
}
//---------------------------------------------------------------
void getSoilMositure()
{
    /*We found through testing that leaving the soil moisture sensor powered
    all the time lead to corrosion of the probes. Thus, this port breaks out
    Digital Pin D5 as the power pin for the sensor, allowing the Photon to
    power the sensor, take a reading, and then disable power on the sensor,
    giving the sensor a longer lifespan.*/
    digitalWrite(SOIL_MOIST_POWER, HIGH);
    delay(200);
    soilMoisture = analogRead(SOIL_MOIST);
    delay(100);
    digitalWrite(SOIL_MOIST_POWER, LOW);

}
//---------------------------------------------------------------
void update18B20Temp(DeviceAddress deviceAddress, double &tempC)
{
  tempC = sensors.getTempC(deviceAddress);
}
//---------------------------------------------------------------
//Read the wind direction sensor, return heading in degrees
int get_wind_direction()
{
  unsigned int adc;
  double volts;

  adc = analogRead(WDIR); // get the current reading from the sensor
  volts = ( ( (double)adc * 3.3)/4095 );
#ifdef EXTENDED_SERIAL_DEBUG
  SERIAL_PORT.print("adc value: ");
  SERIAL_PORT.print(adc);
  SERIAL_PORT.print(" voltage: ");
  SERIAL_PORT.println(volts);
#endif
  // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
  // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
  // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

  //   Direction	Resistance	Voltage	   ADC
  //  (Degrees)	(ohms)	(v=5v, R=10k)	Value
  //  0          	33000   	2.91	3610-3615
  //  22.5	       6570   	2.13	2640-2647
  //  45	         8200   	2.26	2800-2810
  //  67.5	        891   	1.26	1563-1567
  //  90	         1000   	1.29	1595-1600
  //  112.5	        688   	1.20	1491-1494
  //  135	         2200   	1.56	1932-1936
  //  157.5        1410   	1.39	1720-1724
  //  180	         3900   	1.84	2279-2285
  //  202.5	       3140   	1.73	2139-2144
  //  225	        16000   	2.62	3245-3250
  //  247.5       14120   	2.56	3169-3174
  //  270        120000   	3.18	3940-3945
  //  292.5       42120   	2.98	3699-3705
  //  315         64900   	3.08	3825-3829
  //  337.5       21880   	2.76	3419-3427


  //Wind Vanes may vary in the values they return. To get exact wind direction,
  //it is recomended that you AnalogRead the Wind Vane to make sure the values
  //your wind vane output fall within the values listed below.

/* non-seamless values let some readings slip through as -1
  if(volts > 2.89 && volts < 2.93) return (0);
  if(volts > 2.11 && volts < 2.15) return (225);
  if(volts > 2.24 && volts < 2.28) return (450);
  if(volts > 1.24 && volts < 1.27) return (675);
  if(volts > 1.28 && volts < 1.31) return (900);
  if(volts > 1.18 && volts < 1.22) return (1125);
  if(volts > 1.54 && volts < 1.58) return (1350);
  if(volts > 1.37 && volts < 1.41) return (1575);
  if(volts > 1.82 && volts < 1.86) return (1800);
  if(volts > 1.71 && volts < 1.75) return (2025);
  if(volts > 2.60 && volts < 2.64) return (2250);
  if(volts > 2.54 && volts < 2.58) return (2475);
  if(volts > 3.16 && volts < 3.20) return (2700);
  if(volts > 2.96 && volts < 3.00) return (2925);
  if(volts > 3.06 && volts < 3.10) return (3150);
  if(volts > 2.74 && volts < 2.78) return (3375);
*/
  if(volts >= 2.84 && volts < 2.95) return (0);
  if(volts >= 1.99 && volts < 2.20) return (225);
  if(volts >= 2.20 && volts < 2.42) return (450);
  if(volts >= 1.24 && volts < 1.27) return (675);
  if(volts >= 1.27 && volts < 1.35) return (900);
  if(volts >= 1.18 && volts < 1.24) return (1125);
  if(volts >= 1.48 && volts < 1.65) return (1350);
  if(volts >= 1.35 && volts < 1.48) return (1575);
  if(volts >= 1.79 && volts < 1.99) return (1800);
  if(volts >= 1.65 && volts < 1.79) return (2025);
  if(volts >= 2.60 && volts < 2.70) return (2250);
  if(volts >= 2.42 && volts < 2.60) return (2475);
  if(volts >= 3.14 && volts < 3.21) return (2700);
  if(volts >= 2.95 && volts < 3.04) return (2925);
  if(volts >= 3.04 && volts < 3.14) return (3150);
  if(volts >= 2.70 && volts < 2.84) return (3375);


  return (-1); // error, disconnected?
}
//---------------------------------------------------------------
//Returns the instataneous wind speed
float get_wind_speed()
{
  float deltaTime = millis() - lastWindCheck; //750ms

  deltaTime /= 1000.0; //Covert to seconds

  float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

  windClicks = 0; //Reset and start watching for new wind
  lastWindCheck = millis();

  windSpeed *= 1.492; //4 * 1.492 = 5.968MPH

  /* Serial.println();
   Serial.print("Windspeed:");
   Serial.println(windSpeed);*/

  return(windSpeed);
}
//---------------------------------------------------------------
void getWeather()
{
    // Measure Relative Humidity from the HTU21D or Si7021
    humidity = sensor.getRH();

    // Measure Temperature from the HTU21D or Si7021
    tempf = sensor.getTempF();
    tempc = sensor.getTemp();
    // Temperature is measured every time RH is requested.
    // It is faster, therefore, to read it from previous RH
    // measurement with getTemp() instead with readTemp()

    //Measure the Barometer temperature in F from the MPL3115A2
    baroTemp = sensor.readBaroTempF();

    #ifdef ALTITUDE_MODE
    //If in altitude mode, you can get a reading in feet  with this line:
    float altf = sensor.readAltitudeFt();
    #else
    //Measure Pressure from the MPL3115A2
    pascals = sensor.readPressure();
    #endif


    getSoilTemp();//Read the DS18B20 waterproof temp sensor
    getSoilMositure();//Read the soil moisture sensor

    //Calc winddir
//    winddir = get_wind_direction();

    //Calc windspeed
//    windspeedmph = get_wind_speed();

    //Calc windgustmph
    //Calc windgustdir
    //Report the largest windgust today
//    windgustmph = 0;
//    windgustdir = 0;
    //Take a speed and direction reading every second for 2 minute average
    if(++seconds_2m > 119) seconds_2m = 0;

    //Calc the wind speed and direction every second for 120 second to get 2 minute average
    float currentSpeed = get_wind_speed();
    //float currentSpeed = random(5); //For testing
    int currentDirection = get_wind_direction();
    if (currentDirection == -1)
    {
       currentDirection = get_wind_direction(); // retry once
    }

    winddir = currentDirection;
    windspeedmph = currentSpeed;

    windspdavg[seconds_2m] = (int)currentSpeed;
    winddiravg[seconds_2m] = currentDirection;
    //if(seconds_2m % 10 == 0) displayArrays(); //For testing

    //Check to see if this is a gust for the minute
    if(currentSpeed > windgust_10m[minutes_10m])
    {
      windgust_10m[minutes_10m] = currentSpeed;
      windgustdirection_10m[minutes_10m] = currentDirection;
    }

    //Check to see if this is a gust for the day
    if(currentSpeed > windgustmph)
    {
      windgustmph = currentSpeed;
      windgustdir = currentDirection;
    }

    if(++seconds > 59)
    {
      seconds = 0;

      if(++minutes > 59) minutes = 0;
      if(++minutes_10m > 9) minutes_10m = 0;

      rainHour[minutes] = 0; //Zero out this minute's rainfall amount
      windgust_10m[minutes_10m] = 0; //Zero out this minute's gust
    }


    //Calc windspdmph_avg2m
    float temp = 0;
    for(int i = 0 ; i < 120 ; i++)
      temp += windspdavg[i];
    temp /= 120.0;
    windspdmph_avg2m = temp;

    //Calc winddir_avg2m
    temp = 0; //Can't use winddir_avg2m because it's an int
    for(int i = 0 ; i < 120 ; i++)
      temp += winddiravg[i];
    temp /= 120;
    winddir_avg2m = temp;

    //Calc windgustmph_10m
    //Calc windgustdir_10m
    //Find the largest windgust in the last 10 minutes
    windgustmph_10m = 0;
    windgustdir_10m = 0;
    //Step through the 10 minutes
    for(int i = 0; i < 10 ; i++)
    {
      if(windgust_10m[i] > windgustmph_10m)
      {
        windgustmph_10m = windgust_10m[i];
        windgustdir_10m = windgustdirection_10m[i];
      }
    }

    //Total rainfall for the day is calculated within the interrupt
    //Calculate amount of rainfall for the last 60 minutes
    rainin = 0;
    for(int i = 0 ; i < 60 ; i++)
      rainin += rainHour[i];
}

//Prints the amount of time the board has been running
//Does the hour, minute, and second calcs
void printRunTime()
{
  char buffer[50];

  unsigned long runTime = millis();

  int hours = runTime / (60 * 60 * 1000L);
  runTime %= (60 * 60 * 1000L);
  int minutes = runTime / (60 * 1000L);
  runTime %= (60 * 1000L);
  int seconds = runTime / 1000L;

  sprintf(buffer, "RunTime[%02d:%02d:%02d]", hours, minutes, seconds);
  SERIAL_PORT.print(buffer);

  if (hours == 0 && minutes < 20) SERIAL_PORT.print(" RunTime Not yet valid");
}

//printDriverError decodes the CCS811Core::status type and prints the
//type of error to the serial terminal.
//
//Save the return value of any function of type CCS811Core::status, then pass
//to this function to see what the output was.
//printDriverError decodes the CCS811Core::status type and prints the
//type of error to the serial terminal.
//
//Save the return value of any function of type CCS811Core::status, then pass
//to this function to see what the output was.
void printDriverError( CCS811Core::status errorCode )
{
  switch ( errorCode )
  {
    case CCS811Core::SENSOR_SUCCESS:
      SERIAL_PORT.print("SUCCESS");
      break;
    case CCS811Core::SENSOR_ID_ERROR:
      SERIAL_PORT.print("ID_ERROR");
      break;
    case CCS811Core::SENSOR_I2C_ERROR:
      SERIAL_PORT.print("I2C_ERROR");
      break;
    case CCS811Core::SENSOR_INTERNAL_ERROR:
      SERIAL_PORT.print("INTERNAL_ERROR");
      break;
    case CCS811Core::SENSOR_GENERIC_ERROR:
      SERIAL_PORT.print("GENERIC_ERROR");
      break;
    default:
      SERIAL_PORT.print("Unspecified error.");
  }
}

//printSensorError gets, clears, then prints the errors
//saved within the error register.
void printSensorError()
{
  uint8_t error = myCCS811.getErrorRegister();

  if ( error == 0xFF ) //comm error
  {
    SERIAL_PORT.println("Failed to get ERROR_ID register.");
  }
  else
  {
    SERIAL_PORT.print("Error: ");
    if (error & 1 << 5) SERIAL_PORT.print("HeaterSupply");
    if (error & 1 << 4) SERIAL_PORT.print("HeaterFault");
    if (error & 1 << 3) SERIAL_PORT.print("MaxResistance");
    if (error & 1 << 2) SERIAL_PORT.print("MeasModeInvalid");
    if (error & 1 << 1) SERIAL_PORT.print("ReadRegInvalid");
    if (error & 1 << 0) SERIAL_PORT.print("MsgInvalid");
    SERIAL_PORT.println();
  }
}

#ifdef BASELINE_CCS811
void SetupBaselineOperator()
{
  SERIAL_PORT.begin(9600);
  // Make sure your Serial Terminal app is closed before powering your device
  // Now open your Serial Terminal, and hit any key to continue!
  SERIAL_PORT.println("Press any key to begin");
  //This line pauses the Serial port until a key is pressed
  while(!SERIAL_PORT.available()) Particle.process();

  SERIAL_PORT.println();
  SERIAL_PORT.println("CCS811 Baseline Example");

  CCS811Core::status returnCode = myCCS811.begin();
  SERIAL_PORT.print("begin exited with: ");
  printDriverError( returnCode );
  SERIAL_PORT.println();

  //This looks for previously saved data in the eeprom at program start
  if ((EEPROM.read(0) == 0xA5) && (EEPROM.read(1) == 0xB2))
  {
    SERIAL_PORT.println("EEPROM contains saved data.");
  }
  else
  {
    SERIAL_PORT.println("Saved data not found!");
  }
  SERIAL_PORT.println();

  SERIAL_PORT.println("Program running.  Send the following characters to operate:");
  SERIAL_PORT.println(" 's' - save baseline into EEPROM");
  SERIAL_PORT.println(" 'l' - load and apply baseline from EEPROM");
  SERIAL_PORT.println(" 'c' - clear baseline from EEPROM");
  SERIAL_PORT.println(" 'r' - read and print sensor data");
}

void BaselineOperator()
{
  char c;
  unsigned int result;
  unsigned int baselineToApply;
  CCS811Core::status errorStatus;

  if (SERIAL_PORT.available())
  {
    c = SERIAL_PORT.read();
    switch (c)
    {
      case 's':
        //This gets the latest baseline from the sensor
        result = myCCS811.getBaseline();
        SERIAL_PORT.print("baseline for this sensor: 0x");
        if (result < 0x100) SERIAL_PORT.print("0");
        if (result < 0x10) SERIAL_PORT.print("0");
        SERIAL_PORT.println(result, HEX);
        //The baseline is saved (with valid data indicator bytes)
        EEPROM.write(0, 0xA5);
        EEPROM.write(1, 0xB2);
        EEPROM.write(2, (result >> 8) & 0x00FF);
        EEPROM.write(3, result & 0x00FF);
        break;
      case 'l':
        if ((EEPROM.read(0) == 0xA5) && (EEPROM.read(1) == 0xB2))
        {
          SERIAL_PORT.println("EEPROM contains saved data.");
          //The recovered baseline is packed into a 16 bit word
          baselineToApply = ((unsigned int)EEPROM.read(2) << 8) | EEPROM.read(3);
          SERIAL_PORT.print("Saved baseline: 0x");
          if (baselineToApply < 0x100) SERIAL_PORT.print("0");
          if (baselineToApply < 0x10) SERIAL_PORT.print("0");
          SERIAL_PORT.println(baselineToApply, HEX);
          //This programs the baseline into the sensor and monitors error states
          errorStatus = myCCS811.setBaseline( baselineToApply );
          if ( errorStatus == CCS811Core::SENSOR_SUCCESS )
          {
            SERIAL_PORT.println("Baseline written to CCS811.");
          }
          else
          {
            printDriverError( errorStatus );
          }
        }
        else
        {
          SERIAL_PORT.println("Saved data not found!");
        }
        break;
      case 'c':
        //Clear data indicator and data from the eeprom
        SERIAL_PORT.println("Clearing EEPROM space.");
        EEPROM.write(0, 0x00);
        EEPROM.write(1, 0x00);
        EEPROM.write(2, 0x00);
        EEPROM.write(3, 0x00);
        break;
      case 'r':
        if (myCCS811.dataAvailable())
        {
          //Simply print the last sensor data
          myCCS811.readAlgorithmResults();

          SERIAL_PORT.print("CO2[");
          SERIAL_PORT.print(myCCS811.getCO2());
          SERIAL_PORT.print("] tVOC[");
          SERIAL_PORT.print(myCCS811.getTVOC());
          SERIAL_PORT.print("]");
          SERIAL_PORT.println();
        }
        else
        {
          SERIAL_PORT.println("Sensor data not available.");
        }
        break;
      default:
        break;
    }
  }
  delay(10);
}
#endif
