/******************************DATE & TIME*******************************************/
// DS3231_Serial_Easy
// Copyright (C)2015 Rinky-Dink Electronics, Henning Karlsen. All right reserved
// web: http://www.RinkyDinkElectronics.com/

#include <DS3231.h>               

DS3231  rtc(SDA, SCL);
/******************************DATE & TIME*******************************************/
/******************************TEMP & HUM********************************************/
/* https://github.com/adafruit/DHT-sensor-library/blob/master/examples/DHTtester/DHTtester.ino */

#include "DHT.h"                 
#define DHTPIN 2                 
#define DHTTYPE DHT11             

DHT dht(DHTPIN, DHTTYPE);
/******************************TEMP & HUM********************************************/
/******************************MEMORY************************************************/
/* https://www.arduino.cc/en/Tutorial/ReadWrite */

#include <SPI.h>                  
#include <SD.h>                  

File myFile;  //Minneskort
/******************************MEMORY************************************************/
/******************************LUX-SENSOR********************************************/
/* https://learn.adafruit.com/tsl2561/arduino-code */

#include <Wire.h>                 
#include <Adafruit_Sensor.h>      
#include <Adafruit_TSL2561_U.h>   

Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
/******************************LUX-SENSOR********************************************/
/******************************BPM-SENSOR********************************************/
#include <Time.h>                
#include <TimeLib.h>             

const int HR_RX = 7;
byte oldSample, sample;
int beat = 0;
int tiden = 0;
int s = 0;
/******************************BPM-SENSOR********************************************/
/******************************SOUND-SENSOR******************************************/
/* https://gist.github.com/madeintaiwan/6410697 */
/******************************SOUND-SENSOR******************************************/
void setup()
{
    Serial.begin(9600);
        memory_setup();
        delay(600);
        if (myFile) 
          {
            //Description in order
            myFile.println("Datum;Tid;Luftfuktighet;Temperatur;Ljus;Ljud;Puls");
            myFile.close();
          }
        else 
          {
              // if the file didn't open, print an error:
              Serial.println("error opening test.txt");
          }
        rtc.begin();
       // timeSetup();
        dht.begin();
        lightSetup();
        heartSetup();
        memory_setup();
        delay(200);
}
void loop()
{
    timeLoop();
    tempLoop();
    lightLoop();
    soundLoop();
    heartLoop();
    
    //Time interval between the measurements
    //The measurement takes about 10 seconds  
    delay(15000);         
}
void tempLoop()
{
  // Reading temperature and humidity takes about 250 milliseconds!
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  
  memory_setup();
  delay(600);
  if (myFile) 
    {
        myFile.print(h);
        myFile.print(";");
        myFile.print(t);
        myFile.print(";");
        myFile.close();
        Serial.println("Luft & Temp [DONE]");
    } 
  else 
    {
        // if the file didn't open, print an error:
        Serial.println("error opening test.txt");
    }
}
void memory_setup()
{
  while (!Serial) 
    {
      ; // wait for serial port to connect. Needed for native USB port only
    }

  if (!SD.begin(4)) 
    {
        //Serial.println("initialization failed!");
        //  while (1);
        //  Serial.println("initialization done.");
    }
    myFile = SD.open("test.txt", FILE_WRITE);
}

void soundLoop()
{
   const int sampleWindow = 50; // Sample window width in mS (50 mS = 20Hz) 
   unsigned int something;  
   unsigned long startMillis= millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level
   int sample = 0;
   unsigned int signalMax = 0;
   unsigned int signalMin = 1024;
 
   // collect data for 50 mS
   while (millis() - startMillis < sampleWindow)
   {
      sample = analogRead(0);
      if (sample < 1024)  // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;  // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
   double volts = (peakToPeak * 5.0) / 1024;  // convert to volts
   
   memory_setup();
   delay(600);
   if (myFile) 
      {
          myFile.print(volts);
          myFile.print(";");
          myFile.close();
          Serial.println("Ljud [DONE]");
      } 
   else 
      {
          // if the file didn't open, print an error:
          Serial.println("error opening test.txt");
      }
 }
void configureSensor()
{
    /* You can also manually set the gain or enable auto-gain support */
    // tsl.setGain(TSL2561_GAIN_1X);      /* No gain ... use in bright light to avoid sensor saturation */
     tsl.setGain(TSL2561_GAIN_16X);     /* 16x gain ... use in low light to boost sensitivity */
    //  tsl.enableAutoRange(true);            /* Auto-gain ... switches automatically between 1x and 16x */
    
    /* Changing the integration time gives you better sensor resolution (402ms = 16-bit data) */
   tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);      /* fast but low resolution */
    // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_101MS);  /* medium resolution and speed   */
    // tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);  /* 16-bit data but slowest conversions */
}
void lightSetup()
{  
  /* Initialise the sensor */
  if(!tsl.begin())
    {
      /* There was a problem detecting the ADXL345 ... check your connections */
      Serial.print("Ooops, no TSL2561 detected ... Check your wiring or I2C ADDR!");
      while(1);
    }
    /* Setup the sensor gain and integration time */
    configureSensor();
}
void lightLoop()
{
  /* Get a new sensor event */ 
  sensors_event_t event;
  tsl.getEvent(&event);

  memory_setup();
  delay(600);
  if (myFile) 
    {
        if (event.light)
          {
            myFile.print(event.light); 
            myFile.print(";");
          }
        else
          {
            /* If event.light = 0 lux the sensor is probably saturated
               and no reliable data could be generated! */
            myFile.print("E;");
          }
   myFile.close();
   Serial.print(event.light);
   Serial.print("    ");
   Serial.println("Ljus [DONE]");
    }
    else 
      {
          // if the file didn't open, print an error:
          Serial.println("error opening test.txt");
      }
}
void timeSetup()
{
    // The following lines can be uncommented depending on which data you want 
    //rtc.setDOW(MONDAY);     
    rtc.setTime(19, 44, 0);     // Set the time: Hour, Minute, Second (24hr format)
    rtc.setDate(16, 03, 2018);   // Day, Month, Year
}

void timeLoop()
{
   memory_setup();
   delay(600);
   if (myFile) 
      {
        // Send date
        myFile.print(rtc.getDateStr());
        myFile.print(";");
      
        // Send time
        myFile.print(rtc.getTimeStr());
        myFile.print(";");
        myFile.close();
        Serial.println("Tid [DONE]");
      }
    else 
      {
          // if the file didn't open, print an error:
          Serial.println("error opening test.txt");
      }
}
void heartSetup()
{
    pinMode (HR_RX, INPUT); //Signal pin to input
    Serial.println("Waiting for heart beat...");
    //Wait until a heart beat is detected
    while (!digitalRead(HR_RX)) {};
    Serial.println ("Heart beat detected!");
}
int heartLoop()
{
   time_t t = now();
   s = t;        
   tiden = t-s;
   
  while (tiden < 10 )
  {
      sample = digitalRead(HR_RX); //Store signal output
      
      if (sample && (oldSample != sample))
        {
            beat++;
            time_t t = now();
            tiden = t-s;
        }
      oldSample = sample; //Store last signal received
  }
      memory_setup();
      delay(600);
      
  if (myFile)
    {
        myFile.println(beat*6);
        myFile.close();
        Serial.println("Puls [DONE]");
    }
  else 
    {
        // if the file didn't open, print an error:
        Serial.println("error opening test.txt");
    } 
    beat=0;
}
