#include <Wire.h>
#include <AHTxx.h>
#include <SPI.h>
#include <SdFat.h>
#include <LiquidCrystal_I2C.h>
#include "RTClib.h"


RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

//TODO : after millis end, error because variables not updated (probably, think)
void FanTurnOn();//SD Card not insert cause to always Fan run (2 second) and run tempDuration middle of two mode previousTimeFanSpeed and previousTimeFanAction
void initializeSDCard();
void dateTime(uint16_t* date, uint16_t* time);

//pins 11,12,13,4 for SPI
//pins A4,A5 for I2C
//pins not used 2,5,6,7,10

//Buzzer
#define Buzzer 8
//Heater
#define Heater 3
//Fan
#define Fan 9
bool FanTurnOnStatus = false;
bool turnOn = true;
//DataLogger
SdFat SD;
#define chipSelectSD 2
bool createNewFile = true;
int dataLogNumberFile = 1;
String dataLogTargetDirectory = "dataLog/";
String dataLogFileName = "log";
String dataLogFileFormat = ".csv";
String fileName = "";

//test
int durationUpdate = 2000;
unsigned long previousMillis = 0;
unsigned long currentMillis = 0;

//Turn OFF_ON Rapidly Fan
int durationFanSpeedUpdate = 100;
int durationFanActionUpdate = 1500;
unsigned long previousTimeFanSpeed = 0;
unsigned long previousTimeFanAction = 0;
bool firstRunFanSpeed = true;
bool firstRunFanAction = true;

//for Sensor
float temperature;
float humidity;
float baseLineTemp = 37;
float baseLineHumidity = 60;
AHTxx aht10(AHTXX_ADDRESS_X38, AHT1x_SENSOR); //sensor address, sensor type

unsigned int year = 2023;
byte month = 04;
byte day = 28;
byte hour = 16;
byte minute = 30;
byte second = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(Heater, OUTPUT);
  pinMode(Fan, OUTPUT);

  digitalWrite(Heater,HIGH);
  
  // Pins D9 and D10 - 15.6 kHz 10bit
  TCCR1A = 0b00000011; // 10bit
  TCCR1B = 0b00001001; // x1 fast pwm
  
  while (aht10.begin() != true)
  {
    Serial.println(F("AHT1x not connected or fail to load calibration coefficient"));
    delay(2000);
  }

  SdFile::dateTimeCallback(dateTime);
  initializeSDCard();

  lcd.init();                      // initialize the lcd 
  lcd.init();
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(3,0);
  lcd.print("Hello, world!");
  lcd.setCursor(2,1);
  lcd.print("Ywrobot Arduino!");
   lcd.setCursor(0,2);
  lcd.print("Arduino LCM IIC 2004");
   lcd.setCursor(2,3);
  lcd.print("Power By Ec-yuan!");

    if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }
}
void loop()
{
  currentMillis = millis();

  if (FanTurnOnStatus)
    FanTurnOn();

  if (currentMillis - previousMillis >= durationUpdate)
  {
    previousMillis = currentMillis;

    temperature = aht10.readTemperature(); //read 6-bytes via I2C, takes 80 milliseconds
    //humidity = aht10.readHumidity(); //read another 6-bytes via I2C, takes 80 milliseconds
    humidity = aht10.readHumidity(AHTXX_USE_READ_DATA); //takes 0 milliseconds
    //humidity = 60;
    if (temperature != AHTXX_ERROR) //AHTXX_ERROR = 255, library returns 255 if error occurs
    {
      //if(Serial.available())
      //{
      Serial.print("\t");
      Serial.print(String(temperature));
      Serial.print(" ");
      Serial.println(String(humidity));
      //}
    DateTime now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
    
    Serial.println();
      File dataFile = SD.open(fileName, FILE_WRITE);
      if (dataFile)
      {
        dataFile.println(String(temperature) + "," + String(humidity));
        dataFile.close();
      }
      else
      {
        //Serial.println("error opening " + fileName);
        initializeSDCard();
      }
      if (temperature > baseLineTemp)
        digitalWrite(Heater, HIGH);
      else
        digitalWrite(Heater, LOW);

      if (humidity > baseLineHumidity - 1)
      {
        FanTurnOnStatus = true;
      }
      else
      {
        analogWrite(Fan, 0);
        FanTurnOnStatus = false;
        turnOn = true;
      }
    }
  }
}


void dateTime(uint16_t* date, uint16_t* time)
{
  *date = FAT_DATE(year, month, day);
  *time = FAT_TIME(hour, minute, second);
}
void initializeSDCard()
{
  if (SD.begin(chipSelectSD))
  {
    SD.mkdir(dataLogTargetDirectory);
    while (true)
    {
      if (dataLogNumberFile != 1)
        dataLogFileName = "log" + String(dataLogNumberFile);

      if (!SD.exists(dataLogTargetDirectory + dataLogFileName + dataLogFileFormat))
      {
        if (!createNewFile)
        {
          if (dataLogNumberFile != 1)
          {
            if (dataLogNumberFile == 2)
              dataLogFileName = "log";
            else
              dataLogFileName = "log" + String(dataLogNumberFile - 1);
          }
        }

        fileName = dataLogTargetDirectory + dataLogFileName + dataLogFileFormat;
        break;
      }
      dataLogNumberFile++;
    }

    if (createNewFile)
    {
      File dataFile = SD.open(fileName, FILE_WRITE);
      if (dataFile)
      {
        dataFile.println("Temprature,Humidity");
        dataFile.close();
        createNewFile = false;
      }
    }
  }
  //else
  //Serial.println("Card failed, or not present");
}

void FanTurnOn()//SD Card not insert cause to always Fan run (2 second) and run tempDuration middle of two mode previousTimeFanSpeed and previousTimeFanAction
{
  if (humidity > baseLineHumidity + 10)
  {
    durationFanSpeedUpdate = 320;
  }
  else if (humidity > baseLineHumidity + 5)
  {
    durationFanSpeedUpdate = 280;
  }
  else if (humidity > baseLineHumidity + 2)
  {
//      durationFanSpeedUpdate = 220;
    durationFanSpeedUpdate = 250;
  }
  else if (humidity > baseLineHumidity + 1)
  {
//      durationFanSpeedUpdate = 170;
    durationFanSpeedUpdate = 220;
  }
  else if (humidity > baseLineHumidity + 0.5)
  {
    durationFanSpeedUpdate = 120;
  }
  else if (humidity > baseLineHumidity - 1)
  {
    durationFanSpeedUpdate = 100;
  }
    
  if (turnOn)
  {
    if(firstRunFanSpeed)
    {
       analogWrite(Fan, 1023);
       firstRunFanSpeed = false;
       //Serial.println(durationFanSpeedUpdate);
    }
    else if (currentMillis - previousTimeFanSpeed >= durationFanSpeedUpdate)
    {
      previousTimeFanSpeed = currentMillis;
      turnOn = false;
      firstRunFanSpeed = true;
    }
    previousTimeFanAction = currentMillis;
  }
  else
  {
    if(firstRunFanAction)
    {
       analogWrite(Fan, 0);
       firstRunFanAction = false;
       //Serial.println(currentMillis - previousTimeFanAction);
    }
    else if (currentMillis - previousTimeFanAction >= durationFanActionUpdate)
    {
      turnOn = true;
      firstRunFanAction = true;
    }
    previousTimeFanSpeed = currentMillis;
  }
}
