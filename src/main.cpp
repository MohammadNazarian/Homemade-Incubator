#include <Arduino.h>
#include <Wire.h>
#include <TaskScheduler.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <AHTxx.h>
#include <RTClib.h>
#include <AsyncServo.h>
#include <DateConvL.h>
#include <math.h>

/*Tips:
Quail:
  Maintaining Mode: Temperature:10~17C , Humidity: 30~50~70% , until 4 or 5 days , Turn Eggs every 1 hours
  Setter Mode: Temperature:37~38C , Humidity: 50~65% , Turn Eggs every 1 hours
  Hatching Mode: Temperature:37~38C , Humidity: 60~70%
  نحوه قرارگیری تخم ها، نک تیز تخم ها به سمت پایین قرار میگیرد
  فصل مناسب جوجه کشی در بهار است
  پرکردن آب رطوبت رو باید بررسی کنم و الگوشو به دست بیارم
  نطفه سنجی در روز ۸ یا ۱۰ جوجه کشی انجام میشود و بهتره جایی باشه که خیلی محیطش خنک نباشد
  جوجه که به دنیا اومد تا ۲۴ ساعت دونه ای چیزی نمیخواد، فقط بعد از ۱۲ ساعت بعد از به دنیا اومدن محلول آب و قند که ۹۰ درصد آب و ۱۰ درصد قند هستش رو بهشون میدیم
  جوجه که به دنیا اومد، تا چند روز زیر مادر مصنوعی بگذاریم یا حداقل دمای مناسب رو براشون حفظ کنیم
  جوجه که به دنیا اومد، تا دو سه روز دمای ۳۶ تا ۳۷ درجه (مثل دمای هچر) نگه میداریم و در هفته اول دمای ۳۴ درجه رو حفظ کنیم براشون
*/

#define __DEBUG__ false
#define __REPORT__ false

// Tasks
void lcdShow();
void siren();
void sensor();
void fan();
void servoMotor();
void lcdBacklight();

// Menu
void menu();
void setTempratureAndHumidity(double, byte);
void setTime(bool);
void selectMode();
void setBacklight();
void setRotateMode();
void setRotateAngles();
// Other
void initializeEEPROM();
void rotaryEncoderState();

void serialPrintln(String string);
void serialPrint(String string);
void serialPrintReport(String string);

// A4,A5 for I2C
// #define PinRotaryEncoderCLK PC0 // A0
// #define PinRotaryEncoderDT PC1  // A1
#define PinRotaryEncoderCLK A0
#define PinRotaryEncoderDT A1
#define PinButtonRotaryEncoder A2
#define PinServo 3 // PWM
#define PinBuzzer 4
#define PinHeater 5 // PWM
#define PinFan 6    // PWM
// const int rs = PD7, en = PB0, d4 = PB1, d5 = PB2, d6 = PB3, d7 = PB4,backlightLCD = PB5;
// const int rs = 7, en = 8, d4 = 9, d5 = 10, d6 = 11, d7 = 12, backlightLCD = 13;

#define TurnOnHeater HIGH
#define TurnOffHeater LOW

// #define TurnOnFan HIGH
// #define TurnOnFan 255
#define TurnOnFan 1023
#define TurnOffFan LOW

#define ButtonPushed LOW
const int ButtonDebouncingTime = 50;
// const int ButtonLongPress = 2000;

// Task Scheduler
Scheduler scheduler;
#if !__DEBUG__ && __REPORT__
Task sensorTask(1 * TASK_SECOND, TASK_FOREVER, &sensor, &scheduler, false);
#else
Task sensorTask(2 * TASK_SECOND, TASK_FOREVER, &sensor, &scheduler, false);
#endif
Task lcdShowTask(1 * TASK_SECOND, TASK_FOREVER, &lcdShow, &scheduler, false);
Task lcdBacklightTask(1 * TASK_SECOND, TASK_FOREVER, &lcdBacklight, &scheduler, false);
Task sirenTask(500 * TASK_MILLISECOND, TASK_FOREVER, &siren, &scheduler, false);
Task fanTask(10 * TASK_MILLISECOND, TASK_FOREVER, &fan, &scheduler, false);
Task servoMotorTask(1 * TASK_SECOND, TASK_FOREVER, &servoMotor, &scheduler, false);

// EEPROM
// Addreses for EEPROM
byte secondElapsedAddr = 0, minuteElapsedAddr = 1, hourElapsedAddr = 2, dayElapsedAddr = 3, monthElapsedAddr = 4, yearElapsedAddr = 5; // year is int(2 byte)
byte ServoPositionAddr = 7, modeAddr = 8, H_thresholdAddr = 9, T_thresholdAddr = 10, lcdBacklightTimerAddr = 14, turnModeAddr = 16; // T_thresholdAddr is float(4 byte) , lcdBacklightTimerAddr is int(2 byte)
byte StartServoPositionAddr = 17, EndServoPositionAddr = 18;
// for Status
byte timeStatus = 0;
unsigned long autoSetCounter = 0;
unsigned long autoSetDuration = 10000;
byte menuSelection = 0;
byte mode = 3; // Mode 1 is Maintaining , Mode 2 is Incubation , Mode 3 is Hatching

// Rotary Encoder
int currentStateCLK;
int lastStateCLK;
unsigned long lastButtonPress = 0;
boolean buttonActive = false;

// Fan
// Turn OFF_ON Rapidly Fan
unsigned long durationFanSpeedUpdate = 100;
unsigned long durationFanActionUpdate = 1000;
unsigned long previousTimeFanSpeed = 0;
unsigned long previousTimeFanAction = 0;
bool firstRunFanSpeed = true;
bool firstRunFanAction = true;
bool turnOn = true;

// Servo
AsyncServoClass asyncServo;
byte previousServoPosition = 90;
byte turnMode = 1; // 1 : (normal) Every 1 hour, rotate 90 degree, 2 : (advanced) Short rotations every few minutes
byte startServoPosition = 45;
byte middleServoPosition = 90;
byte endServoPosition = 135;
bool initializeServo = false;
byte previousMinuteServo  = 0;
byte currentMinuteServo  = 0;

// Buzzer
byte sirenCounter = 0;
int Buzzerfrequency = 262;

// RTC (I2C)
RTC_DS1307 rtc;
int year = 1401; // for EEPROM.get()
TimeSpan timeElapsedHloder;
DateConvL dateC;
byte previousDay = 0;

// LCD (I2C)
// LiquidCrystal lcd(rs, en, d4, d5, d6, d7, backlightLCD, HIGH);
LiquidCrystal_I2C lcd(0x27, 16, 2);
// LiquidCrystal_I2C lcd(0x27, 16, 2); // set the LCD address to 0x27 for a 16 chars and 2 line display
byte TempIcon[8] = {B01110, B01010, B01010, B01110, B01110, B11111, B11111, B01110};
byte HumidityIcon[8] = {B00100, B00100, B01110, B01110, B11111, B11111, B11111, B01110};
byte degreeIcon[8] = {B11000, B11000, B00000, B00000, B00000, B00000, B00000, B00000};
int lcdShowStatus = 0;
bool changedLcdShowStatus = false;
int backlightCounter;
int lcdBacklightTimer = 30;

// AHT10 Sensor (I2C)
AHTxx AHT10Sensor(AHTXX_ADDRESS_X38, AHT1x_SENSOR);
// Initial Value for Tempurature and Humidity
double T_threshold = 38.0; // temporary Value
byte H_threshold = 60;     // temporary Value
// danger Zone Temperature and Humidity
double T_MaximumDanger = 43.0; // temporary Value
byte H_MaximumDanger = 70;     // temporary Value
double T_MinimumDanger = 30;   // temporary Value
byte H_MinimumDanger = 40;     // temporary Value
// Save Temp and Humidity
float baselineTemp = 37.8;
byte baselineHum = 70;

void setup()
{
  // Serial
  Serial.begin(9600);
  delay(20);
  Serial.println(F("> Temperature and Humidity Controller For Incubator <"));

  // EEPROM
  initializeEEPROM();

  // Rotary Encoder
  pinMode(PinRotaryEncoderCLK, INPUT);
  pinMode(PinRotaryEncoderDT, INPUT);
  pinMode(PinButtonRotaryEncoder, INPUT_PULLUP);
  lastStateCLK = digitalRead(PinRotaryEncoderCLK); // for rotary Encoder

  // Heater
  pinMode(PinHeater, OUTPUT);
  digitalWrite(PinHeater, TurnOffHeater);

  // Fan
  pinMode(PinFan, OUTPUT); // required for digitalWrite
  //  Pins D9 and D10 - 15.6 kHz 10bit
  TCCR1A = 0b00000011; // 10bit
  TCCR1B = 0b00001001; // x1 fast pwm
  analogWrite(PinFan, TurnOffFan);

  // Servo
  middleServoPosition = int((startServoPosition + endServoPosition) /2 );
  asyncServo.begin(PinServo, 1, 20, previousServoPosition);

  // Buzzer
  pinMode(PinBuzzer, OUTPUT);
  digitalWrite(PinBuzzer, LOW);

  // LCD
  // lcd.begin(16, 2);
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, TempIcon);
  lcd.createChar(1, HumidityIcon);
  lcd.createChar(2, degreeIcon);
  // write to LCD
  lcd.clear();
  lcd.print("Incubator");
  delay(1000);
  lcd.clear();
  lcd.print("Testing");
  lcd.setCursor(0, 1);
  lcd.print("sensor...");

  serialPrintln(F("Testing sensor..."));
  // RTC
  if (!rtc.begin())
  {
    serialPrintln("Couldn't find RTC");
    delay(2000);
  }
  if (!rtc.isrunning())
  {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  // rtc.adjust(DateTime(2023, 0, Day, Hrs, Min, Sec));

  // AHT10 Sensor
  while (AHT10Sensor.begin() != true)
  {
    serialPrintln(F("AHT10 not connected or fail to load calibration coefficient"));
    //(F()) save string to flash & keeps dynamic memory free
    lcd.clear();
    lcd.print("AHT10 error");
    delay(2000);
  }
  baselineTemp = AHT10Sensor.readTemperature();
  baselineHum = AHT10Sensor.readHumidity();

  lcd.clear();
  lcd.print("Ready");
  serialPrintln(F("Ready"));
  delay(1000);
  lcd.clear();

  setTempratureAndHumidity(T_threshold, H_threshold);

  DateTime savedTimeStart(EEPROM.get(yearElapsedAddr, year), EEPROM.read(monthElapsedAddr), EEPROM.read(dayElapsedAddr), EEPROM.read(hourElapsedAddr), EEPROM.read(minuteElapsedAddr), EEPROM.read(secondElapsedAddr));
  timeElapsedHloder = rtc.now() - savedTimeStart;
  if(timeElapsedHloder.minutes() < 0 || timeElapsedHloder.hours() < 0 || timeElapsedHloder.days() < 0)//If the stored time was cleared(Because of the low battery of the RTC chip)
  {
    rtc.adjust(savedTimeStart);
    timeElapsedHloder = rtc.now() - savedTimeStart;
  }

  changedLcdShowStatus = true;
  lcdShowTask.enable();
  sensorTask.enable();
  lcdBacklightTask.enable();
  servoMotorTask.enable();
}

void loop()
{
  scheduler.execute();
  asyncServo.update();
  rotaryEncoderState();
}

void rotaryEncoderState()
{
  if (digitalRead(PinButtonRotaryEncoder) == ButtonPushed)
  {
    if (millis() - lastButtonPress > ButtonDebouncingTime && buttonActive == false)
    {
      if (!lcdBacklightTask.isEnabled())
      {
        lcdBacklightTask.enableIfNot();
      }
      else
      {
        buttonActive = true;

        delay(500);
        menu();
      }
      lastButtonPress = millis();
    }
  }
  else
  {
    if (buttonActive == true)
    {
      buttonActive = false;
    }
    lastButtonPress = millis();
  }

  currentStateCLK = digitalRead(PinRotaryEncoderCLK);
  if (currentStateCLK != lastStateCLK) // for change status
  {
    lcdBacklightTask.enableIfNot();
    changedLcdShowStatus = true;;
    if (digitalRead(PinRotaryEncoderDT) == currentStateCLK) // Counter Clockwise rotate
    {

      if (lcdShowStatus > 0)
        lcdShowStatus--;
      else
        lcdShowStatus = 2;
    }
    else
    {
      if (lcdShowStatus < 2)
        lcdShowStatus++;
      else
        lcdShowStatus = 0;
    }
    lastStateCLK = currentStateCLK;
  }
}

void setTime(bool isSetTimeElapsed)
{
  lcdShowTask.disable();
  servoMotorTask.disable();

  autoSetCounter = millis();
  timeStatus = 1;

  DateTime time;
  int Year;
  byte Month;
  byte Day;
  byte Hrs;
  byte Min;
  byte Sec;

  if (isSetTimeElapsed)
  {
    DateTime time2(EEPROM.get(yearElapsedAddr, year), EEPROM.read(monthElapsedAddr), EEPROM.read(dayElapsedAddr), EEPROM.read(hourElapsedAddr), EEPROM.read(minuteElapsedAddr), EEPROM.read(secondElapsedAddr));
    time = time2;
    dateC.ToShamsi(time.year(), time.month(), time.day()); // converts global values of date and stores them to dateC

    Year = dateC.global_year;
    Month = dateC.global_month;
    Day = dateC.global_day;
    Hrs = time.hour();
    Min = time.minute();
    Sec = time.second();
  }
  else
  {
    time = rtc.now();
    dateC.ToShamsi(time.year(), time.month(), time.day()); // converts global values of date and stores them to dateC

    Year = dateC.global_year;
    Month = dateC.global_month;
    Day = dateC.global_day;
    Hrs = time.hour();
    Min = time.minute();
    Sec = time.second();
  }

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Y:");
  lcd.print(Year);
  lcd.setCursor(7, 0);
  lcd.print("M:");
  lcd.print(Month);
  lcd.setCursor(12, 0);
  lcd.print("D:");
  lcd.print(Day);

  lcd.setCursor(1, 1);
  lcd.print("H:");
  lcd.print(Hrs);
  lcd.setCursor(6, 1);
  lcd.print("M:");
  lcd.print(Min);
  lcd.setCursor(11, 1);
  lcd.print("S:");
  lcd.print(Sec);

  serialPrint(F("Set Time "));

  while (true)
  {
    scheduler.execute();

    currentStateCLK = digitalRead(PinRotaryEncoderCLK);

    if (currentStateCLK != lastStateCLK) // for change status
    {
      if (digitalRead(PinRotaryEncoderDT) == currentStateCLK) // Counter Clockwise rotate
      {
        autoSetCounter = millis();
        if (timeStatus == 1)
        {
          if (Year >= 1400)
          {
            Year = Year - 1;
            lcd.setCursor(2, 0);
            lcd.print("    ");
            lcd.setCursor(2, 0);
            lcd.print(Year);
          }
          else
          {
            Year = 1600;
            lcd.setCursor(2, 0);
            lcd.print("    ");
            lcd.setCursor(2, 0);
            lcd.print(Year);
          }
        }
        else if (timeStatus == 2)
        {
          if (Month > 1)
          {
            Month = Month - 1;
            lcd.setCursor(9, 0);
            lcd.print("  ");
            lcd.setCursor(9, 0);
            lcd.print(Month);
          }
          else
          {
            Month = 12;
            lcd.setCursor(9, 0);
            lcd.print("  ");
            lcd.setCursor(9, 0);
            lcd.print(Month);
          }
        }
        else if (timeStatus == 3)
        {
          if (Day > 1)
          {
            Day = Day - 1;
            lcd.setCursor(14, 0);
            lcd.print("   ");
            lcd.setCursor(14, 0);
            lcd.print(Day);
          }
          else
          {
            Day = 31;
            lcd.setCursor(14, 0);
            lcd.print("   ");
            lcd.setCursor(14, 0);
            lcd.print(Day);
          }
        }

        else if (timeStatus == 4)
        {

          if (Hrs != 0)
          {
            Hrs = Hrs - 1;
            lcd.setCursor(3, 1);
            lcd.print("  ");
            lcd.setCursor(3, 1);
            lcd.print(Hrs);
          }
          else
          {
            Hrs = 23;
            lcd.setCursor(3, 1);
            lcd.print("  ");
            lcd.setCursor(3, 1);
            lcd.print(Hrs);
          }
        }
        else if (timeStatus == 5)
        {
          if (Min != 0)
          {
            Min = Min - 1;
            lcd.setCursor(8, 1);
            lcd.print("  ");
            lcd.setCursor(8, 1);
            lcd.print(Min);
          }
          else
          {
            Min = 59;
            lcd.setCursor(8, 1);
            lcd.print("  ");
            lcd.setCursor(8, 1);
            lcd.print(Min);
          }
        }
        else if (timeStatus == 6)
        {
          if (Sec != 0)
          {
            Sec = Sec - 1;
            lcd.setCursor(13, 1);
            lcd.print("  ");
            lcd.setCursor(13, 1);
            lcd.print(Sec);
          }
          else
          {
            Sec = 59;
            lcd.setCursor(13, 1);
            lcd.print("  ");
            lcd.setCursor(13, 1);
            lcd.print(Sec);
          }
        }
      }

      else // Clockwise rotate
      {
        autoSetCounter = millis();
        if (timeStatus == 1)
        {
          if (Year < 1600)
          {
            Year = Year + 1;
            lcd.setCursor(2, 0);
            lcd.print("    ");
            lcd.setCursor(2, 0);
            lcd.print(Year);
          }
          else
          {
            Year = 1400;
            lcd.setCursor(2, 0);
            lcd.print("    ");
            lcd.setCursor(2, 0);
            lcd.print(Year);
          }
        }
        else if (timeStatus == 2)
        {
          if (Month < 12)
          {
            Month = Month + 1;
            lcd.setCursor(9, 0);
            lcd.print("  ");
            lcd.setCursor(9, 0);
            lcd.print(Month);
          }
          else
          {
            Month = 1;
            lcd.setCursor(9, 0);
            lcd.print("  ");
            lcd.setCursor(9, 0);
            lcd.print(Month);
          }
        }
        else if (timeStatus == 3)
        {
          if (Day < 31)
          {
            Day = Day + 1;
            lcd.setCursor(14, 0);
            lcd.print("   ");
            lcd.setCursor(14, 0);
            lcd.print(Day);
          }
          else
          {
            Day = 0;
            lcd.setCursor(14, 0);
            lcd.print("   ");
            lcd.setCursor(14, 0);
            lcd.print(Day);
          }
        }

        else if (timeStatus == 4)
        {
          if (Hrs < 23)
          {
            Hrs = Hrs + 1;
            lcd.setCursor(3, 1);
            lcd.print("  ");
            lcd.setCursor(3, 1);
            lcd.print(Hrs);
          }
          else
          {
            Hrs = 0;
            lcd.setCursor(3, 1);
            lcd.print("  ");
            lcd.setCursor(3, 1);
            lcd.print(Hrs);
          }
        }
        else if (timeStatus == 5)
        {
          if (Min < 59)
          {
            Min = Min + 1;
            lcd.setCursor(8, 1);
            lcd.print("  ");
            lcd.setCursor(8, 1);
            lcd.print(Min);
          }
          else
          {
            Min = 0;
            lcd.setCursor(8, 1);
            lcd.print("  ");
            lcd.setCursor(8, 1);
            lcd.print(Min);
          }
        }
        else if (timeStatus == 6)
        {
          if (Sec < 59)
          {
            Sec = Sec + 1;
            lcd.setCursor(13, 1);
            lcd.print("  ");
            lcd.setCursor(13, 1);
            lcd.print(Sec);
          }
          else
          {
            Sec = 0;
            lcd.setCursor(13, 1);
            lcd.print("  ");
            lcd.setCursor(13, 1);
            lcd.print(Sec);
          }
        }
      }
    }
    lastStateCLK = currentStateCLK;

    if (millis() - autoSetCounter >= autoSetDuration)
    {
      autoSetCounter = millis();
      if (timeStatus == 1)
      {
        lcd.setCursor(6, 0);
        lcd.print(">");
      }
      else if (timeStatus == 2)
      {
        lcd.setCursor(11, 0);
        lcd.print(">");
        lcd.setCursor(6, 0);
        lcd.print(" ");
      }
      else if (timeStatus == 3)
      {
        lcd.setCursor(0, 1);
        lcd.print(">");
        lcd.setCursor(11, 0);
        lcd.print(" ");
      }

      else if (timeStatus == 4)
      {
        lcd.setCursor(5, 1);
        lcd.print(">");
        lcd.setCursor(0, 1);
        lcd.print(" ");
      }
      else if (timeStatus == 5)
      {
        lcd.setCursor(10, 1);
        lcd.print(">");
        lcd.setCursor(5, 1);
        lcd.print(" ");
      }
      else if (timeStatus == 6)
      {
        lcd.setCursor(10, 1);
        lcd.print(" ");
        timeStatus = 0;
        serialPrintln(F("\nStart"));
        lcd.clear();
        delay(300);
        dateC.ToGregorian(Year, Month, Day);
        initializeServo = true;
        if (isSetTimeElapsed)
        {
          EEPROM.write(secondElapsedAddr, Sec);
          EEPROM.write(minuteElapsedAddr, Min);
          EEPROM.write(hourElapsedAddr, Hrs);
          EEPROM.write(dayElapsedAddr, dateC.global_day);
          EEPROM.write(monthElapsedAddr, dateC.global_month);
          EEPROM.put(yearElapsedAddr, dateC.global_year);
        }
        else
          rtc.adjust(DateTime(dateC.global_year, dateC.global_month, dateC.global_day, Hrs, Min, Sec));
        break;
      }
      timeStatus++;
    }

    if (digitalRead(PinButtonRotaryEncoder) == ButtonPushed)
    {
      if (millis() - lastButtonPress > ButtonDebouncingTime && buttonActive == false)
      {
        buttonActive = true;
        autoSetCounter = millis();
        if (timeStatus == 1)
        {
          lcd.setCursor(6, 0);
          lcd.print(">");
        }
        else if (timeStatus == 2)
        {
          lcd.setCursor(11, 0);
          lcd.print(">");
          lcd.setCursor(6, 0);
          lcd.print(" ");
        }
        else if (timeStatus == 3)
        {
          lcd.setCursor(0, 1);
          lcd.print(">");
          lcd.setCursor(11, 0);
          lcd.print(" ");
        }

        else if (timeStatus == 4)
        {
          lcd.setCursor(5, 1);
          lcd.print(">");
          lcd.setCursor(0, 1);
          lcd.print(" ");
        }
        else if (timeStatus == 5)
        {
          lcd.setCursor(10, 1);
          lcd.print(">");
          lcd.setCursor(5, 1);
          lcd.print(" ");
        }
        else if (timeStatus == 6)
        {
          lcd.setCursor(10, 1);
          lcd.print(" ");
          timeStatus = 0;
          lcd.clear();
          serialPrint(F("Start"));
          delay(300);
          dateC.ToGregorian(Year, Month, Day);
          initializeServo = true;
          if (isSetTimeElapsed)
          {
            EEPROM.write(secondElapsedAddr, Sec);
            EEPROM.write(minuteElapsedAddr, Min);
            EEPROM.write(hourElapsedAddr, Hrs);
            EEPROM.write(dayElapsedAddr, dateC.global_day);
            EEPROM.write(monthElapsedAddr, dateC.global_month);
            EEPROM.put(yearElapsedAddr, dateC.global_year);
          }
          else
            rtc.adjust(DateTime(dateC.global_year, dateC.global_month, dateC.global_day, Hrs, Min, Sec));
          break;
        }
        timeStatus++;
        lastButtonPress = millis();
      }
    }
    else
    {
      if (buttonActive == true)
      {
        buttonActive = false;
      }

      lastButtonPress = millis();
    }
  }
  changedLcdShowStatus = true;
  lcdShowTask.enable();
  servoMotorTask.enable();
}
void setTempratureAndHumidity(double t_threshold, byte h_threshold)
{
  sensorTask.disable();
  lcdShowTask.disable();

  autoSetCounter = millis();
  T_threshold = t_threshold;
  H_threshold = h_threshold;
  lcd.clear();
  lcd.print("Set Temperature:");
  lcd.setCursor(0, 1);
  lcd.print((float)T_threshold);
  lcd.write((byte)2);
  serialPrint(F("Set Temperature "));

  while (true) // for Temperature
  {
    scheduler.execute();

    currentStateCLK = digitalRead(PinRotaryEncoderCLK);

    if (currentStateCLK != lastStateCLK) // for change status
    {
      if (digitalRead(PinRotaryEncoderDT) == currentStateCLK) // Counter Clockwise rotate
      {

        if (T_threshold > 1.0)
        {
          T_threshold = T_threshold - 0.1;
        }
        else
        {
          T_threshold = 50.0;
        }

        EEPROM.put(T_thresholdAddr, T_threshold);
        lcd.setCursor(0, 1);
        lcd.print(T_threshold);
        lcd.write((byte)2);
        lcd.print("  ");
        autoSetCounter = millis();
      }

      else // Counter Clockwise rotate
      {
        if (T_threshold < 50.0)
        {
          T_threshold = T_threshold + 0.1;
        }
        else
        {
          T_threshold = 1.0;
        }

        EEPROM.put(T_thresholdAddr, T_threshold);
        lcd.setCursor(0, 1);
        lcd.print(T_threshold);
        lcd.write((byte)2);
        lcd.print("  ");
        autoSetCounter = millis();
      }
    }
    lastStateCLK = currentStateCLK;

    if (digitalRead(PinButtonRotaryEncoder) == ButtonPushed)
    {
      if (millis() - lastButtonPress > ButtonDebouncingTime && buttonActive == false)
      {
        buttonActive = true;
        autoSetCounter = millis();
        delay(300);
        EEPROM.put(T_thresholdAddr, T_threshold);
        serialPrint(String(T_threshold));
        serialPrint(F("C"));
        lastButtonPress = millis();
        break;
      }
    }
    else
    {
      if (buttonActive == true)
      {
        buttonActive = false;
      }
      lastButtonPress = millis();
    }

    if (millis() - autoSetCounter >= autoSetDuration)
    {
      autoSetCounter = millis();
      EEPROM.put(T_thresholdAddr, T_threshold);
      serialPrint(String(T_threshold));
      serialPrint(F("C"));
      break;
    }
  }

  lcd.clear();
  lcd.print("Set Humidity:");
  lcd.setCursor(0, 1);
  lcd.print(H_threshold);
  lcd.print("%");
  serialPrint(F("\nSet Humidity "));

  while (true) // for Humidity
  {
    scheduler.execute();

    currentStateCLK = digitalRead(PinRotaryEncoderCLK);

    if (currentStateCLK != lastStateCLK) // for change status
    {

      if (digitalRead(PinRotaryEncoderDT) == currentStateCLK) // Counter Clockwise rotate
      {
        if (H_threshold > 1)
        {
          H_threshold--;
        }
        else
        {
          H_threshold = 99;
        }

        EEPROM.write(H_thresholdAddr, H_threshold);
        lcd.setCursor(0, 1);
        lcd.print(H_threshold);
        lcd.print("%");
        lcd.print("  ");
        autoSetCounter = millis();
      }
      else // Counter Clockwise rotate
      {
        if (H_threshold < 100)
        {
          H_threshold++;
        }
        else
        {
          H_threshold = 1;
        }
        EEPROM.write(H_thresholdAddr, H_threshold);
        lcd.setCursor(0, 1);
        lcd.print(H_threshold);
        lcd.print("%");
        lcd.print("  ");
        autoSetCounter = millis();
      }
    }
    lastStateCLK = currentStateCLK;

    if (digitalRead(PinButtonRotaryEncoder) == ButtonPushed)
    {
      if (millis() - lastButtonPress > ButtonDebouncingTime && buttonActive == false)
      {
        buttonActive = true;
        autoSetCounter = millis();
        delay(300);
        EEPROM.write(H_thresholdAddr, H_threshold);
        serialPrint(String(H_threshold));
        serialPrint(F("%"));
        lastButtonPress = millis();
        break;
      }
    }
    else
    {
      if (buttonActive == true)
      {
        buttonActive = false;
      }
      lastButtonPress = millis();
    }

    if (millis() - autoSetCounter >= autoSetDuration)
    {
      autoSetCounter = millis();
      EEPROM.write(H_thresholdAddr, H_threshold);
      serialPrint(String(H_threshold));
      serialPrint(F("%"));
      break;
    }
  }
  lcd.clear();
  serialPrintln(F("\nStart"));
  T_MaximumDanger = T_threshold + 2;
  H_MaximumDanger = H_threshold + 10;
  T_MinimumDanger = T_threshold - 2;
  H_MinimumDanger = H_threshold - 20;

  initializeServo = true;
  sensorTask.enable();
  changedLcdShowStatus = true;
  lcdShowTask.enable();
}
void selectMode()
{
  lcdShowTask.disable();
  byte modeSelection = 1;
  switch (mode)
  {
    case 1:
      modeSelection = 1;
      lcd.clear();
      lcd.print(">1_ Maintaining");
      lcd.setCursor(0, 1);
      lcd.print(" 2_ Incubation");
      break;
    case 2:
      modeSelection = 2;
      lcd.clear();
      lcd.print(" 1_ Maintaining");
      lcd.setCursor(0, 1);
      lcd.print(">2_ Incubation");
      break;

    case 3:
      modeSelection = 3;
      lcd.clear();
      lcd.print(" 2_ Incubation");
      lcd.setCursor(0, 1);
      lcd.print(">3_ Hatching");
      break;

    default:
      modeSelection = 1;
      lcd.clear();
      lcd.print(">1_ Maintaining");
      lcd.setCursor(0, 1);
      lcd.print(" 2_ Incubation");
      break;
  }

  while (true)
  {
    scheduler.execute();

    currentStateCLK = digitalRead(PinRotaryEncoderCLK);
    if (currentStateCLK != lastStateCLK) // for change status
    {
      if (digitalRead(PinRotaryEncoderDT) == currentStateCLK)
      {
        if (modeSelection <= 1)
        {
          modeSelection = 4;
          lcd.clear();
          lcd.print(" 3_ Hatching");
          lcd.setCursor(0, 1);
          lcd.print(">Cancel");
        }
        else
        {
          modeSelection--;

          if (modeSelection == 1)
          {
            lcd.clear();
            lcd.print(">1_ Maintaining");
            lcd.setCursor(0, 1);
            lcd.print(" 2_ Incubation");
          }
          else if (modeSelection == 2)
          {
            lcd.clear();
            lcd.print(">2_ Incubation");
            lcd.setCursor(0, 1);
            lcd.print(" 3_ Hatching");
          }
          else if (modeSelection == 3)
          {
            lcd.clear();
            lcd.print(">3_ Hatching");
            lcd.setCursor(0, 1);
            lcd.print(" Cancel");
          }
        }
      }
      else
      {
        if (modeSelection >= 4)
        {
          modeSelection = 1;
          lcd.clear();
          lcd.print(">1_ Maintaining");
          lcd.setCursor(0, 1);
          lcd.print(" 2_ Incubation");
        }
        else
        {
          modeSelection++;

          if (modeSelection == 2)
          {
            lcd.clear();
            lcd.print(" 1_ Maintaining");
            lcd.setCursor(0, 1);
            lcd.print(">2_ Incubation");
          }
          else if (modeSelection == 3)
          {
            lcd.clear();
            lcd.print(" 2_ Incubation");
            lcd.setCursor(0, 1);
            lcd.print(">3_ Hatching");
          }
          else if (modeSelection == 4)
          {
            lcd.clear();
            lcd.print(" 3_ Hatching");
            lcd.setCursor(0, 1);
            lcd.print(">Cancel");
          }
        }
      }
    }
    lastStateCLK = currentStateCLK;

    if (digitalRead(PinButtonRotaryEncoder) == ButtonPushed)
    {
      if (millis() - lastButtonPress > ButtonDebouncingTime && buttonActive == false)
      {
        buttonActive = true;
        delay(300);
        switch (modeSelection)
        {
        case 1:
          mode = 1;
          serialPrintln(F("Selected Maintaining"));
          setTempratureAndHumidity(13.0, 50);
          break;
        case 2:
          mode = 2;
          serialPrintln(F("Selected Incubation(Setter)"));
          setTempratureAndHumidity(37.5, 60);
          break;
        case 3:
          mode = 3;
          serialPrintln(F("Selected Hatcher"));
          setTempratureAndHumidity(37.2, 70);
          break;
        case 4:
          serialPrintln(F("Selected Cancel"));
          break;
        }
        lcd.clear();
        modeSelection = 0;

        DateTime dateTimeNow = rtc.now();
        EEPROM.write(secondElapsedAddr, dateTimeNow.second());
        EEPROM.write(minuteElapsedAddr, dateTimeNow.minute());
        EEPROM.write(hourElapsedAddr, dateTimeNow.hour());
        EEPROM.write(dayElapsedAddr, dateTimeNow.day());
        EEPROM.write(monthElapsedAddr, dateTimeNow.month());
        EEPROM.put(yearElapsedAddr, dateTimeNow.year());

        EEPROM.update(modeAddr, mode);

        initializeServo = true;
        delay(300);
        lastButtonPress = millis();
        break;
      }
    }
    else
    {
      if (buttonActive == true)
      {
        buttonActive = false;
      }
      lastButtonPress = millis();
    }
  }
  changedLcdShowStatus = true;
  lcdShowTask.enable();
}
void menu()
{
  //  Modes:
  //  lcd.print("set Mode");
  //  lcd.print("set Temp & humidity");
  //  lcd.print("set Time");
  //  lcd.print("set TimeElapsed");
  //  lcd.print("set Backlight");
  //  lcd.print("set Rotate Mode");
  //  lcd.print("Cancel");

  lcdShowTask.disable();
  lcdBacklightTask.disable();
  lcd.backlight();

  autoSetCounter = millis();
  menuSelection = 1;
  lcd.clear();
  lcd.print(">set Mode");
  lcd.setCursor(0, 1);
  lcd.print(" set ");
  lcd.write((byte)0);
  lcd.print(" & ");
  lcd.write((byte)1);
  serialPrintln(F("Setting Menu"));

  while (menuSelection >= 1)
  {
    scheduler.execute();

    currentStateCLK = digitalRead(PinRotaryEncoderCLK);
    if (currentStateCLK != lastStateCLK) // for change status
    {
      if (digitalRead(PinRotaryEncoderDT) == currentStateCLK)
      {
        autoSetCounter = millis();
        if (menuSelection <= 1)
        {
          menuSelection = 7;
          lcd.clear();
          lcd.print(" set Rotate Mode");
          lcd.setCursor(0, 1);
          lcd.print(">Cancel");
        }
        else
        {
          menuSelection--;

          if (menuSelection == 1)
          {
            lcd.clear();
            lcd.print(">set Mode");
            lcd.setCursor(0, 1);
            lcd.print(" set ");
            lcd.write((byte)0);
            lcd.print(" & ");
            lcd.write((byte)1);
          }
          else if (menuSelection == 2)
          {
            lcd.clear();
            lcd.print(">set ");
            lcd.write((byte)0);
            lcd.print(" & ");
            lcd.write((byte)1);
            lcd.setCursor(0, 1);
            lcd.print(" set Time");
          }
          else if (menuSelection == 3)
          {
            lcd.clear();
            lcd.print(">set Time");
            lcd.setCursor(0, 1);
            lcd.print(" set TimeElapsed");
          }
          else if (menuSelection == 4)
          {
            lcd.clear();
            lcd.print(">set TimeElapsed");
            lcd.setCursor(0, 1);
            lcd.print(" set Backlight");
          }
          else if (menuSelection == 5)
          {
            lcd.clear();
            lcd.print(">set Backlight");
            lcd.setCursor(0, 1);
            lcd.print(" set Rotate Mode");
          }
          else if (menuSelection == 6)
          {
            lcd.clear();
            lcd.print(">set Rotate Mode");
            lcd.setCursor(0, 1);
            lcd.print(" Cancel");
          }
        }
      }
      else
      {
        autoSetCounter = millis();
        if (menuSelection >= 7)
        {
          menuSelection = 1;
          lcd.clear();
          lcd.print(">set Mode");
          lcd.setCursor(0, 1);
          lcd.print(" set ");
          lcd.write((byte)0);
          lcd.print(" & ");
          lcd.write((byte)1);
        }
        else
        {
          menuSelection++;

          if (menuSelection == 2)
          {
            lcd.clear();
            lcd.print(" set Mode");
            lcd.setCursor(0, 1);
            lcd.print(">set ");
            lcd.write((byte)0);
            lcd.print(" & ");
            lcd.write((byte)1);
          }
          else if (menuSelection == 3)
          {
            lcd.clear();
            lcd.print(" set ");
            lcd.write((byte)0);
            lcd.print(" & ");
            lcd.write((byte)1);

            lcd.setCursor(0, 1);
            lcd.print(">set Time");
          }
          else if (menuSelection == 4)
          {
            lcd.clear();
            lcd.print(" set Time");
            lcd.setCursor(0, 1);
            lcd.print(">set TimeElapsed");
          }
          else if (menuSelection == 5)
          {
            lcd.clear();
            lcd.print(" set TimeElapsed");
            lcd.setCursor(0, 1);
            lcd.print(">set Backlight");
          }
          else if (menuSelection == 6)
          {
            lcd.clear();
            lcd.print(" set Backlight");
            lcd.setCursor(0, 1);
            lcd.print(">set Rotate Mode");
          }
          else if (menuSelection == 7)
          {
            lcd.clear();
            lcd.print(" set Rotate Mode");
            lcd.setCursor(0, 1);
            lcd.print(">Cancel");
          }
        }
      }
    }
    lastStateCLK = currentStateCLK;

    if (digitalRead(PinButtonRotaryEncoder) == ButtonPushed)
    {
      if (millis() - lastButtonPress > ButtonDebouncingTime && buttonActive == false)
      {
        buttonActive = true;
        autoSetCounter = millis();
        delay(300);
        switch (menuSelection)
        {
        case 1:
          serialPrintln(F("Selected set Mode"));
          selectMode();
          break;
        case 2:
          serialPrintln(F("Selected set Temperature and Humidity"));
          setTempratureAndHumidity(T_threshold, H_threshold);
          break;
        case 3:
          serialPrintln(F("Selected set Time"));
          setTime(false);
          break;
        case 4:
          serialPrintln(F("Selected set Time Elapsed"));
          setTime(true);
          break;
        case 5:
          serialPrintln(F("Selected set Backlight"));
          setBacklight();
          break;
        case 6:
          serialPrintln(F("Selected set Rotate Mode"));
          setRotateMode();
          break;
        case 7:
          lcd.clear();
          serialPrintln(F("Selected Cancel"));
        }
        delay(300);
        lastButtonPress = millis();
        changedLcdShowStatus = true;
        lcdShowTask.enable();
        lcdBacklightTask.enable();
        break;
      }
    }
    else
    {
      if (buttonActive == true)
      {
        buttonActive = false;
      }
      lastButtonPress = millis();
    }

    if (millis() - autoSetCounter >= autoSetDuration)
    {
      autoSetCounter = millis();
      lcd.clear();
      delay(300);
      changedLcdShowStatus = true;
      lcdShowTask.enable();
      lcdBacklightTask.enable();
      break;
    }
  }
  
  initializeServo = true;
}

void setBacklight()
{
  lcdShowTask.disable();
  autoSetCounter = millis();
  if (lcdBacklightTimer == 0)
  {
    lcd.clear();
    lcd.print("Backlight");
    lcd.setCursor(0, 1);
    lcd.print("Always ON");
  }
  else
  {
    lcd.clear();
    lcd.print("Backlight ON in");
    lcd.setCursor(0, 1);
    lcd.print(String(lcdBacklightTimer) + " seconds");
  }

  while (true)
  {
    scheduler.execute();

    currentStateCLK = digitalRead(PinRotaryEncoderCLK);
    if (currentStateCLK != lastStateCLK) // for change status
    {
      if (digitalRead(PinRotaryEncoderDT) == currentStateCLK)
      {
        lcdBacklightTimer -= 10;

        autoSetCounter = millis();

        if (lcdBacklightTimer < 0)
          lcdBacklightTimer = 240;

        if (lcdBacklightTimer > 0)
        {
          lcd.clear();
          lcd.print("Backlight ON in");
          lcd.setCursor(0, 1);
          lcd.print(String(lcdBacklightTimer) + " seconds");
        }
        else if (lcdBacklightTimer == 0)
        {
          lcd.clear();
          lcd.print("Backlight");
          lcd.setCursor(0, 1);
          lcd.print("Always ON");
        }
      }
      else
      {
        lcdBacklightTimer += 10;

        autoSetCounter = millis();

        if (lcdBacklightTimer > 240)
          lcdBacklightTimer = 0;

        if (lcdBacklightTimer == 0)
        {
          lcd.clear();
          lcd.print("Backlight");
          lcd.setCursor(0, 1);
          lcd.print("Always ON");
        }
        else if (lcdBacklightTimer < 240)
        {
          lcd.clear();
          lcd.print("Backlight ON in");
          lcd.setCursor(0, 1);
          lcd.print(String(lcdBacklightTimer) + " seconds");
        }
      }
    }
    lastStateCLK = currentStateCLK;

    if (digitalRead(PinButtonRotaryEncoder) == ButtonPushed)
    {
      if (millis() - lastButtonPress > ButtonDebouncingTime && buttonActive == false)
      {
        EEPROM.put(lcdBacklightTimerAddr, lcdBacklightTimer);
        lcd.clear();
        delay(300);
        lastButtonPress = millis();
        break;
      }
    }
    else
    {
      if (buttonActive == true)
      {
        buttonActive = false;
      }
      lastButtonPress = millis();
    }

    if (millis() - autoSetCounter >= autoSetDuration)
    {
      autoSetCounter = millis();
      EEPROM.put(lcdBacklightTimerAddr, lcdBacklightTimer);
      lcd.clear();
      delay(300);
      break;
    }
  }
  changedLcdShowStatus = true;
  lcdShowTask.enable();
}

void setRotateMode()
{
  lcdShowTask.disable();
  byte rotateModeSelection = 1;

  bool selected = false;
  switch (turnMode)
  {
  case 1:
    rotateModeSelection = 1;
    lcd.clear();
    lcd.print(">Rotate per hour");
    lcd.setCursor(0, 1);
    lcd.print(" Rotate slowly");
    break;
  case 2:
    rotateModeSelection = 2;
    lcd.clear();
    lcd.print(" Rotate per hour");
    lcd.setCursor(0, 1);
    lcd.print(">Rotate slowly");
    break;
  default:
    rotateModeSelection = 1;
    lcd.clear();
    lcd.print(">Rotate per hour");
    lcd.setCursor(0, 1);
    lcd.print(" Rotate slowly");
    break;
  }

  while (true)
  {
    scheduler.execute();

    currentStateCLK = digitalRead(PinRotaryEncoderCLK);
    if (currentStateCLK != lastStateCLK) // for change status
    {
      if (digitalRead(PinRotaryEncoderDT) == currentStateCLK)
      {
        if (rotateModeSelection <= 1)
        {
          rotateModeSelection = 3;
          lcd.clear();
          lcd.print(" Rotate slowly");
          lcd.setCursor(0, 1);
          lcd.print(">Cancel");
        }
        else
        {
          rotateModeSelection--;

          if (rotateModeSelection == 1)
          {
            lcd.clear();
            lcd.print(">Rotate per hour");
            lcd.setCursor(0, 1);
            lcd.print(" Rotate slowly");
          }
          else if (rotateModeSelection == 2)
          {
            lcd.clear();
            lcd.print(">Rotate slowly");
            lcd.setCursor(0, 1);
            lcd.print(" Cancel");
          }
        }
      }
      else
      {
        if (rotateModeSelection >= 3)
        {
          rotateModeSelection = 1;
          lcd.clear();
          lcd.print(">Rotate per hour");
          lcd.setCursor(0, 1);
          lcd.print(" Rotate slowly");
        }
        else
        {
          rotateModeSelection++;

          if (rotateModeSelection == 2)
          {
            lcd.clear();
            lcd.print(" Rotate per hour");
            lcd.setCursor(0, 1);
            lcd.print(">Rotate slowly");
          }
          else if (rotateModeSelection == 3)
          {
            lcd.clear();
            lcd.print(" Rotate slowly");
            lcd.setCursor(0, 1);
            lcd.print(">Cancel");
          }
        }
      }
    }
    lastStateCLK = currentStateCLK;

    if (digitalRead(PinButtonRotaryEncoder) == ButtonPushed)
    {
      if (millis() - lastButtonPress > ButtonDebouncingTime && buttonActive == false)
      {
        buttonActive = true;
        delay(300);
        switch (rotateModeSelection)
        {
        case 1:
          if(!selected)
          {
            turnMode = 1;
            EEPROM.write(turnModeAddr,1);

            serialPrintln(F("Selected Rotate per hour"));

            break;
          }
        case 2:
          if(!selected)
          {
            turnMode = 2;
            EEPROM.write(turnModeAddr,2);
            
            serialPrintln(F("Selected Rotate slowly"));
            break;
          }

          case 3:
            if(!selected)
            {
              serialPrintln(F("Selected Cancel"));
              break;
            }
        }
        lcd.clear();
        rotateModeSelection = 0;

        EEPROM.write(StartServoPositionAddr, startServoPosition);
        EEPROM.write(EndServoPositionAddr, endServoPosition);
        middleServoPosition = int((startServoPosition + endServoPosition) /2 );

        delay(300);
        lastButtonPress = millis();
        break;
      }
    }
    else
    {
      if (buttonActive == true)
      {
        buttonActive = false;
      }
      lastButtonPress = millis();
    }
  }
  
  changedLcdShowStatus = true;
  lcdShowTask.enable();
  initializeServo = true;
}

void lcdShow() // delay 53 millisecond
{
  DateTime savedTimeStart(EEPROM.get(yearElapsedAddr, year), EEPROM.read(monthElapsedAddr), EEPROM.read(dayElapsedAddr), EEPROM.read(hourElapsedAddr), EEPROM.read(minuteElapsedAddr), EEPROM.read(secondElapsedAddr));
  timeElapsedHloder = rtc.now() - savedTimeStart;

  if (baselineTemp != AHTXX_ERROR)
  {
    if (lcdShowStatus == 0)
    {
      if (baselineTemp < T_MaximumDanger || mode == 1)
      {
        if(changedLcdShowStatus)
        {
          changedLcdShowStatus = false;
          lcd.clear();

          lcd.setCursor(0, 0);
          lcd.write((byte)0);
          lcd.setCursor(15, 0);
          lcd.write((byte)2);

          lcd.setCursor(0, 1);
          lcd.write((byte)1);
          lcd.setCursor(15, 1);
          lcd.print("%");
        }
        
        lcd.setCursor(2, 0);
        lcd.print(baselineTemp);
        lcd.setCursor(9, 0);
        lcd.print(T_threshold);

        lcd.setCursor(5, 1);
        lcd.print(baselineHum);
        lcd.setCursor(10, 1);
        lcd.print(H_threshold);

        if (baselineTemp < T_threshold)
        {
          lcd.setCursor(8, 0);
          lcd.print("<");
        }
        else if (baselineTemp == T_threshold)
        {
          lcd.setCursor(8, 0);
          lcd.print("=");
        }
        else
        {
          lcd.setCursor(8, 0);
          lcd.print(">");
        }

        if (baselineHum < H_threshold)
        {
          lcd.setCursor(8, 1);
          lcd.print("<");
        }
        else if (baselineHum == H_threshold)
        {
          lcd.setCursor(8, 1);
          lcd.print("=");
        }
        else
        {
          lcd.setCursor(8, 1);
          lcd.print(">");
        }
      }
      else
      {
        changedLcdShowStatus=true;
        lcd.clear();
        lcd.print("Temp is High!!!");
        lcd.setCursor(1, 3);
        lcd.print(baselineTemp);
        lcd.print("C");
      }
    }
    else if (lcdShowStatus == 1)
    {
      DateTime now = rtc.now();
      dateC.ToShamsi(now.year(), now.month(), now.day()); // converts global values of date and stores them to dateC
      if(changedLcdShowStatus)
      {
        changedLcdShowStatus = false;
        previousDay = now.day();
        lcd.clear();
        lcd.print("Date: ");

        lcd.setCursor(0,1);
        lcd.print("Time: ");

        lcd.setCursor(6, 0);
        lcd.print(String(dateC.global_year,DEC) + "/" + String(dateC.global_month,DEC) + "/" + String(dateC.global_day,DEC));
      }
      
      if(now.day() != previousDay)
      {
        previousDay = now.day();
        lcd.setCursor(6, 0);
        lcd.print(String(dateC.global_year,DEC) + "/" + String(dateC.global_month,DEC) + "/" + String(dateC.global_day,DEC));
      }

      lcd.setCursor(6, 1);
      lcd.print(String(now.hour(),DEC) + ":" + String(now.minute(),DEC) + ":" + String(now.second(),DEC));
    }
    else if (lcdShowStatus == 2)
    {

      // serialPrintln("Saved in EEPROM > " + String(EEPROM.get(yearElapsedAddr, year)) + "/" + String(EEPROM.read(monthElapsedAddr)) + "/" + String(EEPROM.read(dayElapsedAddr)) + "|| " + String(EEPROM.read(hourElapsedAddr)) + ":" + String(EEPROM.read(minuteElapsedAddr)) + ":"+ String(EEPROM.read(secondElapsedAddr)));
      // serialPrintln("RTC NOW > " +  String(rtc.now().year()) + "/" + String(rtc.now().month()) + "/" + String(rtc.now().day()) + "|| " + String(rtc.now().hour()) + ":" + String(rtc.now().minute()) + ":"+ String(rtc.now().second()));
      // serialPrintln("timeElapsed > days: " + String(timeElapsed.days()) +  " hours: " + String(timeElapsed.hours())  +  " minutes: " + String(timeElapsed.minutes()) +  " seconds: " + String(timeElapsed.seconds()));

      if(changedLcdShowStatus)
      {
        changedLcdShowStatus = false;

        lcd.clear();
        lcd.print("Day:");
        lcd.setCursor(8, 0);
        lcd.print("Hour:");

        lcd.setCursor(0, 1);
        lcd.print("Min:");

        lcd.setCursor(9, 1);
        lcd.print("Sec:");
      }
      lcd.setCursor(5, 0);
      lcd.print(" ");
      lcd.setCursor(4, 0);
      lcd.print(timeElapsedHloder.days());

      lcd.setCursor(15, 0);
      lcd.print(" ");
      lcd.setCursor(13, 0);
      lcd.print(timeElapsedHloder.hours());

      lcd.setCursor(5, 1);
      lcd.print(" ");
      lcd.setCursor(4, 1);
      lcd.print(timeElapsedHloder.minutes());

      lcd.setCursor(15, 1);
      lcd.print(" ");
      lcd.setCursor(13, 1);
      lcd.print(timeElapsedHloder.seconds());
    }
  }
  else
  {
    lcdBacklightTask.disable(); // If there is a problem, the screen will stay on for information
    lcd.backlight();
    lcd.clear();
    lcd.print("No Sensor data.");
    lcd.setCursor(0, 1);
    lcd.print("System Halted.");
  }
}

void siren()
{

  if (sirenCounter == 3)
  {
    sirenCounter = 0;
    tone(PinBuzzer, Buzzerfrequency);
  }
  else
  {
    sirenCounter++;
    noTone(PinBuzzer);
  }
}

void sensor()
{
  serialPrintReport(String(baselineTemp, 1));
  serialPrintReport(F(","));
  serialPrintReport(String(baselineHum) + "\n");
  baselineTemp = AHT10Sensor.readTemperature();
  scheduler.execute();
  baselineHum = AHT10Sensor.readHumidity();

  // TEMPERATURE
  if (baselineTemp != AHTXX_ERROR)
  {
    if (baselineTemp < T_threshold)
    {
      digitalWrite(PinHeater, TurnOnHeater);
      sirenTask.disable();
      noTone(PinBuzzer);
    }
    else if (baselineTemp >= T_MaximumDanger && mode != 1)
    {
      digitalWrite(PinHeater, TurnOffHeater);
      sirenTask.enableIfNot();
      fanTask.enableIfNot();
    }
    else
    {
      digitalWrite(PinHeater, TurnOffHeater);
      sirenTask.disable();
      noTone(PinBuzzer);
    }

    // HUMIDITY
    if (baselineHum < H_threshold)
    {
      if (baselineTemp < T_MaximumDanger)
      {
        fanTask.disable();
        analogWrite(PinFan, TurnOffFan);
      }
    }
    else
      fanTask.enableIfNot();
  }
  else
  {
    sirenTask.enableIfNot();

    digitalWrite(PinHeater, TurnOffHeater);
    analogWrite(PinFan, TurnOffFan);

    serialPrint(F("Failed to read Sensor _ reset: "));
    serialPrintln(String(AHT10Sensor.softReset()));
    if (AHT10Sensor.softReset())
      serialPrintln(F("softReset is successful"));
    else if (AHT10Sensor.softReset())
      serialPrintln(F("softReset is failed"));
  }
}

void fan() // delay 1 millisecond
{
  if(mode == 2)
  {
    if (fanTask.isFirstIteration())
    {
      turnOn = true;
    }
    unsigned long currentMillis = millis();

    durationFanActionUpdate = 1000;
    if (baselineHum > H_threshold + 15)
    {
      analogWrite(PinFan, TurnOnFan);
      return;
    }
    else if (baselineHum > H_threshold + 10)
    {
      durationFanSpeedUpdate = 1500;
      durationFanActionUpdate = 750;
    }
    else if (baselineHum > H_threshold + 5)
    {
      durationFanSpeedUpdate = 380;
    }
    else if (baselineHum > H_threshold + 2)
    {
      durationFanSpeedUpdate = 350;
    }
    else if (baselineHum > H_threshold + 1)
    {
      durationFanSpeedUpdate = 320;
    }
    else if (baselineHum > H_threshold + 0.5)
    {
      durationFanSpeedUpdate = 250;
    }
    else if (baselineHum > H_threshold - 1)
    {
      durationFanSpeedUpdate = 200;
    }

    if (turnOn)
    {
      if (firstRunFanSpeed)
      {
        analogWrite(PinFan, TurnOnFan);
        firstRunFanSpeed = false;
      }
      else if (currentMillis - previousTimeFanSpeed >= durationFanSpeedUpdate)
      {
        firstRunFanSpeed = true;
        previousTimeFanSpeed = currentMillis;
        turnOn = false;
      }
      previousTimeFanAction = currentMillis;
    }
    else
    {
      if (firstRunFanAction)
      {
        analogWrite(PinFan, TurnOffFan);
        firstRunFanAction = false;
      }
      else if (currentMillis - previousTimeFanAction >= durationFanActionUpdate)
      {
        turnOn = true;
        firstRunFanAction = true;
      }
      previousTimeFanSpeed = currentMillis;
    }
  }
  else // mode 1 and 3 (maintaining and hatching)
  {
    analogWrite(PinFan, TurnOnFan);
  }
}

void servoMotor() // delay most 2 millisecond
{
  DateTime savedTimeStart(EEPROM.get(yearElapsedAddr, year), EEPROM.read(monthElapsedAddr), EEPROM.read(dayElapsedAddr), EEPROM.read(hourElapsedAddr), EEPROM.read(minuteElapsedAddr), EEPROM.read(secondElapsedAddr));
  TimeSpan timeElapsed = rtc.now() - savedTimeStart;
  currentMinuteServo = timeElapsed.minutes();

  if(currentMinuteServo != previousMinuteServo || initializeServo)
  {
    previousMinuteServo = currentMinuteServo;
    initializeServo = false;
    if (mode == 1 || mode == 2) // if maitaining and Setter
    {
      float turnRatio = (endServoPosition - startServoPosition) / 60.00;
      if (timeElapsed.hours() % 2 == 0)
      {
        int servoPos;
        if(turnMode == 1)//(normal) Every 1 hour, rotate 90 degree
        {
          servoPos = startServoPosition;
          asyncServo.goTo(servoPos);
        }
        else if(turnMode == 2)//(advanced) Short rotations every few minutes
        {
            servoPos = startServoPosition + int(ceil(timeElapsed.minutes() * turnRatio  ));
            asyncServo.goTo(servoPos);
        }
        EEPROM.write(ServoPositionAddr, servoPos);
      }
      else if (timeElapsed.hours() % 2 == 1)
      {
        int servoPos;
        if(turnMode == 1)//(normal) Every 1 hour, rotate 90 degree
        {
          servoPos = endServoPosition;
          asyncServo.goTo(servoPos);

        }
        else if(turnMode == 2)//(advanced) Short rotations every few minutes
        {
          servoPos = endServoPosition - int(ceil(timeElapsed.minutes() * turnRatio));
          asyncServo.goTo(servoPos);
        }

        EEPROM.write(ServoPositionAddr, servoPos);
      }
    }
    else if (mode == 3) // if hatching
    {
      asyncServo.goTo(middleServoPosition);
      EEPROM.write(ServoPositionAddr, middleServoPosition);
    }
  }
}

void lcdBacklight()
{
  if (lcdBacklightTask.isFirstIteration())
  {
    backlightCounter = 0;
    lcd.backlight();
  }

  if (lcdBacklightTimer != 0)
  {
    if (backlightCounter >= lcdBacklightTimer)
    {
      lcd.noBacklight();
      lcdBacklightTask.disable();
    }
    backlightCounter++;
  }
}

void initializeEEPROM()
{
  //  EEPROM.write(secondElapsedAddr,00); // for set Values in EEPROM
  //  EEPROM.write(minuteElapsedAddr,52);
  //  EEPROM.write(hourElapsedAddr,21);
  //  EEPROM.write(dayElapsedAddr,6);
  //  EEPROM.write(monthElapsedAddr,5);
  //  EEPROM.put(yearElapsedAddr,2024);

  //  EEPROM.write(ServoPositionAddr,45);
  //  EEPROM.write(modeAddr,3);
  //  EEPROM.write(H_thresholdAddr,60);
  //  EEPROM.put(T_thresholdAddr ,35.0);
  //  EEPROM.put(lcdBacklightTimerAddr,30);
  //  EEPROM.write(turnModeAddr,1);
  //  EEPROM.write(StartServoPositionAddr,45);
  //  EEPROM.write(EndServoPositionAddr,135);

  previousServoPosition = EEPROM.read(ServoPositionAddr);
  startServoPosition = EEPROM.read(StartServoPositionAddr);
  endServoPosition = EEPROM.read(EndServoPositionAddr);
  turnMode = EEPROM.read(turnModeAddr);
  mode = EEPROM.read(modeAddr);

  H_threshold = EEPROM.read(H_thresholdAddr);
  T_threshold = EEPROM.get(T_thresholdAddr, T_threshold);

  lcdBacklightTimer = EEPROM.get(lcdBacklightTimerAddr, lcdBacklightTimer);

  T_MaximumDanger = T_threshold + 2;
  H_MaximumDanger = H_threshold + 10;
  T_MinimumDanger = T_threshold - 2;
  H_MinimumDanger = H_threshold - 20;
}

void serialPrintln(String string)
{
  #if __DEBUG__
    Serial.println(string);
  #endif
}

void serialPrint(String string)
{
  #if __DEBUG__
    Serial.print(string);
  #endif
}

void serialPrintReport(String string)
{
  #if __REPORT__
    Serial.print(string);
  #endif
}