#include <Arduino.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <Servo.h>
// New
#include <DHT.h>

void setTime();
void setTempratureAndHumidity();
void goToSetStatus();
void readSensors();
int average(int *array, int len);

// New
#define DHT11_PIN 4
#define DHTTYPE DHT11
DHT dht(DHT11_PIN, DHTTYPE);
// New
const byte probe = 3; // reading probes
// globals
int index = 0;
int baselineTemp;
int baselineHum;
int tempArr[probe];
int humArr[probe];

// New
void readSensors()
{
  // DHT reading
  tempArr[index] = dht.readTemperature();
  humArr[index] = dht.readHumidity();

  // get average
  baselineTemp = average(tempArr, probe);
  baselineHum = average(humArr, probe);
}
// assuming array is int
int average(int *array, int len)
{
  long sum = 0L; // sum will be larger than an item, long for safety.
  for (int i = 0; i < len; i++)
    sum += array[i];
  return ((int)sum) / len; // average will be fractional, so float may be appropriate.
}

// for EEPROM
// Addreses for EEPROM
int secondAddr = 0, minuteAddr = 1, hourAddr = 2, dayAddr = 3;
int ServoPositionAddr = 4, T_thresholdAddr = 5, H_thresholdAddr = 6;

// Servo //
Servo servo;
int servoPosition = 0;

// for LCD Display Pins //
const int rs = 8;
const int en = 9;
const int d4 = 10;
const int d5 = 11;
const int d6 = 12;
const int d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Lamp , fan //
const int lamp = 3;
const int fan = 6;

// Buttons //
const int ok = A1;
const int UP = A2;
const int DOWN = A3;

// Time //
byte Sec = 0;
byte Min = 0;
byte Hrs = 0;
byte Day = 0;
unsigned long timeRunning = 0;
unsigned long timeTemp = 0;
unsigned long timeDetail = 0;

// Initial Value for Tempurature and Humidity //
int T_threshold = 0;
int H_threshold = 0;

// for Status //
byte SET = 1;
byte timeStatus = 0;
// byte ack = 0;
byte AutoSet = 0;
boolean Time_condition = false;
boolean T_condition = true;
boolean H_condition = true;

unsigned int delaySensorValue = 0;

boolean onlyOnceRotate = true;

void setup()
{
  // Lamp , Fan //
  pinMode(lamp, OUTPUT);
  pinMode(fan, OUTPUT);
  digitalWrite(lamp, LOW);
  digitalWrite(fan, LOW);

  // Buttons //
  pinMode(ok, INPUT);
  pinMode(UP, INPUT);
  pinMode(DOWN, INPUT);
  digitalWrite(ok, HIGH);
  digitalWrite(UP, HIGH);
  digitalWrite(DOWN, HIGH);

  Sec = EEPROM.read(secondAddr);
  Min = EEPROM.read(minuteAddr);
  Hrs = EEPROM.read(hourAddr);
  Day = EEPROM.read(dayAddr);

  // EEPROM.write(T_thresholdAddr , 0);
  // EEPROM.write(H_thresholdAddr , 0);
  T_threshold = EEPROM.read(T_thresholdAddr);
  H_threshold = EEPROM.read(H_thresholdAddr);

  // LCD , Servo //
  servo.attach(A4);
  servoPosition = EEPROM.read(ServoPositionAddr);
  while (servoPosition > 0)
  {
    servo.write(servoPosition); // until 1 degree
    delay(21);
    servoPosition -= 1;
  }
  servo.write(servoPosition); // set servo to 0 degree
  EEPROM.write(ServoPositionAddr, servoPosition);

  lcd.begin(16, 2);
  Serial.begin(9600);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Incubator");
  Serial.println("> Temperature and Humidity Controller For Incubator <");
  delay(1000);

  // New
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Calibrating");
  lcd.setCursor(0, 1);
  lcd.print("sensor...");
  Serial.println("Calibrating sensors...");

  dht.begin();
  // calibration
  for (index = 0; index < probe; index = index + 1)
  {
    readSensors();
    delay(700);
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Ready");
  Serial.println("Ready");
  index = 0;
  delay(1000);
  lcd.clear();
}

void loop()
{
  setTempratureAndHumidity();
  setTime();
  goToSetStatus();

  timeRunning = millis();
  // New
  readSensors();

  index = index + 1;
  if (index > probe - 1)
  {
    index = 0;
  }
  //  int chk = DHTSensor.read11(DHT11_PIN);    // READ DATA
  //    // Check DHTSensor //
  //  switch (chk)
  //  {
  //      case DHTLIB_OK:
  //      ack = 0;
  //      break;
  //      case DHTLIB_ERROR_CHECKSUM:
  //      Serial.print("Sensor : Checksum error,\t");
  //      ack = 1;
  //      delay(200);
  //      break;
  //      case DHTLIB_ERROR_TIMEOUT:
  //      Serial.print("Sensor : Time out error,\t");
  //      ack = 1;
  //      delay(200);
  //      break;
  //      default:
  //      Serial.print("Sensor : Unknown error,\t");
  //      ack = 1;
  //      delay(200);
  //      break;
  //  }

  // DISPLAY DATA in Serial Monitor
  Serial.print("DHT11, \t");
  // New
  Serial.print(baselineTemp, 1);
  Serial.print(",\t");
  // New
  Serial.print(baselineHum, 1);
  Serial.print(",\t");
  Serial.println(millis());

  // if (ack == 0)
  //{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  // New
  lcd.print(baselineTemp);
  lcd.setCursor(4, 0);
  lcd.print(" ");
  lcd.print(T_threshold);

  lcd.setCursor(7, 0);
  lcd.print("  H:");
  // New
  lcd.print(baselineHum);
  lcd.setCursor(13, 0);
  lcd.print(" ");
  lcd.print(H_threshold);

  lcd.setCursor(1, 1);
  lcd.print("d:");
  lcd.print(Day);
  lcd.setCursor(6, 1);
  lcd.print("h:");
  lcd.print(Hrs);
  lcd.setCursor(11, 1);
  lcd.print("m:");
  lcd.print(Min);

  // New
  if (baselineHum >= H_threshold)
  {
    delaySensorValue = millis() - timeRunning;
    delaySensorValue = 500 - delaySensorValue;
    delay(delaySensorValue);

    if ((baselineHum == H_threshold))
    {
      lcd.setCursor(13, 0);
      lcd.print("=");
    }
    else if ((baselineHum > H_threshold))
    {
      lcd.setCursor(13, 0);
      lcd.print(">");
    }

    digitalWrite(fan, HIGH);
  }
  else
  {
    delaySensorValue = millis() - timeRunning;
    delaySensorValue = 500 - delaySensorValue;
    delay(delaySensorValue);

    lcd.setCursor(13, 0);
    lcd.print("<");
    digitalWrite(fan, LOW);
  }
  // New
  if (baselineTemp >= T_threshold)
  {
    if ((baselineTemp == T_threshold))
    {
      lcd.setCursor(4, 0);
      lcd.print("=");
    }
    else if ((baselineTemp > T_threshold))
    {
      lcd.setCursor(4, 0);
      lcd.print(">");
    }
    delay(500);
    if (digitalRead(lamp) == HIGH)
    {
      // baraye inke yek meghdar lamp ro hanooz roshan negah dare ta bishtar garm beshe mohit.
      delay(5000);
    }
    digitalWrite(lamp, LOW);
    delay(2000);
  }
  else
  {
    lcd.setCursor(4, 0);
    lcd.print("<");
    delay(500);

    digitalWrite(lamp, HIGH);
  }
  // Servo //
  if (Hrs == 0 || Hrs == 1 || Hrs == 2 || Hrs == 3 ||
      Hrs == 8 || Hrs == 9 || Hrs == 10 || Hrs == 11 ||
      Hrs == 16 || Hrs == 17 || Hrs == 18 || Hrs == 19 || Hrs == 24)
  {
    if (onlyOnceRotate)
    {
      Serial.println(EEPROM.read(ServoPositionAddr));
      if (EEPROM.read(ServoPositionAddr) >= 90)
      {
        servoPosition = EEPROM.read(ServoPositionAddr);
        Serial.println("  ROTATING BACKWARD  ");
        while (servoPosition > 0)
        {
          servo.write(servoPosition);
          delay(21);
          servoPosition -= 1;
        }
        servo.write(servoPosition);

        EEPROM.write(ServoPositionAddr, servoPosition);
        Serial.println(EEPROM.read(ServoPositionAddr));
        delay(110);
      }
      onlyOnceRotate = false;
    }
  }
  else if (Hrs == 4 || Hrs == 5 || Hrs == 6 || Hrs == 7 ||
           Hrs == 12 || Hrs == 13 || Hrs == 14 || Hrs == 15 ||
           Hrs == 20 || Hrs == 21 || Hrs == 22 || Hrs == 23)
  {
    if (onlyOnceRotate)
    {
      Serial.println(EEPROM.read(ServoPositionAddr));
      if (EEPROM.read(ServoPositionAddr) < 90)
      {
        servoPosition = EEPROM.read(ServoPositionAddr);
        Serial.println("  ROTATING FORWARD  ");
        while (servoPosition < 90)
        {
          servo.write(servoPosition);
          delay(21);
          servoPosition += 1;
        }
        servo.write(servoPosition);

        EEPROM.write(ServoPositionAddr, servoPosition);
        Serial.println(EEPROM.read(ServoPositionAddr));
        delay(110);
      }
      onlyOnceRotate = false;
    }
  }
  timeTemp = timeTemp + (millis() - timeRunning);
  while (timeTemp >= 1000)
  {
    timeTemp = timeTemp - 1000;
    Sec = Sec + 1;
    EEPROM.write(secondAddr, Sec);
  }
  while (Sec >= 60)
  {
    Sec = Sec - 60;
    Min = Min + 1;
    EEPROM.write(secondAddr, Sec);
    EEPROM.write(minuteAddr, Min);
  }
  if (Min == 60)
  {
    Min = 0;
    Hrs = Hrs + 1;

    if (Hrs % 4 == 0)
    {
      onlyOnceRotate = true;
    }
    EEPROM.write(minuteAddr, Min);
    EEPROM.write(hourAddr, Hrs);
  }
  if (Hrs == 24)
  {
    Hrs = 0;
    Day = Day + 1;
    EEPROM.write(hourAddr, Hrs);
    EEPROM.write(dayAddr, Day);
  }
  //}
  //  else if (ack == 1)
  //  {
  //    lcd.clear();
  //    lcd.setCursor(0, 0);
  //    lcd.print("No Sensor data.");
  //    lcd.setCursor(0, 1);
  //    lcd.print("System Halted.");
  //    digitalWrite(lamp, LOW);
  //    digitalWrite(fan, LOW);
  //  }
  Serial.println(millis() - timeRunning);
}

void setTime()
{
  if (timeStatus == 1)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("setTime");
    lcd.setCursor(8, 0);
    lcd.print(">");
    lcd.setCursor(9, 0);
    lcd.print("Day:");
    lcd.setCursor(13, 0);
    lcd.print(Day);
    lcd.setCursor(1, 1);
    lcd.print("h:");
    lcd.print(Hrs);
    lcd.setCursor(6, 1);
    lcd.print("m:");
    lcd.print(Min);
    lcd.setCursor(11, 1);
    lcd.print("s:");
    lcd.print(Sec);

    Serial.print("Set Time ");

    while (Time_condition)
    {
      delay(200);

      if (digitalRead(UP) == LOW) // Pressed
      {
        if (timeStatus == 1)
        {
          if (Day < 50)
          {
            Day = Day + 1;
            lcd.setCursor(13, 0);
            lcd.print("   ");
            lcd.setCursor(13, 0);
            lcd.print(Day);
          }
          else
          {
            Day = 0;
            lcd.setCursor(13, 0);
            lcd.print("   ");
            lcd.setCursor(13, 0);
            lcd.print(Day);
          }
        }
        else if (timeStatus == 2)
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
        else if (timeStatus == 3)
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
        else if (timeStatus == 4)
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
      if (digitalRead(DOWN) == LOW) // Pressed
      {
        if (timeStatus == 1)
        {
          if (Day != 0)
          {
            Day = Day - 1;
            lcd.setCursor(13, 0);
            lcd.print("   ");
            lcd.setCursor(13, 0);
            lcd.print(Day);
          }
          else
          {
            Day = 50;
            lcd.setCursor(13, 0);
            lcd.print("   ");
            lcd.setCursor(13, 0);
            lcd.print(Day);
          }
        }
        else if (timeStatus == 2)
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
        else if (timeStatus == 3)
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
        else if (timeStatus == 4)
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
      if (digitalRead(ok) == LOW) // Pressed
      {
        if (timeStatus == 1)
        {
          EEPROM.write(dayAddr, Day);
          lcd.setCursor(0, 1);
          lcd.print(">");
          lcd.setCursor(8, 0);
          lcd.print(" ");
        }
        else if (timeStatus == 2)
        {
          EEPROM.write(hourAddr, Hrs);
          lcd.setCursor(5, 1);
          lcd.print(">");
          lcd.setCursor(0, 1);
          lcd.print(" ");
        }
        else if (timeStatus == 3)
        {
          EEPROM.write(minuteAddr, Min);
          lcd.setCursor(10, 1);
          lcd.print(">");
          lcd.setCursor(5, 1);
          lcd.print(" ");
        }
        else if (timeStatus == 4)
        {
          EEPROM.write(secondAddr, Sec);
          lcd.setCursor(10, 1);
          lcd.print(" ");

          Time_condition = false;
          onlyOnceRotate = true;
          timeStatus = 0;
          delay(300);
          return;
        }
        timeStatus = timeStatus + 1;
        Serial.print("\nStart ");
      }
    }
  }
}

void setTempratureAndHumidity()
{
  if (SET == 1)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Set Temperature:");
    lcd.setCursor(0, 1);
    lcd.print(T_threshold);
    lcd.print(" *C");
    Serial.print("Set Temperature ");

    while (T_condition)
    {
      delay(200);

      if (digitalRead(UP) == LOW) // Pressed
      {
        T_threshold = T_threshold + 1;
        EEPROM.write(T_thresholdAddr, T_threshold);
        lcd.setCursor(0, 1);
        lcd.print(T_threshold);
        lcd.print(" *C");
        AutoSet = 0;
      }
      if (digitalRead(DOWN) == LOW) // Pressed
      {
        T_threshold = T_threshold - 1;
        EEPROM.write(T_thresholdAddr, T_threshold);
        lcd.setCursor(0, 1);
        lcd.print(T_threshold);
        lcd.print(" *C");
        AutoSet = 0;
      }
      if (digitalRead(ok) == LOW) // Pressed
      {
        AutoSet = 0;
        delay(300);
        T_condition = false;
        EEPROM.write(T_thresholdAddr, T_threshold);
        Serial.print(T_threshold);
        Serial.print("C");
      }

      if (digitalRead(UP) == HIGH && digitalRead(DOWN) == HIGH && digitalRead(ok) == HIGH)
      {
        AutoSet = AutoSet + 1;
      }
      if (AutoSet == 50) // Auto Set in 10 Second ( 10000 ms)
      {
        AutoSet = 0;
        T_condition = false;
        EEPROM.write(T_thresholdAddr, T_threshold);
        Serial.print(T_threshold);
        Serial.print("C");
      }
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Set Humidity:");
    lcd.setCursor(0, 1);
    lcd.print(H_threshold);
    lcd.print("%");
    Serial.print("\nSet Humidity ");

    while (H_condition)
    {
      delay(200);
      if (digitalRead(UP) == LOW)
      {
        H_threshold = H_threshold + 1;
        EEPROM.write(H_thresholdAddr, H_threshold);
        lcd.setCursor(0, 1);
        lcd.print(H_threshold);
        lcd.print("%");
        AutoSet = 0;
      }
      if (digitalRead(DOWN) == LOW)
      {
        H_threshold = H_threshold - 1;
        EEPROM.write(H_thresholdAddr, H_threshold);
        lcd.setCursor(0, 1);
        lcd.print(H_threshold);
        lcd.print("%");
        AutoSet = 0;
      }
      if (digitalRead(ok) == LOW)
      {
        AutoSet = 0;
        delay(300);
        H_condition = false;
        EEPROM.write(H_thresholdAddr, H_threshold);
        Serial.print(H_threshold);
        Serial.print("%");
      }

      if (digitalRead(UP) == HIGH && digitalRead(DOWN) == HIGH && digitalRead(ok) == HIGH)
      {
        AutoSet = AutoSet + 1;
      }
      if (AutoSet == 50) // Auto Set in 10 Second ( 10000 ms)
      {
        EEPROM.write(H_thresholdAddr, H_threshold);
        Serial.print(H_threshold);
        Serial.print("%");
        AutoSet = 0;
        H_condition = false;
      }
    }
    SET = 0;
    Serial.println("\nStart");
  }
}
void goToSetStatus()
{
  if (digitalRead(DOWN) == LOW)
  {
    timeStatus = 1;
    Time_condition = true;
    return;
  }
  if (digitalRead(UP) == LOW)
  {
    SET = 1;
    T_condition = true;
    H_condition = true;
    return;
  }
}
