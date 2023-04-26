#include <Arduino.h>
#include <EEPROM.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include <AHTxx.h>
#include <Wire.h>
#include "EEPROMAnything.h"

void setTime();
void setTempratureAndHumidity();
void goToSetStatus();
void readSensors();
int average (int * array, int len);
void menu();
void Siren(bool Enabled);

#define lampPin 3
#define fanPin 6
#define speakerPin 7
#define servoPin A3
#define CLK 2 //for Rotary Encoder
#define DT 5 //for Rotary Encoder
#define SW 4 //for Rotary Encoder

  //for Rotary Encoder
int currentStateCLK;
int lastStateCLK;
String currentDir ="";
int btnState;
unsigned long lastButtonPress = 0;


  // for AHT10 Sensor
uint8_t readStatus = 0;
AHTxx AHT10Sensor(AHTXX_ADDRESS_X38,AHT1x_SENSOR);

  // for EEPROM
    //Addreses for EEPROM
const int secondAddr = 0 , minuteAddr = 1 , hourAddr = 2 , dayAddr = 3 ;
const int ServoPositionAddr = 4 , T_thresholdAddr = 5 , H_thresholdAddr = 6;

  //Servo //
Servo servo;
byte servoPosition = 0;
 
  // for LCD Display Pins //
const byte rs = 8 , en = 9 , d4 = 10 , d5 = 11 , d6 = 12 , d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);


byte TempIcon[8] = {B01110,B01010,B01010,B01110,B01110,B11111,B11111,B01110};
byte HumidityIcon[8] = {B00100,B00100,B01110,B01110,B11111,B11111,B11111,B01110};
byte degreIcon[8] = {B11000,B11000,B00000,B00000,B00000,B00000,B00000,B00000};

  // Speaker //
float sinVal;
int toneVal;

  // Time //
byte Sec = 0;
byte Min = 0;
byte Hrs = 0;
byte Day = 0;
unsigned long timeRunning = 0;
unsigned long timeTemp = 0;
unsigned long timeDetail = 0;

  // Initial Value for Tempurature and Humidity //
double T_threshold = 35.0;
byte H_threshold = 60;

  //Save Temp and Humidity
float baselineTemp;
byte baselineHum;

byte lcdShow=1;
boolean lcdShowTempAndHum;

  // for Status //
byte SET = 1;
byte timeStatus = 0;
int AutoSet = 0;
byte menuSelection = 0;

boolean Time_condition = false;
boolean T_condition = true;
boolean H_condition = true;
boolean Menu_condition = false;
boolean Mode_condition = false;
boolean onlyOnceRotate = true;

unsigned int delaySensorValue = 0;

void setup()
{
  
  pinMode(lampPin, OUTPUT);
  //pinMode(fanPin, OUTPUT); // required for digitalWrite
  pinMode(speakerPin, OUTPUT);
  
  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  pinMode(SW, INPUT_PULLUP);
  
  digitalWrite(lampPin, LOW);
  analogWrite(fanPin, 0);
  digitalWrite(speakerPin, LOW);
//  digitalWrite(ok, HIGH);// WHAT?
//  digitalWrite(UP, HIGH);// WHAT?
//  digitalWrite(DOWN, HIGH);// WHAT?

  Sec = EEPROM.read(secondAddr);
  Min = EEPROM.read(minuteAddr);
  Hrs = EEPROM.read(hourAddr);
  Day = EEPROM.read(dayAddr);
  
//  EEPROM.put(T_thresholdAddr ,T_threshold);
  T_threshold = EEPROM.get(T_thresholdAddr ,T_threshold);
  H_threshold = EEPROM.read(H_thresholdAddr);
  servoPosition= EEPROM.read(ServoPositionAddr);

  lcd.begin(16, 2);

  lcd.createChar(0, TempIcon);
  lcd.createChar(1, HumidityIcon);
  lcd.createChar(2, degreIcon);
  
  lastStateCLK = digitalRead(CLK);
  
  servo.attach(servoPin);
  Serial.begin(9600);
  
  while(servoPosition > 0)
  {
    servo.write(servoPosition); // until 1 degree
    delay(21);
    servoPosition -= 1;
  }
  servo.write(servoPosition);  // set servo to 0 degree
  EEPROM.write(ServoPositionAddr , servoPosition);

  lcd.clear();
  lcd.print("Incubator");
  Serial.println(F("> Temperature and Humidity Controller For Incubator <"));
  delay(1000);
  lcd.clear();
  lcd.print("Testing");
  lcd.setCursor(0,1);
  lcd.print("sensor...");
  Serial.println(F("Testing sensor..."));

  while (AHT10Sensor.begin() != true)
  {
    Serial.println(F("AHT10 not connected or fail to load calibration coefficient")); 
    //(F()) save string to flash & keeps dynamic memory free
    lcd.clear();
    lcd.print("AHT10 error");
    delay(5000);
  }
  lcd.clear();
  lcd.print("Ready");
  Serial.println(F("Ready"));
  delay(1000);
  lcd.clear();
}

void loop()
{
  Siren(false);
  
  setTempratureAndHumidity();
  setTime();
  menu();
  goToSetStatus();
  
  timeRunning = millis();

  
  baselineTemp = AHT10Sensor.readTemperature();
  baselineHum = AHT10Sensor.readHumidity();
  
    // DISPLAY DATA in Serial Monitor
//  Serial.print(F("AHT10, \t"));
//  Serial.print(baselineTemp,1);
//  Serial.print(F(",\t"));
//  Serial.print(baselineHum,1);
//  Serial.print(F(",\t"));

//  if (baselineTemp != AHT10_ERROR)
//  {

    if(lcdShow <=10)
    {
    lcdShowTempAndHum = true;
    lcd.clear();
    lcd.write((byte)0);
    lcd.print(" ");
    lcd.print(baselineTemp);
    lcd.print("  ");
    lcd.setCursor(8, 0);
    lcd.print("  ");
    lcd.print(T_threshold);
    lcd.setCursor(15, 0);
    lcd.write((byte)2);
    
    lcd.setCursor(0, 1);
    lcd.write((byte)1);
    lcd.print("    ");
    lcd.print(baselineHum);
    lcd.setCursor(8, 1);
    lcd.print("  ");
    lcd.print(H_threshold);
    lcd.setCursor(15, 1);
    lcd.print("%");
    }
    else
    {
    lcdShowTempAndHum = false;
    lcd.clear();
    lcd.print(" Day:");
    lcd.print(Day);
    lcd.setCursor(8, 0);
    lcd.print("Hour:");
    lcd.print(Hrs);
    lcd.setCursor(1, 1);
    lcd.print("Min:");
    lcd.print(Min);
    lcd.print("  Sec:");
    lcd.print(Sec);
    if(lcdShow ==20)
    lcdShow=1;
    }
    lcdShow++;

    delaySensorValue = millis() - timeRunning;
    delaySensorValue = 500 - delaySensorValue;
    delay(delaySensorValue);

  // for Humidity
    if (baselineHum < H_threshold)
    { 
      if(lcdShowTempAndHum)
      {
      lcd.setCursor(8, 1); 
      lcd.print("<");        
      }

      analogWrite(fanPin,0);
      //digitalWrite(fanPin, LOW);
    }
    else if(baselineHum == H_threshold)
    {
      if(lcdShowTempAndHum)
      {
      lcd.setCursor(8, 1);
      lcd.print("=");        
      }
      analogWrite(fanPin,255);
    }
    else  
    {
      if(lcdShowTempAndHum)
      {
      lcd.setCursor(8, 1);
      lcd.print(">");        
      }
      analogWrite(fanPin,255);
      //digitalWrite(fanPin, HIGH);
    }

  // for Temperature
    if (baselineTemp < T_threshold)
    {
      if(lcdShowTempAndHum)
      {
      lcd.setCursor(8, 0);
      lcd.print("<");        
      }
      delay(500);
      digitalWrite(lampPin, HIGH);
    }  
    else if(baselineTemp == T_threshold)
    {
      if(lcdShowTempAndHum)
      {
      lcd.setCursor(8, 0); 
      lcd.print("=");        
      }
      delay(500);
      digitalWrite(lampPin, LOW);
    }  
    else
    {
      while (baselineTemp - T_threshold > 5.0 )// age 5 darage bishtar bood
      {
        lcd.clear();
        lcd.print("Temp is High!!!");
        digitalWrite(lampPin, LOW);
        analogWrite(fanPin,255);
        Siren(true);
        goToSetStatus();
        
        delay(500);
        lcd.setCursor(1,3);
        baselineTemp = AHT10Sensor.readTemperature();
        lcd.print(baselineTemp);
        lcd.print("C");
      }
      if(lcdShowTempAndHum)
      {
      lcd.setCursor(8, 0);
      lcd.print(">");        
      }
      delay(500);
      digitalWrite(lampPin, LOW);
    }

    
              // Servo //
    if( Hrs == 0 || Hrs == 1 || Hrs == 2 || Hrs == 3 ||
        Hrs == 8 || Hrs == 9 || Hrs == 10|| Hrs == 11||
        Hrs == 16|| Hrs ==17 || Hrs == 18|| Hrs == 19|| Hrs == 24)
    {
      if (onlyOnceRotate)
      {
        if(EEPROM.read(ServoPositionAddr) >= 90)
        {
          servoPosition = EEPROM.read(ServoPositionAddr);
          Serial.println(F("  ROTATING BACKWARD  "));
          while(servoPosition > 0)
          {
            servo.write(servoPosition);
            delay(21);
            servoPosition -= 1;
          }
          servo.write(servoPosition);
          
          EEPROM.write(ServoPositionAddr , servoPosition);
          Serial.println(EEPROM.read(ServoPositionAddr));
          delay(110);
        }
        onlyOnceRotate = false; 
       }
     }
    else if( Hrs == 4 || Hrs == 5 || Hrs == 6 || Hrs == 7 ||
             Hrs == 12|| Hrs == 13|| Hrs == 14|| Hrs == 15||
             Hrs == 20|| Hrs == 21|| Hrs == 22|| Hrs == 23)
    {
      if (onlyOnceRotate)
      {
        if(EEPROM.read(ServoPositionAddr) < 90)
        {
          servoPosition = EEPROM.read(ServoPositionAddr);
          Serial.println(F("  ROTATING FORWARD  "));
          while(servoPosition < 90)
          {
            servo.write(servoPosition);
            delay(21);
            servoPosition += 1;
          }
          servo.write(servoPosition);
          
          EEPROM.write(ServoPositionAddr , servoPosition);
          Serial.println(EEPROM.read(ServoPositionAddr));
          delay(110);
        }
        onlyOnceRotate = false;
       }    
     }



    timeTemp = timeTemp + (millis() - timeRunning) ;
    while(timeTemp >= 1000)
    {
      timeTemp = timeTemp - 1000;
      Sec= Sec +1;
      EEPROM.write(secondAddr, Sec);
    }
    while(Sec >= 60)
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

      if( Hrs % 4 == 0 )
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
//
//  }
//  else
//  {
//    Serial.print(F("Failed to read _ reset: ")); 
//    Serial.println(AHT10Sensor.softReset()); 
//    if(AHT10Sensor.softReset() == 0)
//    {
//      Serial.println(F("softReset is failed")); 
//    }
//    else if (AHT10Sensor.softReset() == 1)
//    {
//      Serial.println(F("softReset is successful")); 
//    }
//    Siren(true);
//    lcd.print("No Sensor data.");
//    lcd.setCursor(0, 1);
//    lcd.print("System Halted.");
//    digitalWrite(lampPin, LOW);
//    //digitalWrite(fanPin, LOW);
//    analogWrite(fanPin,0);
//  }
}

void setTime()
{
  if(timeStatus == 1)
  {
    lcd.clear();
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

    Serial.print(F("Set Time "));

    while (Time_condition)
    {
      delay(10);

      AutoSet++;

      currentStateCLK = digitalRead(CLK);
            
      if (currentStateCLK != lastStateCLK)// for change status
      {
        if (digitalRead(DT) == currentStateCLK) // Clockwise rotate
        {
          AutoSet = 0;
          if(timeStatus == 1)
          {
            if(Day < 50)
            {
              Day = Day +1;
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
          else if(timeStatus == 2)
          {
            if(Hrs < 23)
            {
              Hrs = Hrs +1;
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
          else if(timeStatus == 3)
          {
            if(Min < 59)
            {
              Min = Min +1;
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
          else if(timeStatus == 4)
          {
            if(Sec < 59)
            {
              Sec = Sec +1;
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
        else  // Counter Clockwise rotate   -- minus value
        {
          AutoSet = 0;
          if(timeStatus == 1)
          {
            if(Day != 0)
            {
              Day = Day -1;
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
          else if(timeStatus == 2)
          {
            
            if(Hrs != 0)
            {
              Hrs = Hrs -1;
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
          else if(timeStatus == 3)
          {
            if(Min != 0)
            {
              Min = Min -1;
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
          else if(timeStatus == 4)
          {
            if(Sec != 0)
            {
              Sec = Sec -1;
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
      }
      lastStateCLK = currentStateCLK;

      if( AutoSet == 1000 ) // Auto Set in 10 Second ( 10000 ms)
      {
        AutoSet = 0;
        if(timeStatus == 1)
        {
          EEPROM.write(dayAddr, Day);
          lcd.setCursor(0, 1);
          lcd.print(">");
          lcd.setCursor(8, 0);
          lcd.print(" ");
        }
        else if(timeStatus == 2)
        {
          EEPROM.write(hourAddr, Hrs);
          lcd.setCursor(5, 1);
          lcd.print(">");
          lcd.setCursor(0, 1);
          lcd.print(" ");
        }
        else if(timeStatus == 3)
        {
          EEPROM.write(minuteAddr, Min);
          lcd.setCursor(10, 1);
          lcd.print(">");
          lcd.setCursor(5, 1);
          lcd.print(" ");
        }
        else if(timeStatus == 4)
        {
          lcd.setCursor(10, 1);
          lcd.print(" ");
          EEPROM.write(secondAddr, Sec);
          Time_condition = false;
          onlyOnceRotate = true;
          timeStatus = 0;
          Serial.println(F("Start"));
          lcd.clear();
          delay(300);
          return;
        }
        timeStatus++;
      }

      if (digitalRead(SW) == LOW) 
      {
        if (millis() - lastButtonPress > 50) 
        {
          AutoSet=0;
          if(timeStatus == 1)
          {
            EEPROM.write(dayAddr, Day);
            lcd.setCursor(0, 1);
            lcd.print(">");
            lcd.setCursor(8, 0);
            lcd.print(" ");
          }
          else if(timeStatus == 2)
          {
            EEPROM.write(hourAddr, Hrs);
            lcd.setCursor(5, 1);
            lcd.print(">");
            lcd.setCursor(0, 1);
            lcd.print(" ");
          }
          else if(timeStatus == 3)
          {
            EEPROM.write(minuteAddr, Min);
            lcd.setCursor(10, 1);
            lcd.print(">");
            lcd.setCursor(5, 1);
            lcd.print(" ");
          }
          else if(timeStatus == 4)
          {
            lcd.setCursor(10, 1);
            lcd.print(" ");
            EEPROM.write(secondAddr, Sec);
            Time_condition = false;
            onlyOnceRotate = true;
            timeStatus = 0;
            lcd.clear();
            delay(300);
            return;
          }
          timeStatus++ ;
          Serial.print(F("Start"));
        }
        lastButtonPress = millis();
      }
    }
  }
}

void setTempratureAndHumidity()
{
  if (SET == 1)
  {
    lcd.clear();
    lcd.print("Set Temperature:");
    lcd.setCursor(0, 1);
    lcd.print((float) T_threshold);
    lcd.write((byte)2);
    lcd.print("C");
    Serial.print(F("Set Temperature "));
    
    while (T_condition)
    {
      delay(10);
      
      AutoSet++;

      currentStateCLK = digitalRead(CLK);
            
      if (currentStateCLK != lastStateCLK)// for change status
      {
        if (digitalRead(DT) == currentStateCLK) // Clockwise rotate
        {
          if(T_threshold < 90.0)
          {T_threshold =  T_threshold + 0.01;}
          else
          {T_threshold = 1.0;}
          
          EEPROM.put(T_thresholdAddr , T_threshold);            
          //EEPROM.write(T_thresholdAddr , (float) T_threshold);
          lcd.setCursor(0, 1);
          lcd.print(T_threshold);
          lcd.write((byte)2);
          lcd.print("C");
          AutoSet=0;
        }
        else  // Counter Clockwise rotate
        {
          
          if(T_threshold > 1.0)
          {T_threshold =  T_threshold - 0.01;}
          else
          {T_threshold = 90.0;}
          
          EEPROM.put(T_thresholdAddr , T_threshold);
//            EEPROM.write(T_thresholdAddr , (float) T_threshold);
          lcd.setCursor(0, 1);
          lcd.print(T_threshold);
          lcd.write((byte)2);
          lcd.print("C");
          AutoSet=0;
        }
      }
      lastStateCLK = currentStateCLK;

      if (digitalRead(SW) == LOW) 
      {
        if (millis() - lastButtonPress > 50) 
        {
          AutoSet=0;
          delay(300);
          T_condition = false;
          EEPROM.put(T_thresholdAddr ,T_threshold);
//          EEPROM.write(T_thresholdAddr , (float) T_threshold);
          Serial.print(T_threshold);
          Serial.print(F("C"));
        }
        lastButtonPress = millis();
      }
      
      if ( AutoSet == 1000 ) // Auto Set in 10 Second ( 10000 ms)
      {
        AutoSet = 0;
        T_condition = false;
        EEPROM.put(T_thresholdAddr , T_threshold);
//        EEPROM.write(T_thresholdAddr , T_threshold);
        Serial.print(T_threshold);
        Serial.print(F("C"));
      }
    }
    
    lcd.clear();
    lcd.print("Set Humidity:");
    lcd.setCursor(0, 1);
    lcd.print(H_threshold);
    lcd.print("%");
    Serial.print(F("\nSet Humidity "));
  
    while (H_condition)
    {
      delay(10);
      AutoSet = AutoSet + 1;
      
      currentStateCLK = digitalRead(CLK);

      if (currentStateCLK != lastStateCLK)// for change status
      {
        if (digitalRead(DT) == currentStateCLK) // Clockwise rotate
        {
          if(H_threshold < 100)
          {
            H_threshold++;
          }
          else
          {
            H_threshold = 1;
          }
          EEPROM.write(H_thresholdAddr , H_threshold);
          lcd.setCursor(0, 1);
          lcd.print(H_threshold);
          lcd.print("%");
          AutoSet=0;
        }
        else  // Counter Clockwise rotate
        {
          if(H_threshold > 1)
          {H_threshold--;}
          else
          {H_threshold = 99;}
          
          EEPROM.write(H_thresholdAddr , H_threshold);
          lcd.setCursor(0, 1);
          lcd.print(H_threshold);
          lcd.print("%");
          AutoSet=0;
        }
      }
      lastStateCLK = currentStateCLK;

      if (digitalRead(SW) == LOW) 
      {
        if (millis() - lastButtonPress > 50) 
        {
          AutoSet=0;
          delay(300);
          H_condition = false;
          EEPROM.write(H_thresholdAddr , H_threshold);
          Serial.print(H_threshold);
          Serial.print(F("%"));
        }
        lastButtonPress = millis();
      }
      
      if ( AutoSet == 1000 ) // Auto Set in 10 Second ( 10000 ms)
      {
        AutoSet = 0;
        H_condition = false;
        EEPROM.write(H_thresholdAddr , H_threshold);
        Serial.print(H_threshold);
        Serial.print(F("%"));
      }
    }
    SET = 0;
    lcd.clear();
    Serial.println(F("Start"));
    return;
  }
}

void mode()
{
  if(Mode_condition)
  {
    
  }
}
void menu()
{
    //  Modes:
    //  lcd.print("set Mode");
    //  lcd.print("set Time");
    //  lcd.print("set Temp & humidity");
    //  lcd.print("Cancel");
  if(menuSelection >= 1)
  {
    lcd.clear();
    lcd.print(">set Mode");
    lcd.setCursor(0,1);
    lcd.print(" set Time");
    Serial.println(F("Set Mode"));
    
    while(Menu_condition)
    {
      currentStateCLK = digitalRead(CLK);
      if (currentStateCLK != lastStateCLK)// for change status
      {
        if (digitalRead(DT) == currentStateCLK) 
        {
          if(menuSelection >= 4)
          {
            menuSelection = 1;
            lcd.clear();
            lcd.print(">set Mode");
            lcd.setCursor(0,1);
            lcd.print(" set Time");
          }
          else
          {
            menuSelection ++;

            if(menuSelection == 2)
            {
              lcd.clear();
              lcd.print(" set Mode");
              lcd.setCursor(0,1);
              lcd.print(">set Time");
            }
            else if(menuSelection == 3)
            {
              lcd.clear();
              lcd.print(" set Time");
              lcd.setCursor(0,1);
              lcd.print(">set ");
              lcd.write((byte)0);
              lcd.print(" & ");
              lcd.write((byte)1);
            }
            else if(menuSelection == 4)
            {
              lcd.clear();
              lcd.print(" set ");
              lcd.write((byte)0);
              lcd.print(" & ");
              lcd.write((byte)1);
              lcd.setCursor(0,1);
              lcd.print(">Cancel");
            }
          }
        } 
        else 
        {
          if(menuSelection <= 1)
          {
            menuSelection = 4;
            lcd.clear();
            lcd.print(" set ");
            lcd.write((byte)0);
            lcd.print(" & ");
            lcd.write((byte)1);
            lcd.setCursor(0,1);
            lcd.print(">Cancel");
          }
          else
          {
            menuSelection --;
            
            if(menuSelection == 1)
            {
              lcd.clear();
              lcd.print(">set Mode");
              lcd.setCursor(0,1);
              lcd.print(" set Time");
            }
            else if(menuSelection == 2)
            {
              lcd.clear();
              lcd.print(">set Time");
              lcd.setCursor(0,1);
              lcd.print(" set ");
              lcd.write((byte)0);
              lcd.print(" & ");
              lcd.write((byte)1);
            }
            else if(menuSelection == 3)
            {
              lcd.clear();
              lcd.print(">set ");
              lcd.write((byte)0);
              lcd.print(" & ");
              lcd.write((byte)1);
              lcd.setCursor(0,1);
              lcd.print(" Cancel");
            }
          }
        }
      }
      lastStateCLK = currentStateCLK;


      if (digitalRead(SW) == LOW) 
      {
        if (millis() - lastButtonPress > 50) 
        { 
          delay(300);
          switch(menuSelection)
          {
            case 1:
              Mode_condition = true;
              mode();
              Serial.println(F("Selected set Mode"));
              menuSelection = 0;
              Menu_condition = false;
              delay(300);
              break;
            case 2:
              timeStatus = 1;
              Time_condition = true;
              setTime();
              Serial.println(F("Selected set Time"));
              menuSelection = 0;
              Menu_condition = false;
              delay(300);
              break;
            case 3:
              SET = 1;
              T_condition = true;
              H_condition = true;
              setTempratureAndHumidity();
              Serial.println(F("Selected set Temperature and Humidity"));
              menuSelection = 0;
              Menu_condition = false;
              delay(300);
              break;
            case 4:
              lcd.clear();
              delay(200);
              Serial.println(F("Selected Cancel"));
              menuSelection = 0;
              Menu_condition=false;
              return;
          }
        }
        lastButtonPress = millis();
      }          
    }
  }
} 

void goToSetStatus()
{ 
  if (digitalRead(SW) == LOW) 
  {
    if (millis() - lastButtonPress > 50) 
    {
        Menu_condition= true;
        menuSelection = 1;
        delay(500);
        return;
    }
    lastButtonPress = millis();
  }    
}

void Siren(bool Enabled)
{
  if(Enabled)
  {
    for (int x=0; x<180; x++) 
    {
      // convert degrees to radians then obtain sin value
      sinVal = (sin(x*(3.1412/180)));
      // generate a frequency from the sin value
      toneVal = 2000+(int(sinVal*1000));
      tone(7, toneVal);
      delay(2);
    }
  }
  else
  {
    noTone(7);
  }
}
