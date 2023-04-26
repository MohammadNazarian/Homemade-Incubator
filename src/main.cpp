#include <Arduino.h>
#include <EEPROM.h>
#include <dht.h>
#include <LiquidCrystal.h>
#include <Servo.h>

// for EEPROM
// Addreses for EEPROM
int secondAddr = 0, minuteAddr = 1, hourAddr = 2, dayAddr = 3;
int ServoPositionAddr = 4, T_thresholdAddr = 5, H_thresholdAddr = 6;

dht DHTSensor;
#define DHT11_PIN 4

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
int T_threshold = 38;
int H_threshold = 45;

// for Status //
byte SET = 1;
byte timeStatus = 0;
byte ack = 0;
byte AutoSet = 0;
boolean Time_condition = false;
boolean T_condition = true;
boolean H_condition = true;

boolean onlyOnceRotate = true;

void setTime();
void setTempratureAndHumidity();
void goToSetStatus();

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
    lcd.print("Temperature &");
    lcd.setCursor(0, 1);
    lcd.print("Humidity ");
    delay(1000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Controller For");
    lcd.setCursor(0, 1);
    lcd.print("Incubator");
    delay(1000);
    lcd.clear();
    Serial.println("> Temperature and Humidity Controller For Incubator <");
}

void loop()
{
    setTime();
    setTempratureAndHumidity();
    goToSetStatus();

    timeRunning = millis();

    int chk = DHTSensor.read11(DHT11_PIN); // READ DATA
                                           // Check DHTSensor //
    switch (chk)
    {
    case DHTLIB_OK:
        Serial.print("Sensor : OK,\t");
        ack = 0;
        break;
    case DHTLIB_ERROR_CHECKSUM:
        Serial.print("Sensor : Checksum error,\t");
        ack = 1;
        delay(200);
        break;
    case DHTLIB_ERROR_TIMEOUT:
        Serial.print("Sensor : Time out error,\t");
        ack = 1;
        delay(200);
        break;
    default:
        Serial.print("Sensor : Unknown error,\t");
        ack = 1;
        delay(200);
        break;
    }

    // DISPLAY DATA in Serial Monitor
    Serial.print("DHT11, \t");
    Serial.print(DHTSensor.temperature, 1);
    Serial.print(",\t");
    Serial.print(DHTSensor.humidity, 1);
    Serial.print(",\t");
    Serial.println(millis());

    if (ack == 0)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("T:");
        lcd.print(DHTSensor.temperature);
        lcd.setCursor(4, 0);
        lcd.print(" ");
        lcd.print(T_threshold);

        lcd.setCursor(7, 0);
        lcd.print("  H:");
        lcd.print(DHTSensor.humidity);
        lcd.setCursor(13, 0);
        lcd.print(" ");
        lcd.print(H_threshold);

        lcd.setCursor(0, 1);
        lcd.print("d:");
        lcd.print(Day);
        lcd.setCursor(4, 1);
        lcd.print("h:");
        lcd.print(Hrs);
        lcd.setCursor(8, 1);
        lcd.print("m:");
        lcd.print(Min);
        lcd.setCursor(12, 1);
        lcd.print("s:");
        lcd.print(Sec);

        if (DHTSensor.temperature >= T_threshold)
        {
            delay(500);
            if (DHTSensor.temperature >= T_threshold)
            {
                if ((DHTSensor.temperature == T_threshold))
                {
                    lcd.setCursor(4, 0);
                    lcd.print("=");
                }
                else if ((DHTSensor.temperature > T_threshold))
                {
                    lcd.setCursor(4, 0);
                    lcd.print(">");
                }
                if (digitalRead(lamp) == HIGH)
                {
                    delay(5000); // Actualy delay ~ 5 second
                    // Sec= Sec+10;
                    lcd.setCursor(14, 1);
                    lcd.print(Sec);
                    // baraye inke yek meghdar lamp ro hanooz roshan negah dare ta bishtar garm beshe mohit.
                }
                digitalWrite(lamp, LOW);
                delay(9999); // Actualy delay ~ 10 second
            }
        }
        else
        {
            delay(500);
            lcd.setCursor(4, 0);
            lcd.print("<");
            if (DHTSensor.temperature < T_threshold)
            {
                digitalWrite(lamp, HIGH);
            }
        }

        if (DHTSensor.humidity >= H_threshold)
        {
            delay(460);
            if (DHTSensor.humidity >= H_threshold)
            {
                if ((DHTSensor.humidity == H_threshold))
                {
                    lcd.setCursor(13, 0);
                    lcd.print("=");
                }
                else if ((DHTSensor.humidity > H_threshold))
                {
                    lcd.setCursor(13, 0);
                    lcd.print(">");
                }
                digitalWrite(fan, HIGH);
            }
        }
        else
        {
            lcd.setCursor(13, 0);
            lcd.print("<");
            delay(460);
            if (DHTSensor.humidity < H_threshold)
            {
                digitalWrite(fan, LOW);
            }
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
                    delay(83);
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
                    delay(83);
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
    }
    else if (ack == 1)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("No Sensor data.");
        lcd.setCursor(0, 1);
        lcd.print("System Halted.");
        digitalWrite(lamp, LOW);
        digitalWrite(fan, LOW);
    }
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
            }
        }
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Set Humidity:");
        lcd.setCursor(0, 1);
        lcd.print(H_threshold);
        lcd.print("%");
        delay(200);

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
            }

            if (digitalRead(UP) == HIGH && digitalRead(DOWN) == HIGH && digitalRead(ok) == HIGH)
            {
                AutoSet = AutoSet + 1;
            }
            if (AutoSet == 50) // Auto Set in 10 Second ( 10000 ms)
            {
                EEPROM.write(H_thresholdAddr, H_threshold);
                AutoSet = 0;
                H_condition = false;
            }
        }
        SET = 0;
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
