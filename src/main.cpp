#include <Arduino.h>
#include <dht.h>
#include <LiquidCrystal.h>
#include <Servo.h>

dht DHTSensor;
#define DHT11_PIN 4
// #define DHTTYPE DHT11

// Servo //
Servo motor;
//  Motor Position //
int motorPosition = 0;

int Direction = 0;

// for LCD Display Pins //
const int rs = 8;
const int en = 9;
const int d4 = 10;
const int d5 = 11;
const int d6 = 12;
const int d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Lamp , fan //
const int bulb = 3;
const int fan = 6;

// Buttons //
const int ok = A1;
const int UP = A2;
const int DOWN = A3;

int ack = 0;

// Time //
int sec = 0;
int Min = 0;
int hrs = 0;

// Initial Value for Tempurature and Humidity //
int T_threshold = 30;
int H_threshold = 60;

// for Status //
int SET = 0;

boolean T_condition = true;
boolean H_condition = true;

void setup()
{
    // Lamp , Fan //
    pinMode(bulb, OUTPUT);
    pinMode(fan, OUTPUT);

    digitalWrite(bulb, LOW);
    digitalWrite(fan, LOW);

    // Buttons //
    pinMode(ok, INPUT);
    pinMode(UP, INPUT);
    pinMode(DOWN, INPUT);
    digitalWrite(ok, HIGH);
    digitalWrite(UP, HIGH);
    digitalWrite(DOWN, HIGH);

    // LCD , Servo //
    motor.attach(A4);
    motor.write(motorPosition);
    lcd.begin(16, 2);
    Serial.begin(9600);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temperature &");
    lcd.setCursor(0, 1);
    lcd.print("Humidity ");
    delay(1500);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Controller For");
    lcd.setCursor(0, 1);
    lcd.print("Incubator");
    delay(1500);
    lcd.clear();
    Serial.println("  Temperature and Humidity Controller For Incubator");
}
void loop()
{
    if (SET == 0)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Set Temperature:");
        lcd.setCursor(0, 1);
        lcd.print(T_threshold);
        lcd.print(" *C");

        while (T_condition)
        {
            if (digitalRead(UP) == LOW)
            {
                T_threshold = T_threshold + 1;
                lcd.setCursor(0, 1);
                lcd.print(T_threshold);
                lcd.print(" *C");
                delay(200);
            }
            if (digitalRead(DOWN) == LOW)
            {
                T_threshold = T_threshold - 1;
                lcd.setCursor(0, 1);
                lcd.print(T_threshold);
                lcd.print(" *C");
                delay(200);
            }
            if (digitalRead(ok) == LOW)
            {
                delay(200);
                T_condition = false;
            }
        }

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Set Humidity:");
        lcd.setCursor(0, 1);
        lcd.print(H_threshold);
        lcd.print("%");
        delay(100);

        while (H_condition)
        {
            if (digitalRead(UP) == LOW)
            {
                H_threshold = H_threshold + 1;
                lcd.setCursor(0, 1);
                lcd.print(H_threshold);
                lcd.print("%");
                delay(100);
            }
            if (digitalRead(DOWN) == LOW)
            {
                H_threshold = H_threshold - 1;
                lcd.setCursor(0, 1);
                lcd.print(H_threshold);
                lcd.print("%");
                delay(200);
            }
            if (digitalRead(ok) == LOW)
            {
                delay(100);
                H_condition = false;
            }
        }
        SET = 1;
    }
    ack = 0;
    int chk = DHTSensor.read11(DHT11_PIN); // READ DATA

    // Check DHTSensor //
    switch (chk)
    {
    case DHTLIB_OK:
        Serial.print("Sensor : OK,\t");
        break;
    case DHTLIB_ERROR_CHECKSUM:
        Serial.print("Sensor : Checksum error,\t");
        ack = 0;
        break;
    case DHTLIB_ERROR_TIMEOUT:
        Serial.print("Sensor : Time out error,\t");
        ack = 0;
        break;
    default:
        Serial.print("Sensor : Unknown error,\t");
        break;
    }
    // DISPLAT DATA
    Serial.print("DHT11, \t");
    Serial.print(DHTSensor.temperature, 1);
    Serial.print(",\t");
    Serial.println(DHTSensor.humidity, 1);
    delay(100);

    if (ack == 0)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Temp:");
        lcd.print(DHTSensor.temperature);
        lcd.setCursor(0, 1);
        lcd.print("Humidity:");
        lcd.print(DHTSensor.humidity);
        delay(500);

        // temperature and humidity is high //
        if (DHTSensor.temperature >= T_threshold)
        {
            delay(500);
            if (DHTSensor.temperature >= T_threshold)
            {
                digitalWrite(bulb, LOW);
            }
        }
        if (DHTSensor.humidity >= H_threshold)
        {
            delay(500);
            if (DHTSensor.humidity >= H_threshold)
            {
                digitalWrite(fan, HIGH);
            }
        }

        // temperature and humidity is low //

        if (DHTSensor.temperature < T_threshold)
        {
            delay(500);
            if (DHTSensor.temperature < T_threshold)
            {
                digitalWrite(bulb, HIGH);
            }
        }
        if (DHTSensor.humidity < H_threshold)
        {
            delay(500);
            if (DHTSensor.humidity < H_threshold)
            {
                digitalWrite(fan, LOW);
            }
        }
        sec = sec + 1;
        if (sec == 3)
        {
            sec = 0;
            Min = Min + 1;
        }
        if (Min == 3)
        {
            Min = 0;
            hrs = hrs + 1;
        }
        if (hrs == 3 && Min == 0 && sec == 0)
        {
            Serial.println("  ROTATING FORWARD  ");
            for (motorPosition = 0; motorPosition <= 180; motorPosition += 1)
            {
                motor.write(motorPosition);
                delay(25);
            }
        }
        if (hrs == 6 && Min == 0 && sec == 0)
        {
            Serial.println("  ROTATING BACKWARD  ");
            hrs = 0;
            for (motorPosition = 180; motorPosition >= 0; motorPosition -= 1)
            {
                motor.write(motorPosition);
                delay(25);
            }
        }
    }
    if (ack == 1)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("No Sensor data.");
        lcd.setCursor(0, 1);
        lcd.print("System Halted.");
        digitalWrite(bulb, LOW);
        digitalWrite(fan, LOW);
    }
    delay(500);
}
