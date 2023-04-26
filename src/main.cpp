#include <Arduino.h>
#include <LiquidCrystal.h>
#include <Servo.h>
#include <dht11.h>

dht11 DHT;
#define DHT11_PIN 4

const int ok = A1;
const int UP = A2;
const int DOWN = A3;

const int bulb = 3;
const int vap = 6;

const int rs = 8;
const int en = 9;
const int d4 = 10;
const int d5 = 11;
const int d6 = 12;
const int d7 = 13;

int ack = 0;
int pos = 0;
int sec = 0;
int Min = 0;
int hrs = 0;
int T_threshold = 30;
int H_threshold = 60;
int SET = 0;
int Direction = 0;
boolean T_condition = true;
boolean H_condition = true;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
Servo motor;
void setup()
{
    pinMode(ok, INPUT);
    pinMode(UP, INPUT);
    pinMode(DOWN, INPUT);
    pinMode(bulb, OUTPUT);
    pinMode(vap, OUTPUT);
    digitalWrite(bulb, LOW);
    digitalWrite(vap, LOW);
    digitalWrite(ok, HIGH);
    digitalWrite(UP, HIGH);
    digitalWrite(DOWN, HIGH);
    motor.attach(A4);
    motor.write(pos);
    lcd.begin(16, 2);
    Serial.begin(9600);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temperature &");
    lcd.setCursor(0, 1);
    lcd.print("Humidity ");
    delay(3000);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Controller For");
    lcd.setCursor(0, 1);
    lcd.print("Incubator");
    delay(3000);
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
    int chk;
    chk = DHT.read(DHT11_PIN); // READ DATA
    switch (chk)
    {
    case DHTLIB_OK:
        // Serial.print("OK,\t");
        break;
    case DHTLIB_ERROR_CHECKSUM:
        // Serial.print("Checksum error,\t");
        ack = 0;
        break;
    case DHTLIB_ERROR_TIMEOUT:
        // Serial.print("Time out error,\t");
        ack = 0;
        break;
    default:
        // Serial.print("Unknown error,\t");
        break;
    }
    // DISPLAT DATA
    Serial.print("DHT11, \t");
    Serial.print(DHT.temperature, 1);
    Serial.print(",\t");
    Serial.println(DHT.humidity, 1);
    delay(100);

    if (ack == 0)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Temp:");
        lcd.print(DHT.temperature);
        lcd.setCursor(0, 1);
        lcd.print("Humidity:");
        lcd.print(DHT.humidity);
        delay(500);
        if (DHT.temperature >= T_threshold)
        {
            delay(500);
            if (DHT.temperature >= T_threshold)
            {
                digitalWrite(bulb, LOW);
            }
        }
        if (DHT.humidity >= H_threshold)
        {
            delay(500);
            if (DHT.humidity >= H_threshold)
            {
                digitalWrite(vap, HIGH);
            }
        }
        if (DHT.temperature < T_threshold)
        {
            delay(500);
            if (DHT.temperature < T_threshold)
            {
                digitalWrite(bulb, HIGH);
            }
        }
        if (DHT.humidity < H_threshold)
        {
            delay(500);
            if (DHT.humidity < H_threshold)
            {
                digitalWrite(vap, LOW);
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
            for (pos = 0; pos <= 180; pos += 1)
            {
                motor.write(pos);
                delay(25);
            }
        }
        if (hrs == 6 && Min == 0 && sec == 0)
        {
            Serial.println("  ROTATING BACKWARD  ");
            hrs = 0;
            for (pos = 180; pos >= 0; pos -= 1)
            {
                motor.write(pos);
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
        digitalWrite(vap, LOW);
    }
    delay(500);
}