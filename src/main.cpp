#include <Arduino.h>
#include <dht.h>
#include <LiquidCrystal.h>
#include <Servo.h>

// TODO: tanzim daghigh zaman ,, yek meghdaresh baraye charkheshe motore.

dht DHTSensor;
#define DHT11_PIN 4

// Servo //
Servo motor;
//  Motor Position //
int motorPosition = 45;

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
int Sec = 0;
int Min = 59;
int Hrs = 3;
int Day = 0;

// Initial Value for Tempurature and Humidity //
int T_threshold = 38;
int H_threshold = 45;

// for Status //
int SET = 0;
int AutoSet = 0;

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
    for (; motorPosition >= 0; motorPosition -= 1)
    {
        motor.write(motorPosition);
        delay(20);
    }

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
            delay(200);

            if (digitalRead(UP) == LOW) // Pressed
            {
                T_threshold = T_threshold + 1;
                lcd.setCursor(0, 1);
                lcd.print(T_threshold);
                lcd.print(" *C");
                AutoSet = 0;
                // delay(300);
            }
            if (digitalRead(DOWN) == LOW) // Pressed
            {
                T_threshold = T_threshold - 1;
                lcd.setCursor(0, 1);
                lcd.print(T_threshold);
                lcd.print(" *C");
                AutoSet = 0;
                // delay(300);
            }
            if (digitalRead(ok) == LOW) // Pressed
            {
                AutoSet = 0;
                delay(300);
                T_condition = false;
            }

            if (digitalRead(UP) == HIGH && digitalRead(DOWN) == HIGH && digitalRead(ok) == HIGH) // Pressed
            {
                AutoSet = AutoSet + 1;
            }
            if (AutoSet == 50) // Auto Set in 15 Second ( 15000 ms)
            {
                T_condition = false;
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
                lcd.setCursor(0, 1);
                lcd.print(H_threshold);
                lcd.print("%");
                AutoSet = 0;
                // delay(300);
            }
            if (digitalRead(DOWN) == LOW)
            {
                H_threshold = H_threshold - 1;
                lcd.setCursor(0, 1);
                lcd.print(H_threshold);
                lcd.print("%");
                AutoSet = 0;
                // delay(300);
            }
            if (digitalRead(ok) == LOW)
            {
                AutoSet = 0;
                delay(300);
                H_condition = false;
            }

            if (digitalRead(UP) == HIGH && digitalRead(DOWN) == HIGH && digitalRead(ok) == HIGH) // Pressed
            {
                AutoSet = AutoSet + 1;
            }
            if (AutoSet == 75) // Auto Set in 15 Second ( 15000 ms)
            {
                H_condition = false;
            }
        }
        SET = 1;
    }

    if (digitalRead(ok) == LOW)
    {
        SET = 0;
        T_condition = true;
        H_condition = true;
        return;
    }

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
    Serial.println(DHTSensor.humidity, 1);

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
                if (digitalRead(bulb) == HIGH)
                {
                    delay(10000); // Actualy delay ~ 10 second
                    Sec = Sec + 10;
                    lcd.setCursor(14, 1);
                    lcd.print(Sec);
                    // baraye inke yek meghdar lamp ro hanooz roshan negah dare ta bishtar garm beshe mohit.
                }
                digitalWrite(bulb, LOW);
                delay(4000); // Actualy delay ~ 5 second
                Sec = Sec + 4;
            }
        }
        else
        {
            delay(500);
            lcd.setCursor(4, 0);
            lcd.print("<");
            if (DHTSensor.humidity < H_threshold)
            {
                lcd.setCursor(13, 0);
                lcd.print("<");
            }
            if (DHTSensor.temperature < T_threshold)
            {
                digitalWrite(bulb, HIGH);
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

        // CAUTION
        Serial.print(millis());
        Sec = Sec + 1;
        if (Sec >= 60)
        {
            if (Sec % 60 == 0) // if Sec == 60 or 120 or 180
            {
                Sec = 0;
                Min = Min + 1;
            }
            else
            {
                while (Sec > 60)
                {
                    Sec = Sec - 60;
                    Min = Min + 1;
                }
            }
        }
        if (Min == 60)
        {
            Min = 0;
            Hrs = Hrs + 1;
        }
        if (Hrs == 24)
        {
            Hrs = 0;
            Day = Day + 1;
        }

        if (Hrs % 4 == 0 && Min == 0 && Sec == 0) // not work!
        {
            if (motorPosition >= 90)
            {
                Serial.println("  ROTATING BACKWARD  ");
                for (motorPosition = 90; motorPosition >= 0; motorPosition -= 1)
                {
                    motor.write(motorPosition);
                    delay(22);
                }
                Sec = Sec + 2; // Compensation for unaccounted time
            }

            else if (motorPosition < 90) // not Work!!
            {
                Serial.println("  ROTATING FORWARD  ");
                for (motorPosition = 0; motorPosition <= 90; motorPosition += 1)
                {
                    motor.write(motorPosition);
                    delay(20);
                }
                // TODO : Caution
                Sec = Sec + 2; // Compensation for unaccounted time
            }
        }
    }
    else if (ack == 1)
    {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("No Sensor data.");
        lcd.setCursor(0, 1);
        lcd.print("System Halted.");
        digitalWrite(bulb, LOW);
        digitalWrite(fan, LOW);
    }
}
