/* 
AsyncServo
by Chris Fraser <http://blog.chrosfraser.co.za>

https://github.com/chrisfraser/asyncServo
*/

#include <Servo.h>
#include "AsyncServo.h"

AsyncServoClass asyncServo;

void setup()
{
	Serial.begin(9600);

	// Start servo on pin 9 with maxIncrementDegrees of 1 and incrementDelay of 15ms
	// maxIncrementDegrees determines the number of degrees the servo will move in an increment
	// incrementDelay is the delay between increments. Increasing this slows the servo
	// This smooths the travel of the servo
	asyncServo.begin(9, 1, 15);

	// Add some steps to the servo
	asyncServo.add(10);
	asyncServo.add(30);
	asyncServo.add(90);
	asyncServo.add(90);
	asyncServo.add(90);
	asyncServo.add(170);
	asyncServo.add(60);
	asyncServo.add(10);
}

void loop()
{
	// This updates the servo position 
	// Must be called more regularly than incrementDelay or issues may arise
	// Other blocking activities in the loop will cause issues.
	asyncServo.update();

	if (Serial.available())
	{
		char command = Serial.read();

		switch (command)
		{
		case 'p': asyncServo.play();
			Serial.println("Play");
			break;
		case '!': asyncServo.pause(5000);
			Serial.println("Pause");
			break;
		case 'l': asyncServo.loop(2000);
			Serial.println("Loop");
			break;
		case 'g': asyncServo.goTo(150);
			Serial.println("Goto 170");
			break;
		case 'z': asyncServo.goTo(10);
			Serial.println("Goto 10");
			break;
		case 'x': asyncServo.stop();
			Serial.println("Stop");
			break;
		}
	}
}
