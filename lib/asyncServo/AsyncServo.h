/* 
AsyncServo
by Chris Fraser <http://blog.chrosfraser.co.za>

https://github.com/chrisfraser/asyncServo
*/

#ifndef _ASYNCSERVO_h
#define _ASYNCSERVO_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include <Servo.h>

#define ASYNCSERVO_DEBUG 0

class AsyncServoClass
{
public:
	void begin(uint8_t pin, short maxIncrementDegrees = 2, short incrementDelay = 20, int defaultPosition = 90);
	uint8_t getPosition();
	
	bool isRunning();

	// To be called in loop function
	void update();

	uint8_t goTo(uint8_t position, short maxIncrementDegrees = 2, short incrementDelay = 20);

	void play();
	void loop(int delayMillis = 100);
	void pause(int delayMillis = -1);
	void stop();

	void add(uint8_t step);
	void clear();

private:
	Servo _base;
	bool _running;
	bool _loop;
	int _loopDelay;

	short _steps[25];

	// Variables used to determine increments between steps and speed of movement
	// Decreasing _incrementDelay increases servo speed
	short _maxIncrementDegrees, _incrementDelay;

	unsigned long _nextUpdateMillis = 0;
	unsigned long _unpauseMillis = 0;

	short _writeIndex;
	short _readIndex = -1;

	short _currentIncrement;
	short _totalIncrements = -1;

	short _previousStep;
	short _currentStep;

	uint8_t getIncrements(uint8_t previous, uint8_t current);
	uint8_t interpolate(uint8_t previous, uint8_t current);
	void move(uint8_t previous, uint8_t current);
	void reset();
};

extern AsyncServoClass AsyncServo;

#endif
