/* 
AsyncServo
by Chris Fraser <http://blog.chrosfraser.co.za>

https://github.com/chrisfraser/asyncServo

changed by me ;)
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "asyncServo.h"

void AsyncServoClass::begin(uint8_t pin, short servoStep, short incrementDelay , int defaultPosition = 90)
{
	_maxIncrementDegrees = servoStep;
	_incrementDelay = incrementDelay;
	_base.attach(pin);
	_base.write(defaultPosition);
}

// Return current servo position
uint8_t AsyncServoClass::getPosition()
{
	return _base.read();
}

bool AsyncServoClass::isRunning()
{
	return _running;
}

// Update function to be called in loop.
void AsyncServoClass::update()
{
	unsigned long currentMillis = millis();

	// Check for unpause
	if (_unpauseMillis > 0 && currentMillis >= _unpauseMillis)
	{
		_unpauseMillis = 0;
		_previousStep = getPosition();
		play();
	}

	if (currentMillis >= _nextUpdateMillis)
	{
		if (_running)
		{
			//_previousStep = getPosition();
			//_currentStep = _steps[_readIndex];

			// Calculate increments to complete step if not already calculated
			if (_totalIncrements == -1)
			{
				_totalIncrements = getIncrements(_previousStep, _currentStep);
				_currentIncrement = 1;
			}

			move(_previousStep,_currentStep);

			// Check if the increments and hence step are complete
			if (_currentIncrement == _totalIncrements)
			{
				// Reset the increment counters
				_currentIncrement = -1;
				_totalIncrements = -1;

				// If looping then reset and pause for loop delay
				if (_loop == true)
				{
					reset();
					pause(_loopDelay);
				}
				//Done
				else
				{
					stop();
				}
			}
			else
			{
				// Increment _currentIncrement for the next update
				_currentIncrement++;
			}			
		}

		// Update the update Timer by _incrementDelay
		_nextUpdateMillis = currentMillis + _incrementDelay;
	}
}

uint8_t AsyncServoClass::goTo(uint8_t position)
{
	if(!isRunning())
	{
		_previousStep = getPosition();
		_currentStep = position;
		_totalIncrements = getIncrements(_previousStep, _currentStep);
		_currentIncrement = 1;
		_loop = false;
		play();
	}
	else
	{
		Serial.println("isRunning AsyncServo");
	}
}

void AsyncServoClass::play()
{
	_running = true;
}

// Loop steps with a delay between
void AsyncServoClass::loop(int delay)
{
	_running = true;
	_loop = true;
	_loopDelay = delay;
}

void AsyncServoClass::stop()
{
	_running = false;
	_loop = false;
	reset();
}

// Pause the servos
void AsyncServoClass::pause(int delayMillis)
{
	_running = false;
	unsigned long currentMillis = millis();

	if (delayMillis > 0)
	{
		_unpauseMillis = currentMillis + delayMillis;
	}
}


// ------------------------------------------------------------------------
// Private
// ------------------------------------------------------------------------

// Get the number of increments required between previous and current steps
// so as not to exceed _maxIncrementDegrees
uint8_t AsyncServoClass::getIncrements(uint8_t previous, uint8_t current)
{
	// Maximum absolute distance to travel
	float maxMovement = abs(current - previous);

	return fmax((uint8_t)(ceil(maxMovement) / _maxIncrementDegrees), 1);
}

// Linear interpolation between two points
uint8_t AsyncServoClass::interpolate(uint8_t previousStep, uint8_t currentStep)
{
	float delta = (currentStep - previousStep);
	float step = delta / _totalIncrements * _currentIncrement;
	uint8_t result = previousStep + (uint8_t)floor(step);
	
	return result;
}

// Move to the current increment
void AsyncServoClass::move(uint8_t previousStep, uint8_t currentStep)
{
	uint8_t pos = interpolate(previousStep, currentStep);

	_base.write(pos);
}

// Reset counters
void AsyncServoClass::reset()
{
	_totalIncrements = -1;
	_nextUpdateMillis = 0;
	_unpauseMillis = 0;
}

AsyncServoClass AsyncServo;
