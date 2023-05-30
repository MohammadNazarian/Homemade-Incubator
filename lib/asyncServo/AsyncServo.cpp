/* 
AsyncServo
by Chris Fraser <http://blog.chrosfraser.co.za>

https://github.com/chrisfraser/asyncServo
*/

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#include "asyncServo.h"

void AsyncServoClass::begin(uint8_t pin, short maxIncrementDegrees, short incrementDelay , int defaultPosition = 90)
{
	_maxIncrementDegrees = maxIncrementDegrees;
	_incrementDelay = incrementDelay;
	_writeIndex = -1;
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
#if (ASYNCSERVO_DEBUG == 1)
		Serial.println("-----------------------");
		Serial.println("Un-paused");
		Serial.println("-----------------------");
#endif
		_unpauseMillis = 0;
		play();
	}

	if (currentMillis >= _nextUpdateMillis)
	{
		if (_running)
		{
			// Start
			if (_readIndex == -1)
			{
#if (ASYNCSERVO_DEBUG == 1)
				Serial.println("-----------------------");
				Serial.println("Play start");
				Serial.println("-----------------------");
#endif
				// Get the current position of the servo and set that to the previous step
				_previousStep = getPosition();

				// Set read index tot he begining of the steps array
				_readIndex = 0;

				// Get the current step at readIndex
				_currentStep = _steps[_readIndex];
			}


			// Calculate increments to complete step if not already calculated
			if (_totalIncrements == -1)
			{
				_totalIncrements = getIncrements(_previousStep, _currentStep);
				_currentIncrement = 1;
			}

			move(_previousStep,_currentStep);

#if (ASYNCSERVO_DEBUG == 1)
			Serial.print("Moved: ");
			Serial.print(_previousStep);
			Serial.print("-->");
			Serial.print(_currentStep);
			Serial.print(" inc ");
			Serial.print(_currentIncrement);
			Serial.print("/");
			Serial.println(_totalIncrements);
#endif

			// Check if the increments and hence step are complete
			if (_currentIncrement == _totalIncrements)
			{
				// Reset the increment counters
				_currentIncrement = -1;
				_totalIncrements = -1;

				// Proceed to next step
				_readIndex++;

				// Check if we have finished the steps
				if (_readIndex > _writeIndex)
				{
					// If looping then reset and pause for loop delay
					if (_loop == true)
					{
						reset();
						pause(_loopDelay);
					}
					else
					{
						stop();
#if (ASYNCSERVO_DEBUG == 1)
						Serial.println("-----------------------");
						Serial.println("Done!");
						Serial.println("-----------------------");
#endif
					}
				}
				else
				{
					_previousStep = _currentStep;
					_currentStep = _steps[_readIndex];

#if (ASYNCSERVO_DEBUG == 1)
					Serial.println("-----------------------");
					Serial.print("Next Step: ");
					Serial.println(_readIndex);
					Serial.println("-----------------------");
#endif
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

uint8_t AsyncServoClass::goTo(uint8_t position, short maxIncrementDegrees, short incrementDelay)
{
	if(!_running)
	{
		_previousStep = getPosition();
		_currentStep = position;
		_readIndex = _writeIndex;
		_totalIncrements = getIncrements(_previousStep, _currentStep);
		_currentIncrement = 1;
		_loop = false;
		play();
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

// Add a step to the array of steps
void AsyncServoClass::add(uint8_t step)
{
	_steps[_writeIndex++] = step;
}

// Clear steps
void AsyncServoClass::clear()
{
	_writeIndex = 0;
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
	_readIndex = -1;
	_totalIncrements = -1;
	_nextUpdateMillis = 0;
	_unpauseMillis = 0;
}

AsyncServoClass AsyncServo;
