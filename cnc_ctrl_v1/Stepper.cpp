#include "Arduino.h"
#include "Stepper.h"
 
/*
 * two-wire constructor.
 * Sets which wires should control the motor.
 */
Stepper::Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2)
{
  this->step_number = 0;    // which step the motor is on
  this->direction = 0;      // motor direction
  this->last_step_time = 0; // time stamp in us of the last step taken
  this->number_of_steps = number_of_steps; // total number of steps for this motor

  // Arduino pins for the motor control connection:
  this->motor_pin_1 = motor_pin_1;
  this->motor_pin_2 = motor_pin_2;

  // setup the pins on the microcontroller:
  pinMode(this->motor_pin_1, OUTPUT);
  pinMode(this->motor_pin_2, OUTPUT);

  // When there are only 2 pins, set the others to 0:
  this->motor_pin_3 = 0;
  this->motor_pin_4 = 0;
  this->motor_pin_5 = 0;

  // pin_count is used by the stepMotor() method:
  this->pin_count = 2;
}
:
encoder(encoderPin1,encoderPin2)
{
    
    //initialize motor
    motor.setupMotor(pwmPin, directionPin1, directionPin2);
    motor.write(0);
    
    //initialize the PID
    _posPIDController.setup(&_currentSpeed, &_pidOutput, &_targetSpeed, _Kp, _Ki, _Kd, DIRECT);
    _negPIDController.setup(&_currentSpeed, &_pidOutput, &_targetSpeed, _Kp, _Ki, _Kd, DIRECT);
    initializePID();
    
    
}


Stepper::write(float speed){
    /*
    Command the motor to turn at the given speed. Should be RPM is PWM right now.
    */
    
    _targetSpeed = speed;
    
}


Stepper::initializePID(){ 
    //setup positive PID controller
    _posPIDController.SetMode(AUTOMATIC);
    _posPIDController.SetOutputLimits(-255, 255);
    _posPIDController.SetSampleTime(10);
    
    //setup negative PID controller
    _negPIDController.SetMode(AUTOMATIC);
    _negPIDController.SetOutputLimits(-255, 255);
    _negPIDController.SetSampleTime(10);
}




     /*
    Recompute the speed control PID loop and command the motor to move.
    */
	
	
	

 Stepper::computePID(){ 
   
    _currentSpeed = computeSpeed();
    
    
    
    if(_targetSpeed > 0){
        _posPIDController.Compute();
    }
    else{
        _negPIDController.Compute();
    }
    
    motor.write(_pidOutput);
}


    /*
    
    The setPIDAggressiveness() function sets the aggressiveness of the PID controller to
    compensate for a change in the load on the motor.
    
    */


   Stepper::setPIDAggressiveness(float aggressiveness){ 
   
    
    _posPIDController.SetTunings(aggressiveness*_Kp, _Ki, _Kd);
    _negPIDController.SetTunings(aggressiveness*_Kp, _Ki, _Kd);
    
    }
	
	
	
	/*
    
    Returns the motors speed in RPM since the last time this function was called
    
    */
	

	float Stepper::computeSpeed(){
    
    double timeElapsed =  micros() - _lastTimeStamp;
    
    float    distMoved   =  _runningAverage(encoder.read() - _lastPosition);     //because of quantization noise it helps to average these
    
    //Compute the speed in RPM
    float RPM = (7364.0*distMoved)/float(timeElapsed);  //6*10^7 us per minute, 8148 steps per revolution
    
    //Store values for next time
    _lastTimeStamp = micros();
    _lastPosition  = encoder.read();
    
    return -1.0*RPM;
}
	
	
	 /*
    
    Compute a running average from the number passed in.
    
    */
	
	
	float MotorGearboxEncoder::_runningAverage(int newValue){
   
    
    int sum = newValue + _oldValue1 + _oldValue2 + _oldValue3 + _oldValue4 + _oldValue5 + _oldValue6 + _oldValue7 + _oldValue8 + _oldValue9 + _oldValue10;
    float runningAverage = (float(sum))/11.0;
    
    _oldValue10 = _oldValue9;
    _oldValue9 = _oldValue8;
    _oldValue8 = _oldValue7;
    _oldValue7 = _oldValue6;
    _oldValue6 = _oldValue5;
    _oldValue5 = _oldValue4;
    _oldValue4 = _oldValue3;
    _oldValue3 = _oldValue2;
    _oldValue2 = _oldValue1;
    _oldValue1 = newValue;
    
    return runningAverage;
}


		 /*
			Set the name for the object
	     */


	void MotorGearboxEncoder::setName(String newName){
	   
		_motorName = newName;
	}
		
	
	
	
	
	
	/*
	 *   constructor for four-pin version
	 *   Sets which wires should control the motor.
	 */
	 
	 
	 
	 
	Stepper::Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2,
										  int motor_pin_3, int motor_pin_4)
	{
	  this->step_number = 0;    // which step the motor is on
	  this->direction = 0;      // motor direction
	  this->last_step_time = 0; // time stamp in us of the last step taken
	  this->number_of_steps = number_of_steps; // total number of steps for this motor

	  // Arduino pins for the motor control connection:
	  this->motor_pin_1 = motor_pin_1;
	  this->motor_pin_2 = motor_pin_2;
	  this->motor_pin_3 = motor_pin_3;
	  this->motor_pin_4 = motor_pin_4;

	  // setup the pins on the microcontroller:
	  pinMode(this->motor_pin_1, OUTPUT);
	  pinMode(this->motor_pin_2, OUTPUT);
	  pinMode(this->motor_pin_3, OUTPUT);
	  pinMode(this->motor_pin_4, OUTPUT);

	  // When there are 4 pins, set the others to 0:
	  this->motor_pin_5 = 0;

	  // pin_count is used by the stepMotor() method:
	  this->pin_count = 4;
	}

	
	
	
	/*
	 *   constructor for five phase motor with five wires
	 *   Sets which wires should control the motor.
	 */
		Stepper::Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2,
											  int motor_pin_3, int motor_pin_4,
											  int motor_pin_5)
		{
		  this->step_number = 0;    // which step the motor is on
		  this->direction = 0;      // motor direction
		  this->last_step_time = 0; // time stamp in us of the last step taken
		  this->number_of_steps = number_of_steps; // total number of steps for this motor

		  // Arduino pins for the motor control connection:
		  this->motor_pin_1 = motor_pin_1;
		  this->motor_pin_2 = motor_pin_2;
		  this->motor_pin_3 = motor_pin_3;
		  this->motor_pin_4 = motor_pin_4;
		  this->motor_pin_5 = motor_pin_5;

		  // setup the pins on the microcontroller:
		  pinMode(this->motor_pin_1, OUTPUT);
		  pinMode(this->motor_pin_2, OUTPUT);
		  pinMode(this->motor_pin_3, OUTPUT);
		  pinMode(this->motor_pin_4, OUTPUT);
		  pinMode(this->motor_pin_5, OUTPUT);

		  // pin_count is used by the stepMotor() method:
		  this->pin_count = 5;
		}

		/*
		 * Sets the speed in revs per minute
		 */
		void Stepper::setSpeed(long whatSpeed)
		{
		  this->step_delay = 60L * 1000L * 1000L / this->number_of_steps / whatSpeed;
		}

		
		
		
		
		/*
		 * Moves the motor steps_to_move steps.  If the number is negative,
		 * the motor moves in the reverse direction.
		 */
		 
		 
		void Stepper::step(int steps_to_move)
		{
		  int steps_left = abs(steps_to_move);  // how many steps to take

		  // determine direction based on whether steps_to_mode is + or -:
		  if (steps_to_move > 0) { this->direction = 1; }
		  if (steps_to_move < 0) { this->direction = 0; }


		  // decrement the number of steps, moving one step each time:
		  while (steps_left > 0)
		  {
			unsigned long now = micros();
			// move only if the appropriate delay has passed:
			if (now - this->last_step_time >= this->step_delay)
			{
			  // get the timeStamp of when you stepped:
			  this->last_step_time = now;
			  // increment or decrement the step number,
			  // depending on direction:
			  if (this->direction == 1)
			  {
				this->step_number++;
				if (this->step_number == this->number_of_steps) {
				  this->step_number = 0;
				}
			  }
			  else
			  {
				if (this->step_number == 0) {
				  this->step_number = this->number_of_steps;
				}
				this->step_number--;
			  }
			  // decrement the steps left:
			  steps_left--;
			  // step the motor to step number 0, 1, ..., {3 or 10}
			  if (this->pin_count == 5)
				stepMotor(this->step_number % 10);
			  else
				stepMotor(this->step_number % 4);
			}
		  }
		}

		
		
		
		/*
		 * Moves the motor forward or backwards.
		 */
		 
		 
		 
		void Stepper::stepMotor(int thisStep) 
		{
		  if (this->pin_count == 2) {
			switch (thisStep) {
			  case 0:  // 01
				digitalWrite(motor_pin_1, LOW);
				digitalWrite(motor_pin_2, HIGH);
			  break;
			  case 1:  // 11
				digitalWrite(motor_pin_1, HIGH);
				digitalWrite(motor_pin_2, HIGH);
			  break;
			  case 2:  // 10
				digitalWrite(motor_pin_1, HIGH);
				digitalWrite(motor_pin_2, LOW);
			  break;
			  case 3:  // 00
				digitalWrite(motor_pin_1, LOW);
				digitalWrite(motor_pin_2, LOW);
			  break;
			}
		  }
		  if (this->pin_count == 4) {
			switch (thisStep) {
			  case 0:  // 1010
				digitalWrite(motor_pin_1, HIGH);
				digitalWrite(motor_pin_2, LOW);
				digitalWrite(motor_pin_3, HIGH);
				digitalWrite(motor_pin_4, LOW);
			  break;
			  case 1:  // 0110
				digitalWrite(motor_pin_1, LOW);
				digitalWrite(motor_pin_2, HIGH);
				digitalWrite(motor_pin_3, HIGH);
				digitalWrite(motor_pin_4, LOW);
			  break;
			  case 2:  //0101
				digitalWrite(motor_pin_1, LOW);
				digitalWrite(motor_pin_2, HIGH);
				digitalWrite(motor_pin_3, LOW);
				digitalWrite(motor_pin_4, HIGH);
			  break;
			  case 3:  //1001
				digitalWrite(motor_pin_1, HIGH);
				digitalWrite(motor_pin_2, LOW);
				digitalWrite(motor_pin_3, LOW);
				digitalWrite(motor_pin_4, HIGH);
			  break;
			}
		  }

		  if (this->pin_count == 5) {
			switch (thisStep) {
			  case 0:  // 01101
				digitalWrite(motor_pin_1, LOW);
				digitalWrite(motor_pin_2, HIGH);
				digitalWrite(motor_pin_3, HIGH);
				digitalWrite(motor_pin_4, LOW);
				digitalWrite(motor_pin_5, HIGH);
				break;
			  case 1:  // 01001
				digitalWrite(motor_pin_1, LOW);
				digitalWrite(motor_pin_2, HIGH);
				digitalWrite(motor_pin_3, LOW);
				digitalWrite(motor_pin_4, LOW);
				digitalWrite(motor_pin_5, HIGH);
				break;
			  case 2:  // 01011
				digitalWrite(motor_pin_1, LOW);
				digitalWrite(motor_pin_2, HIGH);
				digitalWrite(motor_pin_3, LOW);
				digitalWrite(motor_pin_4, HIGH);
				digitalWrite(motor_pin_5, HIGH);
				break;
			  case 3:  // 01010
				digitalWrite(motor_pin_1, LOW);
				digitalWrite(motor_pin_2, HIGH);
				digitalWrite(motor_pin_3, LOW);
				digitalWrite(motor_pin_4, HIGH);
				digitalWrite(motor_pin_5, LOW);
				break;
			  case 4:  // 11010
				digitalWrite(motor_pin_1, HIGH);
				digitalWrite(motor_pin_2, HIGH);
				digitalWrite(motor_pin_3, LOW);
				digitalWrite(motor_pin_4, HIGH);
				digitalWrite(motor_pin_5, LOW);
				break;
			  case 5:  // 10010
				digitalWrite(motor_pin_1, HIGH);
				digitalWrite(motor_pin_2, LOW);
				digitalWrite(motor_pin_3, LOW);
				digitalWrite(motor_pin_4, HIGH);
				digitalWrite(motor_pin_5, LOW);
				break;
			  case 6:  // 10110
				digitalWrite(motor_pin_1, HIGH);
				digitalWrite(motor_pin_2, LOW);
				digitalWrite(motor_pin_3, HIGH);
				digitalWrite(motor_pin_4, HIGH);
				digitalWrite(motor_pin_5, LOW);
				break;
			  case 7:  // 10100
				digitalWrite(motor_pin_1, HIGH);
				digitalWrite(motor_pin_2, LOW);
				digitalWrite(motor_pin_3, HIGH);
				digitalWrite(motor_pin_4, LOW);
				digitalWrite(motor_pin_5, LOW);
				break;
			  case 8:  // 10101
				digitalWrite(motor_pin_1, HIGH);
				digitalWrite(motor_pin_2, LOW);
				digitalWrite(motor_pin_3, HIGH);
				digitalWrite(motor_pin_4, LOW);
				digitalWrite(motor_pin_5, HIGH);
				break;
			  case 9:  // 00101
				digitalWrite(motor_pin_1, LOW);
				digitalWrite(motor_pin_2, LOW);
				digitalWrite(motor_pin_3, HIGH);
				digitalWrite(motor_pin_4, LOW);
				digitalWrite(motor_pin_5, HIGH);
				break;
			}
		  }
		}

		
		
		/*
		  version() returns the version of the library:
		*/
		
		
		
		int Stepper::version(void)
		{
		  return 5;
		}
		