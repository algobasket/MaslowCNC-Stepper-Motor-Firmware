	#ifndef Stepper_h
	#define Stepper_h

	#include "Arduino.h"
	#include "Encoder.h"
	#include "Motor.h"
	#include "PID_v1.h" 
	

	class Stepper {
	  public:
		// constructors:
		Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2);
		Stepper(int number_of_steps, int motor_pin_1, int motor_pin_2, int motor_pin_3, int motor_pin_4);
        
		#Modified Start by Algobasket
		
		Encoder    encoder;
		Motor      motor; 
		float      computeSpeed();
		void       write(float speed);
		void       computePID();
		void       setName(String newName);
		void       initializePID();
		void       setPIDAggressiveness(float aggressiveness);
		
		
		#Modified End by Algobasket 
		
		// speed setter method:
		void setSpeed(long whatSpeed); 

		// mover method:
		void step(int number_of_steps); 

		int version(void);

	  private:
		void stepMotor(int this_step); 
		
		int direction;              // Direction of rotation
		int speed;                  // Speed in RPMs
		unsigned long step_delay;    // delay between steps, in ms, based on speed
		int number_of_steps;         // total number of steps this motor can take
		int pin_count;               // whether you're driving the motor with 2 or 4 pins
		int step_number;             // which step the motor is on
		
		// motor pin numbers:
		int motor_pin_1;
		int motor_pin_2;
		int motor_pin_3;
		int motor_pin_4;
		
		#Modified Start by Algobasket
		
		
		
		double     _targetSpeed;
		double     _currentSpeed;
		double     _lastPosition;
		double     _lastTimeStamp;
		float      _runningAverage(int newValue);
		String     _motorName;
		double     _pidOutput;
		double     _Kp=20, _Ki=5, _Kd=0;
		PID        _posPIDController;
		PID        _negPIDController;
		int        _oldValue1;
		int        _oldValue2;
		int        _oldValue3;
		int        _oldValue4;
		int        _oldValue5;
		int        _oldValue6;
		int        _oldValue7;
		int        _oldValue8;
		int        _oldValue9;
		int        _oldValue10;
		
		#Modified End by Algobasket
		
		
		
		
		
		long last_step_time;      // time stamp in ms of when the last step was taken
	};

	#endif
