/*
	**************************************************
	** SmoothStepper - Stepper motor speed profiles **
	**************************************************

	By Victor Henrique Salvi (victorsvi@gmail.com)
	
	Functionality:
		Its a interface to use stepper motors in open loop configuration.
		The library supports two kinds of movement:
			(A) Accelerate to a given speed and keep going. It will accelerate to the desired speed and then will keep moving at constant speed.
				This mode is intended to applications which the motor will rotate for a given time and the number of rotations done isn't relevant. 
				This mode supports real time speed profiles: you can change the speed while the motor is rotating and it will account the current
				speed in order to reach the new target speed.
			(B) Displace a given angle of rotation smoothly. It will start stopped and accelerate to the desired speed then move at constant speed than deccelerate ultil stop. 
				The full movement (stopped - at targed speed - stopped) will displace a given ammount. Sometimes the relation between the acceleration and the
				given displacement don't allow the motor to reach the desired speed so it will deccelerate before reaching the targed speed.
				This mode is intended to applications which the motor needs to do precise finite movements.
			    The angle displacement is known before the movement is plannned. The movement is planned and can't be changed in real time. If the motor
				is running and a new displacement is set, the motor will stop immediately.
		This interface is not intended to handle specific stepper drivers. Its reponssible to generate speed profiles and abstract the fact that the
		stepper can just move at discrete steps.
		Please notice that besides the fact that this library specifies the variables as real numbers, the motor might not be able to rotate at
		certain speeds or rotate a certain amount. The problem resides in the discretization of the stepper movement. Problems can occur when:
			- Trying to do an agular displacement smaller then the step angle of the motor. The motor might not rotate or rotate more than specified.
			- Trying to rotate at a very slow speed. The inter-step delay might be too big so the rotation is done in small increments in rotation angle.
			- Trying to rotate at a very high speed. The motor/driver might not support such step frequecy or the program might take longer to loop than the time to do the next step.
			- Trying to accelerate at a very high rate. The motor plus the load have inerce. The motor torque might not be strong enough to speed up the system fast.
		Obs: Positive speeds will make the motor rotate Clock-Wise. Negative Positive speeds will make the motor rotate Counter Clock-Wise.
		
	Changelog:
		08/05/2019:
			Frist version.

*/

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifndef H_SMOOTHSTEPPER
#define H_SMOOTHSTEPPER

/* Data Types */

	enum e_StepDirection {CCW = 0, CW}; //CCW -> Counter Clock-Wise, CW -> Clock-Wise
	enum e_StepperMode {IDDLE = 0, DISPLACE_A, DISPLACE_C, DISPLACE_D, RUN_A, RUN_C}; //DISPLACE -> Move x steps from still state to still state, RUN -> Accelerate to x speed and keep going

	struct s_StepperDef {
		uint8_t stepsPerRev;
		uint8_t stepMultiplier;
		float stepAngle;
	};

	struct s_StepperState {
		enum e_StepDirection dir;
		float targSpeed;
		float startSpeed;
		//float angDisp;
		float angSpeed;
		float angAccel;
		float angDeccel;
		int16_t steps;
		uint32_t timeLastStep;
		uint32_t delayNextStep;
	};

	struct s_StepperPlan {
		enum e_StepperMode mode;
		int16_t stepsAccel;
		int16_t stepsConst;
		int16_t stepsDeccel;
		float targSpeed;
	};

	struct s_Stepper {
		struct s_StepperDef def; //definitions of the current motor
		struct s_StepperState state; //variables that remember the state of the rotational movement
		struct s_StepperPlan plan; //current mode and data of the movement plan
	};

	typedef struct s_Stepper t_Stepper;

/* Functions */

	/* Creates a new stepper 
	 * The stepsPerRev parameter is how many steps the motor needs to do in order to complete one full rotation. It must be greater than zero. 
	 * The stepMultiplier is related to the indexer used (full step = 1, half-step = 2, microstep 1/4 = 4, ...). It must be greater than zero. 
	 * The total steps to do one revolution is the two parameters multiplied.
	 */
	t_Stepper *stepperNew (uint8_t stepsPerRev, uint8_t stepMultiplier);

	/* Set to mode (A) (see library description) and accelerates the speed to the target value.
	 * The angSpeed is the speed that the motor is expected to reach in rad/s. Its sign determines the direction of rotation of the motor.
	 * The angAccel is the acceleration rate of the motor to reach the target speed in rad/s^2. It must be sent in absolute value and must be greater than zero, as the funcion will calculate the sign of the acceleration based on the difference between the current and the target speed.
	 */
	void stepperSetSpeed (t_Stepper *stepper, float angSpeed, float angAccel);

	/* Set to mode (B) (see library description) and displaces the given value.
	 * The angDisp is the angle displacement that the motor needs to do in rad. Its sign determines the direction of rotation of the motor.
	 * The angSpeed is the speed that the motor is expected to reach in rad/s. Its sign determines the direction of rotation of the motor. Both the linDisp and linSpeed must have the same sign.
	 * The angAccel is the acceleration rate of the motor to reach the target speed from rest in rad/s^2. It must be sent in absolute value and must be greater than zero, as the funcion will calculate the sign of the acceleration based on the target speed.
	 * The angDeccel is the acceleration rate of the motor to reach rest from the target speed in rad/s^2. It must be sent in absolute value and must be greater than zero, as the funcion will calculate the sign of the acceleration based on the target speed.
	 */
	void stepperSetDisplacement (t_Stepper *stepper, float angDisp, float angSpeed, float angAccel, float angDeccel);

	/* Executes the motor movement.
	 * Calculates if its time to step. If its true, then calls the stepFunc to do the step.
	 * Updates the movement plan and angular speed.
	 * The time parameter must the the current time of the system in microseconds.
	 * The funcion sent as parameter (stepFunc) should handle the stepper underlying details and do one step when called.
	 * It returns the time delay to the next step in microseconds.
	 * It should be called at small time intervals or right after the step delay is past.
	 */
	uint32_t stepperRun (t_Stepper *stepper, uint32_t time, void (*stepFunc) (enum e_StepDirection dir));
	
	/* Checks if the stepper is running.
	 * Returns 1 if its running, 0 otherwise
	 */
	uint8_t stepperIsActive(t_Stepper *stepper);

	/* Imediatly stops the stepper motor. The stepper state and move plan will be reseted */
	void stepperStop (t_Stepper *stepper);

	/* dealocate a stepper */
	void stepperKill (t_Stepper *stepper);

#endif

#ifdef __cplusplus
}
#endif
