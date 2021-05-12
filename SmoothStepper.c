#include "SmoothStepper.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "Arduino.h"

//#define errorCheck(exp) if(!(exp)) fprintf(stderr,"Assertion error on %s #%d",__FILE__,__LINE__);
#define errorCheck(exp)

//#define DEBUG

//#define PI 3.12159 --defined on arduino.h

#define signAccel(startSpeed,targetSpeed,acceleration) ((targetSpeed) >= (startSpeed) ? (acceleration) : -(acceleration))
#define abs2(a) ((a) >= 0 ? (a) : -(a))

void stepperDump (t_Stepper *stepper);
void stepperReset (t_Stepper *stepper);
float stepperCalcMaxSpeedDisplace(float angDisp, float angAccel, float angDeccel);
int16_t stepperCalcSteps (float stepAngle, float startSpeed, float targSpeed, float angAccel);

/* ------------------ */
/* ----- Public ----- */
/* ------------------ */

t_Stepper *stepperNew (uint8_t stepsPerRev, uint8_t stepMultiplier) {

	//Allocates memory for the Stepper object
	t_Stepper *stepper = NULL;
	stepper = malloc(sizeof(t_Stepper));
	if(stepper == NULL) {
		return NULL;
	}

	//check for programming error as stepsPerRev and stepMultiplier sould be greater than zero
	errorCheck(stepsPerRev > 0.0);
	errorCheck(stepMultiplier > 0.0);

	//sets the physical stepper motor parameter and the stepping mode multiplier
	stepper->def.stepsPerRev = stepsPerRev;
	stepper->def.stepMultiplier = stepMultiplier;
	// calculates the step angle (equation 1)
	stepper->def.stepAngle =  (2.0 * PI / (float)stepsPerRev / (float)stepMultiplier);

	stepperReset(stepper);

	return stepper;
} //stepperNew


void stepperSetSpeed (t_Stepper *stepper, float angSpeed, float angAccel) {

	if(stepper != NULL) {

		errorCheck(angAccel != 0.0);
		if(angAccel == 0) {
			return;
		}

		//the acceleration is recieved in module. The algorithm calculates the signal
		angAccel = abs2(angAccel);

		//Adjust the state, scheduling the acceleration profile
		//keeps dir as its set in the run function
		stepper->state.startSpeed = stepper->state.angSpeed; //the initial speed will be the current speed when the changes occur
		stepper->state.targSpeed = (
			(stepper->state.startSpeed >= 0.0 && angSpeed >= 0.0) || (stepper->state.startSpeed <= 0.0 && angSpeed <= 0.0) ?
				angSpeed :
				0.0
			);
			/* 	if target speed is such that the motor needs to invert direction, the transient is done in 2 steps (frist speed->0 then 0->target)
				 ____				          ____
				|    \				|        /
				|     \				|       /
				-------------		-------------
				|       \			|     /
				|        \____		|____/
			*/

		//stepper->state->angDisp = 0.0;
		//keeps angSpeed
		stepper->state.angAccel = signAccel(stepper->state.startSpeed, stepper->state.targSpeed, angAccel); //sets the new acceleration
		stepper->state.angDeccel = 0.0; //the deceleration isnt used on RUN mode
		stepper->state.steps = 0;
		stepper->state.timeLastStep = 0;
		stepper->state.delayNextStep = 0;

		stepper->plan.mode = RUN_A;
		stepper->plan.stepsAccel = 0;
		stepper->plan.stepsConst = 0;
		stepper->plan.stepsDeccel = 0;
		stepper->plan.targSpeed = angSpeed; //the actual target speed for the RUN mode is set here

	}
} //stepperSetSpeed


void stepperSetDisplacement (t_Stepper *stepper, float angDisp, float angSpeed, float angAccel, float angDeccel) {

	//the sign of the angDisp and angSpeed must be the same!

	float maxSpeed = 0;

	if(stepper != NULL) {

		errorCheck(angSpeed != 0.0);
		errorCheck(angAccel != 0.0);
		errorCheck(angDeccel != 0.0);
		if(angSpeed == 0 || angAccel == 0 || angDeccel == 0) {
			return;
		}
		if(angDisp * angSpeed < 0) { //angdisp and angspeed need to have the same sign
			return;
		}

		//the acceleration is recieved in module. The algorithm calculates the signal
		angAccel = abs2(angAccel);
		angDeccel = abs2(angDeccel);

		/* 	Notice that the target speed (angSpeed) can be either positive or negative.
			A positive speed means that the direction of movement is cockwise and
			a negative speed means that the direction of movement is counter-cockwise
			The sign of the acceleration and deceleration needs to account for this situation,
			as the "acceleration" means the frist ramp of the trapezoid and "deceleration" is the second ramp

		   s|  _____			 ------------t
			| /     \			 |\a     d/
			|/a     d\			 | \_____/
			------------t		s|
		*/
		angAccel = signAccel(0.0, angSpeed, angAccel);
		angDeccel = signAccel(angSpeed, 0.0, angDeccel);

		//Calculates the maximum speed that can be reached while following the set acceleration and deceleration at the set angular displacement (equation 6)
		maxSpeed = stepperCalcMaxSpeedDisplace(angDisp, angAccel, angDeccel); //in module
		maxSpeed = (angSpeed >= 0.0 ? maxSpeed : -maxSpeed); //sets the maximum speed sign as the target speed
		//limits the target speed to the maximum speed (The speed is signed. Take the one with smaller abs2)
		angSpeed = ((abs2(maxSpeed) <= abs2(angSpeed)) ? maxSpeed : angSpeed);
		/* A case which the target speed is greater than the maximum speed:
		   s|  /\
			| /  \
			|/a  d\
			------------t
		*/

		/* Plans the trapezoidal/triangular speed profile */

		//calculates the number of steps to reach the set speed (or the maximum speed) (equation 5)
		stepper->plan.stepsAccel = stepperCalcSteps (stepper->def.stepAngle, 0, angSpeed, angAccel);

		//calculates the number of steps to stop after reaching the target speed (equation 5)
		stepper->plan.stepsDeccel = stepperCalcSteps (stepper->def.stepAngle, angSpeed, 0, angDeccel);

		//the steps at constant speed are the remaining steps from acceleration and decceleration
		stepper->plan.stepsConst = ((int16_t) (angDisp / stepper->def.stepAngle)) - stepper->plan.stepsAccel - stepper->plan.stepsDeccel;
    if((stepper->plan.stepsConst < 0 && angSpeed >= 0.0) || (stepper->plan.stepsConst > 0 && angSpeed < 0.0)) {
      stepper->plan.stepsConst = 0;
    }

		/* Adjust the state, scheduling the acceleration profile */

		//keeps dir as its set in the run function
		stepper->state.startSpeed = 0; //this mode assumes that the initial speed is allways 0
		stepper->state.targSpeed = angSpeed;
		//stepper->state->angDisp = 0.0;
		stepper->state.angSpeed = 0.0; //this mode assumes that the initial speed is allways 0
		stepper->state.angAccel = angAccel; //sets the new acceleration
		stepper->state.angDeccel = angDeccel; //sets the new decceleration
		stepper->state.steps = 0;
		stepper->state.timeLastStep = 0;
		stepper->state.delayNextStep = 0;

		stepper->plan.mode = DISPLACE_A;
		stepper->plan.targSpeed = 0.0; //the DISPLACE mode doesnt use this variables

    if(stepper->plan.stepsAccel == 0) {
      stepper->plan.stepsAccel = (angSpeed >= 0.0 ? 1 : -1);
    }
    if(stepper->plan.stepsDeccel == 0) {
      stepper->plan.stepsDeccel = (angSpeed >= 0.0 ? 1 : -1);
    }

	}

} //stepperSetDisplacement


uint32_t stepperRun (t_Stepper *stepper, uint32_t time, void (*stepFunc) (enum e_StepDirection dir)) {

	uint32_t delay = 0;
	float speed = 0.0;
	float accel = 0.0;

	if(stepper == NULL || stepper->plan.mode == IDDLE) {

		return 0;
	}

	//the time can loop back when enough time has elapsed to reach the maximum value of the variable
	if(((time - stepper->state.timeLastStep) >= stepper->state.delayNextStep) || (time < stepper->state.timeLastStep)) {

		//calc things and step

		//if the mode is constant speed, avoid calculating the equations for the delay/speed as the result won't change
		if(stepper->plan.mode == RUN_C || stepper->plan.mode == DISPLACE_C) {

			delay = (uint32_t) round((1e6 * stepper->def.stepAngle) / abs2(stepper->state.angSpeed));
      if( stepper->state.dir == CW ) {
				stepper->state.steps++;
			}
			else {
				stepper->state.steps--;
			}
		}
		else { //RUN_A, DISPLACE_A or DISPLACE_D

			//the speed is a signed number and it's sign denotates the rotation direction
			stepper->state.dir = (stepper->state.targSpeed > 0.0 || stepper->state.startSpeed > 0.0 ? CW : CCW);
			//set the step to the next step to calculate the next values in order to calculate the delay to the next value.
			if( stepper->state.dir == CW ) { 
				stepper->state.steps++;
			}
			else {
				stepper->state.steps--;
			}

			//sets the "acceleration" rate
			if(stepper->plan.mode == DISPLACE_D) {
				accel = stepper->state.angDeccel;
			}
			else { //RUN_A or DISPLACE_A
				accel = stepper->state.angAccel;
			}

			//sets the speed (equation 4)
			speed = sqrt( (stepper->state.startSpeed * stepper->state.startSpeed) + (2.0 * stepper->state.steps * stepper->def.stepAngle * accel) );//in modulus
			speed = (stepper->state.dir == CW ? speed : -speed);

			//if the speed has overshoot, fix the speed to the target.
			if ((accel >= 0 && speed > stepper->state.targSpeed)
				|| (accel <= 0 && speed < stepper->state.targSpeed)) {
				speed = stepper->state.targSpeed;
			}

			//calculates the delay (equation 7)
			delay = (uint32_t) round(((speed - stepper->state.angSpeed) / accel)*1e6);

			stepper->state.angSpeed = speed;

		}
   stepper->state.delayNextStep = delay;

		//não deve dar passo no primeiro!
		stepFunc(stepper->state.dir);

		if(stepper->plan.mode == RUN_A) {

			//if the speed profile chages direction, the process is done in two steps (curr -> 0; 0 -> targ). This sets the second step
			if(stepper->plan.targSpeed != stepper->state.targSpeed && stepper->state.angSpeed == stepper->state.targSpeed) {

				stepper->state.startSpeed = 0.0;
				stepper->state.targSpeed = stepper->plan.targSpeed;
				//stepper->state->angDisp = 0.0;
				stepper->state.angSpeed = 0.0;
				stepper->state.steps = 0;

			}
			else if (stepper->plan.mode == RUN_A && stepper->state.angSpeed == stepper->state.targSpeed) {

				stepper->plan.mode = RUN_C;
			}
		}

        //if there's no remaining steps in the plan, the displacement is complete
        if(stepper->plan.mode == DISPLACE_D && stepper->plan.stepsDeccel == stepper->state.steps) {

            stepperReset (stepper);
        }

        //sets decelerate mode
        else if(stepper->plan.mode == DISPLACE_C && stepper->plan.stepsConst == stepper->state.steps) {

            stepper->plan.mode = DISPLACE_D;
            stepper->state.angSpeed = stepper->state.targSpeed;
            stepper->state.startSpeed = stepper->state.angSpeed;
            stepper->state.targSpeed = 0.0;
            stepper->state.steps = 0;

        }

        //sets decelerate mode. This is the special case where the speedxtime graphic looks like a triangle
        else if(stepper->plan.mode == DISPLACE_A && stepper->plan.stepsAccel == stepper->state.steps && stepper->plan.stepsConst == 0) {

            stepper->plan.mode = DISPLACE_D;
            stepper->state.angSpeed = stepper->state.targSpeed;
            stepper->state.startSpeed = stepper->state.angSpeed;
            stepper->state.targSpeed = 0.0;
            //stepper->state->angDisp = 0.0;
            stepper->state.steps = 0;

        }

        //sets constant speed mode
        else if(stepper->plan.mode == DISPLACE_A && stepper->plan.stepsAccel == stepper->state.steps && stepper->plan.stepsConst != 0) {

            stepper->plan.mode = DISPLACE_C;
            stepper->state.angSpeed = stepper->state.targSpeed;
            stepper->state.startSpeed = stepper->state.angSpeed;
            //stepper->state->angDisp = 0.0;
            stepper->state.steps = 0;

        }

		stepper->state.timeLastStep = time;
		stepper->state.delayNextStep = delay;

	}

	if(delay == 0) { //if it wasn't time to step, the delay is not calculed again
        delay = stepper->state.delayNextStep;
	}
    return delay;
} //stepperRun

uint8_t stepperIsActive(t_Stepper *stepper) {
	
	if(stepper != NULL) {
		if(stepper->plan.mode == IDDLE) {
			return 0;
		}
		else {
			return 1;
		}
	}
	
	return 0;
} //stepperIsActive

void stepperStop (t_Stepper *stepper) {

	if(stepper != NULL) {
		stepperReset(stepper);
	}
} //stepperStop


void stepperKill (t_Stepper *stepper) {

	if(stepper != NULL) {
		free(stepper);
	}

} //stepperKill


/* ------------------- */
/* ----- Private ----- */
/* ------------------- */

int16_t stepperCalcSteps (float stepAngle, float startSpeed, float targSpeed, float angAccel){

	float angDisp = 0;

	//calculates the number of steps to reach some speed (equation 5)
	angDisp = ((targSpeed * targSpeed) - (startSpeed * startSpeed)) / ( 2.0 * angAccel);

	return (int16_t) round(angDisp/stepAngle);

} //stepperCalcSteps

float stepperCalcMaxSpeedDisplace(float angDisp, float angAccel, float angDeccel){

	float maxSpeed = 0;

	//Calculates the maximum speed that can be reached while following the set acceleration and deceleration at the set angular displacement (equation 6)
	maxSpeed = ((2.0 * angDisp * angAccel * angDeccel) / (angDeccel - angAccel));
	maxSpeed = sqrt(maxSpeed);

	return maxSpeed;

} //stepperCalcMaxSpeedDisplace

void stepperReset (t_Stepper *stepper){

	if(stepper != NULL) {

		stepper->state.dir = CW;
		stepper->state.targSpeed = 0.0;
		stepper->state.startSpeed = 0.0;
		//stepper->state->angDisp = 0.0;
		stepper->state.angSpeed = 0.0;
		stepper->state.angAccel = 0.0;
		stepper->state.angDeccel = 0.0;
		stepper->state.steps = 0;
		stepper->state.timeLastStep = 0;
		stepper->state.delayNextStep = 0;

		stepper->plan.mode = IDDLE;
		stepper->plan.stepsAccel = 0;
		stepper->plan.stepsConst = 0;
		stepper->plan.stepsDeccel = 0;
		stepper->plan.targSpeed = 0.0;
	}
} //stepperReset

#ifdef DEBUG
void stepperDump (t_Stepper *stepper) {

    printf("\nDumping stepper\n");
    if(stepper != NULL) {

        printf("\t\tstate.stepAngle \t%f\n",stepper->def.stepAngle);

        printf("\t\tstate.dir \t\t%s\n",(stepper->state.dir == CW ? "CW" : "CCW"));
        printf("\t\tstate.targSpeed \t%f\n",stepper->state.targSpeed);
        printf("\t\tstate.startSpeed \t%f\n",stepper->state.startSpeed);
        printf("\t\tstate.angSpeed \t\t%f\n",stepper->state.angSpeed);
        printf("\t\tstate.angAccel \t\t%f\n",stepper->state.angAccel);
        printf("\t\tstate.angDeccel \t%f\n",stepper->state.angDeccel);
        printf("\t\tstate.steps \t\t%hd\n",stepper->state.steps);
        printf("\t\tstate.timeLastStep \t%lu\n",stepper->state.timeLastStep);
        printf("\t\tstate.delayNextStep \t%lu\n",stepper->state.delayNextStep);

        printf("\t\tstate.mode \t\t%d\n",stepper->plan.mode);
        printf("\t\tstate.stepsAccel \t%hd\n",stepper->plan.stepsAccel);
        printf("\t\tstate.stepsConst \t%hd\n",stepper->plan.stepsConst);
        printf("\t\tstate.stepsDeccel \t%hd\n",stepper->plan.stepsDeccel);
	}
} //stepperDump
#endif
