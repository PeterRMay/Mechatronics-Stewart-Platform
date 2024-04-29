
#include <PWM.h>
#include <StewartPlatformConstants.h>
#include <MyRio.h>

/*
 * Converting the input angle to output angle.
 * The servo operates from 0 to 200 degrees but we want only 0 to 180.
 * max pulse width = 2500 micro seconds
 * min pulse width = 500 micro seconds
 * total pulse = 2o milliseconds
 * max counter = 3125 of 25000
 * min counter = 625 of 25000
 *
 * Steps:
 * 1) Intakes desired angle
 * 2) Compensate for zero position
 * 3) Find Percent of angle movement
 * 4) Convert to percent of pulse width
 * 5) Send pulse width to signal generation
 * 6) Repeat until told to stop
 *
 *
 * ZeroPosition = 90 degrees
 * MaxAngle = 200
 * MaxCount = 3125
 * MinCount = 625
 *
 * Compensate by wanted wanted from zero if we consider 90 to be zero.
 *
 * GoAngle = DesiredAngle + ZeroPosition
 *
 * AnglePercent = GoAngle/MaxAngle
 *
 * PulseCount = ((MaxCount - MinCount) * AnglePercent) + MinCount
 */

double GetPulse(double DesiredAngle) {
	double PulseCount;
	double GoAngle;
	double AnglePercent;
    double AngleRange = 200; 
    double MaxCount = 3125; 
    double MinCount = 625; 
    double ZeroPosition = 200;


	GoAngle = DesiredAngle + ZeroPosition;

	AnglePercent = GoAngle / AngleRange;

	PulseCount = ((MaxCount - MinCount) * AnglePercent) + MinCount;

	return PulseCount;
}

void MoveServos(double Angles[6], MyRio_Pwm *PWM_Channels) {

	//Create Pointers to the Individual PWM Structures that we fed in
	MyRio_Pwm *PWM1 = PWM_Channels;
	MyRio_Pwm *PWM2 = PWM_Channels + 1;
	MyRio_Pwm *PWM3 = PWM_Channels + 2;
	MyRio_Pwm *PWM4 = PWM_Channels + 3;
	MyRio_Pwm *PWM5 = PWM_Channels + 4;
	MyRio_Pwm *PWM6 = PWM_Channels + 5;

	//Change the PWM Counter compare value to the needed one for the given desired angle for that specific signal
	Pwm_CounterCompare(PWM1, GetPulse(Angles[0]));
	Pwm_CounterCompare(PWM2, GetPulse(Angles[1]));
	Pwm_CounterCompare(PWM3, GetPulse(Angles[2]));
	Pwm_CounterCompare(PWM4, GetPulse(Angles[3]));
	Pwm_CounterCompare(PWM5, GetPulse(Angles[4]));
	Pwm_CounterCompare(PWM6, GetPulse(Angles[5]));

}






