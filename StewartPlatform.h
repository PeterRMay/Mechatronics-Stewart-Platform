
#include <PWM.h>
//#include <MyRio.h>



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






