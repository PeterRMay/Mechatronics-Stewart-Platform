/* Lab <PWM Testing>
 * Author: <Julius Wichert>
 * Date: <3/3/24>
 * Description: <lab exercise description>
 */

/* includes */
#include <stdio.h>
#include "MyRio.h"
#include "T1.h"
#include "PWM.h"
#include "time.h"
#include <TimerIRQ.h>
#include "Encoder.h"
#include "pthread.h"
#include <unistd.h>
#include <MyRio1900.h>
#include "DIO.h"
#include "StewartPlatformConstants.h"
#include "StewartPlatform.c"
#include "conC_Encoder_initialize.h"
#include "conC_Encoder_initialize.c"
#include "matrixMath.h"
#include "baseToStaticPlatformPosition.h"

extern NiFpga_Session myrio_session;

/* definitions */

typedef struct {
	NiFpga_IrqContext irqContext; 			// context
	NiFpga_Bool irqThreadRdy; 			// ready flag
	NiFpga_Bool errorFlag;					//ErrorFlag for calculations
	NiFpga_Bool servoMode;				//Servo movement Mode
} ThreadResource;

#define EncoderCountRev 14400 // counts per revolution
#define DegPerRev 360 // degrees per revolution

MyRio_Encoder encC0,encC1; //Declare all the encoder information

double EncoderPos[2];	//Make sure to specify which position is X and Y axis
static double ZeroPosition = 200;
static double MaxAngle = 200;
static double MaxCount = 3125;
static double MinCount = 625;

double ZeroAngles[] = { 0, 0, 0, 0, 0, 0 };
double DesiredAngles[] = { -90, -90, -90, -90, -90, -90 };

MyRio_Pwm pwmA0, pwmA1, pwmA2, pwmB0, pwmB1, pwmB2; //Declare all the PWM channels

static MyRio_Dio Ch0;	//Declare the digital input channels
static MyRio_Dio Ch1;

/* prototypes */
double GetPulse(double DesiredAngle);
void MoveServos(double *Angles, MyRio_Pwm *PWM_Channels);

void *Timer_Irq_Thread(void* resource);

void InitializePWM();

void home(void);
void responding(void);
void error(void);

/* Define an enumerated type for states */

typedef enum {Home=0, Responding, Error, Exit} State_Type;

/* Define a table of pointers to the functions for each state */

static void (*state_table[])(void)={home, responding, error};

//Has to go after the enum type declaration to declare current state holder variable
static State_Type curr_state;

/**
 * Overview:
 * Demonstrates using the PWM. Generates a PWM signal from PWM 0 on
 * connector A.
 *
 * Instructions:
 * 1. Connect an oscilloscope to the PWM 0 pin on connector A.
 * 2. Run this program.
 *
 * Output:
 * The program generates a signal at 50 Hz
 *
 * Note:
 * The Eclipse project defines the preprocessor symbol for the myRIO-1900.
 * Change the preprocessor symbol to use this example with the myRIO-1950.
 */
int main(int argc, char **argv) {
	NiFpga_Status status;

	/*
	 * Open the myRIO NiFpga Session.
	 * This function MUST be called before all other functions. After this call
	 * is complete the myRIO target will be ready to be used.
	 */
	status = MyRio_Open();
	if (MyRio_IsNotSuccess(status)) {
		return status;
	}

	//Making the Timer Thread-----------------------------------------------------------------------------------------------------------------------

	//Initialize the Timer values
	static ThreadResource irqThread0;		//Thread Resource
	static pthread_t thread;							//Thread Identifier
	static MyRio_IrqTimer irqTimer0;		//
	uint32_t timeoutValue;						//Timeout Value in nano-second

	// Registers Corresponding to the IRQ Channel
	// And set the indicator to allow the new thread
	irqTimer0.timerWrite = IRQTIMERWRITE;
	irqTimer0.timerSet = IRQTIMERSETTIME;
	timeoutValue = 200;

	//Set the default parameters for the error flag and servo mode
	irqThread0.errorFlag = 0;
	irqThread0.servoMode = 0;

	//Register the Timer IRQ
	status = Irq_RegisterTimerIrq(&irqTimer0, &irqThread0.irqContext,
			timeoutValue);

	//Set the indicator to allow the new thread
	irqThread0.irqThreadRdy = NiFpga_True;

	//Create new thread to catch the IRQ
	status = pthread_create(&thread,
	NULL, Timer_Irq_Thread, &irqThread0);

	//Other Main Tasks below----------------------------------------------------------------------------------------------------------------------------------

	while (irqThread0.irqThreadRdy != NiFpga_False) {

	}

	//Wait for thread to terminate--------------------------------------------------------------------------------------------------------------------------------
	irqThread0.irqThreadRdy = NiFpga_False;
	status = pthread_join(thread, NULL);

	//Unregister the IRQ
	status = Irq_UnregisterTimerIrq(&irqTimer0, irqThread0.irqContext);

	//Close the myRIO NiFpga Session.

	status = MyRio_Close();

	return status;
}

void InitializePWM(void) {

	uint8_t selectReg;
	NiFpga_Status status;

	//Create the PWM channel structures

	MyRio_Pwm PWM_Channels[] = { pwmA0, pwmA1, pwmA2, pwmB0, pwmB1, pwmB2 };

	/*
	 * Initialize the PWM struct with registers from the FPGA personality.
	 */
	pwmA0.cnfg = PWMA_0CNFG;
	pwmA0.cs = PWMA_0CS;
	pwmA0.max = PWMA_0MAX;
	pwmA0.cmp = PWMA_0CMP;
	pwmA0.cntr = PWMA_0CNTR;
	pwmA1.cnfg = PWMA_1CNFG;
	pwmA1.cs = PWMA_1CS;
	pwmA1.max = PWMA_1MAX;
	pwmA1.cmp = PWMA_1CMP;
	pwmA1.cntr = PWMA_1CNTR;
	pwmA2.cnfg = PWMA_2CNFG;
	pwmA2.cs = PWMA_2CS;
	pwmA2.max = PWMA_2MAX;
	pwmA2.cmp = PWMA_2CMP;
	pwmA2.cntr = PWMA_2CNTR;
	pwmB0.cnfg = PWMB_0CNFG;
	pwmB0.cs = PWMB_0CS;
	pwmB0.max = PWMB_0MAX;
	pwmB0.cmp = PWMB_0CMP;
	pwmB0.cntr = PWMB_0CNTR;
	pwmB1.cnfg = PWMB_1CNFG;
	pwmB1.cs = PWMB_1CS;
	pwmB1.max = PWMB_1MAX;
	pwmB1.cmp = PWMB_1CMP;
	pwmB1.cntr = PWMB_1CNTR;
	pwmB2.cnfg = PWMB_2CNFG;
	pwmB2.cs = PWMB_2CS;
	pwmB2.max = PWMB_2MAX;
	pwmB2.cmp = PWMB_2CMP;
	pwmB2.cntr = PWMB_2CNTR;

	/*
	 * Set the waveform, enabling the PWM onboard device.
	 */
	Pwm_Configure(&pwmA0, Pwm_Invert | Pwm_Mode, Pwm_NotInverted | Pwm_Enabled);
	Pwm_Configure(&pwmA1, Pwm_Invert | Pwm_Mode, Pwm_NotInverted | Pwm_Enabled);
	Pwm_Configure(&pwmA2, Pwm_Invert | Pwm_Mode, Pwm_NotInverted | Pwm_Enabled);
	Pwm_Configure(&pwmB0, Pwm_Invert | Pwm_Mode, Pwm_NotInverted | Pwm_Enabled);
	Pwm_Configure(&pwmB1, Pwm_Invert | Pwm_Mode, Pwm_NotInverted | Pwm_Enabled);
	Pwm_Configure(&pwmB2, Pwm_Invert | Pwm_Mode, Pwm_NotInverted | Pwm_Enabled);

	/*
	 * Set the clock divider. The internal PWM counter will increments at
	 * f_clk / 4
	 *
	 * where:
	 *  f_clk = the frequency of the myRIO FPGA clock (40 MHz default)
	 */
	Pwm_ClockSelect(&pwmA0, Pwm_32x);
	Pwm_ClockSelect(&pwmA1, Pwm_32x);
	Pwm_ClockSelect(&pwmA2, Pwm_32x);
	Pwm_ClockSelect(&pwmB0, Pwm_32x);
	Pwm_ClockSelect(&pwmB1, Pwm_32x);
	Pwm_ClockSelect(&pwmB2, Pwm_32x);

	/*
	 * Set the maximum counter value. The counter counts from 0 to 25000.
	 *
	 * The counter increments at 40 MHz / 32 = 1.25 MHz and the counter counts
	 * from 0 to 1000. The frequency of the PWM waveform is 10 MHz / 25000
	 * = 50 Hz.
	 */
	Pwm_CounterMaximum(&pwmA0, 25000);
	Pwm_CounterMaximum(&pwmA1, 25000);
	Pwm_CounterMaximum(&pwmA2, 25000);
	Pwm_CounterMaximum(&pwmB0, 25000);
	Pwm_CounterMaximum(&pwmB1, 25000);
	Pwm_CounterMaximum(&pwmB2, 25000);

	//Send the servos to the zero positions

	MoveServos(ZeroAngles, PWM_Channels);

	/*
	 * PWM outputs are on pins shared with other onboard devices. To output on
	 * a physical pin, select the PWM on the appropriate SELECT register. See
	 * the MUX example for simplified code to enable-disable onboard devices.
	 *
	 * Read the value of the SYSSELECTA register.
	 */
	status = NiFpga_ReadU8(myrio_session, SYSSELECTA, &selectReg);

	status = NiFpga_ReadU8(myrio_session, SYSSELECTA, &selectReg);

	status = NiFpga_ReadU8(myrio_session, SYSSELECTA, &selectReg);

	status = NiFpga_ReadU8(myrio_session, SYSSELECTB, &selectReg);

	status = NiFpga_ReadU8(myrio_session, SYSSELECTB, &selectReg);

	status = NiFpga_ReadU8(myrio_session, SYSSELECTB, &selectReg);

	/*
	 * Set bit2 of the SYSSELECTA register to enable PWMA_0 functionality.
	 * The functionality of the bit is specified in the documentation.
	 */

	selectReg = selectReg | (1 << 2);
	selectReg = selectReg | (1 << 3);
	selectReg = selectReg | (1 << 4);
	selectReg = selectReg | (1 << 2);
	selectReg = selectReg | (1 << 3);
	selectReg = selectReg | (1 << 4);

	/*
	 * Write the updated value of the SYSSELECTA register.
	 */
	status = NiFpga_WriteU8(myrio_session, SYSSELECTA, selectReg);

	status = NiFpga_WriteU8(myrio_session, SYSSELECTA, selectReg);

	status = NiFpga_WriteU8(myrio_session, SYSSELECTA, selectReg);

	status = NiFpga_WriteU8(myrio_session, SYSSELECTB, selectReg);

	status = NiFpga_WriteU8(myrio_session, SYSSELECTB, selectReg);

	status = NiFpga_WriteU8(myrio_session, SYSSELECTB, selectReg);

}


void pos(double PosDeg[3]) {
 	static int first = 1; 	//first time calling function
	int Cn1,Cn2;		//variable to hold the current encoder count
	static int Cn11,Cn12;		//variable to hold the previous encoder count
	double EncoderPosBDI[2];

	if (first ==1) {
		Cn11 = Encoder_Counter(&encC0);		//set the previous encoder count for the first time though
		Cn12 = Encoder_Counter(&encC1);
		first = 0;					//check to see if this is the first time we are doing this
	}

	Cn1 = Encoder_Counter(&encC0);			//set the current encoder count
	Cn2 = Encoder_Counter(&encC1);

	EncoderPosBDI[0] = Cn1 - Cn11;		//get the difference in encoder counts to return for the position of the encoder
	EncoderPosBDI[1] = Cn2 - Cn12;
	PosDeg[0] = (EncoderPosBDI[0] / EncoderCountRev) * DegPerRev;
	PosDeg[1] = (EncoderPosBDI[1] / EncoderCountRev) * DegPerRev;

}

void *Timer_Irq_Thread(void* resource) {

	// Cast the Input argument back to its intended form---------------------------------------------------------------------------------------------------------
	ThreadResource* threadResource = (ThreadResource*) resource;

	NiFpga_Bool Status;

	//Initialize all Encoders, Servos, State Machines and parameters needed--------------------------------------------------------------------------------


	//Initialize PWM channels and there parameters
	InitializePWM();

	//Initialize Encoders
	Status = conC_Encoder_Initialize(myrio_session, encC0, 0);
	Status = conC_Encoder_Initialize(myrio_session, encC1, 1);

	//Initialize the digital input/output channels
	Ch0.dir = DIOA_70DIR;
	Ch0.out = DIOA_70OUT;
	Ch0.in = DIOA_70IN;
	Ch0.bit = 0;

	Ch1.dir = DIOA_70DIR;
	Ch1.out = DIOA_70OUT;
	Ch1.in = DIOA_70IN;
	Ch1.bit = 1;

	//Declare all variables and constants for the interrupt
	MyRio_Pwm PWM_Channels[] = { pwmA0, pwmA1, pwmA2, pwmB0, pwmB1, pwmB2 };

	// Platform dimensions
	const double radius = 5.0; // inches
	const double angleBetweenServos = 60; // degrees
	const double servoRotationOffset = 120; // degrees
	const double s = 10.0;
	const double a = 2.0;
	const double l_max = s+a; // maximum extension of leg
	const double l_min = s-a; // minimum extension of leg
	const double h0 = 10.198039; // home height
	const double beta[6] = {510.0, -30.0, 270.0, 90.0, 390.0, 210.0};
	const double B[3][6] = {{8.660254, 0.0, -8.660254, -8.660254, 0.0, 8.660254},
							{5.0, 10.0, 5.0, -5.0, -10.0, -5.0},
							{0, 0, 0, 0, 0, 0}}; // base dimensions in base reference frame
	const double P[3][6] = B; // platform dimensions in platform reference frame
	const double alpha0[6] = {11.309932, 11.309932, 11.309932, 11.309932, 11.309932, 11.309932}; // home servo position, servo arms are at 90deg to legs
	const double servoRotationRange = 180; // degrees
	const double alphaMax = *alpha0 + 0.5*servoRotationRange;
	const double alphaMin = *alpha0 - 0.5*servoRotationRange;

	// Platform position
	double T[3];
	double Phi[3];
	double P_base[3][6];
	double legLengths[6];
	double T_desired[3] = {0, 0, 0};
	double Phi_desired[3] = {0, 0, 0};
	double D[3] = {0, 0, 0};
	double Gamma[3]; // pitch and roll of platform
	
	int ErrorFlag = 0;

	NiFpga_Bool OnButton = NiFpga_False;

	NiFpga_Bool OffButton = NiFpga_False;

	/* The While loop below will perform two tasks while waiting for a signal to stop---------------------------------------------------------------------
	 *  - It will wait for the occurrence( or timeout) of the IRQ
	 *  		- If it has, "Schedule" the next interrupt
	 *  - if the Timer IRQ has asserted
	 *  		- Perform operations to service the interrupt
	 *  		- Acknowledge the interrupt
	 */

	while (threadResource->irqThreadRdy == NiFpga_True) {
		//Reset the on and off button boolean values
		OnButton = NiFpga_False;
		OffButton = NiFpga_False;

		uint32_t timeoutValue = 20000;

		uint32_t irqAssert = 0;

		Irq_Wait(threadResource->irqContext,
		TIMERIRQNO, &irqAssert, (NiFpga_Bool*) &(threadResource->irqThreadRdy));

		
		extern NiFpga_Session myrio_session;

		if (irqAssert) {
			//Your Interrupt Service Code here-------------------------------------------------------------------------------------------------------------------
			NiFpga_WriteU32(myrio_session, IRQTIMERWRITE, timeoutValue);
      		NiFpga_WriteBool(myrio_session, IRQTIMERSETTIME, NiFpga_True);

			//Use pos() to get the position of each encoder relative to the starting position
			pos(Gamma);

			//Check to see if the platform is out of range or the platforms min/max settings
				//If so, set the error flag to 1 and servo flag to 0
				//If not continue
			baseToStaticPlatformPosition(T,Phi,D,Gamma,T_desired,Phi_desired,h0);

			// Calculate required leg lengths
			ErrorFlag = GetLegLengths(P_base, legLengths, T, Phi, B, l_max, l_min);

			//Take the Base position and calculate the required servo correction
				//If the correction is outside the servos range, set the error flag to 1 and servo flag to 0
				//If not continue
			// servoCalc(P_base, )

			//Set the appropriate state
			OnButton = Dio_ReadBit(Ch0);		//Check On Button
			OffButton = Dio_ReadBit(Ch1);		//Check Off Button

				//Check for Home State
				if(curr_state == Home) {
					if(OnButton == NiFpga_True && ErrorFlag == NiFpga_False && OffButton == NiFpga_False) {
						curr_state = Responding;
					} else if(ErrorFlag == NiFpga_True && OffButton == NiFpga_False) {
						curr_state = Error;
					} else if(OffButton == NiFpga_True) {
						curr_state = Exit;
					} else {
						curr_state = Home;
					}
				}

				//Check for Responding State
				if(curr_state == Responding) {
					if(ErrorFlag == NiFpga_True && OffButton ==NiFpga_False) {
						curr_state = Error;
					} else if(OffButton == NiFpga_True) {
						curr_state = Home;
					} else {
						curr_state = Responding;
					}
				}

				//Check for Error State
				if(curr_state == Error) {
					if(OnButton == NiFpga_True && ErrorFlag == NiFpga_False && OffButton == NiFpga_False) {
						curr_state = Home;
					} else if(ErrorFlag == NiFpga_True && OffButton == NiFpga_False) {
						curr_state = Error;
					} else if(OffButton == NiFpga_True) {
						curr_state = Home;
					} else {
						curr_state = Error;
					}

				}

			//run the current state
			state_table[curr_state]();

			//Acknowledge the interrupt-----------------------------------------------------------------------------------------------------------------------------
			Irq_Acknowledge(irqAssert);
		}
	}

	//Terminate the new thread and return the function
	pthread_exit(NULL);
	return NULL;
}

int GetLegLengths(double P_base[3][6], double legLengths[6], double T[3], double Phi[3], double P[3][6], double B[3][6], double l_max, double l_min) {
	double R[3][3];
	double l[3][6];
	int i;
	int errorFlag = 0;
	
	// calculate rotation matrix
	rotZYX(Phi, R);

	// find vector of each leg, l = (T + R*P) - B
	l = matrixSubtract33x33(matrixAdd33x33(T,matrixMultiply33x31(R,P)), B);

	// find length of each leg vector
	for (i = 0; i < 6; i++) {
		legLengths[i] = sqrt(l[1][i]*l[1][i] + l[2][i]*l[2][i] + l[3][i]*l[3][i]);
		// check if leg lengths are too long or short
		if (legLengths[i] < l_min || legLengths[i] > l_max) {
			errorFlag = 1;
		}
		vectorAdd(l[3][i], P[3][i], P_base[3][i]); // find platform positions 
	}

	return errorFlag;
}

void home(void) {
	//Setup PWM channels array
	static MyRio_Pwm PWM_Channels[] = { pwmA0, pwmA1, pwmA2, pwmB0, pwmB1, pwmB2 };

	// Send the servos to the home position
	MoveServos(ZeroAngles, PWM_Channels);

}

void responding(void) {
	//Setup PWM channels array
	static MyRio_Pwm PWM_Channels[] = { pwmA0, pwmA1, pwmA2, pwmB0, pwmB1, pwmB2 };

	// Send the servos to the home position
	MoveServos(DesiredAngles, PWM_Channels);

}

void error(void) {

}



