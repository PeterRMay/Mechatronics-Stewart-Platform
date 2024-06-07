/* Stewart Platform Controls
 * Author: Team ?
 * Date: 4/30/24
 * Description: 
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
#include "conC_Encoder_initialize.h"
#include "matrixMath.h"
#include "baseToStaticPlatformPosition.h"
#include "servoCalc.h"
#include "matlabfiles.h"


extern NiFpga_Session myrio_session;

/* definitions */

typedef struct { // define thread resource structure
	NiFpga_IrqContext irqContext; 			// context
	NiFpga_Bool irqThreadRdy; 			// ready flag
} ThreadResource;
struct biquad {
    double b0; double b1; double b2; // numerator
    double a0; double a1; double a2; // denominator
    double x0; double x1; double x2; // input
    double y1; double y2; }; // output
#define BUFLENGTH 3000 // 1 min of data at 50hz
#define ALPHA0 15.9587 // home servo position
#define SATURATE(x,lo,hi) ((x) < (lo) ? (lo) : (x) > (hi) ? (hi) : (x)) // limits a signal to a low or high value


/* global variables */
MyRio_Encoder encC0,encC1; //Declare all the encoder information
static double ZeroAngles[] = { 0, 0, 0, 0, 0, 0 };
//static double DesiredAngles[] = { -90, -90, -90, -90, -90, -90 };
static double alpha[6] = {0, 0, 0, 0, 0, 0};
MyRio_Pwm pwmA0, pwmA1, pwmA2, pwmB0, pwmB1, pwmB2; //Declare all the PWM channels

//Declare the digital input channels
static MyRio_Dio Ch0;	
static MyRio_Dio Ch1;

// servo data
static double alpha0[6] = {ALPHA0, ALPHA0, ALPHA0, ALPHA0, ALPHA0, ALPHA0}; // home servo position, servo arms are at 90deg to legs
double AngleRange = 180;
double MaxCount = 3125;
double MinCount = 625;
double ZeroPosition = 90;
// encoder data
double EncoderCountRev = 8094;
double DegPerRev = 360;

// Matlab arrays
static double gammaArrayX[BUFLENGTH];
static double gammaArrayY[BUFLENGTH];
static double gammaArrayZ[BUFLENGTH];
//static double IMUArrayX[BUFLENGTH];
//static double IMUArrayY[BUFLENGTH];
//static double IMUArrayZ[BUFLENGTH];
static double TArrayX[BUFLENGTH];
static double TArrayY[BUFLENGTH];
static double TArrayZ[BUFLENGTH];
static double PhiArrayX[BUFLENGTH];
static double PhiArrayY[BUFLENGTH];
static double PhiArrayZ[BUFLENGTH];
static double legLengthsArray1[BUFLENGTH];
static double legLengthsArray2[BUFLENGTH];
static double legLengthsArray3[BUFLENGTH];
static double legLengthsArray4[BUFLENGTH];
static double legLengthsArray5[BUFLENGTH];
static double legLengthsArray6[BUFLENGTH];
static double alphaArray1[BUFLENGTH];
static double alphaArray2[BUFLENGTH];
static double alphaArray3[BUFLENGTH];
static double alphaArray4[BUFLENGTH];
static double alphaArray5[BUFLENGTH];
static double alphaArray6[BUFLENGTH];
static double outOfRange[BUFLENGTH];
// pointers for stepping through arrays
static int ArrayCounter = 0;


/* prototypes */
NiFpga_Status conC_Encoder_initialize(NiFpga_Session myrio_session, MyRio_Encoder *encCp, int iE);
double GetPulse(double DesiredAngle);
void MoveServos(double Angles[6], MyRio_Pwm *PWM_Channels);
void *Timer_Irq_Thread(void* resource);
void InitializePWM();
void pos(double PosDeg[3]);
void home(void);
void responding(void);
void error(void);
int GetLegLengths(double P_base[3][6], double l[3][6], double legLengths[6], double T[3], double Phi[3], double B[3][6], double P[3][6], double l_max, double l_min);
double cascade( double xin, // input
                struct biquad *fa, // biquad array
                int ns, // no. segments
                double ymin, // min output
                double ymax ); // max output

/* Define an enumerated type for states */
typedef enum {Home, Responding, Error, Exxit} State_Type;

/* Define a table of pointers to the functions for each state */
static void (*state_table[])(void)={home, responding, error};

//Has to go after the enum type declaration to declare current state holder variable
static State_Type curr_state = Home;


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

	//Register the Timer IRQ
	status = Irq_RegisterTimerIrq(&irqTimer0, &irqThread0.irqContext,
			timeoutValue);

	//Set the indicator to allow the new thread
	irqThread0.irqThreadRdy = NiFpga_True;

	//Create new thread to catch the IRQ
	status = pthread_create(&thread,
	NULL, Timer_Irq_Thread, &irqThread0);

	//Other Main Tasks below----------------------------------------------------------------------------------------------------------------------------------
	while (curr_state != Exxit) {

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

void *Timer_Irq_Thread(void* resource) {

	// Cast the Input argument back to its intended form---------------------------------------------------------------------------------------------------------
	ThreadResource* threadResource = (ThreadResource*) resource;
	NiFpga_Bool Status;

	//Initialize all Encoders, Servos, State Machines and parameters needed--------------------------------------------------------------------------------
	//Initialize PWM channels and there parameters
	InitializePWM();

	//Initialize Encoders
	Status = conC_Encoder_initialize(myrio_session, &encC0, 0);
	Status = conC_Encoder_initialize(myrio_session, &encC1, 1);

	//Initialize the digital input/output channels
	Ch0.dir = DIOA_70DIR;
	Ch0.out = DIOA_70OUT;
	Ch0.in = DIOA_70IN;
	Ch0.bit = 0;

	Ch1.dir = DIOA_70DIR;
	Ch1.out = DIOA_70OUT;
	Ch1.in = DIOA_70IN;
	Ch1.bit = 1;


	//Declare all variables and constants for the interrupt-----------------------
//	MyRio_Pwm PWM_Channels[] = { pwmA0, pwmA1, pwmA2, pwmB0, pwmB1, pwmB2 };

	// Platform dimensions
//	const double radius = 5.0; // inches
//	const double angleBetweenServos = 60; // degrees
//	const double servoRotationOffset = 120; // degrees
	const double s = 9.0;
	const double a = 1.85;
	const double l_max = s+a; // maximum extension of leg
	const double l_min = s-a; // minimum extension of leg
	const double h0 = 8.9943; // home height
	double beta[6] = {270.0, 210.0, 30.0, 330.0, 150.0, 450.0};
	double B[3][6] = {{4.3301, 0.0, -4.3301, -4.3301, 0.0, 4.3301},
							{2.5, 5.0, 2.50, -2.50, -5.0, -2.50},
							{0, 0, 0, 0, 0, 0}}; // base dimensions in base reference frame
	double P[3][6] = {{2.5680, 1.4422, -4.0102, -4.0102, 1.4422, 2.5680},
							{3.1479, 3.7979, 0.6500, -0.6500, -3.7979, -3.1479},
							{0, 0, 0, 0, 0, 0}}; // platform dimensions in platform reference frame
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
	double Gamma[3] = {0, 0, 0}; // pitch and roll of platform
	double GammaFiltered[3];
	double l[3][6];
	
	// Platform transition logic variables
	int PreviousOffButton = 0;
	int ErrorFlag = 0;
	NiFpga_Bool OnButton = NiFpga_False;
	NiFpga_Bool OffButton = NiFpga_False;

	int i; // for loop counter
	double timeCounter = 0;

	// low pass filter
	int lowPass_ns = 1; // No. of biquad sections
	double tau = 0.05; // time constant, s
	double TS = 0.02; // sample time, s
	double alphaTF = TS / (2*tau + TS);
    static struct biquad lowPass[3];
	for (i = 0; i < 3; i++) {
		(lowPass+i)->a0 = 1.000;
		(lowPass+i)->a1 = (2*alphaTF - 1);
		(lowPass+i)->a2 = 0.000;
		(lowPass+i)->b0 = alphaTF;
		(lowPass+i)->b1 = alphaTF;
		(lowPass+i)->b2 = 0.000;
		(lowPass+i)->x0 = 0.000;
		(lowPass+i)->x1 = 0.000;
		(lowPass+i)->x2 = 0.000;
		(lowPass+i)->y1 = 0.000;
		(lowPass+i)->y2 = 0.000;
	}

	// error correction variables
	// box with spacer, box, two bars with spacer, two bars, 1 bar with spacer x rotation
	// box with thick spacer, box with thin spacer, box, two bars and both spacers, two bars and thick spacer y rotation
	double errorAngles[2][13] = {{-16.9014, -14.3217, -11.2528, -8.8065, -6.8050, 0, 0, 0, 6.8050, 8.8065, 11.2528, 14.3217, 16.9014},
								{-19.9258, -14.6775, -11.9199, -8.3617,-6.0934, 0, 0, 0, 6.0934, 8.3617, 11.9199, 14.6775, 19.9258}};
	double errorVector[2][13] = {{-2.3, -2.1, -1.5, -1.4, -1.1, 0, 0, 0, 1.5, 1.9, 2.3, 2.3, 2.4},
								{-2.7, -2.7, -2.6, -2.1, -1.7, 0, 0, 0, 1.1, 1.6, 2.2, 2.3, 2.5}};
	int errorLength = 12;
	double correctionAngle[3] = {0, 0, 0};
	double correction1, correction2, angle1, angle2;
	int index1, index2;





	/* The While loop below will perform two tasks while waiting for a signal to stop---------------------------------------------------------------------
	 *  - It will wait for the occurrence( or timeout) of the IRQ
	 *  		- If it has, "Schedule" the next interrupt
	 *  - if the Timer IRQ has asserted
	 *  		- Perform operations to service the interrupt
	 *  		- Acknowledge the interrupt
	 */

	while (threadResource->irqThreadRdy == NiFpga_True) {
		//Reset the on and off button boolean values
		OnButton = 0;
		OffButton = 0;
		ErrorFlag = 0;
		uint32_t timeoutValue = 20000;
		uint32_t irqAssert = 0;
		Irq_Wait(threadResource->irqContext,
		TIMERIRQNO, &irqAssert, (NiFpga_Bool*) &(threadResource->irqThreadRdy));

		extern NiFpga_Session myrio_session;

		if (irqAssert) {
			//Your Interrupt Service Code here-------------------------------------------------------------------------------------------------------------------



			timeCounter = timeCounter + TS;

			NiFpga_WriteU32(myrio_session, IRQTIMERWRITE, timeoutValue);
			NiFpga_WriteBool(myrio_session, IRQTIMERSETTIME, NiFpga_True);

			//Use pos() to get the position of each encoder relative to the starting position
			pos(Gamma);

			// Run a low pass filter on the encoder readings
			for (i=0;i<3;i++){
				GammaFiltered[i] = cascade(Gamma[i],
											(lowPass+i),
											lowPass_ns,
											-180,
											180);
			}

			// Use error lookup table to add correction
			/* commented out until new data is collected
			for (i=0;i<2;i++) { // interpolate correction angle
				if (Gamma[i] <= errorAngles[i][0] ){
					correctionAngle[i] = -errorVector[i][0];
				} else if (Gamma[i] >= errorAngles[i][errorLength]){
					correctionAngle[i] = -errorVector[i][errorLength];
				} else {
					index1 = 0;
					index2 = 1;
					// calculate what 2 values it is between
					while (!(Gamma[i] <= errorAngles[i][index2] && Gamma[i] >= errorAngles[i][index1])) {
						index1++;
						index2++;
					}
					correction1 = errorVector[i][index1];
					correction2 = errorVector[i][index2];
					angle1 = errorAngles[i][index1];
					angle2 = errorAngles[i][index2];
					correctionAngle[i] = ( correction1 + (Gamma[i] - angle1)*(correction2-correction1)/(angle2-angle1) );
					Phi_desired[i] = -correctionAngle[i];
				}
				}*/

			// run initial movement routine
						if (timeCounter < 1000){
							if (timeCounter < 5) {
								T_desired[0] = sin(timeCounter*3)*1.5;
								T_desired[1] = cos(timeCounter*3)*1.5;
							} else if (timeCounter > 5.5 && timeCounter < 10) {
								Phi_desired[2] = sin(timeCounter*2)*25;
							} else if (timeCounter > 10.5 && timeCounter < 15) {
								T_desired[2] = sin(timeCounter)*1.5;
							} else if (timeCounter > 15.5 && timeCounter < 1000) {
								Phi_desired[0] = sin(timeCounter*4)*10;
								Phi_desired[1] = cos(timeCounter*4)*10;
							} else {
								for (i=0;i<3;i++){
									T_desired[i] = 0;
									Phi_desired[i] = 0;
								}
							}
						}


			// Convert disturbance to correction translation and angles
			baseToStaticPlatformPosition(T,Phi,D,GammaFiltered,T_desired,Phi_desired,h0);

			// Calculate required leg lengths
			ErrorFlag = GetLegLengths(P_base, l, legLengths, T, Phi, B, P, l_max, l_min);

			//Take the Base position and calculate the required servo correction
				//If the correction is outside the servos range, set the error flag to 1 and servo flag to 0
				//If not continue
			ErrorFlag = servoCalc(alpha, P_base, B, legLengths, s, a, beta, alphaMax, alphaMin);

			//Set the appropriate state
			if(Dio_ReadBit(&Ch0) == 0) {
				OnButton = 1;
			}

			if(Dio_ReadBit(&Ch1) == 0) {
				OffButton = 1;
			} 

			//Check for Home State
			if(curr_state == Home) {
				if(OnButton == 1 && ErrorFlag == 0 && OffButton == 0) {
					curr_state = Responding;
					timeCounter = 0;
				} else if(ErrorFlag == 1 && OffButton == 0) {
					curr_state = Error;
				} else if(OffButton == 1 && PreviousOffButton != 1) {
					//curr_state = Home;
					curr_state = Exxit;
				}
			}

			//Check for Responding State
			if(curr_state == Responding) {
				if (OnButton == 1 && timeCounter >= 1) {
					timeCounter = 1000;
					for (i=0;i<3;i++){
														T_desired[i] = 0;
														Phi_desired[i] = 0;
					}
				}
				if(ErrorFlag == 1 && OffButton == 0) {
					curr_state = Error;
				} else if(OffButton == 1) {
					curr_state = Home;
				}
			}

			//Check for Error State
			if(curr_state == Error) {
				if(OnButton == 1 && ErrorFlag == 0 && OffButton == 0) {
					curr_state = Home;
				} else if(ErrorFlag == 1 && OffButton == 0) {
					curr_state = Error;
				} else if(OffButton == 1) {
					curr_state = Exxit;
				}
			}

			//run the current state
			if(curr_state != Exxit) {
				state_table[curr_state]();
			}

			if (ArrayCounter < BUFLENGTH) {
				gammaArrayX[ArrayCounter] = Gamma[0];
				gammaArrayY[ArrayCounter] = Gamma[1];
				gammaArrayZ[ArrayCounter] = Gamma[2];
				TArrayX[ArrayCounter] = T[0];
				TArrayY[ArrayCounter] = T[1];
				TArrayZ[ArrayCounter] = T[2];
				PhiArrayX[ArrayCounter] = Phi[0];
				PhiArrayY[ArrayCounter] = Phi[1];
				PhiArrayZ[ArrayCounter] = Phi[2];
				legLengthsArray1[ArrayCounter] = legLengths[0];
				legLengthsArray2[ArrayCounter] = legLengths[1];
				legLengthsArray3[ArrayCounter] = legLengths[2];
				legLengthsArray4[ArrayCounter] = legLengths[3];
				legLengthsArray5[ArrayCounter] = legLengths[4];
				legLengthsArray6[ArrayCounter] = legLengths[5];
				alphaArray1[ArrayCounter] = alpha[0];
				alphaArray2[ArrayCounter] = alpha[1];
				alphaArray3[ArrayCounter] = alpha[2];
				alphaArray4[ArrayCounter] = alpha[3];
				alphaArray5[ArrayCounter] = alpha[4];
				alphaArray6[ArrayCounter] = alpha[5];

				if (ErrorFlag == 1) {
					outOfRange[ArrayCounter] = 1;
				} else {
					outOfRange[ArrayCounter] = 0;
				}

				PreviousOffButton = OffButton;
				ArrayCounter++;
			}


			//Acknowledge the interrupt-----------------------------------------------------------------------------------------------------------------------------
			Irq_Acknowledge(irqAssert);
		}
	}

	int err; // indicates matlab file save success
	MATFILE* mf; // matlab file
	mf=openmatfile("StewartData.mat", &err);
    if(!mf) printf("Can't open matfile%d\n",err);
    else printf("MATLAB file opened successfully.\n");
    matfile_addstring(mf,"day", "05152024");
    matfile_addmatrix(mf, "gammaX", gammaArrayX, BUFLENGTH, 1, 0);
    matfile_addmatrix(mf, "gammaY", gammaArrayY, BUFLENGTH, 1, 0);
    matfile_addmatrix(mf, "gammaZ", gammaArrayZ, BUFLENGTH, 1, 0);
    matfile_addmatrix(mf, "TX", TArrayX, BUFLENGTH, 1, 0);
    matfile_addmatrix(mf, "TY", TArrayY, BUFLENGTH, 1, 0);
    matfile_addmatrix(mf, "TZ", TArrayZ, BUFLENGTH, 1, 0);
    matfile_addmatrix(mf,"PhiX", PhiArrayX, BUFLENGTH, 1, 0);
    matfile_addmatrix(mf,"PhiY", PhiArrayY, BUFLENGTH, 1, 0);
    matfile_addmatrix(mf,"PhiZ", PhiArrayZ, BUFLENGTH, 1, 0);
	matfile_addmatrix(mf,"legLengths1", legLengthsArray1, BUFLENGTH, 1, 0);
	matfile_addmatrix(mf,"legLengths2", legLengthsArray2, BUFLENGTH, 1, 0);
	matfile_addmatrix(mf,"legLengths3", legLengthsArray3, BUFLENGTH, 1, 0);
	matfile_addmatrix(mf,"legLengths4", legLengthsArray4, BUFLENGTH, 1, 0);
	matfile_addmatrix(mf,"legLengths5", legLengthsArray5, BUFLENGTH, 1, 0);
	matfile_addmatrix(mf,"legLengths6", legLengthsArray6, BUFLENGTH, 1, 0);
	matfile_addmatrix(mf,"alpha1", alphaArray1, BUFLENGTH, 1, 0);
	matfile_addmatrix(mf,"alpha2", alphaArray2, BUFLENGTH, 1, 0);
	matfile_addmatrix(mf,"alpha3", alphaArray3, BUFLENGTH, 1, 0);
	matfile_addmatrix(mf,"alpha4", alphaArray4, BUFLENGTH, 1, 0);
	matfile_addmatrix(mf,"alpha5", alphaArray5, BUFLENGTH, 1, 0);
	matfile_addmatrix(mf,"alpha6", alphaArray6, BUFLENGTH, 1, 0);
	matfile_addmatrix(mf,"outOfRange", outOfRange, BUFLENGTH, 1, 0);
    matfile_close(mf);

	//Terminate the new thread and return the function
	pthread_exit(NULL);
	return NULL;
}

int GetLegLengths(double P_base[3][6], double l[3][6], double legLengths[6], double T[3], double Phi[3], double B[3][6], double P[3][6], double l_max, double l_min) {
	double R[3][3];
	double P_i[3];
	double dummyVector1[3];
	double dummyVector2[3];
	int i, j;
	int errorFlag = 0;
	
	// calculate rotation matrix
	rotZYX(Phi, R);

	// find vector of each leg, l = (T + R*P) - B
	for (i = 0; i<6; i++) {
		for (j = 0; j<3; j++) {
			P_i[j] = P[j][i];
		}
		matrixMultiply33x31(R,P_i, dummyVector1);
		for (j=0; j<3; j++) {
			dummyVector2[j] = T[j] + dummyVector1[j];
			l[j][i] = dummyVector2[j] - B[j][i];
		}
		
	}
	
	// find length of each leg vector
	for (i = 0; i < 6; i++) {
		legLengths[i] = sqrt(l[0][i]*l[0][i] + l[1][i]*l[1][i] + l[2][i]*l[2][i]);
		// check if leg lengths are too long or short
		if (legLengths[i] < l_min || legLengths[i] > l_max) {
			errorFlag = 1;
		}
		for (j = 0; j < 3; j++){
			P_base[j][i] = l[j][i] + B[j][i]; // find platform positions 
		}
	}

	return errorFlag;
}



void home(void) {
	//Setup PWM channels array
	MyRio_Pwm PWM_Channels[] = { pwmA0, pwmA1, pwmA2, pwmB0, pwmB1, pwmB2 };

	// Send the servos to the home position
	MoveServos(alpha0, PWM_Channels);

}

void responding(void) {
	//Setup PWM channels array
	MyRio_Pwm PWM_Channels[] = { pwmA0, pwmA1, pwmA2, pwmB0, pwmB1, pwmB2 };

	// Send the servos to the home position
	MoveServos(alpha, PWM_Channels);

}

void error(void) {

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
	PosDeg[0] = -(EncoderPosBDI[0] / EncoderCountRev) * DegPerRev;
	PosDeg[1] = (EncoderPosBDI[1] / EncoderCountRev) * DegPerRev;

}


/* Converting the input angle to output angle.
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


	GoAngle = DesiredAngle + ZeroPosition;

	AnglePercent = GoAngle / AngleRange;

	PulseCount = ((MaxCount - MinCount) * AnglePercent) + MinCount;

	return PulseCount;
}

void MoveServos(double Angles[6], MyRio_Pwm *PWM_Channels) {

	double anglesCorrected[6];
	int i;

	// Account for servo arm offset
	for (i = 0; i < 6; i++) {
		anglesCorrected[i] = Angles[i] - alpha0[i];
	}

	//Create Pointers to the Individual PWM Structures that we fed in
	MyRio_Pwm *PWM1 = PWM_Channels;
	MyRio_Pwm *PWM2 = PWM_Channels + 1;
	MyRio_Pwm *PWM3 = PWM_Channels + 2;
	MyRio_Pwm *PWM4 = PWM_Channels + 3;
	MyRio_Pwm *PWM5 = PWM_Channels + 4;
	MyRio_Pwm *PWM6 = PWM_Channels + 5;

	//Change the PWM Counter compare value to the needed one for the given desired angle for that specific signal
	Pwm_CounterCompare(PWM1, GetPulse(-anglesCorrected[0]));
	Pwm_CounterCompare(PWM2, GetPulse(anglesCorrected[1]));
	Pwm_CounterCompare(PWM3, GetPulse(-anglesCorrected[2]));
	Pwm_CounterCompare(PWM4, GetPulse(anglesCorrected[3]));
	Pwm_CounterCompare(PWM5, GetPulse(-anglesCorrected[4]));
	Pwm_CounterCompare(PWM6, GetPulse(anglesCorrected[5]));

}

double cascade(
    double xin, // input
    struct biquad *fa, // biquad array
    int ns, // no. segments
    double ymin, // min output
    double ymax) // max output
    {
        int i; // loop counter
        struct biquad* f = fa; // biquad pointer
        double y0 = xin;

        for (i=0;i<ns;i++) {
            f->x0 = y0; // pass input to current biquad
            y0 = (1/f->a0)*(f->b0*f->x0 + f->b1*f->x1 + f->b2*f->x2 - f->a1*f->y1 - f->a2*f->y2);
            f->x2 = f->x1; // update biquad values
            f->x1 = f->x0;
            f->y2 = f->y1;
            f->y1 = y0;
            f++;         // go to next biquad
        }
        y0 = SATURATE(y0,ymin,ymax);
        return y0;
    }
