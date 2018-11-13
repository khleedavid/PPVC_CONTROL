/*
Name:       PPVC_ControlV10.ino
Created:  7/27/2018 12:36:09 AM
Author:     David Lee 
Built for Arduino DUE
*/
/*
This software may be distributed and modified under the terms of the GNU
General Public License version 2 (GPL2) as published by the Free Software
Foundation and appearing in the file GPL2.TXT included in the packaging of
this file. Please note that GPL2 Section 2[b] requires that all works based
on this software must also be made publicly available under the terms of
the GPL2 ("Copyleft").
*/

// ================================================================
// ===               DEPENDENT LIBRARIES              ===
// ================================================================

#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter
#include <MPU6050_6Axis_MotionApps20.h>
#include <I2Cdev.h>
#define ENCODER_OPTIMIZE_INTERRUPT
#include <Encoder.h>
#include <PID_v1.h>
#include <DueTimer.h>




//DEFINE THESE PARAMETERS BEFORE BEGINNING TESTING
//motor FIT0520 datasheet https://www.dfrobot.com/product-1619.html
//gyroscope MPU6050 datasheet https://components101.com/sensors/mpu6050-module
///////////////////////////////////   UNIT-FRAME CONFIGURATION (IN mm)  /////////////////////////////
#define drum_diameter 30  //in mm
#define total_counts (11*34.02*50*3.5) //enccounts per rev (11)* gearmotor gearbox ratio (34:1) * wormgear ratio (50:1) * experimental multiplier(exp_mult)
#define deg_encount (360/total_counts)//degrees per count of encoder
// (for ~44000 count/rev (exp_mult=4): 0.0080213903743315508025),  (for ~11000 count/rev (exp_mult=1): 0.03208556149732620321), (for ~2805 count/rev (exp_mult=0.25): 0.12834224598930481284)
#define rev_dist (M_PI*drum_diameter/360)*deg_encount //distance traveled per count
#define xLen  600 //x plane length
#define yLen  250 //y plane length

///////////////////////////////////   MPU CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the sketch to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=10;     //Accelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=2;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
///////////////////////////////////   PID CONFIGURATION   /////////////////////////////
//Define the aggressive and conservative Tuning Parameters
double aggKp=0.02, aggKi=0.0, aggKd=2.0;
double consKp=0.02, consKi=0.00, consKd=2.0;//Specify the links and initial tuning parameters
///////////////////////////////////   LOADCELL CONFIGURATION   /////////////////////////////
const float stopLimit = 5.0; //load bearing limit for each load cell
// ================================================================
// ===             MPU  VARIABLES                ===
// ================================================================
//PPVC function
int ppvcState = 0;
/* IMU Data */
// Variables to store the values from the sensor readings
#define RESTRICT_PITCH // Comment out to restrict roll to ?90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
//mpu config vars
MPU6050 mpu;
bool mpu_isinit = false;
bool mpu_iscalibrated = false;
//Kalman filter measurement variables
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
volatile double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double roll, pitch;
//calibration variables are on Offboard

// ================================================================
// ===               LOADCELL VARIABLES              ===
// ================================================================
//main variables are on off board
volatile float LoadRead[4] = {0,0,0,0}; //Stores latest Load Value
// ================================================================
// ===             i2c  VARIABLES                ===
// ================================================================
uint32_t globalTimer = 0;
uint8_t i2cData[14]; // Buffer for I2C data 
const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication
// ================================================================
// ===             TIMER VARIABLES                ===
// ================================================================
unsigned long currentMicros = 0;
uint32_t blinkTimer, waitTimer = 0;
const unsigned long blink_period = 100000;  //blink timer
bool blinkState = false;
// ================================================================
// ===             BUTTON  VARIABLES                ===
// ================================================================
int button[5] = {30, 31, 32, 33, 34}; //declare pins 30-34 for button control. PIN34: ESTOP
// ================================================================
// ===             PID  VARIABLES                ===
// ================================================================
int measureState = 0;
double Setpoint[4], Input[4], Output[4];
PID motorPID[4] = {
	PID(&Input[0], &Output[0], &Setpoint[0], consKp, consKi, consKd,P_ON_E, DIRECT),
	PID(&Input[1], &Output[1], &Setpoint[1], consKp, consKi, consKd,P_ON_E, DIRECT),
	PID(&Input[2], &Output[2], &Setpoint[2], consKp, consKi, consKd,P_ON_E, DIRECT),
PID(&Input[3], &Output[3], &Setpoint[3], consKp, consKi, consKd,P_ON_E, DIRECT)};
// ================================================================
// ===             ENCODER  VARIABLES                ===
// ================================================================
Encoder Enc[4] = {	Encoder(36,37),	Encoder(43,40),	Encoder(42,41),Encoder(39,38)};
//					0,				1,				2,				3,
volatile long oldPos[4]  = {-999,-999,-999,-999};
volatile long newPos[4] = {0,0,0,0};
volatile long savePos[4]={0,0,0,0};
// ================================================================
// ===             MOTOR  VARIABLES                ===
// ================================================================
float testDist = 255;
//              0,  1,  2,  3,
int motor[8] = {22, 28, 26, 24, 
				23, 29, 27, 25}; //declare pins 22-29 for motor control. 2 pins >> 1 motor
//              4,  5,  6,  7
int pwm[4] = {4, 7, 6, 5};  //declare pins 4-7 for motor PWM control.
// ================================================================
// ===             SERIAL INTERFACE VARIABLES                ===
// ================================================================
const byte buffSize = 32; //input buffer size
const char startMarker = '<'; //command start mark
const char endMarker = '>'; //command end mark

const byte msgSize = 32; //message buffer size
const char msgStartMarker = '{'; //message start mark
const char msgEndMarker = '}'; //message end mark

char inputBuffer[buffSize]; //command input buffer
volatile byte bytesRecvd = 0; //bytes count received by cmd buffer
volatile boolean readInProgress = false; //cmd read
volatile boolean newDataFromPC = false; //new data from pc flag

char msgBuffer[msgSize]; //message input buffer
volatile byte msgbytesRecvd = 0; //bytes count received by cmd buffer
volatile boolean msgreadInProgress = false; //msg read
volatile boolean newMsgFromOB = false; //new msg from OB flag

char OBinputBuffer[buffSize]; //ob command buffer size
volatile byte OBbytesRecvd = 0; //ob bytes count received by ob input buffer
volatile boolean OBreadInProgress = false; //ob cmd read
volatile boolean newDataFromOB = false; //new data from ob flag

//variables to store parsed data from PC
char messageFromPC[buffSize] = {0};
volatile int newMotorSel = 0;
volatile int newTurnDist = 0; 

//variables to store parsed data from OB
char messageFromOB[buffSize] = {0};
volatile double OBSel = 0;
volatile double OBDist = 0; 
volatile boolean okFlag = false;	//acknowledge OK

// ================================================================
// ===             FUNCTION PROTOTYPES			                ===
// ================================================================
static void MPUready();
static void MOTOR_init();
static void MOTOR_reset();
static void MOTOR_loop();
static void MOTOR_off();
static void MOTOR_off(int motorSel);
static void MOTOR_turn(int motorSel, int turndist);
static int ENCODER_update(int encSel);
static void ENCODER_print();
static void ENCODER_save(int encoderSaveCount);
static void PID_init();
static void PID_end();
static void PID_on();
static void button_init();
static void buttonCheck();
static void wait(unsigned long wait_time);
static void blinkLED();
static void MPU_calibrate();
static void timer_init();
static void SENSORS_init();
static void MPU_update();
static void MPU_turnCalc(double x, double y, int* dx, int* dy);
static void MPU_settlewait(double pastKalAngleX, double pastKalAngleY);
static void MPU_print();
static void LOADCELL_update();
static void LOADCELL_print();
static void	LOADCELL_lift(float threshload);
static void LOADCELL_warning(float threshload);
static int checkHighest(double rollX, double pitchY);
static void doMeasureLevel(int SetpointManual);
static void PPVC();
static void getDataFromPC();
static void getDataFromOB();
static void parseData();
static void OBparseData();
static void updateSerialESTOP();
static void updateSerialOB();
static void updateSerialOut();
static void printHelp();

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool stopInterrupt = false;     // indicates whether STOP interrupt pin has gone high
static void MPUready(){
	getDataFromOB(); //wait for okFlag acknowledgment from off-board
	getDataFromPC();	//check serial2 port
	updateSerialESTOP();
	if (digitalRead(button[4]) == LOW)
	{stopInterrupt = true;  }
}

// ================================================================
// ===            MOTOR_FUNCTIONS                       ===
// ================================================================
static void MOTOR_init() {  //set motors as outputs
	for (int i = 0; i < 8; i++) { //init motor pins as output
		pinMode(motor[i], OUTPUT);
	}
	for (int i = 0; i < 4; i++) { //init pwm pins as output
		pinMode(pwm[i], OUTPUT);
	}
	button_init(); // turn on buttons
	Serial2.println(F("{(2 of 3)Motors Initialised}"));
}
static void MOTOR_reset(){  //unwind motors for 5 seconds
	for (int i = 0; i<4; i++)	{	//Set motors to turn CCW
		MOTOR_turn(i,-testDist);
	}
	wait(5000000);	//Wait 5 seconds
	MOTOR_off();	//Turn off motors
}
static void MOTOR_loop() { //loops through all 4 motors CW and CCW, checking motors and encoders are responding
	for (int i = 0; i < 4; i++) {
		ENCODER_update(i);	//update encoder values
		digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
		MOTOR_turn(i, testDist);  //turn CW
		wait(1000000);                       // wait for a second
		MOTOR_off();
		ENCODER_print();	//print new encoder values
		digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
		MOTOR_turn(i, -testDist); //turn CCW
		wait(1000000);                       // wait for a second
		MOTOR_off();
		ENCODER_print();	//print new encoder values
		wait(10000);
	}
}
static void MOTOR_off() { //turns off all motors
	for (int i = 0; i < 4; i ++) {
		digitalWrite(motor[i], LOW);
		digitalWrite(motor[i+4], LOW);
		analogWrite(pwm[i], 0);
	}
}
static void MOTOR_off(int motorSel) { //turns off selected motors
	digitalWrite(motor[motorSel], LOW);
	digitalWrite(motor[motorSel+4], LOW);
	analogWrite(pwm[motorSel], 0);
}
static void MOTOR_turn(int motorSel, int turndist) { //turns selected motors clockwise
	//use turndist to set motor PWM and direction
	int i = motorSel;	//select motors 1-4
	if(i == 0 || i == 3){
		if(turndist >=0){ //turn CW
			digitalWrite(motor[i], HIGH);
			digitalWrite(motor[i + 4], LOW);
			analogWrite(pwm[i],turndist);
		}
		else{ //turn CCW
			digitalWrite(motor[i], LOW);
			digitalWrite(motor[i + 4], HIGH);
			analogWrite(pwm[i],abs(turndist));
		}
	}
	else if(i == 1 || i == 2){
		if(turndist >=0){ //turn CW
			digitalWrite(motor[i], LOW);
			digitalWrite(motor[i + 4], HIGH);
			analogWrite(pwm[i],turndist);
		}
		else{ //turn CCW
			digitalWrite(motor[i], HIGH);
			digitalWrite(motor[i + 4], LOW);
			analogWrite(pwm[i],abs(turndist));
		}
	}
}
// ================================================================
// ===					 ENCODER_FUNCTIONS                     ===
// ================================================================
static int ENCODER_update(int encSel){	//update encoder readings
	newPos[encSel] = Enc[encSel].read();
	if (newPos[encSel] != oldPos[encSel]) {		oldPos[encSel] = newPos[encSel];}
	return oldPos[encSel];
}
static void ENCODER_print(){	//print encoder readings
	Serial2.println(F("{Current Encoder 1...2...3...4"));
	Serial2.print(ENCODER_update(0)); Serial2.print("\t");
	Serial2.print(ENCODER_update(1)); Serial2.print("\t");
	Serial2.print(ENCODER_update(2)); Serial2.print("\t");
	Serial2.print(ENCODER_update(3)); Serial2.print("}");
}
static void ENCODER_save(int encoderSaveCount){	 //save and restore encoder counts
	if(encoderSaveCount == 0){	
//take current encoder status and save it so the unit can return to a saved state later
		for (int i = 0; i<4; i++){
			savePos[i] = ENCODER_update(i);
		}
		return;
	}
	if (encoderSaveCount == 1){
	//return setpoint to saved state
		for (int i = 0; i<4; i++){
			Setpoint[i] = savePos[i];
		}
		return;
	}
}
// ================================================================
// ===				     PID_FUNCTIONS                       ===
// ================================================================
static void PID_init(){ //Initialise PID settings
	for (int i=0; i<4; i++)
	{
		motorPID[i].SetMode(MANUAL);
		motorPID[i].SetSampleTime(10);
		motorPID[i].SetOutputLimits(-255,0,255);
	}
	Serial2.println(F("{(3 of 3)PID initialised}"));
}
static void PID_end(){ //set PID to MANUAL mode, stop calculating
	for (int i=0; i<4; i++)
	{motorPID[i].SetMode(MANUAL);
		MOTOR_off(i);
	}
}
static void PID_on(){ //set PID to AUTOMATIC mode, start calculating
	for (int i = 0; i<4; i++) {
		motorPID[i].SetMode(AUTOMATIC);
		motorPID[i].SetTunings(consKp, consKi, consKd);
	}
}
// ================================================================
// ===					  BUTTON_CONTROLS                       ===
// ================================================================
static void button_init(){
	pinMode(LED_BUILTIN, OUTPUT);// initialize LED_BUILTIN as an output.
	pinMode(button[4], INPUT_PULLUP); //init button4 for interrupt stop
	for (int i = 0; i < 4; i++) { //init button pins as input, pwm pins as output
		pinMode(button[i], INPUT_PULLUP);
	}
}
static void buttonCheck() { //Polled function for hardware buttons
	if (digitalRead(button[0]) == LOW) { //button0 prints current sensor readings
		MPU_print();
		ENCODER_print();
		LOADCELL_print();
	}
	if (digitalRead(button[1]) == LOW) {//button1 performs calibration of MPU6050
		MPU_calibrate();
	}
	if (digitalRead(button[2]) == LOW) { //button2 performs balancing process
		doMeasureLevel(0);
	}
	if (digitalRead(button[3]) == LOW) {//button3 motor test loop
		MOTOR_loop();
	}
	//button[4] is used as an emergency stop interrupt
}
// ================================================================
// ===                CALIBRATION_ROUTINE                       ===
// ================================================================
static void MPU_calibrate(){ //calls offboard to run calibration of gyro
	int MPUCalibrateMode = 0;	//Set Calibration Stage to 0
	unsigned long timeout_period = 1000000;	//set timeout period for serial port messaging
	
	digitalWrite(LED_BUILTIN,HIGH);
	Serial2.println("{MPU6050 Calibration Sketch}"); //notify user of calibration progress
	wait(20000);
	Serial2.println(F("{Your MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.}"));
	wait(30000);	//ensure user places the gyroscope in a level position on the PPVC unit for proper calibration
	Serial1.print(F("<C>")); //send initial calibration command
	do 
	{
		if (MPUCalibrateMode == 0 && stopInterrupt != true) //send the calibrate command to offboard and wait for acknowledgement
		{
			currentMicros = micros();  //get the current "time" (actually the number of microseconds since the program started)
			if (currentMicros - blinkTimer >= timeout_period) { //test whether the timeout period has elapsed without offboard response
				blinkTimer = currentMicros;  //IMPORTANT to save the start time of the current LED state.
				Serial1.print(F("<C>"));//resend calibrate command if timeout is reached
			}
			getDataFromOB(); //wait for acknowledgment from off-board
			updateSerialOB();
			if (okFlag == true){ //if the acknowledgment flag is set, then increment calibration stage
				okFlag = false;	//reset acknowledgment flag for next message
				MPUCalibrateMode++;
			}
		}
		if (MPUCalibrateMode == 1 && stopInterrupt != true){ //wait for calibration to finish, in the mean time print progress messages sent from the offboard
			getDataFromOB(); //wait for CALIFIN reply from off-board
			updateSerialOB();
			if(mpu_iscalibrated == true){
				//if mpu calibrated flag is set, then increment calibration stage
				MPUCalibrateMode++;
			}
		}
		getDataFromPC();
		updateSerialESTOP();
	} while (MPUCalibrateMode < 2 && stopInterrupt != true);
		
	if (stopInterrupt == true){
		//if emergency stop button is pressed, the stopInterrupt flag will force the loop to exit, then an emergency stop message is sent to the off-board and waits for an acknowledging response
			currentMicros = micros();  //get the current "time" (actually the number of microseconds since the program started)
			if (currentMicros - blinkTimer >= timeout_period) { //test whether the timeout period has elapsed without offboard response
				blinkTimer = currentMicros;  //IMPORTANT to save the start time of the current state.
				Serial1.print(F("<E>"));	//resend emergency stop command
			}
			getDataFromOB(); //wait for acknowledge reply <R> from off-board
			updateSerialOB();	//main board sends "EMERGENCY STOP" message to user, and sets the acknowledgment flag
			if (okFlag == true){ //if the acknowledge flag is set, then end the process
				okFlag = false;	//reset acknowledgment flag for next message
				stopInterrupt = false;	//reset emergency stop only after acknowledgment
			}
	}
	Serial2.println(mpu_iscalibrated ? "{MPU Calibrated!}": "{Calibration failed. Something went wrong}"); //check if calibration was successful
	ENCODER_save(0);//Save current encoder readings
	ENCODER_print();
	mpu_iscalibrated = false; //reset flag for next calibration
}
// ================================================================
// ===                TIMER_PROTOCOLS                          ===
// ================================================================
static void timer_init(){        // initialize timer1, and set a 1/2 second period
	Serial1.begin(115200);	//serial port for OFFBOARD
	Serial2.begin(115200);	//serial port for PC CONTROL
	Timer1.attachInterrupt(MPUready).start(50000);  // attaches callback() as a timer overflow interrupt
	globalTimer = micros();
	blinkTimer = micros();
	waitTimer = micros();
}
static void wait(unsigned long wait_time){ //nonblocking wait function that sets a timer in microseconds
	waitTimer = currentMicros;  //reset waitTimer
	currentMicros = micros(); //get current time
	while (currentMicros - waitTimer <= wait_time) { //check if wait_time has elapsed
		currentMicros = micros(); //update time while waiting
	}
}
static void blinkLED() {  //blink LED to indicate activity
	currentMicros = micros();  //get the current "time" (actually the number of microseconds since the program started)
	if (currentMicros - blinkTimer >= blink_period) { //test whether the period has elapsed
		blinkState = !blinkState;
		digitalWrite(LED_BUILTIN, blinkState);
		blinkTimer = currentMicros;  //IMPORTANT to save the start time of the current LED state.
	}
}

// ================================================================
// ===                MPU_PROTOCOLS                          ===
// ================================================================
static void SENSORS_init()	//calls offboard to check if sensors have been initialised
{
	unsigned long timeout_period = 10000;	//set timeout period for serial port messaging

	Serial1.print("<AT>");	//continue requesting mpu init check
	while (mpu_isinit != true) //send the command to offboard and wait for acknowledgement
	{
		currentMicros = micros();  //get the current "time" (actually the number of microseconds since the program started)
		if (currentMicros - blinkTimer >= timeout_period) { //test whether the timeout period has elapsed without offboard response
			blinkTimer = currentMicros;  //IMPORTANT to save the start time of the current LED state.
			Serial1.print(F("<AT>"));//resend command if timeout is reached
		}
		getDataFromOB(); //wait for acknowledgment from off-board
		updateSerialOB();
	}
	Serial2.println(mpu_isinit ? "{(1 of 3) MPU initialized": "MPU NOT INITIALIZED, CHECK YOUR CONNECTIONS}");
	Serial1.print(F("<P>"));//request print from offboard
	while(okFlag == false){
		currentMicros = micros();  //get the current "time" (actually the number of microseconds since the program started)
		if (currentMicros - blinkTimer >= timeout_period) { //test whether the timeout period has elapsed without offboard response
			blinkTimer = currentMicros;  //IMPORTANT to save the start time of the current LED state.
			Serial1.print(F("<P>"));//resend print command if timeout is reached
		}
		getDataFromOB(); //wait for acknowledgment from off-board
		updateSerialOB();
	}
	okFlag = false;
}

static void MPU_update() { //update current roll, pitch from offboard
	unsigned long timeout_period = 10000;	//set timeout period for serial port messaging
	Serial1.print(F("<P>"));//request print from offboard

	while(okFlag == false){
		currentMicros = micros();  //get the current "time" (actually the number of microseconds since the program started)
		if (currentMicros - blinkTimer >= timeout_period) { //test whether the timeout period has elapsed without offboard response
			blinkTimer = currentMicros;  //IMPORTANT to save the start time of the current LED state.
			Serial1.print(F("<P>"));//resend print command if timeout is reached
		}
		getDataFromOB(); //wait for okFlag acknowledgment from off-board
		updateSerialOB();
	}
	okFlag = false; //reset acknowledgment flag
}

static void MPU_turnCalc(double x, double y, int* dx, int* dy){
	//returns counts required to adjust imbalance for x, y planes
	*dx = round((xLen * sin(fabs(x) * DEG_TO_RAD)) / (2 * rev_dist));
	*dy = round((yLen * sin(fabs(y) * DEG_TO_RAD)) / (2 * rev_dist));
// 	Serial2.println(xLen * sin(fabs(x) * DEG_TO_RAD));	//print distance to adjust x and y axis
// 	Serial2.println(yLen * sin(fabs(y) * DEG_TO_RAD));	//commented out for brevity
}
static void MPU_settlewait(double pastKalAngleX, double pastKalAngleY){ //waits until roll and pitch has stopped fluctuating and swinging is slower
	int settled = 0;
	MPU_update(); //update current roll, pitch
	while ((fabs(kalAngleY - pastKalAngleY) > 0.03) || (fabs(kalAngleX - pastKalAngleX) > 0.03)){	
		//while current roll or pitch is fluctuating, current value will be far from past value, hence loop continues
		settled = 1;	//set settled counter to update once loop exits
		Serial2.println(F("{SWINGING!!}"));	//print notification that unit is still swinging hence functions are blocked
		pastKalAngleY = kalAngleY;	//update past roll, pitch
		pastKalAngleX = kalAngleX;
		wait(100000);	//wait 10ms for unit to settle before measuring again
		MPU_update();	//update roll, pitch again
	}
	if(settled == 1)Serial2.println(F("{OK Settled}")); //once loop exits, print notification that unit has stopped swinging and functions resume
}
static void MPU_print(){ //print current roll and pitch
	Serial2.print(F("<KalRoll,")); Serial2.print(kalAngleX); Serial2.print(">");

	Serial2.print(F("<KalPitch,")); Serial2.print(kalAngleY); Serial2.print(">");
	Serial2.print("\r\n");
}
// ================================================================
// ===               LOADCELL FUNCTIONS             ===
// ================================================================
static void LOADCELL_update() { //update loadcell readings
	unsigned long timeout_period = 10000;	//set timeout period for serial port messaging
	
	Serial1.print(F("<LP>"));//resend print command if timeout is reached
	while(okFlag == false){
		currentMicros = micros();  //get the current "time" (actually the number of microseconds since the program started)
		if (currentMicros - blinkTimer >= timeout_period) { //test whether the timeout period has elapsed without offboard response
			blinkTimer = currentMicros;  //IMPORTANT to save the start time of the current state.
			Serial1.print(F("<LP>"));//resend print command if timeout is reached
		}
		getDataFromOB(); //wait for okFlag acknowledgment from off-board
		updateSerialOB();
	}
	okFlag = false;	 //reset acknowledgment flag
}

static void LOADCELL_print(){  //print current load cell readings
	for (int i = 0; i < 4; i++){
		Serial2.print("<LoadCell, ");
		Serial2.print(i);
		Serial2.print(",");
		Serial2.print(LoadRead[i]); 
		Serial2.print(">");
	}
}

static void LOADCELL_lift(float threshload){
/*load cell is required to detect when the pulley becomes taut and threshold weight is detected
when pulley is taut, system can begin balancing process
state 0: get loadcell readings. turn on motors while checking readings
		increment to next loadcell when loadcell readings exceed threshold load
state 1: print loadcell readings
*/
int liftstate = 0;
	do{
		LOADCELL_update();
		MOTOR_off();
		LOADCELL_print();
		LOADCELL_update();
		wait(100000);
		if (liftstate == 0 && stopInterrupt!= true){
			if(LoadRead[0] < threshload){
				MOTOR_turn(0,125);
			}
			else {
				MOTOR_off(0);
				liftstate++;
			}
		}
		else if (liftstate == 1 && stopInterrupt!= true){
			if(LoadRead[1] < threshload){
				MOTOR_turn(1,125);
			}
			else {
				MOTOR_off(1);
				liftstate++;
			}
		}
		else if (liftstate == 2 && stopInterrupt!= true){
			if(LoadRead[2] < threshload){
				MOTOR_turn(2,125);
			}
			else {
				MOTOR_off(2);
				liftstate++;
			}
		}
		else if (liftstate == 3 && stopInterrupt!= true){
			if(LoadRead[3] < threshload){
				MOTOR_turn(3,125);
			}
			else {
				MOTOR_off(3);
				liftstate++;
			}
		}
	} while ( liftstate <4 && stopInterrupt!= true);
	MOTOR_off();
	LOADCELL_print();
}

static void LOADCELL_warning(float threshload){
// when load is exceeded, warning flag is triggered causing stop
	for (int i = 0; i<4; i++){
		if (LoadRead[i] >= threshload){
			Serial2.println(F("{LOAD LIMIT ERROR!!!}"));
			stopInterrupt = true;
		}
	}
}
// ================================================================
// ===             PPVC CONTROL ROUTINES                ===
// ================================================================
static int checkHighest(double rollX, double pitchY){
	if (rollX >= 0 && pitchY >= 0){return 1;} //return highest point
	else if (rollX <= 0 && pitchY >= 0){return 4;}
	else if (rollX <= 0 && pitchY <= 0){return 3;}
	else if (rollX >= 0 && pitchY <= 0){return 2;}
	else{Serial2.println(F("{GYROSCOPE ERROR!!!}"));
		stopInterrupt = true;
	return 0;}
}
static void doMeasureLevel(int SetpointManual){
	//find highest point
	//measureState 0: set dx,dy setpoint based on highest point
	//state 1: turn on pid to prime the system
	//state 2: adjust dx,dy to setpoint. set input to encoder, output to motor pwm 
	//state 3: remeasure, ensure within tolerance, else repeat from state 0
	int encoderState[4] = {0,0,0,0};	//count when each encoder is within set range to setpoint
	int gapDist = 1000;	//set range when encoder input is 'close enough' to setpoint
	int encSteady = 1; //set number of encoderState counts until the system reports that the setpoint has been achieved
	float thresholdAngle = 0.5; //set threshold angle for deciding level
	int dx, dy; //mpu setpoints for each axis
	long gap;	//count difference between input and setpoint
	double measureX, measureY;	//input initial values for roll, pitch angles
	
	MPU_settlewait(kalAngleX,kalAngleY);  //wait for unit to stop swaying
	wait(5000000); //encoder values may not have settled after movement. wait for 5 seconds.
	MPU_update();
	measureX = kalAngleX; //sample roll, pitch angles at start
	measureY = kalAngleY; //pitch
	MPU_turnCalc(measureX,measureY, &dx, &dy);  //get mpu setpoints
		
	Serial2.println(F("Performing Measure and Adjust"));
	do
	{	
		if (measureState == 0 && stopInterrupt!= true && SetpointManual == 0){ //xy measure checks which is highest point, then sets setpoints for each motor
			Serial2.println(F("{STATE 0: Measuring x,y axis...}"));
			MOTOR_off();	//ensure all motors are off before starting
			MPU_settlewait(kalAngleX,kalAngleY);	//wait for unit to stop swinging
			if(checkHighest(measureX,measureY) == 1){	//set setpoints based on which corner is highest
				Setpoint[0]= ENCODER_update(0) -dy -dx;	//setpoints use a superposition of x and y planes
				Setpoint[1]= ENCODER_update(1) +dy -dx;	//by setting the adjustment direction for y planes and x planes separately
				Setpoint[2]= ENCODER_update(2) +dy +dx;	//different target positions for each encoder can be achieved
				Setpoint[3]= ENCODER_update(3) -dy +dx;
			}
			else if(checkHighest(measureX,measureY) == 2){
				Setpoint[0]= ENCODER_update(0) +dy -dx;
				Setpoint[1]= ENCODER_update(1) -dy -dx;
				Setpoint[2]= ENCODER_update(2) -dy +dx;
				Setpoint[3]= ENCODER_update(3) +dy +dx;
			}
			else if(checkHighest(measureX,measureY) == 3){
				Setpoint[0]= ENCODER_update(0) +dy +dx;
				Setpoint[1]= ENCODER_update(1) -dy +dx;
				Setpoint[2]= ENCODER_update(2) -dy -dx;
				Setpoint[3]= ENCODER_update(3) +dy -dx;
			}
			else if(checkHighest(measureX,measureY) == 4){
				Setpoint[0]= ENCODER_update(0) -dy +dx;
				Setpoint[1]= ENCODER_update(1) +dy +dx;
				Setpoint[2]= ENCODER_update(2) +dy -dx;
				Setpoint[3]= ENCODER_update(3) -dy -dx;
			}
			measureState++;	//update measureState for next step
			MPU_update();
			MPU_print();	//print initial gyro and encoder readings, highest corner, and setpoints for each corner
			ENCODER_print();
			Serial2.print(dx); Serial2.println("\t");
			Serial2.print(dy); Serial2.println("\t");
			Serial2.print(F("{Highest: "));
			Serial2.println(checkHighest(measureX,measureY));
			Serial2.println(F("Setpoint Y+X 1...2...3...4"));
			Serial2.print(Setpoint[0]); Serial2.print("\t");
			Serial2.print(Setpoint[1]); Serial2.print("\t");
			Serial2.print(Setpoint[2]); Serial2.print("\t");
			Serial2.print(Setpoint[3]); Serial2.print("\t");
			Serial2.println(F("Continue?....}"));
// 	      return; //exit loop to allow for user to decide whether to continue, comment out for automatic levelling
		}
		if (measureState == 0 && stopInterrupt!= true && SetpointManual == 1){ //set setpoints to encoder values at calibration state
			Serial2.println(F("STATE 0a: Returning to calibrated state..."));
			MOTOR_off();
			MPU_settlewait(kalAngleX,kalAngleY);
			ENCODER_save(1);
			measureState++;
			MPU_update();
			MPU_print();
			ENCODER_print();
			Serial2.println(F("{Setpoint Y+X 1...2...3...4"));
			Serial2.print(Setpoint[0]); Serial2.print("\t");
			Serial2.print(Setpoint[1]); Serial2.print("\t");
			Serial2.print(Setpoint[2]); Serial2.print("\t");
			Serial2.print(Setpoint[3]); Serial2.print("\t");
			Serial2.println(F("Continue?....}"));
// 	      return; //exit loop to allow for user to decide whether to continue, comment out for automatic levelling
		}
		if (measureState == 1 && stopInterrupt!= true){	//turn on PID Controller, and then carry on
			Serial2.println(F("{STATE 1: Turning on PID...}"));
			PID_on();
			measureState++;
		}
		if (measureState == 2 && stopInterrupt!= true){ //xy adjust loop
			Serial2.println(F("{STATE 2: Adjusting x,y axis...}"));
			for(int i = 0; i<4; i++) //repeat process for each motor
			{
				Input[i] = ENCODER_update(i);//encoder input sent through ENCODER_update()
				gap = (long)fabs(Setpoint[i]-Input[i]); //distance away from setpoint
				ENCODER_print();
				//======================AUTO MODE==========================================
// 				if (gap < 100){  //we're close to setpoint, use conservative PID tuning parameters so motor turns slower
// 					motorPID[i].SetTunings(consKp, consKi, consKd);
// 				}
// 				else{ //we're far from setpoint, use aggressive PID tuning parameters so motor turns faster
// 					motorPID[i].SetTunings(aggKp, aggKi, aggKd);
// 				}
// 				motorPID[i].Compute();
// 				MOTOR_turn(i,Output[i]); //set output of PID to individual motor pwm control
				//==========================================================================
				
				//=====================MANUAL MODE==============================
				if (Setpoint[i] < Input[i] && encoderState[i]==0){ //if setpoint less than input, lower
					MOTOR_turn(i,-125);
				}
				if ((Setpoint[i]) > Input[i] && encoderState[i]==0){ //else, raise
					MOTOR_turn(i,125);
				}
				//===============================================================
				if(gap <= (long)gapDist){ //if we have lifted to within limits, turn off motor and increment encoder
					encoderState[i]++;
					MOTOR_off(i);
				}// end if (gap<=50)

			}
			getDataFromPC();	//check serial for new commands
			updateSerialESTOP();	//check input command against list
			updateSerialOut();
// 			LOADCELL_warning(stopLimit);//check if loadcell threshold is exceeded
			if((encoderState[0] >= encSteady)&&(encoderState[1] >= encSteady)&&(encoderState[2] >= encSteady)&&(encoderState[3] >= encSteady)){
				//if all the encoders report that they are near the desired setpoint, move to the next measureState
				PID_end();  //turn PID adjustment OFF
				for (int i = 0 ; i<4; i++){encoderState[i] = 0;} //reset encoderState to 0
				measureState++; //increment measureState
				MOTOR_off();	//turn off all motors
				MPU_print();
				ENCODER_print();
			}
		}
		if (measureState == 3 && stopInterrupt!= true){ 
			//remeasure to check if leveled
			Serial2.println(F("{PLEASE WAIT...."));
			wait(10000000);	//wait 2 seconds for unit to stabilise
			MPU_settlewait(kalAngleX,kalAngleY);	//confirm that swinging has stopped
			ENCODER_print();	//print final encoder and setpoint values for the user to confirm if the unit is leveled
			Serial2.println(F("{STATE 3: Setpoint X 1...2...3...4"));
			Serial2.print(Setpoint[0]); Serial2.print("\t");
			Serial2.print(Setpoint[1]); Serial2.print("\t");
			Serial2.print(Setpoint[2]); Serial2.print("\t");
			Serial2.print(Setpoint[3]); Serial2.print("\t");
			Serial2.print(F("}"));
			MPU_update();
			MPU_print(); //print the current gyroscope readings
			getDataFromOB(); //wait for acknowledgment from off-board
			updateSerialOB();

			if (kalAngleX >= -thresholdAngle && kalAngleX <= thresholdAngle && kalAngleY >= -thresholdAngle && kalAngleY <= thresholdAngle) {
				//if pitch and roll are within tolerance, here set to 0.2degrees, then move to endstate
				Serial2.println(F("{System leveled}"));
				measureState++;   
			}
			else { //if values are not within tolerance repeat the leveling process from the start
				measureState = 0;
				measureX = kalAngleX; //sample roll, pitch angles again
				measureY = kalAngleY;
				MPU_turnCalc(measureX,measureY, &dx, &dy);  //get mpu setpoints
			}
		}
	} while (measureState <4 && stopInterrupt!= true); //if emergency stop is pressed, exit the loop immediately
	Serial2.println(F("{System Stopped}"));
	MPU_update();
	MPU_print(); //print pitch and roll
	MOTOR_off(); //turn off all motors
	measureState = 0;//reset measurestate for next leveling
	stopInterrupt = false;//reset emergency stop flag
}
static void PPVC(){
	/*
	On Ground:
	1. release cables (MOTOR_reset)
	2. motor loop test (MOTOR_loop)
	[Calibrate Y/N?]
	3. calibrate
	[cali done. crane lifts]
	[crane stops. carry on Y/N?]
	Suspended:
	1. tighten cables till taut, wait till unit settled
	[ready to measurelevel Y/N?]
	2. measure xy adjust xy
	3. done. exit.
	*/
	
	if (ppvcState == 0)
	{Serial2.println(F("{State 0: Releasing cables.. Check that all motors are turning in CCW}"));
// 		MOTOR_reset();
// 		Serial2.println(F("{Cables released. Performing motor loop test..}"));
		MPU_print();
		LOADCELL_print();
		MOTOR_loop();
		Serial2.println(F("{Send Y to continue to calibration }"));
		ppvcState++;
		return;
	}
	if (ppvcState == 1)
	{Serial2.println(F("{State 1: Calibrating..}"));
		MPU_calibrate();
		MPU_update();
		getDataFromOB(); //wait for okFlag acknowledgment from off-board
		updateSerialOB();
		MPU_print();
		Serial2.println(F("{Unit calibrated and ready for lifting, send Y to lift unit }"));
		ppvcState++;
		return;
	}
	if (ppvcState == 2)
	{Serial2.println(F("{State 2: Tightening motor cables till taut}"));
		//tighten motor cables using loadcell input as stop point
		LOADCELL_lift(0.01); //tighten cables until 0.2kg load is detected on each cable
		Serial2.println(F("{Send Y to continue to balancing }"));
		ppvcState++;
		return;
	}
	if (ppvcState == 3)
	{
// 		Serial2.println(F("{State 3: Measuring unit...}"));
		doMeasureLevel(0);
		Serial2.println(F("{XY axis measured...END adjustment...}"));
		ppvcState=0;
		return;
	}
}
// ================================================================
// ===             SERIAL COMMUNICATION PROTOCOLS                ===
// ================================================================
static void getDataFromPC() {
	// receive data from PC and save it into inputBuffer
	if(Serial2.available() > 0) {
		char x = Serial2.read();
		// the order of these IF clauses is significant
		if(readInProgress) {
			if (x != endMarker){	
				//end marker not detected yet, continue reading
				inputBuffer[bytesRecvd] = x;
				bytesRecvd ++;
				if (bytesRecvd == buffSize) {
					bytesRecvd = buffSize - 1;
				}
			}
			else {		
				//end marker detected!, stop reading and parse data received
				readInProgress = false;
				newDataFromPC = true;
				inputBuffer[bytesRecvd] = '\0';
				parseData();
			}
		}
		else if (x == startMarker) {
			//if startMarker "<" is detected, start reading, and set the bytes received to zero
			bytesRecvd = 0;
			readInProgress = true;
		}
	}
}
static void getDataFromOB() {
	// receive data from Offboard and save it into inputBuffer
	if(Serial1.available() > 0) {
		char x = Serial1.read();
		// the order of these IF clauses is significant
		if(OBreadInProgress) {
			if (x != endMarker){
				OBinputBuffer[OBbytesRecvd] = x;
				OBbytesRecvd ++;
				if (OBbytesRecvd == buffSize) {
					OBbytesRecvd = buffSize - 1;
				}
			}
			else {
				OBreadInProgress = false;
				newDataFromOB = true;
				OBinputBuffer[OBbytesRecvd] = '\0';
				OBparseData();
			}
		}
		if (msgreadInProgress) {
			if (x != msgEndMarker){
				msgBuffer[msgbytesRecvd] = x;
				msgbytesRecvd ++;
				if (msgbytesRecvd == msgSize){
					msgbytesRecvd = msgSize - 1;
				}
			}
			else{
				msgreadInProgress = false;
				newMsgFromOB = true;
				msgBuffer[msgbytesRecvd] = '\0';
				strcpy(messageFromOB, msgBuffer); // copy msgBuffer to messageFromOB
			}
		}
		else if (x == startMarker) {
			OBbytesRecvd = 0;
			OBreadInProgress = true;
		}
		else if (x == msgStartMarker){
			msgbytesRecvd = 0;
			msgreadInProgress = true;
		}
	}
}
//=============
static void parseData() {
	// split the data into its parts: string for funcsel, int for motorsel, 2nd int for turndist
	char * strtokIndx; // this is used by strtok() as an index
	
	strtokIndx = strtok(inputBuffer,",");      // get the first part - the string
	strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
	
	strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
	newMotorSel = atof(strtokIndx);     // convert this part to an integer
	
	strtokIndx = strtok(NULL, ",");
	newTurnDist = atoi(strtokIndx);     // convert this part to an integer

}
static void OBparseData() {
	// split the data into its parts: string for funcsel, int for motorsel, 2nd int for turndist
	char * strtokIndx; // this is used by strtok() as an index
	
	strtokIndx = strtok(OBinputBuffer,",");      // get the first part - the string
	strcpy(messageFromOB, strtokIndx); // copy it to messageFromPC
	
	strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
	OBSel = atof(strtokIndx);     // convert this part to an integer
	
	strtokIndx = strtok(NULL, ",");
	OBDist = atof(strtokIndx);     // convert this part to an integer
}
static void updateSerialESTOP() {
	if (newDataFromPC) {newDataFromPC = false;
		if (strcmp(messageFromPC, "E") == 0) {  //run ppvc adjustment sequence
			stopInterrupt = true;
		}
	}
	messageFromPC[0] = '\0'; //clear the msg buffer
}
static void updateSerialOB() {
	if (newDataFromOB) {
		newDataFromOB = false;
		if (strcmp(messageFromOB, "X") == 0) {  //update roll, pitch
			kalAngleX = OBSel;
			kalAngleY = OBDist;
			okFlag = true;
		}
		if (strcmp(messageFromOB, "L12") == 0) {  //update LOADCELL 1, 2
			LoadRead[0] = OBSel;
			LoadRead[1] = OBDist;
		}
		if (strcmp(messageFromOB, "L34") == 0) {  //update LOADCELL 3, 4
			LoadRead[2] = OBSel;
			LoadRead[3] = OBDist;
			okFlag = true;
		}
		if (strcmp(messageFromOB, "AB") == 0) {  //is MPU initialised?
			mpu_isinit = true;
		}
		if (strcmp(messageFromOB, "CALIFIN") == 0) {  //run ppvc adjustment sequence
			mpu_iscalibrated = true;
		}
		if (strcmp(messageFromOB, "R") == 0) {  //run ppvc adjustment sequence
			Serial2.print(F("{EMERGENCY STOP}"));
		}
		if (strcmp(messageFromOB, "OK") == 0) {  //acknowledge message
			okFlag = true;
		}
	}
	else if (newMsgFromOB){
		newMsgFromOB = false;
		Serial2.print(F("{"));
		Serial2.print(messageFromOB);
		Serial2.print(F("}"));
	}

	messageFromOB[0] = '\0'; //clear the msg buffer
}

static void updateSerialOut() {
	if (newDataFromPC) {newDataFromPC = false;

	if (strcmp(messageFromPC, "P") == 0) {  //MPU PRINT
		MPU_update();
		MPU_print();
		ENCODER_print();
	}
	if (strcmp(messageFromPC, "LP") == 0) {  //LOADCELL PRINT
		LOADCELL_update();
		LOADCELL_print();	
	}
	if (strcmp(messageFromPC, "LE") == 0) {  //LOADCELL PRINT
		LOADCELL_lift(0.02);
	}
	if (strcmp(messageFromPC, "C") == 0) {  //calibrate
// 		Serial2.print(F("{OK}"));
		MPU_calibrate();
		
	}
	if (strcmp(messageFromPC, "ME") == 0) { //do measurelevel
// 		Serial2.print(F("{OK}"));
		doMeasureLevel(0);
		
	}
	if (strcmp(messageFromPC, "L") == 0) {  //motor loop
		ENCODER_print();
		MOTOR_loop();
		ENCODER_print();
		
	}
	if (strcmp(messageFromPC, "M") == 0) {  //turn selected motor
// 		Serial2.print(F("{OK}"));
		ENCODER_print();
		MOTOR_turn(newMotorSel,newTurnDist);
		
	}
	if (strcmp(messageFromPC, "MS") == 0) { //stop all motors, set pid to manual
// 		Serial2.print(F("{OK}"));
		ENCODER_print();
		MOTOR_off();
		PID_end();
		
	}
	if (strcmp(messageFromPC, "RR") == 0) { //return to calibration state
// 		Serial2.print(F("{OK}"));
		MOTOR_reset();
// 		ENCODER_save(1);
// 		doMeasureLevel(1);
		
	}
	if (strcmp(messageFromPC, "PPVC") == 0) { //run ppvc adjustment sequence
// 		Serial2.print(F("{OK}"));
		if (ppvcState==0)
		{PPVC();    }
			
	}
	if (strcmp(messageFromPC, "Y") == 0) {  //continue ppvc adjustment sequence
// 		Serial2.print(F("{OK}"));
		if(ppvcState !=0){
		PPVC();}
		
	}
// 	if (strcmp(messageFromPC, "PID") == 0) {  //show PID settings
// 		for (int i = 0; i<4 ; i++){
// 			Serial2.print("{PID");
// 			Serial2.print(i);
// 			Serial2.print(": \t");
// 			Serial2.print(motorPID[i].GetKp());
// 			Serial2.print("\t");
// 			Serial2.print(motorPID[i].GetKi());
// 			Serial2.print("\t");
// 			Serial2.println(motorPID[i].GetKd());
// 			Serial2.print("\t");
// 		Serial2.println(motorPID[i].GetMode());
// 		Serial2.print(F("}"));    
// 		
// 		}
// 	}
// 	if (strcmp(messageFromPC, "H") == 0){
// 		printHelp();
// 		
// 	}
}
	messageFromPC[0] = '\0'; //clear the msg buffer
}
static void printHelp(){
// 	Serial2.println(F("{LIST OF FUNCTIONS [PPVC_CONTROLV10] :}"));
// 	Serial2.println(F("{P: Print current accelerometer readings}"));
// 	Serial2.println(F("{LP: Print current load cell readings}"));	
// 	Serial2.println(F("{C: Calibrate accelerometer}"));
// 	Serial2.println(F("{ME: Begin measuring and adjusting sequence [manual]}"));
// 	Serial2.println(F("{L: test CW [+ve encoder count] and CCW [-ve encoder count] motor movement. }"));
// 	Serial2.println(F("{M, intx, inty: M,[select motor], [desired pwm speed: +ve CW, -ve CCW]}"));
// 	Serial2.println(F("{MS: Stop all motors}"));
// 	Serial2.println(F("{RR: Unwind cables for 5 seconds (CCW)}"));
// 	Serial2.println(F("{PPVC: Begin PPVC balancing}"));
// 	Serial2.println(F("{Y: Continue PPVC balancing Sequence}"));
// 	Serial2.println(F("{PID: display current pid settings}"));
}

// The setup() function runs once each time the micro-controller starts
void setup()
{
	timer_init();	//turn on timers and serial ports
	SENSORS_init(); //Start MPU, LOADCELLS
	MOTOR_init();	//turn on motors
	PID_init();	//set PID settings
}

// Add the main program code into the continuous loop() function
void loop()
{
	buttonCheck();	//check physical button press
	blinkLED();	//blink for activity
	getDataFromPC();	//check serial2 port
	updateSerialOut();	//action based on serial2 input
	getDataFromOB();	//check serial1 port
	updateSerialOB();	//action based on serial1 input

}

