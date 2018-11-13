/*
    Name:       PPVC_OffboardV1.ino
    Created:	8/30/2018 5:53:53 PM
    Author:     David Lee
	Built for Arduino MEGA
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
#include <HX711.h>
#include <TimerOne.h>
#include "TimerThree-master/TimerThree.h"

//DEFINE THESE PARAMETERS BEFORE BEGINNING TESTING
///////////////////////////////////   MPU CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the sketch to your needs.
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=10;     //Accelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=2;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
const float calibration_factor[4] = {123000,66300,102000,76000}; //set calibration factor for load cells

// ================================================================
// ===             MPU  VARIABLES                ===
// ================================================================
//gyroscope MPU6050 datasheet https://components101.com/sensors/mpu6050-module
/* IMU Data */
// Variables to store the values from the sensor readings
#define RESTRICT_PITCH // Comment out to restrict roll to ?90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf
#define INTERRUPT_PIN 2 //dmpDataReady interrupt notifies when data is ready to be read
Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;
//mpu config vars
MPU6050 mpu;
bool mpu_isinit = false;
//kalman filter measurement variables
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter
double roll, pitch;
//calibration variables
int16_t ax, ay, az;
int16_t gx, gy, gz;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,caliState=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

// ================================================================
// ===               LOADCELL VARIABLES              ===
// ================================================================

HX711 Load[4] = { //DOUT, CLK
	HX711(26,27), //this one i messed up on the board so i have to switch the connections
	HX711(24,25),
	HX711(23,22),
HX711(28,29)};
float LoadRead[4] = {0,0,0,0}; //Stores latest Load Value
bool load_isinit = false;
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
const unsigned long calibrate_period = 10000000;  //calibration wait
const unsigned long blink_period = 100000;  //blink timer
bool blinkState = false;

// ================================================================
// ===             SERIAL INTERFACE VARIABLES                ===
// ================================================================

const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;
char messageFromPC[buffSize] = {0};
int newMotorSel = 0;
int newTurnDist = 0; // fraction of servo range to move

// ================================================================
// ===               FUNCTION PROTOTYPES              ===
// ================================================================

static void MPUready();
static void wait(unsigned long wait_time);
static void blinkLED();
static void MPU_calibrate();
static void meansensors();
static void calibration();
static uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
static uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
static uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);
static void timer_init();
static bool SENSORS_testinit();
static void MPU_init();
static void MPU_update();
static void MPU_turnCalc(double x, double y, int* dx, int* dy);
static void MPU_settlewait(double pastKalAngleX, double pastKalAngleY);
static void MPU_print();
static void LOADCELL_init();
static void LOADCELL_update();
static void LOADCELL_print();
static void getDataFromControl();
static void emptyBuffer();
static void parseData();
static void updateSerialOut();
static void updateSerialCali();

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool stopInterrupt = false; //Emergency stop interrupt
volatile bool MPUcallback = false;
volatile bool LCcallback = false;
static void MPUready(){ //MPU Interrupt Service Routine
	MPUcallback = true;
}
static void LCready(){	//LOADCELL Interrupt Service Routine
	LCcallback = true;
}
// ================================================================
// ===                TIMER_ROUTINES                       ===
// ================================================================
static void timer_init(){        // initialize timers
	globalTimer = micros();
	blinkTimer = micros();
	waitTimer = micros();
	Timer1.initialize(100000);
	Timer3.initialize(100000);
	Timer1.attachInterrupt(MPUready);  // attaches MPUready() as a timer overflow interrupt
	Timer3.attachInterrupt(LCready,400000);	// attaches LCready() as a timer overflow interrupt
}
static void wait(unsigned long wait_time){ //waits a set amount of time in microseconds
	waitTimer = currentMicros;  //reset globaltimer
	currentMicros = micros();
	while (currentMicros - waitTimer <= wait_time) {
		currentMicros = micros();
	}
}
static void blinkLED(){  //blink LED to indicate activity
	currentMicros = micros();  //get the current "time" (actually the number of microseconds since the program started)
	if (currentMicros - blinkTimer >= blink_period) { //test whether the period has elapsed
		blinkState = !blinkState;
		digitalWrite(LED_BUILTIN, blinkState);
		blinkTimer = currentMicros;  //IMPORTANT to save the start time of the current LED state.
	}
}
// ================================================================
// ===                CALIBRATION_ROUTINE                       ===
// ================================================================
static void MPU_calibrate(){
	MPU_settlewait(kalAngleX,kalAngleY);	//wait for unit to stop swinging
	digitalWrite(LED_BUILTIN,HIGH);
	Serial1.print("{MPU6050 Calibration Sketch}");	//print message to notify user
	wait(20000);
// 	Serial1.println(F("{\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n}"));
	wait(30000);
	// verify offboard connection to mpu
	Serial1.print(mpu.testConnection() ? "{MPU6050 connection successful}" : "{MPU6050 connection failed}");
	while(mpu.testConnection() == false);	//if connection is not made, wait until user solves the issue
	wait(10000);
	// reset mpu raw measurement offsets
	mpu.setXAccelOffset(0);
	mpu.setYAccelOffset(0);
	mpu.setZAccelOffset(0);
	mpu.setXGyroOffset(0);
	mpu.setYGyroOffset(0);
	mpu.setZGyroOffset(0);
	do{ 
		getDataFromControl();	//check for emergency stop message
		updateSerialCali();
		if (caliState==0 && stopInterrupt!= true){
			Serial1.print(F("{Reading sensors...}"));
			meansensors();	//find mean of raw sensor readings for first pass of offset
			caliState++;	//increment calibration to next stage
			wait(100000);   
		}
		if (caliState==1 && stopInterrupt!= true) {
			Serial1.print(F("{Calculating offsets...}"));
			calibration();	//set offsets to mean values, measure mean of sensor readings again, until sensor mean readings are within desired range of the predefined optimal level measurements
			caliState++;	//increment calibration to next stage
			wait(100000);   
		}
		if (caliState==2 && stopInterrupt!= true) {
			meansensors();	//calculate mean of sensors again for final pass
			// set new offsets for sensor readings
			mpu.setXAccelOffset(ax_offset);
			mpu.setYAccelOffset(ay_offset);
			mpu.setZAccelOffset(az_offset);
			mpu.setXGyroOffset(gx_offset);
			mpu.setYGyroOffset(gy_offset);
			mpu.setZGyroOffset(gz_offset);
			wait(100000);
			caliState++;    //increment calibration to next stage
		}
	} while (caliState < 3 && stopInterrupt!= true);
	for (int i=0; i<4; i++)	{ //tare the pulleys after calibration
		Load[i].tare();
		Serial1.print(F("{TARING}"));
	}
	 			Serial1.print(F("<CALIFIN>"));	//send <CALIFIN> message to end calibration on Main board

	caliState = 0; //reset calibration stages for next calibrate routine
	if (stopInterrupt == true){
		//if stopInterrupt is pressed, calibration process will exit
		//send <R> to confirm that system has exited calibration
		//reset stopInterrupt flag
		Serial1.print("<R>");
		stopInterrupt = false;
	}
}
static void meansensors(){
	long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

	while (i<(buffersize+101)){
		// read raw accel/gyro measurements from device
		mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		
		if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
			buff_ax=buff_ax+ax;
			buff_ay=buff_ay+ay;
			buff_az=buff_az+az;
			buff_gx=buff_gx+gx;
			buff_gy=buff_gy+gy;
			buff_gz=buff_gz+gz;
		}
		if (i==(buffersize+100)){
			mean_ax=buff_ax/buffersize;
			mean_ay=buff_ay/buffersize;
			mean_az=buff_az/buffersize;
			mean_gx=buff_gx/buffersize;
			mean_gy=buff_gy/buffersize;
			mean_gz=buff_gz/buffersize;
		}
		i++;
		delay(2); //Needed so we don't get repeated measures
	}
}
static void calibration(){ //sets offsets by averaging test offset values within a shrinking range
	ax_offset=-mean_ax/8;
	ay_offset=-mean_ay/8;
	az_offset=(16384-mean_az)/8;

	gx_offset=-mean_gx/4;
	gy_offset=-mean_gy/4;
	gz_offset=-mean_gz/4;
	while (1){
		int ready=0;
		mpu.setXAccelOffset(ax_offset);
		mpu.setYAccelOffset(ay_offset);
		mpu.setZAccelOffset(az_offset);

		mpu.setXGyroOffset(gx_offset);
		mpu.setYGyroOffset(gy_offset);
		mpu.setZGyroOffset(gz_offset);

		meansensors();
		Serial1.println("{...}");

		if (abs(mean_ax)<=acel_deadzone) ready++;
		else ax_offset=ax_offset-mean_ax/acel_deadzone;

		if (abs(mean_ay)<=acel_deadzone) ready++;
		else ay_offset=ay_offset-mean_ay/acel_deadzone;

		if (abs(16384-mean_az)<=acel_deadzone) ready++;
		else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

		if (abs(mean_gx)<=giro_deadzone) ready++;
		else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

		if (abs(mean_gy)<=giro_deadzone) ready++;
		else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

		if (abs(mean_gz)<=giro_deadzone) ready++;
		else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

		if (ready==6 || stopInterrupt == true) break;
	}
}
// ================================================================
// ===                I2C_PROTOCOLS                          ===
// ================================================================
static uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop) 
{
	return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
}
static uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop) 
{
	Wire.beginTransmission(IMUAddress);
	Wire.write(registerAddress);
	Wire.write(data, length);
	uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
	if (rcode) {
		Serial1.print(F("{i2cWrite failed: "));
		Serial1.print(rcode);
		Serial1.println("}");
	}
	return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
}
static uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes) 
{
	uint32_t timeOutTimer;
	Wire.beginTransmission(IMUAddress);
	Wire.write(registerAddress);
	uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
	if (rcode) {
		Serial1.print(F("{i2cRead failed: "));
		Serial1.print(rcode);
		Serial1.println("}");
		return rcode; // See: http://arduino.cc/en/Reference/WireEndTransmission
	}
	Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
	for (uint8_t i = 0; i < nbytes; i++) {
		if (Wire.available())
		data[i] = Wire.read();
		else {
			timeOutTimer = micros();
			while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
			if (Wire.available())
			data[i] = Wire.read();
			else {
				Serial1.println(F("{i2cRead timeout}"));
				return 5; // This error value is not already taken by endTransmission
			}
		}
	}
	return 0; // Success
}
// ================================================================
// ===                MPU_PROTOCOLS                          ===
// ================================================================
static bool SENSORS_testinit(){	//returns true if mpu is initialised
	if (mpu_isinit == true)
	{
		return true;
	}
	else{	 return false;}
}
static void MPU_init() 
{
	pinMode(INTERRUPT_PIN, INPUT);
	Serial1.begin(115200);
	Wire.begin();
	#if ARDUINO >= 157
	Wire.setClock(400000UL); // Set I2C frequency to 400kHz
	#else
	TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
	#endif

	i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
	i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
	i2cData[2] = 0x00; // Set Gyro Full Scale Range to ?250deg/s
	i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ?2g
	
	while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
	while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
	
	while (i2cRead(0x75, i2cData, 1));
	if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
		Serial1.print(F("{Error reading sensor}"));
		while (1);
	}
	wait(10000);

	/* Set kalman and gyro starting angle */
	while (i2cRead(0x3B, i2cData, 6));
	accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
	accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
	accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
	
	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -? to ? (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
	#ifdef RESTRICT_PITCH // Eq. 25 and 26
	roll  = atan2(accY, accZ) * RAD_TO_DEG;
	pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	#else // Eq. 28 and 29
	roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	pitch = atan2(-accX, accZ) * RAD_TO_DEG;
	#endif

	kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);
	gyroXangle = roll;
	gyroYangle = pitch;
	mpu_isinit = true; //set mpu initialised flag for mainboard query
}
static void MPU_update(){
	if (!MPUcallback) return;	//check if MPUCallback Flag is true for timing purposes
	MPUcallback = false;	//reset MPUCallback flag
	while (i2cRead(0x3B, i2cData, 14));	//read mpu6050 through i2c register 0x3B
	accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
	accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
	accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
	gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
	gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
	gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);
	double dt = (double)(micros() - globalTimer) / 1000000; // Calculate delta time, change in time since last reading
	globalTimer = micros();	//update current time

	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -? to ? (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
	#ifdef RESTRICT_PITCH // Eq. 25 and 26
	// convert accelerometer roll and pitch from radians to degrees
	roll  = atan2(accY, accZ) * RAD_TO_DEG;
	pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	#else // Eq. 28 and 29
	roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	pitch = atan2(-accX, accZ) * RAD_TO_DEG;
	#endif
	// convert gyroscope roll and pitch rotation from radians/second to degrees/second
	double gyroXrate = gyroX / 131.0; // Convert to deg/s
	double gyroYrate = gyroY / 131.0; // Convert to deg/s

	#ifdef RESTRICT_PITCH
	//if raw roll reading goes beyond 90 degrees for each side, reset kalman filter angles to prevent roll value from "jumping" between high and low readings
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
		kalmanX.setAngle(roll);
		kalAngleX = roll;
		gyroXangle = roll;
	} else
	kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the roll angle using a Kalman filter

	if (abs(kalAngleX) > 90)	//if calculated roll reading exceeds 90 degree limit, invert the gyroscope reading
	gyroYrate = -gyroYrate; // Invert rate, so it fits the restricted accelerometer reading
	kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);	// Calculate the pitch angle using a Kalman filter
	#else
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
		kalmanY.setAngle(pitch);
		kalAngleY = pitch;
		gyroYangle = pitch;
	} else
	kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalAngleY) > 90)
	gyroXrate = -gyroXrate; // Invert rate, so it fits the restricted accelerometer reading
	kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
	#endif

	gyroXangle += gyroXrate * dt; // Calculate raw gyro angle without any filter
	gyroYangle += gyroYrate * dt;

	// Reset the gyro angle to kalman filtered angle when it has drifted beyond center point
	if (gyroXangle < -180 || gyroXangle > 180)
	gyroXangle = kalAngleX;
	if (gyroYangle < -180 || gyroYangle > 180)
	gyroYangle = kalAngleY;
	if (kalAngleX >= -0.05 && kalAngleX <= 0.05) {
		kalAngleX = 0.0;
	}
	if(kalAngleY >= -0.05 && kalAngleY <= 0.05){
		kalAngleY = 0.0;
	}
	
// 	LOADCELL_update();
}
static void MPU_settlewait(double pastKalAngleX, double pastKalAngleY){ //waits until kalAngle is settled
	int settled = 0;
	MPU_update();
	while ((fabs(kalAngleY - pastKalAngleY) > 0.03) || (fabs(kalAngleX - pastKalAngleX) > 0.03))
	{ settled = 1;
		pastKalAngleY = kalAngleY;
		pastKalAngleX = kalAngleX;
		wait(100000);
	MPU_update();       }
}
static void MPU_print(){	//sends roll, and pitch data to mainboard in 1 packet
	Serial1.print(F("<X,")); Serial1.print(kalAngleX,2);
	Serial1.print(F(",")); Serial1.print(kalAngleY,2);Serial1.print(F(">"));
}

// ================================================================
// ===               LOADCELL FUNCTIONS             ===
// ================================================================

static void LOADCELL_init(){	//initialise load cell
	Serial1.println("{HX711 scale init}");
	for (int i = 0; i < 4; i++){
		Load[i].set_scale(calibration_factor[i]); //This value is obtained by using the SparkFun_HX711_Calibration sketch
		Load[i].tare(); //Assuming there is no weight on the scale at start up, reset the scale to 0
		Serial1.print("{Ready:");
		Serial1.print(i);
		Serial1.print("}");
	}
	load_isinit = true;
}
static void LOADCELL_update(){	//update loadcell readings
		if (!LCcallback) return;	//check if MPUCallback Flag is true for timing purposes
		LCcallback = false;	//reset MPUCallback flag

	for (int i = 0; i < 4; i++){
		LoadRead[i] = Load[i].get_units();
	}
}
static void LOADCELL_print(){ //Send loadcell readings to main board
	Serial1.print("<L12");	//send readings in packets of 2 each
	for (int i = 0; i < 2; i++){	//send loadcell 1,2 readings
		Serial1.print(",");
		Serial1.print(fabs(LoadRead[i]),2); 
	}
	Serial1.print(">");
	Serial1.print("<L34"); //send loadcell 3,4 readings
	for (int i = 2; i < 4; i++){
		Serial1.print(",");
		Serial1.print(fabs(LoadRead[i]),2); 		
	}
	Serial1.print(">");

}

// ================================================================
// ===             SERIAL COMMUNICATION PROTOCOLS               ===
// ================================================================
static void getDataFromControl() {
	// receive data from Control and save it into inputBuffer
	if(Serial1.available() > 0) {
		char x = Serial1.read();
		// the order of these IF clauses is significant
		if (x == endMarker) {
			//if endMarker ">" is detected, stop reading, parse Data received, and empty buffer 
			readInProgress = false;
			newDataFromPC = true;
			inputBuffer[bytesRecvd] = 0;
			parseData();
			emptyBuffer();			
		}
		if(readInProgress) {
			//while reading, save each incoming byte into inputBuffer
			inputBuffer[bytesRecvd] = x;
			bytesRecvd ++;	//track number of bytes received
			if (bytesRecvd == buffSize) {
				//if bytes received reaches maximum buffer size, replace the current byte with the next incoming byte
				bytesRecvd = buffSize - 1;
			}
		}
		if (x == startMarker) {
			//if startMarker "<" is detected, start reading, and set the bytes received to zero
			bytesRecvd = 0;
			readInProgress = true;
		}
	}
}
static void emptyBuffer()
{
	while (Serial1.available() > 0) {
		Serial1.read();
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
static void updateSerialCali() {
	if (newDataFromPC) {newDataFromPC = false;}
	if (strcmp(messageFromPC, "E") == 0) {  //stop calibration sequence
		stopInterrupt = true;
		//only acknowledge message after system has actually stopped calibrating
	}
	messageFromPC[0] = '\0'; //clear the msg buffer
}
static void updateSerialOut() {
	if (newDataFromPC) {newDataFromPC = false;}
		//if new message is received, reset the flag, then select the function based on the message contents

	if (strcmp(messageFromPC, "P") == 0) {  //MPU PRINT
		MPU_print();
	}
	if (strcmp(messageFromPC, "C") == 0) {  //calibrate
		Serial1.print("<OK>");
		MPU_calibrate();
	}
	if (strcmp(messageFromPC, "AT") == 0) {  //MPU TEST INIT
		if(SENSORS_testinit()) Serial1.print("<AB>");
	}
	if (strcmp(messageFromPC, "LP") == 0) {  //LOADCELL PRINT
		LOADCELL_update();
		LOADCELL_print();
	}
	messageFromPC[0] = '\0'; //clear the msg buffer after the function is performed
}

// The setup() function runs once each time the micro-controller starts
void setup()
{
	timer_init(); //Initialize timers
	LOADCELL_init(); //Initialize Load cells
	MPU_init(); //Start MPU and serial connection
	pinMode(LED_BUILTIN, OUTPUT);// initialize LED_BUILTIN as an output.
}

// Add the main program code into the continuous loop() function
void loop()
{
	blinkLED();
	MPU_update(); //only updates when mpucallback is high from timer
	LOADCELL_update();
	getDataFromControl();
	updateSerialOut();
}
