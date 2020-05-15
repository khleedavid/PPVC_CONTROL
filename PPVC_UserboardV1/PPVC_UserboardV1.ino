#include <Bounce2.h>
#include <digitalWriteFast.h>
#include "Arduino.h"
#include <Wire.h>

#define LCD05  0x63                   // LCD05 address
#define BUTTON1 12
#define BUTTON2 10
#define BUTTON3 8
#define LED_BUILTIN 13

#define NUM_BUTTONS 3
const uint8_t BUTTON_PINS[NUM_BUTTONS] = {8,10,12};
	
Bounce * buttons = new Bounce[NUM_BUTTONS];


byte buffer[3];
char stringBuf[24];
uint32_t blinkTimer,calibrationTimer, printTimer, waitTimer = 0;
unsigned long currentMillis = 0;
const unsigned long blink_period = 15*60*1000;  //calibrate timer for 15 minutes
const unsigned long print_period = 1000;  //print timer for 10 seconds
const unsigned long	busy_period = 5000;
bool blinkState = false;
bool control_isinit = false;

const byte buffSize = 32; //input buffer size
const char startMarker = '<'; //command start mark
const char endMarker = '>'; //command end mark

const byte msgSize = 32; //message buffer size
const char msgStartMarker = '{'; //message start mark
const char msgEndMarker = '}'; //message end mark

char OBinputBuffer[buffSize]; //ob command buffer size
volatile byte OBbytesRecvd = 0; //ob bytes count received by ob input buffer
volatile boolean OBreadInProgress = false; //ob cmd read
volatile boolean newDataFromOB = false; //new data from ob flag

char msgBuffer[msgSize]; //message input buffer
volatile byte msgbytesRecvd = 0; //bytes count received by cmd buffer
volatile boolean msgreadInProgress = false; //msg read
volatile boolean newMsgFromOB = false; //new msg from OB flag

//variables to store parsed data from OB
char messageFromOB[buffSize] = {0};
volatile char *OBSel;
volatile double OBDist = 0;
volatile boolean busyFlag = false;	//acknowledge OK

void LCDclear();
void LCDMessage(String message);
void LCDcr();
static void getDataFromOB();
static void OBparseData();
static void updateSerialOB();

void LCDclear(){
	buffer[0] = 0;                       // Clear the screen
	buffer[1] = 12;
	Wire.beginTransmission(LCD05);
	Wire.write(buffer,2);
	Wire.endTransmission();
}

void LCDcr(){
	buffer[0] = 0;                       // Carriage return
	buffer[1] = 13;
	Wire.beginTransmission(LCD05);
	Wire.write(buffer,2);
	Wire.endTransmission();
}

void LCDMessage(String message){
	int len = message.length() + 1;       // Length of the message
	message.toCharArray(stringBuf, len);  // Convert the message to a char array
	stringBuf[0] = 0;                     // First byte of message to 0 (LCD05 command register)
	Wire.beginTransmission(LCD05);
	Wire.write(stringBuf, len);
	Wire.endTransmission();
}
static void wait(unsigned long wait_time){ //nonblocking wait function that sets a timer in milliseconds
	waitTimer = currentMillis;  //reset waitTimer
	currentMillis = millis(); //get current time
	while (currentMillis - waitTimer <= wait_time) { //check if wait_time has elapsed
		currentMillis = millis(); //update time while waiting
	}
}

static void timedCalibrate() {  //blink LED to indicate activity, perform calibration
	currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
	if (currentMillis - calibrationTimer >= blink_period && busyFlag!= true) { //test whether the period has elapsed
		digitalWriteFast(LED_BUILTIN, HIGH);
		wait(50);
		digitalWriteFast(LED_BUILTIN, LOW);
		Serial1.print(F("<C>")); //send periodic calibration command
		calibrationTimer = currentMillis;  //IMPORTANT to save the start time of the current LED state.
 	}
	if (currentMillis - printTimer >= print_period && busyFlag!= true) { //test whether the period has elapsed
		digitalWriteFast(LED_BUILTIN, HIGH);
		wait(50);
		digitalWriteFast(LED_BUILTIN, LOW);
		Serial1.print(F("<P>")); //send periodic calibration command
		printTimer = currentMillis;  //IMPORTANT to save the start time of the current LED state.
// 		busyFlag = true;
	}
}
static void SENSORS_init()	//calls offboard to check if sensors have been initialised
{
	unsigned long timeout_period = 100;	//set timeout period for serial port messaging

	Serial1.print("<UT>");	//continue requesting mpu init check
	while (control_isinit != true) //send the command to offboard and wait for acknowledgement
	{
		currentMillis = millis();  //get the current "time" (actually the number of microseconds since the program started)
		if (currentMillis - blinkTimer >= timeout_period) { //test whether the timeout period has elapsed without offboard response
			blinkTimer = currentMillis;  //IMPORTANT to save the start time of the current LED state.
			Serial1.print(F("<UT>"));//resend command if timeout is reached
		}
		getDataFromOB(); //wait for acknowledgment from off-board
		updateSerialOB();
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

static void OBparseData() {
	// split the data into its parts: string for funcsel, int for motorsel, 2nd int for turndist
	char * strtokIndx; // this is used by strtok() as an index
	
	strtokIndx = strtok(OBinputBuffer,",");      // get the first part - the string
	strcpy(messageFromOB, strtokIndx); // copy it to messageFromPC
	
	strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
	OBSel = strtokIndx;     // convert this part to an integer
	
	strtokIndx = strtok(NULL, ",");
	OBDist = atof(strtokIndx);     // convert this part to an integer
}

static void updateSerialOB() {
	char pitch[20];
	char str[50];
	char str2[50];
	if (newDataFromOB) {
		newDataFromOB = false;
		if (strcmp(messageFromOB, "ut") == 0) {  //set control board ready flag
			control_isinit = true;
		}
		if (strcmp(messageFromOB, "OK") == 0) {  //acknowledge message [redundant]
			busyFlag = false;
		}
		if (strcmp(messageFromOB, "Roll") == 0) {  //acknowledge message [redundant]
			
			sprintf(str, " Roll: %s", OBSel);
			LCDclear();
			LCDMessage(str);
		}
		if (strcmp(messageFromOB, "Pitch") == 0) {  //acknowledge message [redundant]
			sprintf(str2, " Pitch: %s", OBSel);
			LCDcr();
			LCDMessage(str2);
		}
	}
	else if (newMsgFromOB){
		newMsgFromOB = false;
			LCDclear();
			LCDMessage(" "+(String)messageFromOB);
	}
	messageFromOB[0] = '\0'; //clear the msg buffer
}
static bool busyFlag_testinit(){	//returns false if okflag is high>> system busy

	while(busyFlag == true){
		currentMillis = millis();  //get the current "time" (actually the number of microseconds since the program started)
		if (currentMillis - printTimer >= busy_period) { //test whether the timeout period has elapsed without offboard response
			blinkTimer = currentMillis;  //IMPORTANT to save the start time of the current LED state.
			Serial1.print(F("<OK>"));//resend print command if timeout is reached
		}
		getDataFromOB(); //wait for okFlag acknowledgment from off-board
		updateSerialOB();
	}
	busyFlag = false; //reset acknowledgment flag
}

void setup() {
	//start serial connection
// 	Serial.begin(9600);
	Serial1.begin(115200);
	calibrationTimer = millis();
	blinkTimer = millis();
	//configure pin 12-8 as an input and enable the internal pull-up resistor
	for (int i = 0; i < NUM_BUTTONS; i++) {
		buttons[i].attach( BUTTON_PINS[i] , INPUT_PULLUP  );       //setup the bounce instance for the current button
		buttons[i].interval(25);              // interval in ms
	}
  	pinMode(LED_BUILTIN, OUTPUT);	//configure 13 as LED activity indicator
	Wire.begin();
	LCDclear();
	LCDMessage(" PPVC INTELLIFT");
	SENSORS_init();
	Serial1.print(F("<C>")); //send initial calibration command
	LCDcr();
	LCDMessage(" READY");
}


void loop() {
	//timed functions
	timedCalibrate();
// 	busyFlag_testinit();
	getDataFromOB();
	updateSerialOB();
	//button functions
	for (int i = 0; i < NUM_BUTTONS; i++)  {
		// Update the Bounce instance :
		buttons[i].update();
	}
	if (buttons[0].fell() ) {
		Serial1.print(F("<RR>")); //send lower motors command
		LCDcr();
		LCDMessage(" LOWER");
// 		busyFlag = true;
	}  
	
	if (buttons[1].fell()) {		
		Serial1.print(F("<E>"));	//send emergency stop command
		LCDcr();
		LCDMessage(" ESTOP"); 
// 		busyFlag = true;
	}//E-stop
	
	
	if (buttons[2].fell()) {    
		Serial1.print(F("<ME>")); //send balance motor command
		LCDcr();
		LCDMessage(" PPVC");
// 		busyFlag = true;
	}  
}