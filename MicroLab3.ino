// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       MicroLab3.ino
    Created:	3/11/2019 12:24:15 PM
    Author:     DESKTOP-IGB49L9\lukeg
*/

// Includes
#include <SoftwareSerial.h>

// Mode Defines
#define IDLE 0
#define RECORD 1
#define PLAY 2

// IO Pins
#define FREQ 2
#define IR A0
#define LED A5
#define BT_TX A1
#define BT_RX A2
#define LCD_SS 11
#define LCD_CLK 12
#define LCD_DATA 13

const int KEY_ROWS[] = { 3, 4, 5, 6 };
const int KEY_COLS[] = { 7, 8, 9, 10 };

//Global Vars
int mode;
volatile int frequency;
SoftwareSerial bluetooth(BT_TX, BT_RX);

// The setup() function runs once each time the micro-controller starts
void setup()
{
	attachInterrupt(2, freqCalc, RISING);

	Serial.begin(9600);  // Begin the serial monitor at 9600bps

	bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
	bluetooth.print("$");  // Print three times individually
	bluetooth.print("$");
	bluetooth.print("$");  // Enter command mode
	delay(100);  // Short delay, wait for the Mate to send back CMD
	bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
	// 115200 can be too fast at times for NewSoftSerial to relay the data reliably
	bluetooth.begin(9600);  // Start bluetooth serial at 9600
}

// Add the main program code into the continuous loop() function
void loop()
{
	updateLED();

	if (bluetooth.available) {
		String line = bluetooth.readString();
		if (line.equals("record")) mode = RECORD;
		else if (line.equals("playback")) mode = PLAY;
		else if (line.equals("help")) help();
		else if (line.equals("clear")) clear();
		else if (line.equals("stop")) mode = IDLE;
		else if (line.equals("song")); // Print song segment
		else if (line.equals("seg")); // Switch song segment
		else error();
	}
	
	switch (mode) {
	case RECORD:
		record();
		break;
	case PLAY:
		play();
		break;
	case IDLE:

		break;
	default:
		mode = IDLE;
	}
}

//Bullet 4
void updateLED(){}

//Menu
void help(){}

//Menu
void error(){}

//Bullet 1
void record(){}

//Bullet 2
void play(){}

//QOL
void stop(){}

//Bullet 3 partial
void printLCD(){}

//Menu
void clear(){}

void freqCalc(){
	static unsigned long last = 0;
	frequency = 1000000.0 / (micros() - last);
	last = micros();
}
