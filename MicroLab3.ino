// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       MicroLab3.ino
    Created:	3/11/2019 12:24:15 PM
    Author:     DESKTOP-IGB49L9\lukeg
*/

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

// The setup() function runs once each time the micro-controller starts
void setup()
{
	attachInterrupt(2, freqCalc, RISING);

}

// Add the main program code into the continuous loop() function
void loop()
{
	
}

void menu(){}

void record(){}

void play(){}

void stop(){}

void printLCD(){}

void clear(){}

void freqCalc(){
	static unsigned long last = 0;
	frequency = 1000000.0 / (micros() - last);
	last = micros();
}
