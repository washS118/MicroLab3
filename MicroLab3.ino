// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       MicroLab3.ino
    Created:	3/11/2019 12:24:15 PM
    Author:     DESKTOP-IGB49L9\lukeg
*/

// Includes
#include <SoftwareSerial.h>
#include <Keypad.h>
#include <Key.h>

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
#define SPEAKER A4

const int KEY_ROWS[] = { 3, 4, 5, 6 };
const int KEY_COLS[] = { 7, 8, A3, 10 };



//Global Vars
int mode;
volatile int frequency;
SoftwareSerial bluetooth(BT_TX, BT_RX);
float tones[TONES] = {261.6, 293.6, 329.6, 349.2, 392, 440, 493.8, 523.2};

//Song Segment settings
#define NOTES 30
#define SEGMENTS 2

typedef struct {
	int tone; //Index of tones array
	int duration; //In Millis
} Note;

Note song[SEGMENTS][NOTES];

//Keypad Settings
const byte ASCII0 = 48;
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
char keys[ROWS][COLS] = {
	{ '1','2','3','A' },
	{ '4','5','6','B' },
	{ '7','8','9','C' },
	{ '*','0','#','D' }
};

Keypad keypad = Keypad(makeKeymap(keys), KEY_ROWS, KEY_COLS, ROWS, COLS);

// IR-FuncGen Settings
#define TOP ICR1
#define DUTY OCR1A
#define CLOCKRATE 16000000
#define PWM_PIN 9
#define PWM_FREQ 1000
#define COUNT 5
const int cutoffs[] = { 500, 100, 10, 5, 0 };
const int prescalers[] = { 1, 8, 64, 256, 1024 };
#define READ_PIN 2
long avgFreq;
float avgDuty;
#define A_PIN A0
#define TOGGLE_PIN 2



//LCD Function Prototypes
		void char2LCD(const byte);
		void char2LCD(const int dataPin, const int clockPin, 
              const int ssPin, const byte ch, 
              const int byteOrder);
		void str2LCD(unsigned char *str);
		void turnBlinkingCursorOn();
		void turnBlinkingCursorOff();
	// Put cursor at location position
	// Values 0x00-0x0F for row 1, 0x40-0x4F for row 2
		void setCursorPos(unsigned char pos);
	// clear LCD and move cursor to line 1 column 1
		void clearScreen();
	// setup code for communication with LCD
		void lcdSPISetup();

// The setup() function runs once each time the micro-controller starts
void setup()
{
	attachInterrupt(2, freqCalc, RISING);

	Serial.begin(9600);  // Begin the serial monitor at 9600bps
	Serial.println("Starting Program");

	bluetooth.begin(115200);  // The Bluetooth Mate defaults to 115200bps
	bluetooth.print("$");  // Print three times individually
	bluetooth.print("$");
	bluetooth.print("$");  // Enter command mode
	delay(100);  // Short delay, wait for the Mate to send back CMD
	bluetooth.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
	// 115200 can be too fast at times for NewSoftSerial to relay the data reliably
	bluetooth.begin(9600);  // Start bluetooth serial at 9600
	bluetooth.print("Starting Program");

	lcdSPISetup();

	// IR-FuncGen Setup
	pinMode(PWM_PIN, OUTPUT);
	pinMode(READ_PIN, INPUT);
	pinMode(A_PIN, INPUT);
	initPWM();
	setPWM(PWM_FREQ, (analogRead(A_PIN) / 1024.0) * 100);




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

//LCD Sub-Functions
		void char2LCD(const int dataPin, const int clockPin, 
              		const int ssPin, const byte ch, 
              		const int byteOrder = MSBFIRST)
		{
  			byte compareValue = 0x80;
  			// initialize compareValue
  			if(byteOrder == MSBFIRST)
  			{
   		 	compareValue = 0x80;
 		 	} else 
 		 	{
  		  	compareValue = 0x01;
 		 	}
  
  			// enable slave select
 		 	digitalWrite(ssPin, LOW);
  
  			// shift out data
  			for (int i = 0; i < 8; i++)
  			{
   		 	// send bit to data pin
   		 	digitalWrite(dataPin, (ch&compareValue)?HIGH:LOW);
    
  		  	// shift compare value
  		 	 if(byteOrder == MSBFIRST)
   		 	{
  		    	  compareValue = compareValue >> 1;
  		  	} else 
  		 	 {
  		  	    compareValue = compareValue << 1;
  		  	}
    
  		  	// trigger clk rising edge(toggle low then high)
  		  	digitalWrite(clockPin, LOW);
  		  	// wait before trigger
  		  	delayMicroseconds(4);
   		 	digitalWrite(clockPin, HIGH);
 		  	 // wait, LCD can only handle up to 100KHz
 		   	delayMicroseconds(14);
    
 		 	}
 		 	// disable slave select
 		 	digitalWrite(ssPin, HIGH);
		 }

		// convenience function for char2LCD using global pin variables
		void char2LCD(const byte ch)
		{
  			char2LCD(DATA_PIN, CLK_PIN, SS_PIN, ch);
		}

		// Output string to LCD
		void str2LCD(const char *str) 
		{
  		  int i = 0;
   		 while(str[i]) {
        		char2LCD(str[i]);
        		i++;
    		}  
		}

		void turnBlinkingCursorOn() 
		{
  		  char2LCD(0xFE);
   		 char2LCD(0x4B);
   		 // 100 micro second execution time
   		 delayMicroseconds(101);      
		}

		void turnBlinkingCursorOff() 
		{
    		char2LCD(0xFE);
    		char2LCD(0x4C);
    		// 100 micro second execution time
    		delayMicroseconds(101);      
		}

		// Put cursor at location position
		// Values 0x00-0x0F for row 1, 0x40-0x4F for row 2
		void setCursorPos(unsigned char pos) 
		{
    		char2LCD(0xFE);
    		char2LCD(0x45);
    		char2LCD(pos);
    		// 100 us execution time
    		delayMicroseconds(110);  
		}

		// clear LCD and move cursor to line 1 column 1
		void clearScreen()
		{
  		  char2LCD(0xFE);
   		 char2LCD(0x51);
   		 // 1.5 ms execution time
   		 delayMicroseconds(2000);
		}

		// setup code for communication with LCD
		void lcdSPISetup()
		{
  
  		// Setup SPI pins as output
 		 pinMode(DATA_PIN, OUTPUT);
  		pinMode(CLK_PIN, OUTPUT);
  		pinMode(SS_PIN, OUTPUT); 
  
  
  		turnBlinkingCursorOn();
}

//Play-Record Sub-Functions
	void initSeg(int seg) {
		for (int j = 0; j < NOTES; j++) {
			song[seg][j].duration = 0;
			song[seg][j].tone = 0;
		}
	}
	void printSegment(int seg) {
		Serial.print(song[seg][0].tone);
		for (int i = 1; i < NOTES; i++) {
			Serial.print(", ");
			Serial.print(song[seg][i].tone);
		}
	}

//LED Control Sub-Functions
		void readPWM() {
			long totalHigh = pulseIn(READ_PIN, HIGH);
			long totalLow = pulseIn(READ_PIN, LOW);
			avgFreq = 1000000 / (totalHigh + totalLow);
			avgDuty = (totalHigh / (float)(totalHigh + totalLow)) * 100;
		}

		void initPWM() {
			TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
			TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS12) | _BV(CS10);
			TOP = 0xFFFF;
			DUTY = 0x0;
		}

		void setPWM(double frequency, double duty) {
			//Find proper prescaler
			int index = 0;
			while (frequency < cutoffs[index]) {
				//Serial.println(cutoffs[index]);
				index++;
			}
			unsigned int scale = prescalers[index];

			//SET SCALE
			TCCR1B &= 0xF8;
			switch (scale)
			{
			case 1:
				TCCR1B |= _BV(CS10);
				break;
			case 8:
				TCCR1B |= _BV(CS11);
				break;
			case 64:
				TCCR1B |= _BV(CS11);
				TCCR1B |= _BV(CS10);
				break;
			case 256:
				TCCR1B |= _BV(CS12);
				break;
			case 1024:
				TCCR1B |= _BV(CS12);
				TCCR1B |= _BV(CS10);
				break;
			default:
				break;
			}

			//SET frequency and duty cycle
			long t = (CLOCKRATE / scale / frequency) - 1;
			if (t > 0xFFFF) t = 0xFFFF;

			TOP = t;
			DUTY = (duty / 100) * TOP;

			//print data
			Serial.println("Setting Frequency and Duty");
			Serial.print("TOP:");
			Serial.print(TOP);

			Serial.print("\tDUTY:");
			Serial.print(DUTY);

			Serial.print("\tSCALE:");
			Serial.println(scale);
		}		

void setDuty(double duty) {
	//SET duty cycle
	DUTY = (duty / 100) * TOP;
}


