// Visual Micro is in vMicro>General>Tutorial Mode
// 
/*
    Name:       MicroLab3.ino
    Created:	3/11/2019 12:24:15 PM
    Author:     DESKTOP-IGB49L9\lukeg
*/

#pragma region Includes
#include <SoftwareSerial.h>
#include <Keypad.h>
#include <Key.h>
#pragma endregion

#pragma region Modes
#define MODE_IDLE 0
#define MODE_RECORD 1
#define MODE_PLAY 2
#pragma endregion

#pragma region IO Pins
#define FREQ_PIN 2
#define IR_PIN A0
#define LED_PIN 9
#define BT_TX A1
#define BT_RX A2
#define LCD_SS 11
#define LCD_CLK 12
#define LCD_DATA 13
#define SPEAKER_PIN A4
#pragma endregion

#pragma region Keypad Vars
byte KEY_ROWS[] = { 3, 4, 5, 6 };
byte KEY_COLS[] = { 7, 8, A3, 10 };

const byte ASCII0 = 48;
const byte ROWS = 4; //four rows
const byte COLS = 4; //four columns
char keys[ROWS][COLS] = {
	{ '1','2','3','A' },
	{ '4','5','6','B' },
	{ '7','8','9','C' },
	{ '*','0','#','D' }
};

bool keyIdle = false;
bool keyWasIdle = false;
int pressTime = 0;
char key;
int keyCount = 0;
#pragma endregion

#pragma region Globals
int mode;
volatile int frequency;
volatile float pause;
SoftwareSerial bluetooth(BT_TX, BT_RX);
Keypad keypad(makeKeymap(keys), KEY_ROWS, KEY_COLS, ROWS, COLS);
#pragma endregion

#pragma region Tone Vars
#define TONES 8
const float tones[TONES] = { 261.6, 293.6, 329.6, 349.2, 392, 440, 493.8, 523.2 };
#pragma endregion

#pragma region Song Vars
#define NOTES 30
#define SEGMENTS 2

typedef struct {
	int tone; //Index of tones array
	int duration; //In Millis
} Note;

Note song[SEGMENTS][NOTES];
#pragma endregion

#pragma region PWM
#define TOP ICR1
#define DUTY OCR1A
#define CLOCKRATE 16000000
#define PWM_PIN 9
#define PWM_FREQ 1000
#define PRESCALER_COUNT 5
const int cutoffs[] = { 500, 100, 10, 5, 0 };
const int prescalers[] = { 1, 8, 64, 256, 1024 };
bool toggle = true;
#pragma endregion

#pragma region Playback-Record vars
bool measuring = false;
int segment = 0;
bool  isPressed = false;
int note = 0;
int stopTime = 0;
#pragma endregion

// The setup() function runs once each time the micro-controller starts
void setup()
{
	attachInterrupt(digitalPinToInterrupt(FREQ_PIN), freqCalc, RISING);

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
	bluetooth.println("Starting Program");

	lcdSPISetup();
	str2LCD("Starting Program");

	// IR-FuncGen Setup
	pinMode(PWM_PIN, OUTPUT);
	pinMode(IR_PIN, INPUT);
	pinMode(FREQ_PIN, INPUT);
	initPWM();
	setPWM(PWM_FREQ, 100);
}

// Add the main program code into the continuous loop() function
void loop()
{
	
	updateLED();
	
	if (bluetooth.available()) {
		String line = bluetooth.readString();
		line.trim();
		line.toLowerCase();
		Serial.println(line);
		if (line.equals("record")) mode = MODE_RECORD;
		else if (line.equals("play")) mode = MODE_PLAY;
		else if (line.equals("help")) help();
		else if (line.equals("clear")) clear();
		else if (line.equals("stop")) stop();
		else if (line.equals("song")) printSegment();
		else if (line.equals("seg")) isPressed = !isPressed; // Switch song segment
		else error();
	}
    if (mode == MODE_IDLE){
		clearScreen();
		str2LCD("MAIN MENU");
	}
	key = keypad.getKey();

	keyWasIdle = keyIdle;
	keyIdle = keypad.getState() == IDLE;

	if (key == 'D') updateDuty(analogRead(IR_PIN));
	
	switch (mode) {
	case MODE_RECORD:
		record();
		break;
	case MODE_PLAY:
		play();
		break;
	case MODE_IDLE:
		//Serial.println(frequency);
		break;
	default:
		mode = MODE_IDLE;
	}
}

//Bullet 4
void updateLED(){
	static long lastMilli = 0;

	float pause = 1000 / ((float)frequency / 2000.0) / 2;
	

	//Toggle the led state
	if(millis() - (lastMilli + pause) > 0)
	{
		/*
		Serial.println(toggle);
		if (toggle) {

			setPWM(PWM_FREQ, dutyCycle);
			Serial.println("ON");
		}
		else {
			Serial.println("OFF");
			TCCR1B &= 0xF8;
		}
		*/

		TCCR1A ^= _BV(COM1A1);

		lastMilli = millis();
		toggle = !toggle;
	}
}

//Menu
void help()
{
	Serial.println("HELP");
	clearScreen();
	str2LCD("HELP");
}

//Menu
void error()
{
	bluetooth.println("ERROR: Invalid command");
	clearScreen();
	str2LCD("ERROR:");
	setCursorPos(0x40);
	str2LCD("Invalid command");
}

//Bullet 1
void record()
{
	
	//Serial.println("RECORDING");
	clearScreen();
	str2LCD("RECORDING");
	setCursorPos(0x40);
	if (keyWasIdle && !keyIdle) { // Rising Edge
		Serial.println(key);
		char2LCD(key);
		keyCount++;
		if(keyCount >=15){
			clearScreen();
			str2LCD("RECORDING");
			setCursorPos(0x40);
			keyCount = 0;
		}
		key = key - ASCII0;
		if (key >= 1 && key <= TONES) {
			pressTime = millis();
			song[segment][note].tone = key - 1;
			tone(SPEAKER_PIN, tones[key - 1]);
			measuring = true;
		}
	}
	else if (!keyWasIdle && keyIdle) { // Falling Edge
		if (measuring) {
			song[segment][note].duration = millis() - pressTime;

			Serial.println("Falling Edge");
			/*
			Serial.print("Saving note ");
			Serial.print(note);
			Serial.print(" as tone ");
			Serial.print(song[segment][note].tone);
			Serial.println(" and duration ");
			Serial.print(song[segment][note].duration);
			*/
			note++;
			if (note >= NOTES) {
				note = 0;
				mode = MODE_IDLE;
				Serial.println("DONE RECORDING");
				clearScreen();
				str2LCD("DONE RECORDING");
			}
			noTone(SPEAKER_PIN);
			measuring = false;
		}
	}
}

//Bullet 2
void play()
{
	Serial.println("PLAY");
	clearScreen();
	if(isPressed == false){
		str2LCD("PLAYING SONG 1");
	}
	else if(isPressed == true){
		str2LCD("PLAYING SONG 1");
	}
	
	if (millis() >= stopTime) {
		if (song[segment][note].duration == 0) {
			mode = MODE_IDLE;
		}
		else {
			Note n = song[segment][note];
			int toneNum = n.tone;
			Serial.println(toneNum);
			Serial.println(tones[toneNum]);
			// Plays note duration according to frequency input
			tone(SPEAKER_PIN, tones[toneNum], (n.duration*2*pause/1000));
			stopTime = millis() + (n.duration*2*pause/1000);
			note++;
			if (note >= NOTES) {
				note = 0;
				mode = MODE_IDLE;
			}
		}
	}
	// Shut off speaker
	else if (millis() >= stopTime) {
		note = 0;
		noTone(SPEAKER_PIN);
	}
}


//QOL
void stop()
{
	Serial.println("STOP");
	clearScreen();
	str2LCD("STOP");
	setCursorPos(0X40);
	switch (mode) {
	case MODE_RECORD:
		bluetooth.println("Leaving record mode");
		str2LCD("Leaving RECORD");
		break;
	case MODE_PLAY:
		bluetooth.println("Leaving playback mode");
		str2LCD("Leaving PLAYBACK");
		break;
	default:
		bluetooth.println("Nothing to stop, returning to idle");
		clearScreen();
		str2LCD("Nothing to stop,");
		setCursorPos(0X40);
		str2LCD("Return to idle");
	}

	mode = MODE_IDLE;
}

//Bullet 3 partial
void printLCD(){}

//Menu
void clear(){}

void freqCalc(){
	static unsigned long last = 0;
	frequency = 1000000.0 / (micros() - last);
	last = micros();
}

#pragma region LCD Functions
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
 	} 
	else 
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
  		}
		else 
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
	char2LCD(LCD_DATA, LCD_CLK, LCD_SS, ch);
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
	pinMode(LCD_DATA, OUTPUT);
	pinMode(LCD_CLK, OUTPUT);
	pinMode(LCD_SS, OUTPUT); 

	turnBlinkingCursorOn();
}

#pragma endregion

#pragma region Play-Record Helper Funcs
void initSeg(int seg) {
	for (int j = 0; j < NOTES; j++) {
		song[seg][j].duration = 0;
		song[seg][j].tone = 0;
	}
}

void printSegment() {
	bluetooth.print("Current segment: ");
	bluetooth.println(segment);
	for (int j = 0; j < SEGMENTS; j++) {
		bluetooth.print("\nSegment ");
		bluetooth.println(j);
		for (int i = 0; i < NOTES; i++) {
			Note note = song[j][i];
			if (note.duration == 0) {
				if (i == 0) bluetooth.println("EMPTY");
				break;
			}
			bluetooth.print("Tone: ");
			bluetooth.print(note.tone + 1);
			bluetooth.print("\tDuration: ");
			bluetooth.println(note.duration);
		}
	}
}
#pragma endregion

#pragma region PWM Funcs
void initPWM() {
	TCCR1A = _BV(COM1A1) | _BV(WGM11); //_BV(COM1B1) |
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
	updateDuty(duty);
	/*
	//print data
	Serial.println("Setting Frequency and Duty");
	Serial.print("TOP:");
	Serial.print(TOP);

	Serial.print("\tDUTY:");
	Serial.print(DUTY);

	Serial.print("\tSCALE:");
	Serial.println(scale);
	*/
}

void updateDuty(double duty) {
	DUTY = (duty / 100) * TOP;
	Serial.println("UPDATE DUTY CYCLE");
}
#pragma endregion
