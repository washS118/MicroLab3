/*
#include <Keypad.h>
#include <Key.h>

//Button Settings
#define BUTTON 0
#define DB_WAIT 10

//Speaker settings
#define SPEAKER 7
#define TONES 8
#define NOTE_DELAY 40
float tones[TONES] = { 261.6, 293.6, 329.6, 349.2, 392, 440, 493.8, 523.2 };

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
byte rowPins[ROWS] = { 5, 4, 3, 2 }; //connect to the row pinouts of the keypad
byte colPins[COLS] = { 13, 12, 11, 10 }; //connect to the column pinouts of the keypad

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

void setup()
{
	for (int i = 0; i < SEGMENTS; i++) {
		initSeg(i);
	}
	pinMode(BUTTON, INPUT_PULLUP);
	Serial.begin(9600);
	Serial.println("Starting Program");
}

void loop()
{
	//Playback controlls
	static bool record = false;
	static bool playback = false;
	static bool measuring = false;
	static int segment = 0;
	static int note = 0;
	static int stopTime = 0;

	//Keypad Controlls
	static bool idle = false;
	static bool wasIdle = false;
	static int pressTime = 0;

	//Button Controlls
	static bool wasPressed = false;
	bool isPressed = getButton();

	//Switch song segment
	if (wasPressed == true && isPressed == false) {
		noTone(SPEAKER);
		record = false;
		playback = false;
		note = 0;
		segment++;
		if (segment >= SEGMENTS) {
			segment = 0;
		}

		Serial.print("Switching to segment ");
		Serial.println(segment);
	}

	wasPressed = isPressed;


	//Check device mode - record gets priority if conflict
	char key = keypad.getKey();

	wasIdle = idle;
	idle = keypad.getState() == IDLE;

	if (key == '*') {
		Serial.println("Begin Recording");
		initSeg(segment);
		record = true;
		playback = false;
		note = 0;
	}
	else if (key == '#') {
		Serial.println("Begin Playback");
		record = false;
		playback = true;
		note = 0;
	}
	else if (record) {
		if (wasIdle && !idle) { // Rising Edge
			Serial.println(key);
			key = key - ASCII0;
			if (key >= 1 && key <= TONES) {
				pressTime = millis();
				song[segment][note].tone = key - 1;
				tone(SPEAKER, tones[key - 1]);
				measuring = true;
			}
		}
		else if (!wasIdle && idle) { // Falling Edge
			if (measuring) {
				Serial.println("Falling Edge");

				song[segment][note].duration = millis() - pressTime;

				Serial.print("Saving note ");
				Serial.print(note);
				Serial.print(" as tone ");
				Serial.print(song[segment][note].tone);
				Serial.println(" and duration ");
				Serial.print(song[segment][note].duration);
				note++;
				if (note >= NOTES) {
					note = 0;
					record = false;

					printSegment(segment);
				}
				noTone(SPEAKER);
				measuring = false;
			}
		}
	}
	else if (playback) {
		if (millis() >= stopTime) {
			if (song[segment][note].duration == 0) {
				playback = false;
			}
			else {
				Note n = song[segment][note];
				int toneNum = n.tone;
				Serial.println(toneNum);
				Serial.println(tones[toneNum]);
				tone(SPEAKER, tones[toneNum], n.duration);
				stopTime = millis() + n.duration + NOTE_DELAY;
				note++;
				if (note >= NOTES) {
					note = 0;
					playback = false;
				}
			}


		}
	}
	else { // Shut off speaker
		if (millis() >= stopTime) {
			note = 0;
			noTone(SPEAKER);
		}
	}
}

void initSeg(int seg) {
	for (int j = 0; j < NOTES; j++) {
		song[seg][j].duration = 0;
		song[seg][j].tone = 0;
	}
}

bool getButton()
{
	static bool isPressed = false;
	static int pressedTime = 0;
	int currentTime = millis();
	int reading = digitalRead(BUTTON);

	if (reading != isPressed) {
		// reset the debouncing timer
		pressedTime = currentTime;
	}

	if (currentTime - pressedTime > DB_WAIT) {
		if (reading == LOW) {
			isPressed = true;
		}
		else {
			isPressed = false;
		}
	}

	return isPressed;
}

void printSegment(int seg) {
	Serial.print(song[seg][0].tone);
	for (int i = 1; i < NOTES; i++) {
		Serial.print(", ");
		Serial.print(song[seg][i].tone);
	}
}
*/