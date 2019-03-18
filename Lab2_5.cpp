/*
#define TOP ICR1
#define DUTY OCR1A

#define CLOCKRATE 16000000

#define PWM_PIN 9

#define PWM_FREQ 1000

#define COUNT 5
const int cutoffs[] = { 500, 100, 10, 5, 0 };
const int prescalers[] = { 1, 8, 64, 256, 1024 };

#define READ_PIN 8
long avgFreq;
float avgDuty;

#define A_PIN A0
#define TOGGLE_PIN 2

void setup()
{
	Serial.begin(9600);

	pinMode(PWM_PIN, OUTPUT);
	pinMode(READ_PIN, INPUT);
	pinMode(TOGGLE_PIN, OUTPUT);
	pinMode(A_PIN, INPUT);

	initPWM();
	setPWM(PWM_FREQ, (analogRead(A_PIN) / 1024.0) * 100);
}

void loop()
{
	static long lastMilli = 0;

	readPWM();
	float pause = 1000 / ((float)avgFreq / 2000.0) / 2;
	Serial.println(pause);

	//Set led brightness
	setDuty((analogRead(A_PIN) / 1024.0) * 100);

	//Toggle the toggle pin
	if (millis() - (lastMilli + pause) > 0) {
		digitalWrite(TOGGLE_PIN, !digitalRead(TOGGLE_PIN));
		lastMilli = millis();
	}
}

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
*/