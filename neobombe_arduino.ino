#include <AccelStepper.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"

#define TOP (0)
#define MIDDLE (1)
#define BOTTOM (2)
#define TYPE (BOTTOM)

#define NUM_SHIELDS (TYPE + 1)
#define NUM_MOTORS (NUM_SHIELDS * 2)

#define LED_PIN (13)

Adafruit_MotorShield AFMS[NUM_SHIELDS];
Adafruit_StepperMotor *motors[NUM_MOTORS];

#if TYPE == TOP
AccelStepper accels[NUM_MOTORS] = {
	AccelStepper(forwardstep_1, backwardstep_1),
	AccelStepper(forwardstep_2, backwardstep_2),
};
#endif

#if TYPE == TOP
	#define MOTOR_SPEED (300.0)
#elif TYPE == MIDDLE
	#define MOTOR_SPEED (20)
#elif TYPE == BOTTOM
	#define MOTOR_SPEED (60)
#endif

#define MOTOR_DELAY (1000)

static bool isProcessing;

static int index; // current motor that _should_ be moving (only used for BOTTOM)
static long loggedTime; // (only used for BOTTOM)

void setup() {
	Serial.begin(9600);
	Serial.write(TYPE);

	for (int i = 0; i < NUM_SHIELDS; i++) {
		AFMS[i] = Adafruit_MotorShield(0x60 + i);
		AFMS[i].begin();
		motors[i * 2] = AFMS[i].getStepper(200, 1);
		motors[i * 2 + 1] = AFMS[i].getStepper(200, 2);
	}

	TWBR = ((F_CPU / 400000l) - 16) / 2; // Change the i2c clock to 400KHz

	#if TYPE == TOP
		accels[0].setSpeed(MOTOR_SPEED); //steps per second
		accels[1].setSpeed(MOTOR_SPEED);
	#else
		for (int i = 0; i < NUM_MOTORS; i++) {
			motors[i]->setSpeed(MOTOR_SPEED);
		}
	#endif

	Serial.flush();

	// LED for debugging purpose
	pinMode(LED_PIN, OUTPUT);
	digitalWrite(LED_PIN, LOW);
}

void loop() {
	if (Serial.available() > 0) {
		int b = Serial.read();
		if (b == 65) {
			isProcessing = true;
			digitalWrite(LED_PIN, HIGH);
		} else if (b == 66) {
			isProcessing = false;
			for (int i = 0; i < NUM_MOTORS; i++) {
				motors[i]->release();
			}
			digitalWrite(LED_PIN, LOW);
			index = 0;
		}
	}

	if (isProcessing) {
		process();
	}
}

#if TYPE == TOP
void process() {
	for (int i = 0; i < NUM_MOTORS; i++) {
		accels[i].runSpeed();
	}
}

#elif TYPE == MIDDLE
void process() {
	for (int i = 0; i < NUM_MOTORS; i++) {
		motors[i]->step(8, FORWARD, SINGLE);
	}
}

#elif TYPE == BOTTOM
void process() {
	if (millis() - loggedTime > MOTOR_DELAY) {
		motors[index]->step(8, FORWARD, SINGLE);
		loggedTime = millis();
		index = (index + 1) % NUM_MOTORS;
	}
}
#endif

#if TYPE == TOP
void forwardstep_1() {
  motors[0]->onestep(FORWARD, SINGLE);
}
void backwardstep_1() {
  motors[0]->onestep(BACKWARD, SINGLE);
}

void forwardstep_2() {
  motors[1]->onestep(FORWARD, SINGLE);
}
void backwardstep_2() {
  motors[1]->onestep(BACKWARD, SINGLE);
}
#endif
