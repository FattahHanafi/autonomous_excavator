#include <AutoPID.h>

#define PWM_PORT PORTB  // Pins 10 - 13
#define DIR_PORT PORTC  // Pins 30 - 37
#define SEN_PORT PORTF  // Pins A0 - A7
#define NUM_ACT 4       // Number of actuators

double target[NUM_ACT];
double actual[NUM_ACT];
double voltage[NUM_ACT];

#define KP 3.5d
#define KI 0.0d
#define KD 5.5d
#define MIN_VOLTAGE FROM_LOW + 10

#define OUTPUT_MIN -150.0d
#define OUTPUT_MAX 150.0d

#define FROM_LOW 25.0d    // Minimum value of sensor
#define FROM_HIGH 882.0d  // Maximum value of sensor
#define TO_LOW 408.0d     // Stroke related to minimum value of sensor
#define TO_HIGH 600.0d    // Stroke related to maximum value of sensor

AutoPID PID_1(&actual[0], &target[0], &voltage[0], OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID PID_2(&actual[1], &target[1], &voltage[1], OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID PID_3(&actual[2], &target[2], &voltage[2], OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);
AutoPID PID_4(&actual[3], &target[3], &voltage[3], OUTPUT_MIN, OUTPUT_MAX, KP, KI, KD);

const int pwm_pins[] = { 13, 12, 11, 10 };
const int sen_pins[] = { A0, A1, A2, A3 };

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  DDRB = 0xFF;  // Set as output
  DDRC = 0xFF;  // Set as output
  DDRA = 0xFF;  // Set as output
  DDRF = 0x00;  // Set as input

  PID_1.setBangBang(50.0);
  PID_2.setBangBang(50.0);
  PID_3.setBangBang(50.0);
  PID_4.setBangBang(50.0);

  PID_1.setTimeStep(1);
  PID_2.setTimeStep(1);
  PID_3.setTimeStep(1);
  PID_4.setTimeStep(1);

  PWM_PORT = 0x00;
  DIR_PORT = 0x00;
  for (uint8_t i = 0; i < NUM_ACT; i++) {
    analogWrite(pwm_pins[i], 0.0);
  }
}

void checkSerial() {

  if (Serial.available() < 6)  {
    return;
  }

  for (uint8_t i = 0; i < NUM_ACT; ++i) {
    target[i] = double(Serial.parseFloat());
  }
}

void readSensors() {
  for (uint8_t i = 0; i < NUM_ACT; ++i) {
    actual[i] = double(map(analogRead(sen_pins[i]), FROM_LOW, FROM_HIGH, TO_LOW, TO_HIGH));
  }
}

uint8_t direction;

void loop() {
  checkSerial();
  readSensors();

  PID_1.run();
  PID_2.run();
  PID_3.run();
  PID_4.run();

  direction = 0;
  for (uint8_t i = 0; i < NUM_ACT; ++i) {
    Serial.print(actual[i]);
    Serial.print(" ");
    // Serial.print(target[i]);
    // Serial.print(" ... ");

    if (voltage[i] >= 0) {
      direction |= (0b01) << (2 * i);
    }
    else {
      direction |= (0b10) << (2 * i);
    }
  }
  Serial.print('\n');

  DIR_PORT = direction;
  for (uint8_t i = 0; i < NUM_ACT; i++) {
    analogWrite(pwm_pins[i], (abs(voltage[i]) > MIN_VOLTAGE) ?  int(map(abs(voltage[i]), TO_LOW, TO_HIGH, FROM_LOW, FROM_HIGH)) : 0);
  }
  delay(1);
}