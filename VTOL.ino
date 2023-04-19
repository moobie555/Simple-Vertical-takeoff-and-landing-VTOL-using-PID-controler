#include <PID_v1.h>

// Motor control pins
const int motorPin1 = 2;
const int motorPin2 = 4;
const int motorenable = 3;

// Potentiometer pins
const int setpointPotPin = A0;
const int inputPotPin = A1;
const int kpPotPin = A2;

// PID variables
double setpoint, input, output;
double kp, ki, kd;

// PID controller
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);

  // Start the PID controller
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
}

void loop() {
  // Read potentiometer values
  setpoint = map(analogRead(setpointPotPin), 0, 1023, 0, 255);
  input = map(analogRead(inputPotPin), 0, 1023, 0, 255);
  kp = map(analogRead(kpPotPin), 0, 1023, 0, 255) / 100.0; // Adjust scaling as needed

  // Update PID gains
  myPID.SetTunings(kp, ki, kd);

  // Compute PID
  myPID.Compute();

  // Control motor
  analogWrite(motorenable, output);
  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);
}
