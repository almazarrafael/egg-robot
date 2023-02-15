#include "Wire.h"
#include <MPU6050_light.h>

// MPU
MPU6050 mpu(Wire);

// Pin definitions
#define enA 11 // PWM 
#define in1 12 // Forward
#define in2 13 // Backward

#define enB 3 // PWM
#define in3 4 // Backward
#define in4 5 // Forward

// PID values
float yRotation; // PID input
int output;
float setpoint = 0;
int sampleTime = 10;
int outMin = -255;
int outMax = 255;

// PID calculation
unsigned long lastTime;
float errSum;
float lastErr;

// PID tuning
double kp = 100; // Ku = 35
double ki = 0.5 * sampleTime;
double kd = 0.05 / sampleTime;

bool fallDetected = false;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    // MPU
    Serial.println("MPU6050 calibration: START");
    byte status = mpu.begin();
    while (status != 0) { }
    delay(1000);
    mpu.calcOffsets();
    Serial.println("MPU6050 calibration: END");
    
    // Motor A
    pinMode(enA, OUTPUT);
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);

    // Motor B
    pinMode(enB, OUTPUT);
    pinMode(in3, OUTPUT);
    pinMode(in4, OUTPUT);
}

void loop() {
  while (true) {
    mpu.update();
    yRotation = mpu.getAngleY();

    // Stops moving when the angle gets too steep and unrecovarable
    if (abs(yRotation) > 30) {
      motor_control(0);
      break;
    }
    
    Serial.print("Y: "); Serial.println(yRotation);
    
    output = PID_compute(yRotation);
    Serial.println(output);
    
    motor_control(output);
  }
}

int PID_compute (float input) {
  int output;
  double iTerm;
  
  unsigned long now = millis();
  int timeChange = (double)(now - lastTime);

  if (timeChange >= sampleTime) {
    double error = setpoint - input;
    errSum += error;
    double dErr = (error - lastErr);

    iTerm = ki*errSum;
    // reset windup mitigation: integral
    if (iTerm > outMax) iTerm = outMax;
    else if (iTerm < outMin) iTerm = outMin;

    output = (kp*error) + (iTerm) + (kd*dErr);
    // reset windup mitigation: output
    if (output > outMax) output = outMax;
    else if (output < outMin) output = outMin;

    lastErr = error;
    lastTime = now;
  }
  
  return output;
}

void motor_control(int val) {
    // Val should range from -255 to 255, but just in case
    val = constrain(val, -255, 255);

    // Values less than 30 doesn't make the motor move
    if (abs(val) < 30) {
      val = 0;
    }
    
    if (val > 0) {
        digitalWrite(in1, HIGH);
        digitalWrite(in4, HIGH);

        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);

        analogWrite(enA, val);
        analogWrite(enB, val);
    }
    else if (val < 0) {
        digitalWrite(in1, LOW);
        digitalWrite(in4, LOW);

        digitalWrite(in2, HIGH);
        digitalWrite(in3, HIGH);

        analogWrite(enA, -1*val);
        analogWrite(enB, -1*val);
    }
    else {
        digitalWrite(in1, LOW);
        digitalWrite(in4, LOW);

        digitalWrite(in2, LOW);
        digitalWrite(in3, LOW);

        analogWrite(enA, 0);
        analogWrite(enB, 0);
    }
}
