// Pin definitions
#define enA 11 // PWM 
#define in1 12 // Forward
#define in2 13 // Backward

#define enB 3 // PWM
#define in3 4 // Forward
#define in4 5 // Backward

// Tuning parameters
double kp;
double ki;
double kd;

float setpoint = 180; // 180 degrees nominal, adjust according to IMU readings when robot is straight up

void setup() {
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

}

void PID_compute () {

}

void motor_control(int val) {
    if (val > 0) {
        // forward
    }
    else if (val < 0) {
        // backward
    }
    else {
        // stop
    }
}