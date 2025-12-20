#include <Arduino.h>
#include <Servo.h>

// motor driver pins
const int pwm_pins[] = {7, 9, 11, 12};
const int dir_pins[] = {8, 10, 50, 48};

// encoder pins
const int enc_out_A[] = {18, 2, 19, 3};
const int enc_out_B[] = {20, 4, 21, 5};

const float wheel_diameter = 7.0; // cm
const float track_width = 27.65; 
const float pulses_per_revolution = 1796.0;
const bool invert_dir[] = {true, true, false, false};

const float cm_per_rev = PI * wheel_diameter;
const float pulses_per_cm = pulses_per_revolution / cm_per_rev;

Servo cr_servo; // CR servo
Servo mg_servo; // MG996R
const int cr_pin = 14;
const int mg_pin = 15;
const int cr_stop = 1500;

volatile long pulse_count[4] = {0,0,0,0};


void encoder_0();
void encoder_1();
void encoder_2();
void encoder_3();
void drive_motors(int dirs[4], int speed);
void stop_motors();
void travel(int dir, float distance_M, int speed);
void turn(int turn_dir, float secs, int speed);

void setup() {
  Serial.begin(115200);

  // motor pins
  for (int i = 0; i < 4; i++){
    pinMode(pwm_pins[i], OUTPUT);
    pinMode(dir_pins[i], OUTPUT);
    pinMode(enc_out_A[i], INPUT_PULLUP);
    pinMode(enc_out_B[i], INPUT_PULLUP);
  }

  attachInterrupt(digitalPinToInterrupt(enc_out_A[0]), encoder_0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_out_B[0]), encoder_0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_out_A[1]), encoder_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_out_B[1]), encoder_1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_out_A[2]), encoder_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_out_B[2]), encoder_2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_out_A[3]), encoder_3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(enc_out_B[3]), encoder_3, CHANGE);

  cr_servo.attach(cr_pin);
  mg_servo.attach(mg_pin);
  mg_servo.write(100); // start at 100 to prevent dragging on ground
}

void loop(){
  if (Serial.available()){
    String input = Serial.readStringUntil('\n');

    // travel forward
    if (input.startsWith("travel ")){
      int dir;
      if(input.indexOf("forward") >= 0){
        dir = 1;
      }else{
        dir = -1;
      }
      float dist = input.substring(input.lastIndexOf(' ') + 1).toFloat();
      travel(dir, dist, 200);
      Serial.println("Done");
    }
    // turn
    else if (input.startsWith("turn ")){
      int tdir;
      if(input.indexOf("right") >= 0){
        tdir = 1;
      }else{
        tdir = -1;
      }
      float secs = input.substring(input.lastIndexOf(' ') + 1).toFloat();
      turn(tdir, secs, 200);
      Serial.println("Done");
    }
    // spin CR servo forward
    else if (input.startsWith("spin ")){
      cr_servo.writeMicroseconds(1300);
      Serial.println("Done");
    }
    // Spin CR servo backward
    else if (input.startsWith("spinback ")){
      cr_servo.writeMicroseconds(1700);
      Serial.println("Done");
    }
    else if (input.startsWith("stop")){
      cr_servo.writeMicroseconds(1500);
      Serial.println("Done");
    }
    // MG996R position
    else if (input.startsWith("mg996r ")){
      int angle = input.substring(7).toInt();
      mg_servo.write(angle);
      Serial.println("Done");
    }
  }
}

void travel(int dir, float distance_M, int speed){
  long target = (long)(distance_M * 100.0 * pulses_per_cm);
  for (int i = 0; i < 4; i++){
    pulse_count[i] = 0;
  }
  int dirs[4] = {dir, dir, dir, dir};
  drive_motors(dirs, speed);
  while (true){
    bool moving = false;
    for (int i = 0; i < 4; i++){
      if (abs(pulse_count[i]) < target) moving = true;
    }
    if (!moving) break;
  }
  stop_motors();
}

void turn(int turn_dir, float secs, int speed){
  int dirs[4] = {turn_dir, turn_dir, -turn_dir, -turn_dir};
  for (int i = 0; i < 4; i++){
    pulse_count[i] = 0;
  }
  drive_motors(dirs, speed);
  delay((int)(secs * 1000));
  stop_motors();
}

void drive_motors(int directions[4], int speed){
  for (int i = 0; i < 4; i++){
      int d;
      if (invert_dir[i]){
        d = -directions[i];
      }else{
        d = directions[i];
    }
    
    if (d > 0){
      digitalWrite(dir_pins[i], HIGH);
    }else{
      digitalWrite(dir_pins[i], LOW);
    }
    analogWrite(pwm_pins[i], speed);
  }
}

void stop_motors(){
  for (int i = 0; i < 4; i++) analogWrite(pwm_pins[i], 0);
}

// update encoder counts
void encoder_0() {pulse_count[0]++;}
void encoder_1() {pulse_count[1]++;}
void encoder_2() {pulse_count[2]++;}
void encoder_3() {pulse_count[3]++;}