#include <math.h>

#define LEFT_ENCODER_A 23
#define LEFT_ENCODER_B 22
#define RIGHT_ENCODER_A 21
#define RIGHT_ENCODER_B 19

#define R_speed_pin 27
#define R_dir_1_pin 14
#define R_dir_2_pin 13

#define L_speed_pin 26
#define L_dir_1_pin 33
#define L_dir_2_pin 25

float wheel_diameter = 8.4; // meters

long int left_count = 0;
long int left_count_spd = 0;
long int right_count = 0;
long int right_count_spd = 0;

int left_encoder_CPR = 207;
int right_encoder_CPR = 207;
int desired_count = 1;

// Odometry variables
long int del_right_count = 0;
long int last_right_count = 0;
long int del_left_count = 0;
long int last_left_count = 0;
float wheel_seperation = 31.6; // centimeters
float left_wheel_distance = 0;
float right_wheel_distance = 0;
float robot_distance = 0;
float x = 0;
float y = 0;
float phi = 0;

// controller parameters
float set_x = 0;
float set_y = 100;
float distance_tolerance = 5.00 ;
float V = 0;
float W = 0; 
float error = 0;
float kp = 10;
int PWML = 0;
int PWMR = 0;
int MAXPWM = 180;
int MINPWM = 0;

volatile unsigned current_time = 0;
volatile unsigned previous_time = 0;
volatile unsigned delta_time = 0;
volatile unsigned sample_time = 100; // in milliseconds

volatile unsigned L_enc_cur_time = 0;
volatile unsigned L_enc_prev_time = 0;
volatile unsigned L_enc_del_time = 0;
double freq_l = 0;
double Wl = 0;
double Vl = 0;

volatile unsigned R_enc_cur_time = 0;
volatile unsigned R_enc_prev_time = 0;
volatile unsigned R_enc_del_time = 0;
double freq_r = 0;
double Wr = 0;
double Vr = 0;

const int filterSize = 20;
double leftVelocities[filterSize];
double rightVelocities[filterSize];
int leftIndex = 0;
int rightIndex = 0;
double leftVelocitySum = 0;
double rightVelocitySum = 0;

void setup() {
  Serial.begin(115200);

  pinMode(R_speed_pin, OUTPUT);
  pinMode(R_dir_1_pin, OUTPUT);
  pinMode(R_dir_2_pin, OUTPUT);

  pinMode(L_speed_pin, OUTPUT);
  pinMode(L_dir_1_pin, OUTPUT);
  pinMode(L_dir_2_pin, OUTPUT);
  
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), do_left_motor, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), do_left_motor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), do_right_motor, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), do_right_motor, CHANGE);
  
  for (int i = 0; i < filterSize; i++) {
    leftVelocities[i] = 0;
    rightVelocities[i] = 0;
  }

  delay(5000);
}

void loop() {
  current_time = millis();
  R_enc_cur_time = millis();
  L_enc_cur_time = millis();
  delta_time = current_time - previous_time;

  if (delta_time >= sample_time) {

    float set_phi = atan2(set_y-y,set_x-x);
    R_enc_del_time = R_enc_cur_time - R_enc_prev_time;
    L_enc_del_time = L_enc_cur_time - L_enc_prev_time;

    // Compute wheel velocities
    Wl = desired_count * (2 * PI * freq_l) / left_encoder_CPR;
    Vl = Wl * wheel_diameter / 2;
    Wr = desired_count * (2 * PI * freq_r) / right_encoder_CPR;
    Vr = Wr * wheel_diameter / 2;
//    V = (Vl+Vr)/2.0; 
    V = 200;
    error = set_phi - phi;
    W = (Vr-Vl)/wheel_seperation + kp*error;
    PWMR = V + (W*wheel_seperation)/2;
    PWML = V - (W*wheel_seperation)/2;
    
    if(PWMR>MAXPWM)
    {
      PWMR = MAXPWM;
    }
    if(PWMR<MINPWM)
    {
      PWMR = MINPWM;
    }
    if(PWML>MAXPWM)
    {
      PWML = MAXPWM;
    }
    if(PWML<MINPWM)
    {
      PWML = MINPWM;
    }

    if(abs(x-set_x) < distance_tolerance && abs(y-set_y) < distance_tolerance)
    {
      MoveMotor_R(0, 0);
      MoveMotor_L(0, 0);
    }
    else
    {
      MoveMotor_R(PWMR, 1);
      MoveMotor_L(PWML, 1);
    }

    Odometry();
    sendPositionError();
    previous_time = current_time;
  }
  
}

void sendFilteredVelocity() {
  double filteredVl = movingAverageFilter(Vl, leftVelocities, leftIndex, leftVelocitySum);
  double filteredVr = movingAverageFilter(Vr, rightVelocities, rightIndex, rightVelocitySum);
  Serial.print(filteredVr);
  Serial.print(",");
  Serial.println(filteredVl);
}

void sendRawVelocity() {
  Serial.print(Vr);
  Serial.print(",");
  Serial.println(Vl);
}

void sendOdometry() {
  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
//  Serial.println(y);
  Serial.print(",");
  Serial.println(phi);
}

void sendPositionError() {
  Serial.print(x-set_x);
  Serial.print(",");
  Serial.println(y-set_y);
}

void do_left_motor() {
  int b = digitalRead(LEFT_ENCODER_B);
  if (b > 0) {
    left_count--;
    left_count_spd--;
    if (abs(left_count_spd) == desired_count) {
      if (L_enc_del_time > 0) {
        freq_l = (1000) / (double)L_enc_del_time;
      }
      left_count_spd = 0;
      L_enc_prev_time = L_enc_cur_time;
    }
  } else {
    left_count++;
    left_count_spd++;
    if (abs(left_count_spd) == desired_count) {
      if (L_enc_del_time > 0) {
        freq_l = (1000) / (double)L_enc_del_time;
      }
      left_count_spd = 0;
      L_enc_prev_time = L_enc_cur_time;
    }
  }
}

void do_right_motor() {
  int b = digitalRead(RIGHT_ENCODER_B);
  if (b > 0) {
    right_count++;
    right_count_spd++;
    if (abs(right_count_spd) == desired_count) {
      if (R_enc_del_time > 0) {
        freq_r = (1000) / (double)R_enc_del_time;
      }
      right_count_spd = 0;
      R_enc_prev_time = R_enc_cur_time;
    }
  } else {
    right_count--;
    right_count_spd--;
    if (abs(right_count_spd) == desired_count) {
      if (R_enc_del_time > 0) {
        freq_r = (1000) / (double)R_enc_del_time;
      }
      right_count_spd = 0;
      R_enc_prev_time = R_enc_cur_time;
    }
  }
}

void MoveMotor_L(int speed, int dir) {
  if (dir == 1) { // forward
    digitalWrite(L_dir_1_pin, LOW);
    digitalWrite(L_dir_2_pin, HIGH);
  } else if (dir == -1) { // reverse
    digitalWrite(L_dir_1_pin, HIGH);
    digitalWrite(L_dir_2_pin, LOW);
  } else if (dir == 0) {
    digitalWrite(L_dir_1_pin, LOW);
    digitalWrite(L_dir_2_pin, LOW);
  }
  analogWrite(L_speed_pin, speed);
}

void MoveMotor_R(int speed, int dir) {
  if (dir == 1) { // forward
    digitalWrite(R_dir_1_pin, LOW);
    digitalWrite(R_dir_2_pin, HIGH);
  } else if (dir == -1) { // reverse
    digitalWrite(R_dir_1_pin, HIGH);
    digitalWrite(R_dir_2_pin, LOW);
  } else if (dir == 0) {
    digitalWrite(R_dir_1_pin, LOW);
    digitalWrite(R_dir_2_pin, LOW);
  }
  analogWrite(R_speed_pin, speed);
}

double movingAverageFilter(double newValue, double* values, int& currentIndex, double& runningSum) {
  runningSum -= values[currentIndex];
  values[currentIndex] = newValue;
  runningSum += newValue;
  currentIndex = (currentIndex + 1) % filterSize;
  double average = runningSum / filterSize;
  return average;
}

void Odometry() {
  del_right_count = right_count - last_right_count;
  del_left_count = left_count - last_left_count;
  right_wheel_distance = (del_right_count * PI * wheel_diameter) / (double)right_encoder_CPR;
  left_wheel_distance = (del_left_count * PI * wheel_diameter) / (double)left_encoder_CPR;

  robot_distance = (right_wheel_distance + left_wheel_distance) / 2;

  x = x + robot_distance * cos(phi);
  y = y + robot_distance * sin(phi);

  phi = phi + (right_wheel_distance - left_wheel_distance) / wheel_seperation;
  phi = atan2(sin(phi), cos(phi)); // keep phi between pi and -pi 
  

  last_right_count = right_count;
  last_left_count = left_count;
}
