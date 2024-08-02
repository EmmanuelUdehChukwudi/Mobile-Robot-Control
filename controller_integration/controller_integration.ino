#include <math.h>
#include <MPU6050_tockn.h>
#include <Wire.h>


MPU6050 mpu6050(Wire);
float orientation = 0;
#define LEFT_ENCODER_A 23
#define LEFT_ENCODER_B 17    //22
#define RIGHT_ENCODER_A 16 //21
#define RIGHT_ENCODER_B 19

#define R_speed_pin 27
#define R_dir_1_pin 14
#define R_dir_2_pin 13

#define L_speed_pin 26
#define L_dir_1_pin 33
#define L_dir_2_pin 25

float wheel_diameter = 8.4; // cm

volatile long int left_count = 0;
volatile long int left_count_spd = 0;
volatile long int right_count = 0;
volatile long int right_count_spd = 0;

int left_encoder_CPR = 207;
int right_encoder_CPR = 207;
int desired_count = 1;

// Odometry variables
volatile long int del_right_count = 0;
volatile long int last_right_count = 0;
volatile long int del_left_count = 0;
volatile long int last_left_count = 0;
float wheel_separation = 31.6; // cm
float left_wheel_distance = 0;
float right_wheel_distance = 0;
float robot_distance = 0;
float x = 0;
float y = 0;
float phi = 0;

// controller parameters
float set_x = 0;
float set_y = 100;
float goal_tolerance = 5.00;
float V = 0;
float W = 0; 
float error_V = 0;
float previous_error = 0;
double proportional_term = 0;
double integral_term = 0;
double derivative_term = 0;
double output = 0;

// orientation controller
float Xd = 100;
float Yd = -200;
float Kp_phi = 30;
float Ki_phi = 0;
float Kd_phi = 0.01;
float error_phi = 0;
float phi_d = 0;

// velocity controller
float Kp_v = 30;
float Ki_v = 0;
float Kd_v = 0.1;
float error_vx = 0;
float error_vy = 0;
float error_v = 0;
int left_dir = 1;
int right_dir = 1;

volatile unsigned current_time = 0;
volatile unsigned previous_time = 0;
volatile unsigned delta_time = 0;
volatile unsigned sample_time = 100; // ms

volatile unsigned L_enc_cur_time = 0;
volatile unsigned L_enc_prev_time = 0;
volatile unsigned L_enc_del_time = 0;
double freq_l = 0;
double Wl = 0;
int Vl = 0;

volatile unsigned R_enc_cur_time = 0;
volatile unsigned R_enc_prev_time = 0;
volatile unsigned R_enc_del_time = 0;
double freq_r = 0;
double Wr = 0;
int Vr = 0;

const int filterSize = 5;
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

  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  delay(5000); // Initial delay
}

void loop() {
  mpu6050.update();
  current_time = millis();
  delta_time = current_time - previous_time;

  if (delta_time >= sample_time) 
  {
    phi_d = atan2(Yd - y, Xd - x);
    R_enc_del_time = millis() - R_enc_prev_time;
    L_enc_del_time = millis() - L_enc_prev_time;

    // Compute wheel velocities
    Wl = desired_count * (2 * PI * freq_l) / left_encoder_CPR;
    Vl = Wl * (wheel_diameter) / 2;
    Wr = desired_count * (2 * PI * freq_r) / right_encoder_CPR;
    Vr = Wr * (wheel_diameter) / 2;
    
    Odometry(); 
//    angular velocity controller
//    error_phi = phi_d - phi;
    orientation = mpu6050.getAngleZ()*PI/180;
    error_phi = phi_d - orientation;
    double phi_output = Controller(error_phi, Kp_phi, Ki_phi, Kd_phi);
    W = phi_output; // robots angular velovity as computed by controller

//    Velocity controller
    error_vx = Xd - x;
    error_vy = Yd - y;
    error_v = sqrt( (error_vx)*(error_vx) + (error_vy)*(error_vy) );
    int v_out = Controller(error_v,Kp_v,Ki_v,Kd_v);
    v_out = constrain(v_out,-180,180);

    Vl = v_out - (W * wheel_separation)/2 ;
    Vr = v_out + (W * wheel_separation)/2 ;

    Vl = constrain(Vl,-180,180);
    Vr = constrain(Vr,-180,180);
//    Vl = movingAverageFilter(Vl, leftVelocities, leftIndex, leftVelocitySum);
//    Vr = movingAverageFilter(Vr, rightVelocities, rightIndex, rightVelocitySum);


    if(error_vx <= goal_tolerance && error_vy <= goal_tolerance)
    {
          left_dir = 0;
          right_dir = 0;
          Vl = 0;
          Vr = 0;
          freq_r = 0;
          freq_l = 0;
          MoveMotor_L(Vl,left_dir);
          MoveMotor_R(Vr,right_dir);
    }

    else
    {
          MoveMotor_L(Vl,left_dir);
          MoveMotor_R(Vr,right_dir);
    }

    Serial.print(Vr);
    Serial.print(",");
    Serial.print(Vl);
    Serial.print("-----   ");
    Serial.print(x);
    Serial.print(",");
    Serial.println(y);
    sendOdometry();
//    Serial.println(orientation);
    previous_time = current_time;
    
  }
}

double Controller(float error, float Kp, float Ki, float Kd)
{
  proportional_term = Kp * error;
  integral_term += Ki * error * (sample_time / 1000.0);
  derivative_term = Kd * (error - previous_error) / (sample_time / 1000.0);
  output = proportional_term + integral_term + derivative_term;
  previous_error = error;
  return output;
}
void SendPhiControllerOutput()
{
    Serial.print(phi_d);
    Serial.print(",");
    Serial.print(phi);
    Serial.print(",");
    Serial.print(error_phi);
    Serial.print(",");
    Serial.println(W);
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
  Serial.print(",");
  Serial.println(phi);
}

void sendPositionError() {
  Serial.print(x - set_x);
  Serial.print(",");
  Serial.println(y - set_y);
}

void do_left_motor() {
  int b = digitalRead(LEFT_ENCODER_B);
  if (b > 0) {
    left_count--;
  } else {
    left_count++;
  }
  left_count_spd++;
  if (abs(left_count_spd) == desired_count) {
    if (L_enc_del_time > 0) {
      freq_l = 1000.0 / (double)L_enc_del_time;
    }
    left_count_spd = 0;
    L_enc_prev_time = millis();
  }
}

void do_right_motor() {
  int b = digitalRead(RIGHT_ENCODER_B);
  if (b > 0) {
    right_count++;
  } else {
    right_count--;
  }
  right_count_spd++;
  if (abs(right_count_spd) == desired_count) {
    if (R_enc_del_time > 0) {
      freq_r = 1000.0 / (double)R_enc_del_time;
    }
    right_count_spd = 0;
    R_enc_prev_time = millis();
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
  return runningSum / filterSize;
}

void Odometry() {
  del_right_count = right_count - last_right_count;
  del_left_count = left_count - last_left_count;
  right_wheel_distance = (del_right_count * PI * (wheel_diameter )) / right_encoder_CPR;
  left_wheel_distance = (del_left_count * PI * (wheel_diameter )) / left_encoder_CPR;

  robot_distance = (right_wheel_distance + left_wheel_distance) / 2;

  x += robot_distance * cos(phi);
  y += robot_distance * sin(phi);

  phi += (right_wheel_distance - left_wheel_distance) / wheel_separation;
  phi = atan2(sin(phi), cos(phi)); // keep phi between pi and -pi 

  last_right_count = right_count;
  last_left_count = left_count;
}
