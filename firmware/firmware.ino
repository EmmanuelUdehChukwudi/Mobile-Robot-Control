
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

float wheel_diameter = 8.4; // centimeters

long int left_count = 0;
long int right_count = 0;

int left_encoder_CPR = 207;
int right_encoder_CPR = 207;

volatile unsigned current_time = 0;
volatile unsigned previous_time = 0;
volatile unsigned delta_time = 0;
volatile unsigned sample_time = 100; /// in milliseconds

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
  
  pinMode(LEFT_ENCODER_A,INPUT);
  pinMode(LEFT_ENCODER_B,INPUT);
  pinMode(RIGHT_ENCODER_A,INPUT);
  pinMode(RIGHT_ENCODER_B,INPUT);
  
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
  R_enc_cur_time = millis();
  L_enc_cur_time = millis();

  double filteredVl = movingAverageFilter(Vl, leftVelocities, leftIndex, leftVelocitySum);
  double filteredVr = movingAverageFilter(Vr, rightVelocities, rightIndex, rightVelocitySum);

  Serial.print(filteredVr);
  Serial.print(",");
  Serial.print(filteredVl);
  Serial.print(",");
  Serial.print(Vr);
  Serial.print(",");
  Serial.println(Vl);
  MoveMotor_R(100,1);
  MoveMotor_L(100,1);
}

void do_left_motor()
{
  L_enc_del_time = L_enc_cur_time - L_enc_prev_time;
  L_enc_prev_time = L_enc_cur_time;
//  int b = digitalRead(LEFT_ENCODER_B);
//  if(b > 0)
//  {
//    left_count--;
//  }
//  else
//  {
//    left_count++;
//  }
  if(L_enc_del_time)
  {
    freq_l = (1000)/(double) L_enc_del_time;
    Wl = (2 * PI * freq_l)/ left_encoder_CPR;
    Vl = Wl * wheel_diameter/2;
  }
}

void do_right_motor()
{
  R_enc_del_time = R_enc_cur_time - R_enc_prev_time;
  R_enc_prev_time = R_enc_cur_time;
//  int b = digitalRead(RIGHT_ENCODER_B);
//  if(b > 0)
//  {
//    right_count++;
//  }
//  else
//  {
//    right_count--;
//  }

  if(R_enc_del_time>0)
  {
    freq_r = (1000)/(double) R_enc_del_time;
    Wr = (2 * PI * freq_r)/ right_encoder_CPR;
    Vr = Wr * wheel_diameter/2;
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
