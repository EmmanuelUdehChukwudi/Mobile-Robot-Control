#include <MPU6050_tockn.h>
#include <Wire.h>

#define LEFT_ENCODER_A 23
#define LEFT_ENCODER_B 17 //22

#define RIGHT_ENCODER_A 16 // 21
#define RIGHT_ENCODER_B 19

volatile long int left_count = 0;
volatile long int right_count = 0;
MPU6050 mpu6050(Wire);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LEFT_ENCODER_A, INPUT);
  pinMode(LEFT_ENCODER_B, INPUT);
  pinMode(RIGHT_ENCODER_A, INPUT);
  pinMode(RIGHT_ENCODER_B, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A), do_left_motor, RISING);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_B), do_left_motor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_A), do_right_motor, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_B), do_right_motor, CHANGE);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

}

void loop() {
  mpu6050.update();
  Serial.print("angleZ : ");
  Serial.print(mpu6050.getAngleZ()*PI/180);
  Serial.print("   Left Count: ");
  Serial.print(left_count);
  Serial.print("   Right Count: ");
  Serial.println(right_count);
}

void do_left_motor() {
  int b = digitalRead(LEFT_ENCODER_B);
  if (b > 0) {
    left_count--;
  } else {
    left_count++;
  }
  
}

void do_right_motor() {
  int b = digitalRead(RIGHT_ENCODER_B);
  if (b > 0) {
    right_count++;
  } else {
    right_count--;
  }
  
}
