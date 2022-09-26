#include <MPU6050_light.h>
#include <Wire.h>
MPU6050 mpu(Wire);

unsigned long timer = 0;


int ENA = 6;
int ENB = 5;
int IN1 = 2;
int IN2 = 7;
int IN3 = 12;
int IN4 = 13;


int offButton = A1;
int button;
int prevButton;
int start = 0;

int Trig = 9;
int Echo = 10;

long rawPulse;
int distance;
int obstacle = 0;

int straight;
int right;
int left;

float angle;
float prevAngle;
int dt;
int currTime;
int prevTime;

int rightSpeed = 240;
int leftSpeed = 240;

int calibCount = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //initialize each of the H-bridge configuration pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // initialize PWM control pins
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // initialize stop button to stop the loop
  //pinMode(offButton, INPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  //initialize the trigger and echo pins
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);

  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  
  currTime = millis();
  dt = currTime - prevTime;

  mpu.update();

  start = run();
  if(start == 0){

    brake();

  } else if (start == 1){

    obstacle = ultrasonic();

    if(obstacle == 1){
      brake();
    }

    angle = mpu.getAngleZ();
    driveStraight(angle, prevAngle, dt);
    prevAngle = angle;

  }
  
  prevTime = currTime;
}

int getYawAngle(int dt){

}

void driveStraight(float currYaw, float prevYaw, int dt) {

  float errorIntegral;
  // Directions of motors compared to inputs IN1 and IN2
  // IN1'IN2' = off
  // IN1'IN2 = Reverse
  // IN1IN2' = Forward
  // IN1IN2 = off

  // Directions of motors compared to inputs IN3 IN4
  // IN3'IN4' = off
  // IN3'IN4 = Forward
  // IN3IN4' = Reverse
  // IN3IN4 = off

  // Drive both wheels forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  // P term 
  int kp = 10;

  // calculate the desired speed of the right motor
  rightSpeed = rightSpeed + (currYaw - prevYaw) * kp;
  if(rightSpeed > 255){
    rightSpeed = 255;
  }else if (rightSpeed < 0){
    rightSpeed = 0;
  }

  // calculate the desired speed of the left motor
  leftSpeed = leftSpeed - (currYaw - prevYaw) * kp;
  if(leftSpeed > 255){
    leftSpeed = 255;
  }else if (leftSpeed < 0){
    leftSpeed = 0;
  }
/*
  // I term
  int ki = 10;


  errorIntegral = (currYaw - prevYaw) * dt;

  // right motor
  rightSpeed += errorIntegral * ki;

  // left motor
  leftSpeed += -errorIntegral * ki;
*/
  // set motor speed
  analogWrite(ENA, rightSpeed);
  analogWrite(ENB, leftSpeed);
}

void brake() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

int run() {
  static int start = 0;

  button = analogRead(offButton);
  //Serial.println(button);
  if(button <= 500 && prevButton > 500 && start == 0){
    start = 1;
    Serial.println("on xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
  } else if(button <= 500 && prevButton > 500 && start != 0){
    start = 0;
    Serial.println("off");
  }

  prevButton = button;

  return start;
}

int ultrasonic(){
  // set the trigger low for 2us to ensure that the signal is clear
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);

  // set trigger high for 10us
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);

  // gets the time it took in microseconds
  rawPulse = pulseIn(Echo, HIGH);

  // sound travels at about 343m/s, so to get the distance in centimeters
  // one must multiply the time in microseconds it took to retrieve the echo by 0.0343cm/us and divide by two since the signal away from and to the sensor
  distance = (rawPulse * 0.0343)/2;

  

  static int i = 0;

  i++;

  if(i>1000){
    Serial.print("The distance in centimeters is ");
    Serial.print(distance);
    Serial.print("\n");
    i = 0;
  }

  if(distance < 5){
    obstacle = 1;
  }else{
    obstacle = 0;
  }

  return obstacle;
}