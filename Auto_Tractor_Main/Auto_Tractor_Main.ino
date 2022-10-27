#include <MPU6050_light.h>
#include <Wire.h>
#include <SoftwareSerial.h>

MPU6050 mpu(Wire);

int timer = 0;

// the pin for the IR sensor
int IR = A0;

// the pins for the L298N motor driver
int ENA = 6;
int ENB = 5;
int IN1 = 2;
int IN2 = 7;
int IN3 = 12;
int IN4 = 13;

// the pin for the on/off button
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
float desiredAngle = 0;
int dt;
int currTime;
int prevTime;

int rightSpeed = 213;
int leftSpeed = 240;

float error = 0;
int integral = 0;

int calibCount = 0;

// counts the sequential turns
int turnCount = 0;
int masterCount = 0;

int currLight = 0;
int prevLight = 0;

int wait = 25;

int drive = 0;

// initializations for bluetooth communication
SoftwareSerial HM10(8,11); //changed to pins 8 and 11
char appData;  
String inData = "";


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

  // initialize the IR pin as an input
  pinMode(IR, INPUT);

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

  // Bluetooth communication rate
  HM10.begin(9600); 
}

void loop() {
  // put your main code here, to run repeatedly:
  
  drive = bluetooth();

  currTime = millis();
  dt = currTime - prevTime;

  mpu.update();

  if(drive == 1){
    start = 1; 
  }
  
  start = run();
  if(start == 0){

    brake();

  } else if (start == 1){

    obstacle = ultrasonic();
    currLight = analogRead(IR);
    //Serial.println(currLight);

    if(obstacle == 1){
      brake();
    }else{

      
      angle = mpu.getAngleZ();
      
      if(timer<5){
        timer++;
      }else{
        // may turn early do to unsigned int, typecast later if needed
        if(currLight < (prevLight - 150) && turnCount < 2 && wait > 30){
          desiredAngle = desiredAngle + 90;
          turnCount += 1;
          masterCount += 1;
          wait = 0;
          Serial.println(wait);
        } else if(currLight < (prevLight - 150) && turnCount < 4 && wait > 30){
          desiredAngle = desiredAngle - 90;
          turnCount += 1;
          masterCount += 1;
          if(turnCount == 4){
            turnCount = 0;
          }
          wait = 0;
        } else{
          wait += 1;
        }
        timer = 0;
        Serial.println(angle);
        drivePID(angle, prevAngle, desiredAngle, dt);
        prevAngle = angle;

        prevLight = currLight;
      }
      
    }

  }
  
  prevTime = currTime;
}


void drivePID(float currYaw, float prevYaw, float desiredYaw, int dt) {

  int kp;
  static int turning = 0;

  // Directions of left motor compared to inputs IN1 and IN2
  // IN1'IN2' = off
  // IN1'IN2 = Reverse
  // IN1IN2' = Forward
  // IN1IN2 = off

  // Directions of right motor compared to inputs IN3 IN4
  // IN3'IN4' = off
  // IN3'IN4 = Forward
  // IN3IN4' = Reverse
  // IN3IN4 = off

  // positive yaw is counterclockwise

  error = desiredYaw - currYaw;

  if((error) > 15 || turning > 0 && error > 9){
    turning = 100;

    // Turn immediately to the left
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    if(turning > 0){
      turning += -1;
    }

    // turn left P term 
    kp = 0.4;
  }else if((error) < -15 || turning > 0 && error < -11){
    turning = 100; 

    // Turn immediately to the right
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);


    if(turning > 0){
      turning += -1;
    }
    

    // turn right P term 
    kp = 0.4;
  } else if(abs(error) <= 15){
    turning = 0;

    // Drive both wheels forward
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    // Drive straight P term 
    kp = 1.7;
  }

  if(currYaw == 0){
    rightSpeed = 213;
    leftSpeed = 240;
  }


  // calculate the desired speed of the right motor
  rightSpeed = rightSpeed - kp * (error);
  if(rightSpeed > 228){
    rightSpeed = 228;
  }else if (rightSpeed < 198){
    rightSpeed = 195;
  }

  // calculate the desired speed of the left motor
  leftSpeed = leftSpeed + kp * (error);
  if(leftSpeed > 255){
    leftSpeed = 255;
  }else if (leftSpeed < 225){
    leftSpeed = 225;
  }
  /*
  int ki = 1;

  integral += ki * error;
  rightSpeed = rightSpeed - integral;
  leftSpeed = leftSpeed + integral;
*/
  // set motor speed
  analogWrite(ENB, rightSpeed);
  analogWrite(ENA, leftSpeed);
}

// stops the tractor
void brake() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// detects if the button has been pressed and 
int run() {
  static int start = 0;

  button = analogRead(offButton);
  
  if(button <= 500 && prevButton > 500 && start == 0){
    start = 1;
    //Serial.println("on xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx");
  } else if(button <= 500 && prevButton > 500 && start != 0){
    start = 0;
    //Serial.println("off");
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
/*
  if(i>500){
    Serial.print("The distance in centimeters is ");
    Serial.print(distance);
    Serial.print("\n");
    i = 0;
  }
  */
  if(distance < 10){
    obstacle = 1;
  }else{
    obstacle = 0;
  }

  return obstacle;
}

int bluetooth(){

  static int drive = 0;

  HM10.listen();  // listen the HM10 port
  if (HM10.available() > 0){
    Serial.println("reading...");
    HM10.write("reading...");
  }
  

  while (HM10.available() > 0){
    //Serial.println("TEST");
    appData = HM10.read();
    inData = String(appData);  // save the data in string format
    //Serial.write(appData);
    //Serial.println(inData);
  }
  if( inData == "S") 
  {
  Serial.println("Tractor stopped");
  HM10.write("Tractor stopped");
  //insert tractor instructions for switching off
  drive = 0;
  }
  if ( inData == "G") 
  {
  Serial.println("Tractor Moving..");
  HM10.write("Tractor moving");
  //insert tractor instructions for On
  drive = 1;
  }

  inData = "";
  return drive;
}

// Turn function
void turn(float currYaw, float prevYaw, int dt){
  // left yaw is positive, right yaw is negative

  // turn left currYaw + 90
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);


  // turn right currYaw - 90
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  
}