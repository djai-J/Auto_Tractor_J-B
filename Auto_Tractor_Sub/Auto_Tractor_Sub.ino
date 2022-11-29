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
int Buzz = 4;

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

int rightSpeed = 185;
int leftSpeed = 178;

float error = 0;
int integral = 0;

int calibCount = 0;

// counts the sequential turns
int turnCount = 0;
int masterCount = 0;
int turnBool = 0;

int currLight = 1000;
int prevLight = 1000;

int wait = 25;

char drive = 0;

// initializations for bluetooth communication
SoftwareSerial HM10(8,11); //changed to pins 8 and 11
char appData;  
String inData = "";


// battery charge things
int currentPin = A2;
int voltPin = A3;

float current = 0;
float voltage = 0;

int adc = 0;

float vout = 0.0; //do not change
float vin = 0.0; //do not change
float R1 = 30000.0; //onboard resistor 1 value
float R2 = 7500.0; //onboard resistor 2 value
int svalue = 0; //do not change

float chargeAccum = 0;
float SOC = 0;

float kalmanAngle = 0;
float Kn = 1;

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
  pinMode(Buzz, OUTPUT);

  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!= 0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");

  // Bluetooth communication rate
  HM10.begin(9600); 

  pinMode(currentPin, INPUT);
  pinMode(voltPin, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  // receives input from the GUI
  drive = bluetooth();

  // calculates the total run time and time between loops
  currTime = millis();
  dt = currTime - prevTime;

  // updates the gyroscope angles
  mpu.update();

  // receives input from the button on the tractor
  start = run();

  // calculates the measured instantaneous current
  int adc = analogRead(currentPin);
  float voltage = adc*5/1023.0;
  float current = (voltage-2.5)/0.185;
  Serial.print("Current : "); // prints the current header
  Serial.println(current); // prints the current

  // calculates the measured instantaneous voltage
  svalue = analogRead(voltPin);
  vout = (svalue * 5.0) / 1024.0;
  vin = vout / (R2/(R1+R2));
  Serial.print(vin,2); // prints the voltage
  Serial.println(" volts DC"); // prints the words "volts DC"

  // calculates the state of charge of the batteries
  chargeAccum += current*(dt/(60*60*1000));
  SOC = (2 - chargeAccum) / 2; 

  // keeps the tractor in a stopped state until the proper command is received from the button or GUI
  if(start == 0 && drive == 0){

    brake();

    start = 0;
    drive = '0';

  } else if (start == 1 || drive == 1){

    start = 1;
    drive = '0';

    // detects obstacles in front of the tractor
    obstacle = ultrasonic();

    // detects tape below the tractor
    currLight = analogRead(IR);
    //Serial.println(currLight);

    // performs an emergency stop if there is an obstacle
    if(obstacle == 1){
      brake();

      // generates a sound to alert the operator via the buzzer
      digitalWrite(Buzz, HIGH);
      digitalWrite(Buzz, LOW);
    }else{
      digitalWrite(Buzz, LOW);
      
      // updates the yaw angle of the tractor
      angle = mpu.getAngleZ();

      // basic Kalman filter to help reduce impact of gyro drift
      kalmanAngle = kalmanAngle + Kn * (angle - kalmanAngle);
      
      // checks if marker has been encountered and how the tractor should turn if so
      if(currLight > (prevLight + 60) && turnCount < 2 && wait > 2 && currLight > 600){ // Left turn
        desiredAngle = desiredAngle + 90;
        turnCount += 1;
        masterCount += 1;
        turnBool = 1;
        wait = 0;
      } else if(currLight > (prevLight + 60) && turnCount < 4 && wait > 2 && currLight > 600){ // Right turn
        desiredAngle = desiredAngle - 90;
        turnCount += 1;
        masterCount += 1;
        if(turnCount == 4){
          turnCount = 0;
        }
        turnBool = 1;
        wait = 0;
      } else if (!turnBool && wait < 4){ // wait to pass over the tape
        Serial.println(wait);
        wait += 1;
      }
      //Serial.println(angle);

      // causes the tractor to drive straight or turn
      turnBool = drivePID(kalmanAngle, prevAngle, desiredAngle, dt, turnBool);

      // stores the previous yaw of the tractor
      prevAngle = kalmanAngle;
      
      // a dampener for the number of times to store the previous IR level, helpful if the loop becomes too taxing
      if(timer < 0){
        timer++;
      } else{
        prevLight = currLight;
        timer = 0;
      }
      
    }

      // several print statements to help debugging
      Serial.print("The current IR level is ");
      Serial.print(currLight);
      Serial.print("          The previous IR level is ");
      Serial.println(prevLight);  

      Serial.print("                     The angle is ");
      Serial.println(angle);

      Serial.print("                     The previous angle is ");
      Serial.println(prevAngle);
  }

  Serial.print("dt = ");
  Serial.println(dt);

  // stores the previous time
  prevTime = currTime;
}


int drivePID(float currYaw, float prevYaw, float desiredYaw, int dt, int turning) {
  static int stop = 0;

  int kp;

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

  // calculates the error of the current heading
  error = desiredYaw - currYaw;

  if((error) > 2 && turning == 1){ // executes the left turn when it has hit the correct checkpoints

    // Turn immediately to the left
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

    // set motor speed
    analogWrite(ENA, 180);
    analogWrite(ENB, 120);

    prevYaw = desiredAngle;
    
    stop = 0;

  }else if((error) < -2 && turning == 1){ // executes the left turn when it has hit the correct checkpoints

    // Turn immediately to the right
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    // set motor speed
    analogWrite(ENA, 120);
    analogWrite(ENB, 150);

    prevYaw = desiredAngle;
    
    stop = 0;

  } else if(turning == 0){ // drives straight if it is not turning

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
    }else if (rightSpeed < 115){
      rightSpeed = 115;
    }

    // calculate the desired speed of the left motor
    leftSpeed = leftSpeed - (currYaw - prevYaw) * kp;
    if(leftSpeed > 248){
      leftSpeed = 248;
    }else if (leftSpeed < 108){
      leftSpeed = 108;
    }

    // set motor speed
    analogWrite(ENA, rightSpeed);
    analogWrite(ENB, leftSpeed);

  } else if (turning == 1 && abs(error) <= 2){ // ensures the tractor is still within the correct bounds of the desired angle
    brake();

    if(stop<3){
      stop++;
    } else{
      turning = 2;
    }

    Serial.println(stop);
  } else if (turning == 2){ // recalculates gyro offsets to reduce gyro drift
      delay(1000);
      mpu.calcOffsets(true,true);

      turning = 0;
  }

  return turning;
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

  // returns that an obstacle is detected if the distance is less than 10 cm
  if(distance < 10){
    obstacle = 1;
  }else{
    obstacle = 0;
  }

  return obstacle;
}

// receives input and tries to send output to the bluetooth module
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
  if( inData == "S") // stops the tractor
  {
  Serial.println("Tractor stopped..*******************************************************************************************************************");
  HM10.write("Tractor stopped");
  //insert tractor instructions for switching off
  drive = 0;
  }
  if ( inData == "G") // starts the tractor
  {
    Serial.println("Tractor Moving..*******************************************************************************************************************");
    HM10.write("Tractor moving");
    //insert tractor instructions for On
    drive = 1;
  }

  inData = "";
  return drive;
}