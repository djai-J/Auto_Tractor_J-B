
int ENA = 6;
int ENB = 5;
int IN1 = 2;
int IN2 = 7;
int IN3 = 12;
int IN4 = 13;

int offButton = 4;
int stop;

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
  pinMode(offButton, INPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

}

void loop() {
  // put your main code here, to run repeatedly:

  // set motor speed to full for both motors 0 (off) - 255 (full speed)
  analogWrite(ENA, 255);
  analogWrite(ENB, 255);

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


  // run test to drive both wheels forward for 2 seconds
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  
  delay(2000);
  Serial.print("Hello World\n");

  // run test to turn left for 2 seconds
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  delay(2000);

  // run test to turn right for 2 seconds
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(2000);

  // run test to reverse for 2 seconds
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  delay(2000);

  // run test to stop both wheels
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  delay(2000);
  
  stop = digitalRead(offButton);

  if(stop != 0){
    exit(0);
  }

}
