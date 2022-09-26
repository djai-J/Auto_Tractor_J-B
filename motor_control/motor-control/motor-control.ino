int motor1pin1 = 2;
int motor1pin2 = 7;

int motor2pin1 = 12;
int motor2pin2 = 13;

void setup() {
  // put your setup code here, to run once:
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);

  pinMode(6, OUTPUT); 
  pinMode(5, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:   

  //Controlling speed (0 = off and 255 = max speed):
  analogWrite(6, 100); //ENA pin
  analogWrite(5, 200); //ENB pin

  //Controlling spin direction of motors:
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);

  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);
  delay(1000);

  digitalWrite(motor1pin1, LOW);
  digitalWrite(motor1pin2, HIGH);

  digitalWrite(motor2pin1, LOW);
  digitalWrite(motor2pin2, HIGH);
  delay(1000);
}
