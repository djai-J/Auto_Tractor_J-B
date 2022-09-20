int Trig = 9;
int Echo = 10;

long rawPulse;
int distance;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //initialize the trigger and echo pins
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);

}

void loop() {
  // put your main code here, to run repeatedly:

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

  Serial.print("The distance in centimeters is ");
  Serial.print(distance);
  Serial.print("\n");

  delay(500);

}
