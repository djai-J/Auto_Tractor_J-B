SoftwareSerial HM10(2, 3);
char appData;  
String inData = "";

void setup(){
  Serial.begin(9600);
  Serial.println("HM10 serial started at 9600");
  HM10.begin(9600); 
  pinMode(13, OUTPUT); // setting output
  digitalWrite(13, LOW); // writing to output
}

void loop(){
  HM10.listen();  // listen the HM10 port
  while (HM10.available() > 0) {   // if HM10 sends something then read
    appData = HM10.read();
    inData = String(appData);  // save the data in string format
    Serial.write(appData);
  }

  if ( inData == "O") {
    Serial.println("Tractor stopped");
    //insert tractor instructions for switching off
    delay(500);
  }

  if ( inData == "I") {
    Serial.println("Tractor Moving..");
    //insert tractor instructions for On
    delay(500);
  }

}