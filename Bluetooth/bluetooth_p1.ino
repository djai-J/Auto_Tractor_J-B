#include <SoftwareSerial.h>

SoftwareSerial HM10(8,11); //changed to pins 3 and 11
char appData;  
String inData = "";
//change to arbitrary pins
void setup(){
  //pinMode(1, INPUT); //not needed anymore
  //pinMode(0, OUTPUT);
  Serial.begin(9600);
  Serial.println("HM10 serial started at 9600");
  HM10.begin(9600); 

 

  //pinMode(13, OUTPUT); // setting output
  //digitalWrite(13, LOW); // writing to output
}
//create while loop for listening
void loop(){
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
  }
  if ( inData == "G") 
  {
  Serial.println("Tractor Moving..");
  HM10.write("Tractor moving");
  //insert tractor instructions for On
  }

  inData = "";

  

  

}