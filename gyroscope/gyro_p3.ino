#include <ezButton.h>
#include "MPU9250.h"


#define LOOP_STATE_STOPPED 0
#define LOOP_STATE_STARTED 1

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;
int count = 0; 


ezButton button(4);  // create ezButton object that attach to pin 4;
int loopState = LOOP_STATE_STOPPED;


void setup() {
  // serial to display data
  Serial.begin(38400);
  button.setDebounceTime(50); // set debounce time to 50 milliseconds
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void loop() {
  // read the sensor
  button.loop(); // MUST call the loop() function first

  if (button.isPressed()) {
    if (loopState == LOOP_STATE_STOPPED)
      loopState = LOOP_STATE_STARTED;
    else // if(loopState == LOOP_STATE_STARTED)
      loopState = LOOP_STATE_STOPPED;
  }

if (loopState == LOOP_STATE_STARTED){
  IMU.readSensor();
  // display the data
  Serial.print("AccelX: ");
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("	");
  Serial.print("AccelY: ");  
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("	");
  Serial.print("AccelZ: ");  
  Serial.println(IMU.getAccelZ_mss(),6);
  
  Serial.print("GyroX: ");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("	");
  Serial.print("GyroY: ");  
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("	");
  Serial.print("GyroZ: ");  
  Serial.println(IMU.getGyroZ_rads(),6);

  Serial.print("MagX: ");  
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("	");  
  Serial.print("MagY: ");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("	");
  Serial.print("MagZ: ");  
  Serial.println(IMU.getMagZ_uT(),6);
  
  Serial.print("Temperature in C: ");
  Serial.println(IMU.getTemperature_C(),6);
  Serial.println();
  delay(2000);

}


} 