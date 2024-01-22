#include <Wire.h>
#include "SparkFun_AS7265X.h" 
AS7265X sensor;
 
void setup() {

  Serial.begin(115200);
  Serial.setTimeout(1);
   
  if(sensor.begin() == false)
  {
    while(1);
  }

  Wire.setClock(400000);
  sensor.disableIndicator();  
}
 

void loop() {
  sensor.takeMeasurementsWithBulb(); // This is a hard wait while all 18 channels are measured

  float readings[18];

  readings[0] = sensor.getCalibratedA();
  readings[1] = sensor.getCalibratedB();
  readings[2] = sensor.getCalibratedC();
  readings[3] = sensor.getCalibratedD();
  readings[4] = sensor.getCalibratedE();
  readings[5] = sensor.getCalibratedF();

  readings[6] = sensor.getCalibratedG();
  readings[7] = sensor.getCalibratedH();
  readings[8] = sensor.getCalibratedI();
  readings[9] = sensor.getCalibratedJ();
  readings[10] = sensor.getCalibratedK();
  readings[11] = sensor.getCalibratedL();

  readings[12] = sensor.getCalibratedR();
  readings[13] = sensor.getCalibratedS();
  readings[14] = sensor.getCalibratedT();
  readings[15] = sensor.getCalibratedU();
  readings[16] = sensor.getCalibratedV();
  readings[17] = sensor.getCalibratedW();

  for (int i = 0; i < 18; i++) {
    
    Serial.print(readings[i]);  
    Serial.print(",");
  }

  // Add a delimiter to indicate the end of the data
  Serial.println("");

  delay(1000);
}
