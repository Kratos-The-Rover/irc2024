#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;
std_msgs::Int16MultiArray sensor_data;

ros::Publisher sensor_data_pub("ld_sensor_data", &sensor_data);

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; // I2C

#define MQ2_PIN A0
#define MQ7_PIN A1
#define MQ7_2_PIN A2


unsigned long delayTime;

void setup() {
  nh.initNode();
  unsigned status;
    
  // default settings
  status = bme.begin();  
  

  nh.advertise(sensor_data_pub);

  nh.logerror("bme status : ");
  nh.logerror(status);
  delayTime = 1000; // Initial delay for the first publish
}

int sensor_data_raw[6];



void read_data(){
  sensor_data_raw[0] = analogRead(MQ2_PIN);
  sensor_data_raw[1] = analogRead(MQ7_PIN);
  sensor_data_raw[2] = analogRead(MQ7_2_PIN);
  
  sensor_data_raw[3] = (int)(bme.readTemperature()*100); // with 2 decimal places = divide by 100 on jetson
  sensor_data_raw[4] = (int)(bme.readHumidity()*100); // same here
  sensor_data_raw[5] = (int)(bme.readPressure() / 100.0F); // will give value in  hPa with 2 decimal places so again divide by 100
  
  
}

void loop() {
  
  read_data();
  // char* to_send = "sensor data: "+sensor_data_raw[4];
  // nh.logerror(to_send);
    sensor_data.data = sensor_data_raw;
    // int test[] = {1,2,3};
    // sensor_data.data = test;
    sensor_data.data_length = 6;
  sensor_data_pub.publish(&sensor_data);

  nh.spinOnce();
  delay(delayTime);
}

