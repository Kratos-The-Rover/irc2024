#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <Servo.h>
Servo servo1;
Servo servo2;
Servo servo3;
//Servo servo4;

#define DIR4 6               //connections to be checked
#define PWM4 5
#define DIR6 10                   //1 6inch 2 4inch
#define PWM6 11
#define POT1 A4

#define servo1pin 9
#define servo2pin 8
#define servo3pin 7
//#define servo4pin 4

#define stepperenablepin 3

//float servo11,servo22,servo33;
//int servofun1=0;
//int servofun2=0;

ros::NodeHandle n;
std_msgs::Int16MultiArray msg;

//ros::Publisher pub1("velocitiesm", &vels);
int vel;
void messageCallback(const std_msgs::Int16MultiArray& receivedMsg)
{
  int vel_la1 = receivedMsg.data[0];
  if (abs(receivedMsg.data[0]) > 50) {
    vel_la1 = receivedMsg.data[0];
  } else {
    vel_la1 = 0;
  }

  int vel_la2 = receivedMsg.data[1];
  if (abs(receivedMsg.data[1]) > 50) {
    vel_la2 = receivedMsg.data[1];
  } else {
    vel_la2 = 0;
  }

  int var_st = receivedMsg.data[2];

  if(receivedMsg.data[0] == 0 && receivedMsg.data[1] == 0){
    analogWrite(PWM4, 0);
    analogWrite(PWM6, 0);
    
    digitalWrite(DIR4, LOW);
    digitalWrite(DIR6, LOW);
  }
  if(receivedMsg.data[0] >= 0 && receivedMsg.data[1] >= 0){
    analogWrite(PWM4, abs(vel_la1));     
    analogWrite(PWM6, abs(vel_la2));  //linear actuators 1 and 2 extend
    digitalWrite(DIR4, HIGH);
    digitalWrite(DIR6, HIGH);
  }
  if(receivedMsg.data[0] <= 0 && receivedMsg.data[1] <= 0){
    analogWrite(PWM4, abs(vel_la1));
    analogWrite(PWM6, abs(vel_la2));  //linear actuators 1 and 2 retract 
    digitalWrite(DIR4, LOW);
    digitalWrite(DIR6, LOW);
  }
  if(receivedMsg.data[0] <= 0 && receivedMsg.data[1] >= 0){
    analogWrite(PWM4, abs(vel_la1));
    analogWrite(PWM6, abs(vel_la2));  //linear actuator 1 extends and linear actuator 2 retracts
    digitalWrite(DIR4 , LOW);
    digitalWrite(DIR6, HIGH);
  }
  if(receivedMsg.data[0] >= 0 && receivedMsg.data[1] <= 0){
    analogWrite(PWM4, abs(vel_la1));
    analogWrite(PWM6, abs(vel_la2));  //linear actuator 1 retracts and linear actuator 2 extends
    digitalWrite(DIR4, HIGH);
    digitalWrite(DIR6, LOW);
  }

  if(var_st == 1){
    digitalWrite(stepperenablepin,LOW);
  }
  else if(var_st == 2){
    digitalWrite(stepperenablepin,HIGH);
  }
}
/* if(value_passed==3){
    servo3.write(105);
    delay(200);
    servo3.write(90);
  }
  else if(value_passed==-3){
    servo3.write(75);
    delay(200);
    servo3.write(90);
  }
  else{
    servo3.write(90);
  }
  if(value_passed==4){
    servo2.write(105);
    delay(200);
    servo2.write(90);
  }
  else if(value_passed==-4){
    servo2.write(75);
    delay(200);
    servo2.write(90);
  }
  else{
    servo2.write(90);
  }
//  

//    vels.data=vel1;
//  vels.angular.y=vel2;
//  vels.linear.x=servofun1;
//  vels.linear.y=servofun2;
}*/

ros::Subscriber<std_msgs::Int16MultiArray> sub1("/control1", messageCallback); 

void setup(){
  pinMode(DIR4,OUTPUT);
  pinMode(PWM4,OUTPUT);
  pinMode(DIR6,OUTPUT);
  pinMode(PWM6,OUTPUT);
  pinMode(POT1,INPUT);
  digitalWrite(stepperenablepin, LOW);
  servo3.attach(7);
  servo2.attach(8);

  pinMode(3,OUTPUT);
  n.initNode();
  n.subscribe(sub1);
//  n.advertise(pub1);
}

void loop(){
  
//  pub1.publish(&vels);

  n.spinOnce();
}

