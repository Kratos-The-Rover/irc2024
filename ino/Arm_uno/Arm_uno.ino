#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

#define baserotationpwm 6
#define basedirection 7
#define bevel_right_pwm 10
#define bevel_right_drn 5
#define bevel_left_pwm  9
#define bevel_left_drn  12
const int dirPin = 4; // can be 4 and 3 as well
const int stepPin = 11; // can be 11 and 2 as well
int stepp=0;
const int stepsPerRevolution = 50;

ros::NodeHandle n;
std_msgs::Int16MultiArray msg;

int vel;
int var;
int var_base;
int var_bev;
int var_st;
int vel_base;
int vel_bev;

void messageCallback(const std_msgs::Int16MultiArray& receivedMsg)
{
  var_base = receivedMsg.data[0];
  var_bev = receivedMsg.data[2];
  var_st = receivedMsg.data[4];

  vel_base = receivedMsg.data[1];
  if (receivedMsg.data[1] > 25 && receivedMsg.data[1] < 175) {
    vel_base = receivedMsg.data[1];
  } else {
    vel_base = 0;
  }

  vel_bev = receivedMsg.data[3];
  if (receivedMsg.data[3] > 25 && receivedMsg.data[3] < 175) {
    vel_bev = receivedMsg.data[3];
  } else {
    vel_bev = 0;
  }
}

void control_things(){

  // n.loginfo("mode" + var);
  if(var_base == 0 && var_bev == 0 && var_st == 0){
    digitalWrite(baserotationpwm, LOW);//off
    digitalWrite(bevel_right_pwm, LOW);
    digitalWrite(bevel_left_pwm, LOW);
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, LOW);
  }
  if(var_base == 1){
    digitalWrite(basedirection, HIGH);//base rotates in one direction
    analogWrite(baserotationpwm, vel_base);
    digitalWrite(bevel_right_pwm, LOW);
    digitalWrite(bevel_left_pwm, LOW);
  }
  if(var_base == 2){
    digitalWrite(basedirection, LOW);//base rotates in another direction
    analogWrite(baserotationpwm, vel_base);
    digitalWrite(bevel_right_pwm, LOW);
    digitalWrite(bevel_left_pwm, LOW);
  }

  if(var_bev == 1){
    digitalWrite(bevel_right_drn, LOW);//gripper roll motion in one direction
    digitalWrite(bevel_left_drn, HIGH);
    analogWrite(bevel_right_pwm, vel_bev);
    analogWrite(bevel_left_pwm, vel_bev);
    digitalWrite(baserotationpwm, LOW);
  }
  if(var_bev == 2){
    digitalWrite(bevel_right_drn, HIGH);//gripper roll motion in another direction
    digitalWrite(bevel_left_drn, LOW);
    analogWrite(bevel_right_pwm, vel_bev);
    analogWrite(bevel_left_pwm, vel_bev);
    digitalWrite(baserotationpwm, LOW);
  }
  if(var_bev == 3){
    digitalWrite(bevel_right_drn, HIGH);//gripper pitch motion in one direction
    digitalWrite(bevel_left_drn, HIGH);
    analogWrite(bevel_right_pwm, vel_bev);
    analogWrite(bevel_left_pwm, vel_bev);
    digitalWrite(baserotationpwm, LOW);
  }
  if(var_bev == 4){
    digitalWrite(bevel_right_drn, LOW);//gripper pitch motion in another direction
    digitalWrite(bevel_left_drn, LOW);
    analogWrite(bevel_right_pwm, vel_bev);  
    analogWrite(bevel_left_pwm, vel_bev);
    digitalWrite(baserotationpwm, LOW);
  }

  if(var_st == 1){
    digitalWrite(dirPin, HIGH);
    
    for(int x=0; x<5; x++){
      
      digitalWrite(stepPin, HIGH);//gripper opens or closes
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
    }
  }
  else if(var_st == 2){
    digitalWrite(dirPin, LOW);
    
    for(int x=0; x<5; x++){
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
    }
  }
}
ros::Subscriber<std_msgs::Int16MultiArray> sub1("/control2", messageCallback); 
void setup(){
  pinMode(baserotationpwm,OUTPUT);
  pinMode(basedirection,OUTPUT);
  pinMode(bevel_right_pwm,OUTPUT);
  pinMode(bevel_left_pwm,OUTPUT);
  pinMode(bevel_right_drn,OUTPUT);
  pinMode(bevel_left_drn,OUTPUT);
  pinMode(dirPin,OUTPUT);
  pinMode(stepPin,OUTPUT);
  digitalWrite(3, LOW);

  n.initNode();
  n.subscribe(sub1);
}

void loop(){
  control_things();
  n.spinOnce();
  // delay(1);
}

