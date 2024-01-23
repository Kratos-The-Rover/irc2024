// FOR ARDUINO UNO

#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#define DIR1 7
#define PWM1 6 // channel 6
#define DIR_EL 5
#define PWM_EL 10 //channel 3
#define DIR_MIX 12
#define PWM_MIX 9 // channel 4


const int dirPin = 4; // can be 4 and 3 as well
const int stepPin = 11; // can be 11 and 2 as well
int stepp=0;
const int stepsPerRevolution = 50;


int var_st;


std_msgs::Int16MultiArray str_msg;
ros::NodeHandle nh;

void messageCb( const std_msgs::Int16MultiArray& msg){
  if(msg.data[0]==1){
    analogWrite(PWM1,150);//use peristaltic pump 1
  }
  else if(msg.data[1]==1){
    digitalWrite(DIR_EL,HIGH);
    analogWrite(PWM_EL,150);//bring the auger up
  }
  else if(msg.data[2]==1){
    digitalWrite(DIR_EL,LOW);
    analogWrite(PWM_EL,150);//bring auger down
  }
  else if(msg.data[3]==1){
    digitalWrite(DIR_MIX,HIGH);
    analogWrite(PWM_MIX,150);//rotate mixing chamber clockwise
  }
  else if(msg.data[4]==1){
    digitalWrite(DIR_MIX,LOW);
    analogWrite(PWM_MIX,150);//rotate mixing chamber anti clockwise
  }
  else{
    analogWrite(PWM1,0);
    analogWrite(PWM_EL,0);
    analogWrite(PWM_MIX,0);//make sure everything is switched off 
  }
  var_st = msg.data[5];
}

void control_stepper(){
  if(var_st == 0){
        digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, LOW);

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
ros::Subscriber<std_msgs::Int16MultiArray> sub("/ld_uno", &messageCb );
void setup() {
  nh.initNode();
  nh.subscribe(sub);

  // put your setup code here, to run once:
  pinMode(DIR1,OUTPUT);
  pinMode(PWM1,OUTPUT);
  digitalWrite(DIR1,HIGH);//peristaltic pump 1

  pinMode(DIR_EL,OUTPUT);
  pinMode(PWM_EL,OUTPUT);//Motor used to make auger go up or down.

  pinMode(DIR_MIX,OUTPUT);
  pinMode(PWM_MIX,OUTPUT);//motor used for mixing chamber.

    pinMode(dirPin,OUTPUT);
  pinMode(stepPin,OUTPUT);
  //   digitalWrite(3, LOW);


}

void loop() {
  control_stepper();
  nh.spinOnce();
  //delay(1);
  
  
} 
