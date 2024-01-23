#include <ros.h>
#include <std_msgs/Int16MultiArray.h>
#define DIR2 6//pump2
#define PWM2 5

#define DIR3 12//pump3
#define PWM3 1

#define DIRAS 10//augur spin 
#define PWMAS 11

#define stepperenablepin 3

std_msgs::Int16MultiArray msg;
ros::NodeHandle nh;
int msg_data[4] = {0,0,0,2};

// std_msgs:: str_msg;
std_msgs::Int16MultiArray msg2;
ros::Publisher pub("ld_mega_feedback", &msg2);  

void messageCb( const std_msgs::Int16MultiArray& msg){
  msg2 = msg;
  msg_data[0] = msg.data[0];
  msg_data[1]= msg.data[1];
  msg_data[2] =msg.data[2];
  msg_data[3] = msg.data[3]; // for stepper enable / disable //// 1 for enable 2 (& 0) for disable
}

void excecute_command(){
  if(msg_data[0]==1){
    analogWrite(PWM2,150);//use peristaltic pump 1
  }
  else analogWrite(PWM2,0);

  if(msg_data[1]==1){
    analogWrite(PWM3,150);//use peristaltic pump 2
  }
  else analogWrite(PWM3,0);

  if(msg_data[2]==1){
    digitalWrite(DIRAS,HIGH);
    analogWrite(PWMAS,150);//rotate mixing auger clock
  }
  if(msg_data[2]== -1){
    digitalWrite(DIRAS,LOW);
    analogWrite(PWMAS, 150);//rotate mixing auger anti clock
  }
  if(msg_data[2]==0){
    digitalWrite(DIRAS, LOW);
    analogWrite(PWMAS, 0);
  }
  // else{
  //   analogWrite(PWMAS,0);//make sure everything is switched off 
  // }

  if(msg_data[3] == 2){
    digitalWrite(stepperenablepin, HIGH); // disabled

  }
  if(msg_data[3] == 1){
    digitalWrite(stepperenablepin, LOW);

  }
}
ros::Subscriber<std_msgs::Int16MultiArray> sub("/ld_mega", &messageCb );
void setup() {
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub);


  // put your setup code here, to run once:
  pinMode(DIR2,OUTPUT);
  pinMode(PWM2,OUTPUT);

  pinMode(DIRAS,OUTPUT);
  pinMode(PWMAS,OUTPUT);//Motor used to make auger go up or down.

  pinMode(DIR3,OUTPUT);
  pinMode(PWM3,OUTPUT);//motor used for mixing chamber.

  pinMode(stepperenablepin, OUTPUT);
}

void loop() {
  nh.spinOnce();
  excecute_command(); 
  // pub.publish(&msg2);
  delay(1);
  
  
} 
