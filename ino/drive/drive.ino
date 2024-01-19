#include <ArduinoHardware.h>
#include <geometry_msgs/Point.h>

#include <Cytron_SmartDriveDuo.h>
#include <ros.h>

ros::NodeHandle nh;

#define IN1 6 
#define BAUDRATE 115200
Cytron_SmartDriveDuo smartDriveDuo30(SERIAL_SIMPLIFIED, IN1, BAUDRATE);
geometry_msgs::Point vels;
ros::Publisher pub1("feedback", &vels);

 float right_wheel=0; 
 float left_wheel=0;


void callback(const geometry_msgs::Point& msg)
{
  

  right_wheel = msg.x;
  left_wheel = msg.z;
  vels.x = right_wheel;
  vels.y = left_wheel;
  pub1.publish(&vels);
}

ros::Subscriber<geometry_msgs::Point> sub("/rover",&callback);

void setup()
{ 
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(pub1);
}

void loop()
{
  smartDriveDuo30.control(left_wheel,right_wheel);
  nh.spinOnce();
}

