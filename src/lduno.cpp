#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Point.h>

geometry_msgs::Point commands;

class LDUno{
  private:
    ros::NodeHandle nh;
		ros::Publisher uno_pub;
    ros::Subscriber sub;
  public:
    LDUno(){
      this->uno_pub = this->nh.advertise<geometry_msgs::Point>("/lduno", 20);
      this->sub = this->nh.subscribe("/joy1", 20, &LDUno::joyCallback, this);
    }

    void joyCallback(const sensor_msgs::Joy& msg){
      float ratchet = msg.axes[0];
      float lead_screw = msg.axes[1];
      float stepper_clockwise = msg.buttons[2];
      float stepper_anticlockwise = msg.buttons[0];
      float stepper_command = 0;

      if(abs(ratchet) > 0.5)
        ratchet = 1.0;
      else 
        ratchet = 0;
      if(abs(lead_screw) > 0.5)
        lead_screw = 1.0*abs(lead_screw)/lead_screw;
      else
        lead_screw = 0;
      if(stepper_clockwise == 1)
        stepper_command = 1;
      else if(stepper_anticlockwise == 1)
        stepper_command = -1;
      else
        stepper_command = 0;

      commands.x = ratchet;
      commands.y = lead_screw;
      commands.z = stepper_command;

      this->uno_pub.publish(commands);
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "ld_uno", ros::init_options::AnonymousName);
  LDUno lduno = LDUno();
  ros::spin();
  return 0;
}