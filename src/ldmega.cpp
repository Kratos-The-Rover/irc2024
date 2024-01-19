#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist commands;

class LDMega{
  private:
    ros::NodeHandle nh;
		ros::Publisher mega_pub;
    ros::Subscriber sub;
  public:
    LDMega(){
      this->mega_pub = this->nh.advertise<geometry_msgs::Twist>("/ldmega", 20);
      this->sub = this->nh.subscribe("/joy1", 20, &LDMega::joyCallback, this);
    }

    void joyCallback(const sensor_msgs::Joy& msg){
      float bucket_tilt_forward = msg.buttons[1];   //circle
      float bucket_tilt_backward = msg.buttons[3];  //square
      float cam1_forward = msg.axes[5];             //R2
      float cam1_backward = msg.axes[2];            //L2
      float cam2_forward = msg.buttons[5];          //R1
      float cam2_backward = msg.buttons[4];         //L1

      if(bucket_tilt_forward == 1.0)
        bucket_tilt_forward = 1.0;
      else
        bucket_tilt_forward = 0;
        
			if(bucket_tilt_backward == 1.0)
				bucket_tilt_backward = 1.0;
      else
        bucket_tilt_backward = 0;

      if(cam1_forward < 0.5)
        cam1_forward = 1;
      else
        cam1_forward = 0;

      if(cam1_backward < 0.5)
        cam1_backward = 1;
      else
        cam1_backward = 0;

      if(cam2_forward == 1)
        cam2_forward = 1;
      else
        cam2_forward = 0;

      if(cam2_backward == 1)
        cam2_backward = 1;
      else
        cam2_backward = 0;

      commands.linear.x = bucket_tilt_forward;
      commands.linear.y = bucket_tilt_backward;
      commands.linear.z = cam1_forward;
      commands.angular.x = cam1_backward;
      commands.angular.y = cam2_forward;
      commands.angular.z = cam2_backward;

      this->mega_pub.publish(commands);
    }
};

int main(int argc, char **argv){
  ros::init(argc, argv, "ld_mega", ros::init_options::AnonymousName);
  LDMega ldmega = LDMega();
  ros::spin();
  return 0;
}