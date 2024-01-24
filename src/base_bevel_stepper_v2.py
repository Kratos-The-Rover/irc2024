#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Joy


class JoystickControl:
     def __init__(self):
          self.var_bev = 0 # variable to control bevel motors
          self.vel_bev = 0 # velocity of bevel motors
          self.var_base = 0 # variable to control base
          self.vel_base = 0 # velocity of base
          self.var_st = 0 # variable to control gripper
          self.msg = Int16MultiArray()
        
          self.sub = rospy.Subscriber("/joy1", Joy, self.callback)
          self.pub = rospy.Publisher("/control2", Int16MultiArray, queue_size=1)
        
     def callback(self, data):
          r2 = data.axes[5] # axes[5] //controls Base
          l2 = data.axes[2] # axes[2]

          xAxis1 = data.axes[3] # axes[3]  //controls Bevel
          yAxis1 = data.axes[4] # axes[4]

          triangle = data.buttons[2] # buttons[2] //controls Stepper (for enabling)
          cross = data.buttons[0] # buttons[0]

          self.var_bev = 0
          self.vel_bev = 0
          self.var_base = 0
          self.vel_base = 0
          self.var_st = 0
          
          # this part of if block, will tell base motor to run at a certain speed
          if r2 < 1:
               self.var_base = 1
               self.vel_base = int(abs(r2-1)*255/2)
          elif l2 < 1:
               self.var_base = 2
               self.vel_base = int(abs(l2-1)*255/2)
          
          # this part of if block, will tell if the gripper is performing roll or pitch motion, in which direction and what velocity
          if yAxis1 > 0 and abs(yAxis1) > abs(xAxis1):
               self.var_bev = 1
               self.vel_bev = int(abs(yAxis1)*255)
          elif yAxis1 < 0 and abs(yAxis1) > abs(xAxis1):
               self.var_bev = 2
               self.vel_bev = int(abs(yAxis1)*255)
          elif xAxis1 > 0 and abs(xAxis1) > abs(yAxis1):
               self.var_bev = 3
               self.vel_bev = int(abs(xAxis1)*255)
          elif xAxis1 < 0 and abs(xAxis1) > abs(yAxis1):
               self.var_bev = 4
               self.vel_bev = int(abs(xAxis1)*255)

          #this part of the if block will tell the gripper to open or close
          if triangle == 1:
               self.var_st = 1
          elif cross == 1:
               self.var_st = 2
          else:
               self.var_st = 0

          self.msg.data = [self.var_base, self.vel_base, self.var_bev, self.vel_bev, self.var_st]
          self.pub.publish(self.msg)

#rospy.init_node("joy_control", anonymous=True)
pub = rospy.Publisher("/control2", Int16MultiArray, queue_size=1)

if __name__ == '__main__':
    rospy.init_node("base_bevel_stepper_v2", anonymous=True)
    joystick_control = JoystickControl()
    rospy.spin()
