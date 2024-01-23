#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Joy


class JoystickControl:
     def __init__(self):
          self.pump1 = 0 # variable to control peristalic pump 1
          self.auger_up = 0 # variable to control auger
          self.auger_down = 0 # velocity of control auger
          self.mixer_clock = 0 # variable to control mixer
          self.mixer_anti = 0 # variable to control mixer
          self.var_st = 0 # variable to control gripper
          self.msg = Int16MultiArray()
        
          self.sub = rospy.Subscriber("/joy", Joy, self.callback)
          self.pub = rospy.Publisher("/ld_uno", Int16MultiArray, queue_size=1)
        
     def callback(self, data):
          joy_pump1= data.axes[-1] # axes[5] //controls pumps
      

          xAxis1 = data.axes[3] # axes[3]  //controls mixer
          yAxis1 = data.axes[4] # axes[4]  //controls auger

          triangle = data.buttons[2] # buttons[2] //controls Stepper (for forward)
          cross = data.buttons[0] # buttons[0]
          
          # this part of if block, will run pump 1
          if(joy_pump1==1):
            self.pump1=1
          else:
            self.pump1 = 0
          
          # this part of if block, will control 2 bidirectional motors which are mixer and auger up and down
          # x axis is for the mixer
          # y axis is for auger up/down
          if yAxis1 > 0 and abs(yAxis1) > abs(xAxis1):
               self.auger_up = 1
          else:
               self.auger_up = 0
          if yAxis1 < 0 and abs(yAxis1) > abs(xAxis1):
               self.auger_down = 1
          else:
               self.auger_down = 0
          if xAxis1 > 0 and abs(xAxis1) > abs(yAxis1):
               self.mixer_clock = 1
          else:
               self.mixer_clock = 0
          if xAxis1 < 0 and abs(xAxis1) > abs(yAxis1):
               self.mixer_anti = 1
          else:
               self.mixer_anti = 0

          #this part of the if block will tell the gripper to open or close
          if triangle == 1:
               self.var_st = 1
          elif cross == 1:
               self.var_st = 2
          else:
               self.var_st = 0

          self.msg.data = [self.pump1, self.auger_up, self.auger_down,self.mixer_clock,self.mixer_anti,self.var_st]
          self.pub.publish(self.msg)

rospy.init_node("joy_control", anonymous=True)
# pub = rospy.Publisher("/control2", Int16MultiArray, queue_size=1)
rate = rospy.Rate(10)
if __name__ == '__main__':
    rospy.init_node("joy_control", anonymous=True)
    joystick_control = JoystickControl()
    rospy.spin()
