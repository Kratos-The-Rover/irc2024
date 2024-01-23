#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Joy


class JoystickControl:
     def __init__(self):
          self.pump2 = 0 # variable to control peristalic pump 2
          self.pump3 = 0 # variable to control peristalic pump 3
          self.auger_spinC = 0 # variable to control auger spin clockwise
          self.auger_spinA = 0 # variable to control auger spin anti clockwise
          self.auger_spin = 0
          self.msg = Int16MultiArray()
          self.stepper_status = 2  ## 0 / 2 for disable, 1 for enable
        
          self.sub = rospy.Subscriber("/joy", Joy, self.callback)
          self.pub = rospy.Publisher("/ld_mega", Int16MultiArray, queue_size=1)
        
     def callback(self, data):
          joy_pump2= data.axes[-1] #controls pumps
          joy_pump3=data.axes[-2]
          
          square = data.buttons[3] # enabling stepper
          circle = data.buttons[1] # disableing stepper
          
          self.auger_spin = 0
            
          xaxis=data.axes[0] #controls spin of augur
          if(joy_pump2==-1):
            self.pump2=1 
          else:
            self.pump2 = 0
          if(joy_pump3==1):
            self.pump3=1
          else:
            self.pump3 = 0
            
          if(square==1):
               self.stepper_status = 1
          elif(circle==1):
               self.stepper_status = 2

          
          if(xaxis>0):
               self.auger_spin = 1
          elif(xaxis<0):
               self.auger_spin = -1
          else:
               self.auger_spin = 0
               
          self.msg.data = [self.pump2,self.pump3,self.auger_spin, self.stepper_status]
          self.pub.publish(self.msg)

rospy.init_node("joy_control_mega", anonymous=True)
pub = rospy.Publisher("/mega", Int16MultiArray, queue_size=1)
rate = rospy.Rate(10)
if __name__ == '__main__':
    rospy.init_node("joy_control_mega", anonymous=True)
    joystick_control = JoystickControl()
    rospy.spin()
