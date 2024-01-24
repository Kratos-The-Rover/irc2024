#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Joy

class JoystickControl:
    def __init__(self):
        self.vel_la1 = 0 # velocity of linear actuator 1 
        self.vel_la2 = 0 # velocity of linear actuator 2
        self.var_st = 0 # variable to enabel/disable stepper 
        self.msg = Int16MultiArray()
        
        self.sub = rospy.Subscriber("/joy1", Joy, self.callback)
        self.pub = rospy.Publisher("/control1", Int16MultiArray, queue_size=1)

    def callback(self, data):
        xAxis1 = data.axes[0] 
        yAxis1 = data.axes[1]
        square = data.buttons[3]
        circle = data.buttons[1]
        self.vel_la1 = 0
        self.vel_la2 = 0

        # this part of if block, will tell which linear actuators to run and in which combo
        # For eg: Both extending, one extending and other retracting, etc.
        
        if xAxis1 > 0:
            self.vel_la1 = int((xAxis1)*255) # velocity of linear actuator 1 is proportional to how much you move the joystick (analog)
        if xAxis1 < 0:
            self.vel_la1 = int((xAxis1)*255)
        if yAxis1 < 0:
            self.vel_la2 = int((yAxis1)*255) # velocity of linear actuator 2 is proportional to how much you move the joystick (analog)
        if yAxis1 > 0:
            self.vel_la2 = int((yAxis1)*255)

        # this part will tell if stepper should be enabled or disabled

        if square == 1:
            self.var_st = 1
        elif circle == 1:
            self.var_st = 2
        else:
            self.var_st = 0

        self.msg.data = [self.vel_la1, self.vel_la2, self.var_st]
        self.pub.publish(self.msg)

if __name__ == '__main__':
    rospy.init_node("cam_la_v2", anonymous=True)
    joystick_control = JoystickControl()
    rospy.spin()