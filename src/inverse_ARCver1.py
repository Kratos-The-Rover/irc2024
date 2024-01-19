#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point
# import time
import matplotlib.pyplot as plt

class Kin():
    #lengths in meters
    # pulished in cm 

    def __init__(self):
        self.x1 = 0
        self.y1 = 0
        self.xcoord=0
        self.ycoord=0
        self.x=0.495
        self.y=0.04
        self.l4 = 36.00 * (10**-2)
        self.l6 = 31.00 * (10**-2)
        self.k4 = 7.5 * (10**-2)
        self.k6 = 19.50* (10**-2)
        self.h4 = 4.0 * (10**-2)
        self.h6 = 1.50 * (10**-2)
        self.a4 = 35.0 * (10**-2)           #to be initialised with given value
        self.a6 = 36.5 * (10**-2)            #to be initialised with given value

        self.q6 = 0        #to be measured again
        self.q4 = 0          #to be measured again
        self.f = 38.0
        self.LL6=self.l6+self.h6+self.k4
        self.LL4=self.l4+self.h4
        self.rotangle=0
        self.rotangle=math.radians(28)
        self.forward_kinematics()
        self.inverse_kinematics()
        self.count=0
        #self.x_max=0                        #to be implemented
        #self.y_max=0                        #to be implemented
        self.pub=rospy.Publisher("inverse_kinematics",Point,queue_size=5)
        self.sub=rospy.Subscriber("joy",Joy,self.callback)


    
    def callback(self,msg):
        #map buttons and axes to the function calls
        if(msg.axes[0] < -0.5):
            self.x1 += 0.001
            self.y1+=0
            self.xcoord +=0.001
            self.ycoord +=0
            self.x +=((math.cos(self.rotangle))*self.x1)-((math.asin(self.rotangle))*self.y1)
            self.y +=((math.sin(self.rotangle))*self.x1)+((math.acos(self.rotangle))*self.y1)
            print(f"Added x: {self.x1} , Added y: {self.y1}")
            self.x1=0
            self.y1=0
        elif(msg.axes[0] > 0.5):
            self.x1 -= 0.001
            self.y1+=0
            self.xcoord -=0.001
            self.ycoord +=0
            self.x +=((math.cos(self.rotangle))*self.x1)-((math.asin(self.rotangle))*self.y1)
            self.y +=((math.sin(self.rotangle))*self.x1)+((math.acos(self.rotangle))*self.y1)
            print(f"Added x: {self.x1} , Added y: {self.y1}")
            self.x1=0
            self.y1=0
        if(msg.axes[1] > 0.5):
            self.y1 += 0.001
            self.x1+=0
            self.xcoord +=0
            self.ycoord +=0.001
            self.x +=((math.cos(self.rotangle))*self.x1)-((math.asin(self.rotangle))*self.y1)
            self.y +=((math.sin(self.rotangle))*self.x1)+((math.acos(self.rotangle))*self.y1)
            print(f"Added x: {self.x1} , Added y: {self.y1}")
            self.y1=0
            self.x1=0
        elif(msg.axes[1] < -0.5):
            self.y1 -= 0.001
            self.x1+=0
            self.xcoord +=0
            self.ycoord -=0.001
            self.x +=((math.cos(self.rotangle))*self.x1)-((math.asin(self.rotangle))*self.y1)
            self.y +=((math.sin(self.rotangle))*self.x1)+((math.acos(self.rotangle))*self.y1)
            print(f"Added x: {self.x1} , Added y: {self.y1}")
            self.y1=0
            self.x1=0
        # if(msg.buttons[1]==1):
            # plt.show()
        self.inverse_kinematics()
        #self.forward_kinematics()
        #print(self.a4, self.a6)
        self.pub.publish((self.a4*100), (self.a6*100), -1)
        print(f"xcoord:{self.xcoord} ycoord:{self.ycoord}")
        #print(self.x,self.y)
        self.count+=1
        self.array_x=[]
        self.array_x.append(self.x)
        self.array_y=[]
        self.array_y.append(self.y)
        plt.plot((self.array_x)*100, (self.array_y)*100, 'r*')
        plt.axis([-1, 1, -1, 1])
        # plt.show()



    def inverse_kinematics(self):
        # self.LL6=self.l6+self.h6+self.k4
        # self.LL4=self.l4+self.h4
        print(f"LL4: {self.LL4} LL6: {self.LL6}")
        # self.q4=-(math.acos(((self.x*self.x)+(self.y*self.y)-(self.LL6*self.LL6)-(self.LL4*self.LL4))/(2*self.LL6*self.LL4)))
        # #self.q6=(math.atan(self.y/self.x))-(math.atan((self.LL4*math.sin(self.q4))/(self.LL6+(self.LL4*math.cos(self.q4)))))
        # self.q6=(math.atan(self.y/self.x))-(math.asin((self.LL4*(math.sin(self.q4)))/(math.sqrt((self.x*self.x)+(self.y*self.y)))))
        # self.q4=math.degrees(self.q4)   
        # self.q6=math.degrees(self.q6)
        print(f"x: {self.x} y: {self.y}")
        # print(f"x1: {self.x1} y1: {self.y1}")
        # print(f"q4: {self.q4} q6: {self.q6}")
        # m=180-(self.q6)-(self.f)
        # m=math.radians(m)
        # n=180+(self.q4)
        # n=math.radians(n)
        self.beta=math.atan(self.y/self.x)
        self.d=math.sqrt(((self.LL6)*(self.LL6))-((self.x/2)*(self.x/2))-((self.y/2)*(self.y/2)))
        self.alpha=math.atan(self.d/(math.sqrt(((self.x/2)*(self.x/2))+((self.y/2)*(self.y/2)))))
        self.q6=self.alpha+self.beta
        self.b=math.sqrt(((self.LL6*math.tan(self.q6))*(self.LL6*math.tan(self.q6)))/(1+((math.tan(self.q6))*(math.tan(self.q6)))))
        self.a=self.b/(math.tan(self.q6))
        self.gamma=math.atan((self.y-self.b)/(self.x-(self.a)))
        print(f"gamma: {math.degrees(self.gamma)}")
        print(f"d: {math.degrees(self.d)}")
        self.q4=-self.q6+self.gamma
        self.q4=math.degrees(self.q4)
        self.q6=math.degrees(self.q6)
        print(f"a: {self.a} b: {self.b}")
        m=180-(self.q6)-(self.f)
        n=180+(self.q4)
        m=math.radians(m)
        n=math.radians(n)
        print(f"q4: {self.q4} q6: {self.q6}")

        self.a6=math.sqrt((self.l6*self.l6)+(self.k6*self.k6)-((math.cos(m))*2*self.l6*self.k6))
        self.a4=math.sqrt((self.l4*self.l4)+(self.k4*self.k4)-((math.cos(n))*2*self.k4*self.l4))
        print(f"a4: {self.a4} a6: {self.a6}")
    
    def forward_kinematics(self):
        self.q6=180-(self.f)-(math.degrees(math.acos(((self.k6*self.k6)+(self.l6*self.l6)-((self.a6)*(self.a6)))/(2*self.k6*self.l6))))
        self.q4=-180+(math.degrees(math.acos(((self.k4*self.k4)+(self.l4*self.l4)-(self.a4*self.a4))/(2*self.k4*self.l4))))
        # print(f"q4 {self.q4}")
        self.q6=math.radians(self.q6)
        self.q4=math.radians(self.q4)
        # self.x1=(self.LL6*(math.cos(self.q6)))+((self.LL4)*(math.cos((self.q4)+(self.q6))))
        # self.y1=(self.LL6*(math.sin(self.q6)))+((self.LL4)*(math.cos(-(self.q4)-(self.q6)+90)))
        #self.x1=0.495
        #self.y1=0.04
        # print(f"xfwd: {self.x1} yfwd: {self.y1}")

if(__name__ == "__main__"):
    rospy.init_node("test")
    kin = Kin()
   
    # rate = rospy.Rate(100)
    # while not rospy.is_shutdown():
    #     # kin.pub.publish(((kin.a4)*100),((kin.a6)*100),kin.count)
    #     rate.sleep()
    
    rospy.spin()
