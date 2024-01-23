#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Point

p=Point()


def callback(msg):
   
    p.x=msg.axes[8]
    p.y=msg.axes[9]


    
    pub.publish(p)


rospy.init_node("cam_gimble")
rospy.Subscriber("/joy0",Joy,callback)
pub=rospy.Publisher("/cam_gimble",Point,queue_size=10)
rospy.spin()
