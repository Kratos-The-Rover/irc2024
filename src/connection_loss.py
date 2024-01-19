#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import os

hostname = "192.168.1.11"
linear = 0
angular = 0

def callback(msg):
    global linear, angular
    linear = -msg.linear.x
    angular = -msg.angular.z

def check_connection():
    global linear, angular
    vels = Twist()
    vels.linear.x = linear
    vels.angular.z = angular
    response = os.system("ping -c 1 -W 3 " + hostname)
    if response == 0:
        print(hostname, 'is up!')
    else:
        sub.unregister()
        pub.publish(Twist())
        rospy.sleep(2)
        pub.publish(vels)
        rospy.sleep(4)

        while response:
            pub.publish(Twist())
            response = os.system("ping -c 1 -W 3 " + hostname)
    rate.sleep()

if __name__ == "__main__":
    rospy.init_node("connection_loss", anonymous=True)
    pub = rospy.Publisher("/rover", Twist, queue_size=10)
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        sub = rospy.Subscriber("/rover", Twist, callback=callback)
        check_connection()