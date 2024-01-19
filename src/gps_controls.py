#! /usr/bin/env python3
import rospy 			
import math			
import numpy as np	
import utm
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Twist 
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Int32
import matplotlib.pyplot as plt

class RootClient(object):

    def __init__(self):

        print("[.] Initializing Subscriber and GPS")
       
        self.odom_sub = rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.gps_cb)
        self.cur_east = 0.0
        self.cur_north = 0.0
        self.gps_cnt = 0
        self.set_gps = False
        while not self.set_gps and not rospy.is_shutdown():
            continue
        
        print("[+] Done initializing Subscriber")
    
    def gps_cb(self, msg):
        self.gps_cnt += 1

        if self.gps_cnt <= 100:
            self.start_lat = msg.latitude
            self.start_lon = msg.longitude
            print(f"GPS set at {self.start_lat}, {self.start_lon}")
        else:
            self.set_gps = True
            self.cur_east, self.cur_north, _, _ = utm.from_latlon(msg.latitude, msg.longitude)

    def main(self, goal_x, goal_y, start_east, start_nort):

        r = rospy.Rate(10)
        self.start_east = start_east
        self.start_north = start_nort

        plt_goalx = goal_x - self.cur_east
        plt_goaly = goal_y - self.cur_north
        plt_goaly[0] = 0
        plt_goalx[0] = 0
        plt.plot(plt_goalx, plt_goaly)
        plt.scatter(plt_goalx, plt_goaly, label = "GPS Waypoints", color = "green")
        plt.scatter([0], [0], label = "Current Position", color = "cyan")
        plt.legend()

        while not rospy.is_shutdown():
            
            plt.scatter([self.cur_east - self.start_east], [self.cur_north - self.start_north], label = "Current Position", color = "cyan")
            plt.pause(0.001)
            r.sleep()
        
                    
if __name__ == "__main__":

    rospy.init_node("GPS_Waypoint_Visualizer")
    root = RootClient()
    print("All set!")
    words=[]
    goal_x = [0]
    goal_y = [0]

    co_file = open("/home/anshulraje/Desktop/root_cont_coordinates.txt","r")
    lines = co_file.readlines()

    start_lat = 38.9948541
    start_lon = -110.1614736

    start_east, start_nort, start_reg, start_reg_char=utm.from_latlon(start_lat, start_lon)
    for line in lines:
        easting,northing,region_num,region_char=utm.from_latlon(float(line.split()[0]), float(line.split()[1]))
        goal_x.append(float(easting))
        goal_y.append(float(northing))

    co_file.close()

    print(goal_x)
    print(goal_y)

    root.main(goal_x, goal_y, start_east, start_nort)
    rospy.spin()
    rospy.logwarn("Killing!")


