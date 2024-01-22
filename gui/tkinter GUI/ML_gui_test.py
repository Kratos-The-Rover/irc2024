#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Int8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

str_inp = """Running on GPU...

Total number of images for site 1 = 50
Total breakdown for site 1:
Basalt: 0.211579004590509
Conglomerate: 0.154108799183155
Coprolite: 0.064276531761994
Dolomite: 0.016016354133365
Fossil: 0.053301881200145
Granite: 0.131860339753656
Limestone: 0.965734889819681
Obsidian: 0.189001124311103
Sandstone: 0.071260748078191
Shale: 0.044254417612316

Final Prediction: Limestone"""

def video_publisher():
    
    ml_pub = rospy.Publisher("/ml", String, queue_size=1)
    # Video file path
    video_file = "/home/sathvik/Videos/LD_gui_final/ML_video_feed.mp4"  # Replace with your video file
    
    cap = cv2.VideoCapture(video_file)
    if not cap.isOpened():
        rospy.logerr("Error opening video file")
        return
    
    bridge = CvBridge()
    rate = rospy.Rate(100)  # Adjust the rate as needed
    while not rospy.is_shutdown() and cap.isOpened():
        
        ret, frame = cap.read()
        if ret:
            # Convert OpenCV frame to ROS Image message
            ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # Set ROS Image message header timestamp
            ros_image.header.stamp = rospy.Time.now()
            
            # Publish ROS Image message
            bool_pub.publish(0)
            image_pub.publish(ros_image)
            ml_pub.publish(str_inp)
            rate.sleep()
        else:
            bool_pub.publish(1)
            rate.sleep()
            break

    cap.release()

if __name__ == '__main__':
    rospy.init_node('video_publisher', anonymous=True)
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
    bool_pub = rospy.Publisher('/bool_topic', Int8, queue_size=1)
    
    try:
        video_publisher()
    except rospy.ROSInterruptException:
        pass
