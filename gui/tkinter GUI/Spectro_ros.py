#!/usr/bin/env python3

import serial
import time
import rospy
from gui_stuff.msg import spectro_msg

# Define the serial port and baud rate
ser = serial.Serial('/dev/ttyUSB0', 115200)  # Update the port accordingly

# Create the publisher and initialize the node outside the loop
pub = rospy.Publisher('/spectrometer', spectro_msg, queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(10)

try:
    while not rospy.is_shutdown():
        data = ser.readline().decode('utf-8').strip()
        reading = []
        meow = ""

        for c in data:
            if c == ',' and meow != "":
                reading.append(round(float(meow), 2))
                meow = ""
            else:
                meow += c

        if reading:
            pub_val = spectro_msg()
            pub_val.brad = -1
            pub.publish(pub_val)
            # Publish the message
            for i in reading:
                print(i)
                pub_val.brad = i
                pub.publish(pub_val)
                rate.sleep()
            else:
                pub_val.brad = -1
                pub.publish(pub_val)

        time.sleep(0.1)

except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")