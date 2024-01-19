#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int8
from gui_stuff.msg import *

flag = 0
old_flag = -1

rospy.init_node("gui_test", anonymous=True)

def flag_func(data):
    global old_flag, flag
    if old_flag==flag:
        old_flag = -1
    else:
        old_flag = flag
    flag = data.data
    
# Sensors publisher
pub_sensor = rospy.Publisher("/sensors", sensor_msg, queue_size=1)
pub_spectro = rospy.Publisher("/spectrometer", spectro_msg, queue_size=1)

flag_sub = rospy.Subscriber('/flag_topic', Int8, flag_func)

co2_sensor = {
    0: 600,
    5: 570,
    10: 580,
    15: 610,
    20: 660,
    25: 690,
    30: 680
}


ch4_sensor = {
    0: 1.88,
    5: 1.87,
    10: 1.89,
    15: 1.87,
    20: 1.86,
    25: 1.87,
    30: 1.88
}


co_sensor = {
    0: 5.1,
    5: 5.3,
    10: 5.1,
    15: 5.2,
    20: 5.1,
    25: 5.4,
    30: 5.3
}


bradford_assay = {
    410: 0.08,
    435: 0.04,
    460: 0.005,
    485: 0.008,
    510: 0.015,
    535: 0.03,
    560: 0.08,
    585: 0.15,
    610: 0.241,
    635: 0.19,
    680: 0.11,
    705: 0.07
}


chlorophyll = {
    410: 0.3,
    435: 0.25,
    460: 0.2,
    485: 0.18,
    510: 0.15,
    535: 0.12,
    560: 0.14,
    585: 0.16,
    610: 0.3,
    635: 0.4,
    680: 0.2,
    705: 0.1
}

data_sensor = sensor_msg()
data_spectro = spectro_msg()

while not rospy.is_shutdown():
    if (flag==1):
        try:
            for i in range(len(co2_sensor)):
                if old_flag!=flag:
                    print(1)
                    data_sensor.co2 = list(co2_sensor.values())[i]
                    data_sensor.ch4 = list(ch4_sensor.values())[i]
                    data_sensor.co = list(co_sensor.values())[i]
                    pub_sensor.publish(data_sensor)
                    rospy.sleep(1)
            else:
                flag = old_flag

        except:
            pass
    
    elif (flag==2):
        try:
            for j in range(len(bradford_assay)):
                if old_flag!=flag:
                    print(2)
                    data_spectro.brad = list(bradford_assay.values())[j]
                    data_spectro.chloro = list(chlorophyll.values())[j]
                    pub_spectro.publish(data_spectro)
                    rospy.sleep(1)
            else:
                flag = old_flag

        except:
            pass
        

rospy.spin()

