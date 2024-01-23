import rospy
from std_msgs.msg import Int16MultiArray
from irc2024.msg import sensor_msg
from irc2024.msg import atmos_msg


def ld_sensor_callback(data):
    # Extract values from Int16MultiArray
    co_values = data.data[0]
    ch4_values = data.data[1]
    co2_values = data.data[2]
    temp = data.data[3]
    temp/=100
    humid = data.data[4]
    humid/=100
    air_pressure = data.data[5]
    air_pressure = (air_pressure*0.0009869233)/10 # convert from hpa to atm

    # Create sensor_msg message
    sensor_data = sensor_msg()
    sensor_data.co2 = float(co2_values)
    sensor_data.ch4 = float(ch4_values)
    sensor_data.co = float(co_values)

    # Create atmos_msg message
    atmos_data = atmos_msg()
    atmos_data.temp = float(temp)
    atmos_data.humidity = float(humid)
    atmos_data.pressure = float(air_pressure)

    # Publish sensor_msg and atmos_msg messages
    sensor_pub.publish(sensor_data)
    atmos_pub.publish(atmos_data)

if __name__ == '__main__':
    rospy.init_node('ld_sensor_subscriber')

    # Create publishers for sensor_msg and atmos_msg messages
    sensor_pub = rospy.Publisher('sensor_topic', sensor_msg, queue_size=10)
    atmos_pub = rospy.Publisher('atmos_topic', atmos_msg, queue_size=10)

    # Subscribe to ld_sensor_data topic
    rospy.Subscriber('ld_sensor_data', _Int16MultiArray, ld_sensor_callback)

    # Spin ROS
    rospy.spin()
