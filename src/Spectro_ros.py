import serial
import time
import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayLayout, MultiArrayDimension

# Define the serial port and baud rate
ser = serial.Serial('/dev/ttyUSB0', 115200)  # Update the port accordingly

# Create the publisher and initialize the node outside the loop
pub = rospy.Publisher('Spectro', Float32MultiArray, queue_size=10)
rospy.init_node('talker', anonymous=True)

try:
    while not rospy.is_shutdown():
        data = ser.readline().decode('utf-8').strip()
        #print(data)
        reading = []
        meow = ""

        for c in data:
            if c == ',' and meow != "":
                reading.append(float(meow))
                meow = ""
            else:
                meow += c

        if reading:
            # Create a Float32MultiArray message
            msg = Float32MultiArray()

            # Create layout with one dimension (size of data list)
            layout = MultiArrayLayout()
            layout.dim.append(MultiArrayDimension(label="data", size=len(reading), stride=1))
            msg.layout = layout

            # Set the data field
            msg.data = reading

            # Publish the message
            pub.publish(msg)

        time.sleep(0.1)

except KeyboardInterrupt:
    ser.close()
    print("Serial connection closed.")