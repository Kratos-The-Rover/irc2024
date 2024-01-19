#!/usr/bin/env python3
import serial
import os
import datetime

arduino_port = "/dev/ttyACM3"
baud = 9600
print("Enter Site Number:")
inp = str(input())
fileName="/home/kratos/catkin_ws/src/urc2022/csv_data/site"+inp+"/uno-data"+inp+".csv"

ser = serial.Serial(arduino_port, baud)
print("Connected to Arduino port:" + arduino_port)
file = open(fileName, "w")
print("Created file")

current_time = str(datetime.datetime.now())
print("Current Time: " + current_time)
file.write(current_time + "\n")
file.flush()

samples = 10 #how many samples to collect
print_labels = True
line = 0 #start at 0 because our header is 0 (not real data)
while line <= samples:
    # incoming = ser.read(9999)
    # if len(incoming) > 0:
    if print_labels:
        if line==0:
            print("Methane PPM, Visible, IR, UV, CO2 PPM, CO PPM:")
            file.write("Methane PPM, Visible, IR, UV, CO2 PPM, CO PPM:" + "\n")
        else:
            print("Line " + str(line) + ": writing...")
    getData=str(ser.readline())
    data=getData[0:][:-2]
    print(data)

    #file = open(fileName, "a")
    file.write(data + "\n") #write data with a newline
    file.flush()
    line = line+1

print("Data collection complete!")
file.close()

os.system("scp /home/kratos/catkin_ws/src/urc2022/csv_data/site"+inp+"/uno-data"+inp+".csv anshulraje@192.168.1.11:/home/anshulraje/Desktop/csv_data/site"+inp)