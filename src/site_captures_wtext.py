from unittest import result
import cv2
import threading
import os
import time
timenow=time.time()
currdir=os.getcwd()
user=os.getlogin()
try:
    os.mkdir(f"/home/{user}/Desktop/Captures")
except:
    num=1
try:
    os.mkdir(f"/home/{user}/Desktop/Captures/site_captures_wtext")
except:
    num=1
print(f"Timenow: {timenow}")
camport=int(input("Enter the camport:"))
num=1
counter=0
cam_port = camport
cam = cv2.VideoCapture(cam_port)
# cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
result=0
image=0
startpoint=20

def update_feed():
    global num
    global result
    global image
    while(num==1):
        result, image = cam.read()
        # print("reading")

def capture_image():
    global num
    global image
    global startpoint
    counter=0
    site_counter=int(input("Enter site number: "))

    while(num==1):
        print("Enter 1 to take")
        print(f"Current science site is:{site_counter}")
        try:
            inp=int(input())
            if(inp==1):
                counter=counter+1
                if result:
                    cv2.imwrite(f"/home/{user}/Desktop/Captures/site_captures_wtext/{site_counter}_{counter}.jpg", image)
                    cv2.waitKey(3)
                    print("Written")
                    # cv2.imshow("heloo",image)
                else:
                    print("No image detected. Please! try again")
                    counter=counter-1
            elif(inp==123):
                site_counter=site_counter
            else:
                print("Enter correct number")
        except:
            print("Move on")

if __name__ == "__main__":
    t1 = threading.Thread(target=update_feed)
    t2 = threading.Thread(target=capture_image)

    t1.start()
    t2.start()
    
