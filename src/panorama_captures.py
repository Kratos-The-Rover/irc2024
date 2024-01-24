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
    os.mkdir(f"/home/{user}/Desktop/Captures/panorama_captures")
except:
    num=1
os.mkdir(f"/home/{user}/Desktop/Captures/panorama_captures/{timenow}")
print(f"Timenow: {timenow}")
num=1
counter=0

try:
    cam_port = int(input("enter the camport number: "))
    cam = cv2.VideoCapture(cam_port)
except:
    print("camport not found ,defaulting to 0")
    cam_port=0
    cam = cv2.VideoCapture(cam_port)

# cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)
result=0
image=0
ret=0

def update_feed():
    global num,ret
   
    global image
    while(num==1):
        ret, image = cam.read()
    #        cv2.imshow("feed",image)
        

def capture_image():
    global num,ret
    global image
    counter=0
    site_counter=1

    while(num==1):
        print("Enter 1 to take")
        print(f"Current panaroma site is:{site_counter}")
        try:
            inp=int(input())
            if(inp==1):
                counter=counter+1
                if ret:
                    cv2.imwrite(f"/home/{user}/Desktop/Captures/panorama_captures/{timenow}/{counter}.jpg", image)
                    
                else:
                    print("No image detected. Please! try again")
                    counter=counter-1
            else:
                print("Enter correct number")
        except:
            print("Move on")

if __name__ == "__main__":
    t1 = threading.Thread(target=update_feed)
    t2 = threading.Thread(target=capture_image)

    t1.start()
    t2.start()
    
