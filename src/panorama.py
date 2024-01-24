from re import M
import cv2
import os

import cv2
currdir=os.getcwd()
user=os.getlogin()
try:
    os.mkdir(f"/home/{user}/Desktop/panorama_results")
except:
    num=1

path = f'/home/{user}/Desktop/Captures/panorama_captures'
images = []
myList = os.listdir(path)
listlen=len(myList)
myList.sort()
print(f"Folder list{myList}")
currentfolder=int(input("Enter the index of the folder you want to stich, starts from 0:"))
currdirname=myList[currentfolder]
path = f"/home/{user}/Desktop/Captures/panorama_captures/{currdirname}"
myList = os.listdir(path)
listlen=len(myList)
print(f"Photo list {myList}")
myList_f=[]
for i in range(listlen):
    myList_f.append(f'{i+1}.jpg')
print(myList_f)
counter=1

for imgnames in myList_f:
    print(imgnames)
    curImg = cv2.imread(f'{path}/{imgnames}')
    # print(imgnames)
    # curImg = cv2.resize(curImg,(0,0),None,0.3,0.3)
    images.append(curImg)
    counter=counter+1

stitcher = cv2.Stitcher.create()
(status, result) = stitcher.stitch(images)
# print(result)
if (status == cv2.Stitcher_OK):
    print('Panorama Generated')
    cv2.imwrite("/home/{user}/Desktop/panorama_results/panaroma_done.jpg", result)
#    cv2.imshow('Images',result)
    cv2.waitKey()
else:
    print('Not successful')
