# import the necessary packages
from flask import Response, Flask, render_template, request
import threading
import argparse 
import datetime, time
import imutils
import cv2

# initialize the output frame and a lock used to ensure thread-safe
# exchanges of the output frames (useful when multiple browsers/tabs are viewing the stream)
outputFrame = None
lock = threading.Lock()
 
# initialize a flask object
app = Flask(__name__)

cap = cv2.VideoCapture(0)
time.sleep(2.0)

@app.route("/")
def index():
    # return the rendered template
    return render_template("index.html")

def stream(frameCount):
    global outputFrame, lock
    if cap.isOpened():
        # cv2.namedWindow(('IP camera DEMO'), cv2.WINDOW_AUTOSIZE)
        while True:
            ret_val, frame = cap.read()
            if frame.shape:
                frame = cv2.resize(frame, (1280,720))
                with lock:
                    outputFrame = frame.copy()
            else:
                continue 
    else:
        print('camera open failed')

def generate(camera_id, resolution):
    global outputFrame
    # grab global references to the output frame and lock variables
    global outputFrame, lock

    width_str, height_str = resolution.split('x')

    # Convert the extracted strings to integers
    width = int(width_str)
    height = int(height_str)

    print(camera_id,width,height)
    
    cap = cv2.VideoCapture(camera_id)
    if cap.isOpened():
        # cv2.namedWindow(('IP camera DEMO'), cv2.WINDOW_AUTOSIZE)
        while True:
            ret_val, frame = cap.read()
            if frame.shape:
                frame = cv2.resize(frame, (width,height))
                with lock:
                    outputFrame = frame.copy()
            else:
                continue 

    
    # loop over frames from the output stream
    while True:
        # wait until the lock is acquired
        with lock:
            # check if the output frame is available, otherwise skip
            # the iteration of the loop
            if outputFrame is None:
                continue
 
            # encode the frame in JPEG format
            (flag, encodedImage) = cv2.imencode(".jpg", outputFrame)
 
            # ensure the frame was successfully encoded
            if not flag:
                continue
 
        # yield the output frame in the byte format
        yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + 
            bytearray(encodedImage) + b'\r\n')



@app.route("/video_feed")
def video_feed():
    camera_id = request.args.get('id', '0')
    resolution = request.args.get('res', '640x480')
    # return the response generated along with the specific media
    # type (mime type)
    return Response(generate(camera_id,resolution),
        mimetype = "multipart/x-mixed-replace; boundary=frame")

# check to see if this is the main thread of execution
if __name__ == '__main__':
    # construct the argument parser and parse command line arguments
    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--ip", type=str, required=False, default='192.168.1.10',
        help="ip address of the device")
    ap.add_argument("-c", "--camera-id", type=int, default=0,
        help="ID of the USB camera device")
    ap.add_argument("-o", "--port", type=int, required=False, default=8000, 
        help="ephemeral port number of the server (1024 to 65535)")
    ap.add_argument("-f", "--frame-count", type=int, default=32,
        help="# of frames used to construct the background model")
    args = vars(ap.parse_args())

    t = threading.Thread(target=stream, args=(args["frame_count"],))
    t.daemon = True
    t.start()
 
    # start the flask app
    app.run(host=args["ip"], port=args["port"], debug=True,
        threaded=True, use_reloader=False)
 
# release the video stream pointer
cap.release()
cv2.destroyAllWindows()