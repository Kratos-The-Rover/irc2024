from flask import Flask, render_template, Response, request
import cv2
import threading
import argparse 

import sys
import os

app = Flask(__name__)

# Dictionary to store outputFrames, locks, and resolutions for each camera ID
camera_streams = {}

def get_available_cameras():
    try:
        # Use os to list video devices in /dev
        video_devices = [device for device in os.listdir('/dev') if device.startswith('video')]
        available_cameras = ['/dev/' + device for device in video_devices]
    except Exception as e:
        print(f"Error getting available cameras: {e}")
        available_cameras = []

    return available_cameras

def stream(camera_id, resolution):
    global camera_streams

    width_str, height_str = resolution.split('x')
    width, height = int(width_str), int(height_str)

    cap = cv2.VideoCapture(camera_id, cv2.CAP_V4L)

    if not cap.isOpened():
        print(f'Camera {camera_id} open failed')
        return

    while True:
        ret_val, frame = cap.read()

        if not ret_val:
            print(f'Error reading frame from camera {camera_id}')
            break

        frame = cv2.resize(frame, (width, height))

        # Access the lock, outputFrame, and resolution specific to the camera ID
        with camera_streams[camera_id]["lock"]:
            camera_streams[camera_id]["outputFrame"] = frame.copy()

    cap.release()

@app.route("/")
def index():
    # Get available camera IDs (device names)
    camera_ids = get_available_cameras()

    return render_template("index.html", camera_ids=camera_ids)

def generate(camera_id, resolution):
    global camera_streams

    while True:
        with camera_streams[camera_id]["lock"]:
            if camera_streams[camera_id]["outputFrame"] is None:
                continue

            (flag, encodedImage) = cv2.imencode(".jpg", camera_streams[camera_id]["outputFrame"])

            if not flag:
                continue

        yield (b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' +
               bytearray(encodedImage) + b'\r\n')

@app.route("/stop_video_feed")
def stop_video_feed():
    camera_id = request.args.get('id', '/dev/video0')
    release_camera(camera_id)
    return "Video feed stopped for camera_id: {}".format(camera_id)

def release_camera(camera_id):
    global camera_streams

    if camera_id in camera_streams:
        camera_streams[camera_id]["lock"].acquire()

        # Release the capture and remove camera_id from the dictionary
        if camera_streams[camera_id]["outputFrame"] is not None:
            camera_streams[camera_id]["outputFrame"] = None
        
        del camera_streams[camera_id]

        camera_streams[camera_id]["lock"].release()

        if "cap" in camera_streams[camera_id]:
            cap = camera_streams[camera_id]["cap"]
            if cap.isOpened():
                cap.release()





@app.route("/video_feed")
def video_feed():
    camera_id = request.args.get('id', '/dev/video0')
    resolution = request.args.get('res', '640x480')

    # Create outputFrame, lock, and resolution for the new camera ID if not exists
    if camera_id not in camera_streams:
        camera_streams[camera_id] = {
            "outputFrame": None,
            "lock": threading.Lock(),
            "resolution": resolution
        }

        # Start the stream in a separate thread
        threading.Thread(target=stream, args=(camera_id, resolution)).start()

    # Return the response generated along with the specific media type (mime type)
    return Response(generate(camera_id, resolution), mimetype="multipart/x-mixed-replace; boundary=frame")

if __name__ == "__main__":
    # Parse command-line arguments for IP and port
    # ip_address = sys.argv[1] if len(sys.argv) > 1 else '127.0.0.1'
    # port = int(sys.argv[2]) if len(sys.argv) > 2 else 5000

    ap = argparse.ArgumentParser()
    ap.add_argument("-i", "--ip", type=str, required=False, default='0.0.0.0',
        help="ip address of the device")
    ap.add_argument("-o", "--port", type=int, required=False, default=8000, 
        help="ephemeral port number of the server (1024 to 65535)")
    ap.add_argument("-f", "--frame-count", type=int, default=32,
        help="# of frames used to construct the background model")
    args = vars(ap.parse_args())



    app.run(host=args["ip"], port=args["port"], debug=True)
