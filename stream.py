import time
import picamera

with picamera.PiCamera() as camera:
    try:
        for i, filename in enumerate(camera.capture_continuous('image{counter:02d}.jpg')):
            print(filename)
            time.sleep(1)
            if i == 10:
                break
    finally:
        camera.stop_preview()

# seperate thread for aruco and server
# marker writes in queue and aruco thread will get the data
# use io eventloop