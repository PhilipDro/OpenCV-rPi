import numpy as np
import cv2
import sys
import time
import math
import time
import picamera

import asyncio
import websockets


dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

cameraMatrix = np.array([[1.01138268e+03, 0.00000000e+00, 1.32588738e+03],[0.00000000e+00, 1.01062687e+03, 1.00096125e+03],[0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
distCoeffs = np.array([[-0.13462107, -0.00034976, -0.00724568, -0.00229212,  0.01794853]])

def angles_from_rvec(rvec):
    r_mat, _j_ = cv2.Rodrigues(rvec)
    c = [math.atan2(r_mat[1][0], r_mat[0][0])]
    return c


if __name__ == '__main__':

    async def hello(websocket, path):
        name = await websocket.recv()
        print(f"< {name}")

        greeting = f"Hello {name}!"

        await websocket.send(greeting)
        print(f"> {greeting}")

    start_server = websockets.serve(hello, 10.51.6.5, 8000)

    asyncio.get_event_loop().run_until_complete(start_server)
    asyncio.get_event_loop().run_forever()

    with picamera.PiCamera() as camera:
        camera.resolution = (1024, 768)
        camera.start_preview()
        # Camera warm-up time
        time.sleep(2)
        camera.capture('bar.jpg')

    file = 'bar.jpg'
    print('Processing file:', file)

    # out_file = sys.argv[2]
    # if out_file is None:

    # for debugging
    out_file = 'marker_out.jpeg'

    # Read image
    frame = cv2.imread(file)

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    parameters = cv2.aruco.DetectorParameters_create()

    start_time = time.time()
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    print("Time take: %s seconds" % (time.time() - start_time))

    print(ids)

    rvecs, tvecs, _objPoints = cv2.aruco.estimatePoseSingleMarkers(corners, 2, cameraMatrix, distCoeffs)

    # draw pose estimation markers
    for i in range(0, len(ids)):
        frame = cv2.aruco.drawAxis(frame, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 2);

        cv2.imwrite(out_file, frame)

    #rvec -> r matrix -> winkel z axis -> degree -> change ausrichtung to north (90 degrees)

    # for(x,y,z), value in np.ndenumerate(rvecs):

    for i in range(0, len(ids)):
        rvec = rvecs[i]
        for j in range(0, len(rvec)):
            print('origin: ' + str(rvec[j]))

        angles = angles_from_rvec(rvecs[i])
        print(str(ids[i]) + ' angles')

        for j in range(0, len(angles)):
            print('angle: ' + str(math.degrees(angles[j])))