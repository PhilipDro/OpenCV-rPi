
import numpy as np
import cv2
import glob

# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((6*9,3), np.float32)
objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

images = glob.glob('*.jpg')

for fname in images:
    print("run")

    img = cv2.imread(fname)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (9,6),None)

    # If found, add object points, image points (after refining them)
    if ret == True:
        objpoints.append(objp)

        corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, (9,6), corners2,ret)
        cv2.imwrite('outcome.jpg', img)

        print('drawChessboardCorners')

        # Return the camera matrix, distortion coefficients, rotation and translation vectors etc.
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
        # np.savez('save.npz', mtx, dist, rvecs, tvecs)
        # print('camera matrix ' + mtx)
        # print('distortion coefficient ' + dist)

        # Refine scaling parameters
        img = cv2.imread('outcome.jpg')
        h,  w = img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

        # undistort
        mapx,mapy = cv2.initUndistortRectifyMap(mtx,dist,None,newcameramtx,(w,h),5)
        dst = cv2.remap(img,mapx,mapy,cv2.INTER_LINEAR)

        # crop the image
        x,y,w,h = roi
        dst = dst[y:y+h, x:x+w]
        cv2.imwrite('undistorted.png',dst)

        # Load previously saved data
        # with np.load('save.npz') as X:
        #    mtx, dist, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

        cv2.aruco.estimatePoseSingleMarkers(corners, 2, mtx, dist)

        # [[1.01138268e+03, 0.00000000e+00, 1.32588738e+03],
        #  [0.00000000e+00, 1.01062687e+03, 1.00096125e+03],
        # [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]]
        #
        #
        # [[-0.13462107, -0.00034976, -0.00724568, -0.00229212,  0.01794853]]

        # def draw(img, corners, imgpts):
        # corner = tuple(corners[0].ravel())
        # img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255,0,0), 5)
        # img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0,255,0), 5)
        # img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0,0,255), 5)
        # return img



        # TODO
        # websocket
        # return (Id, x, y, b*) *bearing
        # rotationsvektor -> roatation matrix -> phi ; rad/degree