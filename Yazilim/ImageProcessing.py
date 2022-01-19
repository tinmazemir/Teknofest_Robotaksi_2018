from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import numpy as np
import serial

maxWidth = 160
maxHeight = 120
lastLinePos = 35
camera = PiCamera()
camera.resolution = (maxWidth, maxHeight)
camera.framerate = 32
camera.brightness = 50
camera.saturation = 100
#camera.hflip=True
#camera.vflip=True
rawCapture = PiRGBArray(camera, size=(maxWidth,maxHeight))

time.sleep(0.1)
rect = np.array([
[0, maxHeight - 1],
[70,70],
[maxWidth - 70, 70],
[maxWidth - 1, maxHeight - 1]], dtype = "float32")

dst=np.array([
[50, maxHeight - 1],
[50, 50],
[maxWidth - 50, 50],
[maxWidth - 50, maxHeight - 1]], dtype = "float32")

t1 = time.time()
inittime = 0

roi = np.array([[[80,40],[maxWidth-80,40],[maxWidth-10,maxHeight],[10,maxHeight]]], dtype=np.int32)
try:
    ser = serial.Serial("/dev/ttyACM0",115200,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)
except serial.SerialException:
    ser = serial.Serial("/dev/ttyACM1",115200,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)

kernel = np.ones((1,1),np.uint8)
kernel2 = np.ones((5,5),np.uint8)


for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):

    image = frame.array
    if image is None:
            continue
    image = cv2.GaussianBlur(image, (7,7), 1)

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    #M = cv2.getPerspectiveTransform(rect, dst)
        #warp = cv2.warpPerspective(image, M, (maxWidth, maxHeight), flags=cv2.INTER_LINEAR)
    mask = np.zeros_like(image)
    ignore_mask_color = (255,) * 3
    cv2.fillPoly(mask, roi, ignore_mask_color)
    warp = cv2.bitwise_and(image, mask)

    img_hsv = cv2.cvtColor(warp, cv2.COLOR_BGR2HSV)

    #line = cv2.inRange(img_hsv, np.array([70,0,180], dtype=np.uint8), np.array([140,255,255], dtype=np.uint8))

    line = cv2.inRange(img_hsv, np.array([0,0,190], dtype=np.uint8), np.array([120,255,255], dtype=np.uint8))
    #line  = cv2.erode(line,kernel,iterations = 2)
    #line = cv2.dilate(line,kernel2,iterations = 1)
    #line = cv2.erode(line,kernel,iterations = 1)
    #line = cv2.dilate(line,kernel2,iterations = 1)

    histogram = np.sum(line[maxHeight - 80: maxHeight,:], axis=1)
    linepos = np.mean(histogram)
    stdev = np.std(histogram)
    #print(linepos, stdev)

    #histogram = np.sum(line[maxHeight-90:maxHeight,:], axis=0)
    #linepos = np.mean(histogram[maxWidth/2+20:maxWidth-40])
    #stdev = np.std(histogram[maxWidth/2+20:maxWidth-40])
    #print(linepos, stdev)
    t2 = time.time()
    if (t2 - t1 > 10 or inittime == 1):
       inittime = 1
       if stdev > 2500:
           for i in range(10):
               ser.write("l".encode())
    #if linepos > 20.0 and linepos < 500 and stdev < 800:
    #   for i in range(10):
    #       ser.write("b".encode())
    #       print("STOP")

    #s = "s" + str(linepos) + '\r'
    #ser.write(s.encode())

    sat = img_hsv[:,:,1]
    a,thresh1 = cv2.threshold(sat,170,255,cv2.THRESH_BINARY)
    g  = cv2.erode(thresh1,kernel,iterations = 2)
    g = cv2.dilate(g,kernel2,iterations = 1)
    g = cv2.erode(g,kernel,iterations = 1)
    g = cv2.dilate(g,kernel2,iterations = 1)
    contours,_= cv2.findContours(g.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    maxArea = 0
    maxCnt = -1
    if len(contours) > 0:
            for i,cnt in enumerate(contours):
                area = cv2.contourArea(cnt)
                if area > maxArea:
                    maxArea = area
                    maxCnt = i
            x,y,w,h = cv2.boundingRect(contours[maxCnt])
            ar =(float(w) / float(h))
            #print(ar,w,h,x,y)
            t2 = time.time()
            if t2 - t1 > 10:
               if y > 20 and y < 100 and w < 160 and w > 20 and h < 80 and h > 5 and ar > 1.5 and ar < 4.0:
                  for i in range(10):
                      ser.write("k".encode())
                   #print("STOP")
            #cv2.drawContours(image, contours,-1, (0,255,0),-1)

    #redlines = cv2.HoughLinesP(sat, 1, np.pi/180, 30, 255, 1)
    #if redlines is not None:
    #   for line in redlines:
    #       x1, y1, x2, y2 = line[0]
            #if y1-y2 > 30 or y1-y2:
            #   continue
    #       cv2.line(image, (x1, y1), (x2,y2), (0,255,0), 3)

    #cv2.imshow("warp", warp)
    #cv2.imshow("thresh",line)
    #cv2.imshow("frame1",image)
    key = cv2.waitKey(25) & 0xFF

    rawCapture.truncate(0)

    if key == ord("q"):
        break
