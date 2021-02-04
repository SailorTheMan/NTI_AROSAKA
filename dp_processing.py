import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


def stackImages(scale, imgArray):
    rows = len(imgArray)
    cols = len(imgArray[0])
    rowsAvailable = isinstance(imgArray[0], list)
    width = imgArray[0][0].shape[1]
    height = imgArray[0][0].shape[0]
    if rowsAvailable:
        for x in range(0, rows):
            for y in range(0, cols):
                if imgArray[x][y].shape[:2] == imgArray[0][0].shape[:2]:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (0, 0), None, scale, scale)
                else:
                    imgArray[x][y] = cv2.resize(imgArray[x][y], (imgArray[0][0].shape[1], imgArray[0][0].shape[0]), None, scale, scale)
                if len(imgArray[x][y].shape) == 2: imgArray[x][y] = cv2.cvtColor(imgArray[x][y], cv2.COLOR_GRAY2BGR)
        imageBlank = np.zeros((height, width, 3), np.uint8)
        hor = [imageBlank]*rows
        hor_con = [imageBlank] * rows
        for x in range(0, rows):
            hor[x] = np.hstack(imgArray[x])
        ver = np.vstack(hor)
    else:
        for x in range(0, rows):
            if imgArray[x].shape[:2] == imgArray[0].shape[:2]:
                imgArray[x] = cv2.resize(imgArray[x], (0, 0), None, scale, scale)
            else:
                imgArray[x] = cv2.resize(imgArray[x], (imgArray[0].shape[1], imgArray[0].shape[0]), None, scale, scale)
            if len(imgArray[x].shape) == 2: imgArray[x] = cv2.cvtColor(imgArray[x], cv2.COLOR_GRAY2BGR)
        hor = np.hstack(imgArray)
        ver = hor
    return ver


def getContours(img, imgContour):
    _, contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    maxArea = 0
    max_rect = (0, 0, 0, 0)
    for cnt in contours:
        area = cv2.contourArea(cnt)
        minArea = 800 #cv2.getTrackbarPos('minArea', 'Parameters')
        if area > minArea:
            cv2.drawContours(imgContour, cnt, -1, (255, 0, 255), 3)
            peri = cv2.arcLength(cnt, True)
            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
            x, y, w, h = cv2.boundingRect(approx)
            if w * h > maxArea:
                maxArea = w * h
                max_rect = (x, y, w, h)
            cv2.rectangle(imgContour, (x, y), (x + w, y + h), (0, 255, 0), 5)
    x, y, w, h = max_rect
    cv2.rectangle(imgContour, (x, y), (x + w, y + h), (255, 0, 0), 5)
    return max_rect

 
def image_callback(data):
    img = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

    imgContour = img.copy()
    imgBlur = cv2.GaussianBlur(img, (7, 7), 1)
    imgGray = cv2.cvtColor(imgBlur, cv2.COLOR_BGR2GRAY)
    
    threshold1 = 255 #cv2.getTrackbarPos('Threshold1', 'Parameters')
    threshold2 = 200 #cv2.getTrackbarPos('Threshold2', 'Parameters')
    
    imgCanny = cv2.Canny(imgGray, threshold1, threshold2)
    kernel = np.ones((5, 5))
    imgDil = cv2.dilate(imgCanny, kernel, iterations=1)

    max_rect = getContours(imgDil, imgContour)
    maxRect_pub.publish(str(max_rect[0]) + ' ' + str(max_rect[1]) + ' ' + str(max_rect[2]) + ' ' + str(max_rect[3]))
    imgCenters = imgContour.copy()
    x, y, w, h = max_rect
    cv2.rectangle(imgCenters, (x + w/2-2, y + h/2-2), (x + w/2+2, y + h/2+2), (255, 0, 0), 2)
    cv2.rectangle(imgCenters, (162, 142), (158, 138), (0, 0, 255), 2)

    imgStack = stackImages(1.0, ([img, imgGray, imgCanny], 
                                 [imgDil, imgContour, imgCenters]))

    image_pub.publish(bridge.cv2_to_imgmsg(imgStack, 'bgr8'))

rospy.init_node('computer_vision_sample')
image_pub = rospy.Publisher('~debug', Image, queue_size=1)
maxRect_pub = rospy.Publisher('maxRect', String, queue_size=1)
bridge = CvBridge()

image_sub = rospy.Subscriber('main_camera/image_raw', Image, image_callback)

rospy.spin()

