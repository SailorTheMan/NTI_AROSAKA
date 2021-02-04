import cv2
import numpy as np
CROPP_DIM = 60

def getContours(binary_image):      
    _, contours, hierarchy = cv2.findContours(binary_image, 
                                              cv2.RETR_CCOMP, 
                                               cv2.CHAIN_APPROX_SIMPLE)
    return contours

def draw_contours(image, contours, image_name):
    index = -1 #means all contours
    thickness = 2 #thinkess of the contour line
    color = (255, 0, 255) #color of the contour line
    cv2.drawContours(image, contours, index, color, thickness)
    #cv2.imshow(image_name,image)

def crop_detect(photo):
    # cropping
    #cv2.imshow('orig', photo)
    #cv2.waitKey(0)
    cropped_image = photo[60:180, 100:220].copy()
    #cv2.imshow('crop', cropped_image)
    #cv2.waitKey(0)

def contour_counter(mask):
    blur =  cv2.blur(mask,(5,5))
    contours = getContours(blur)
    counter = 0
    largest_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > largest_area:
            largest_area = area 
        perimeter = cv2.arcLength(cnt, True)
        if area > 100:
            counter+=1
        #print ("Chosen contour area: {}, Perimeter: {}".format(area, perimeter))
    #print('Largest area {0}'.format(largest_area))
    return counter

def count_cargo(img):
    y = 120-CROPP_DIM/2
    x = 160-CROPP_DIM/2

    cropped_image = img[y:y+CROPP_DIM, x:x+CROPP_DIM].copy()
    hsv = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2HSV)

    redLower = np.array((147, 65, 108), np.uint8) 
    redUpper = np.array((243, 150, 254), np.uint8) 
    # redLower = np.array((169, 60, 167), np.uint8) 
    # redUpper = np.array((180, 123, 211), np.uint8) 

    greenLower = (51, 29, 112)
    greenUpper = (86, 217, 255)
    # greenLower = (66, 29, 131)
    # greenUpper = (90, 80, 189)

    yellowLower = (27, 70, 70)
    yellowUpper = (62, 150, 240)
    # yellowLower = (22, 47, 176)
    # yellowUpper = (39, 104, 231)

    # blueLower = (106, 64, 145)
    # blueUpper = (110, 110, 170)
    blueLower = (91, 92, 136)
    blueUpper = (120, 209, 200)

    rgyb_counts = []
    cargo_type = 'NONE'
    red_mask = cv2.inRange(hsv, redLower, redUpper)
    #rgyb_counts.append(contour_counter(red_mask))
    if (contour_counter(red_mask)):
        cargo_type = 'RED'
    
    green_mask = cv2.inRange(hsv, greenLower, greenUpper)
    #rgyb_counts.append(contour_counter(green_mask))
    if (contour_counter(green_mask)):
        cargo_type = 'GREEN'

    yellow_mask = cv2.inRange(hsv, yellowLower, yellowUpper)
    if (contour_counter(yellow_mask)):
        cargo_type = 'YELLOW'

    blue_mask = cv2.inRange(hsv, blueLower, blueUpper)
    if (contour_counter(blue_mask)):
        cargo_type = 'BLUE'

    if __name__ == '__main__':
        cv2.imshow("crop", cropped_image)
        cv2.waitKey(0)
    print('Cargo: ' + cargo_type)
    if (cargo_type != 'NONE'):
        return (1, cargo_type)
    else: 
        return (0, cargo_type)

    #print('Total: {0}'.format(sum(rgyb_counts)))
    #print('rgyb: {0}'.format(rgyb_counts))

    #cv2.imshow("mask image r", red_mask)
    #cv2.imshow("mask image g", green_mask)
    #cv2.imshow("mask image y", yellow_mask)
    
    


if __name__ == '__main__':
    image = cv2.imread('/home/clover/catkin_ws/src/clover/clover_simulation/src/images_testflight/invent/cv_31.png') #4.jpg     3.png
    count_cargo(image)
