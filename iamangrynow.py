import numpy as np
import cv2
import os

CROPP_DIM = 120

###          HOW TO USE          ###
# Call  recognize_digit([rgb_photo])
# Returned value is recognized digit

def compute_weight(file_path, photo, orb):
    img1 = cv2.imread(file_path,0)          # queryImage
    img1 = cv2.adaptiveThreshold(img1, 
                            255, 
                            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                            cv2.THRESH_BINARY_INV, 151, 2)
    ##### TODO: Use skeletons to compare features?
    img2 = cv2.cvtColor(photo, cv2.COLOR_BGR2GRAY)
    img2 = cv2.adaptiveThreshold(img2, 
                            255, 
                            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                            cv2.THRESH_BINARY_INV, 151, 2)


    '''### SHAPE MATCHING TEST ####
    _, contours,hierarchy = cv2.findContours(img1,2,1)
    cnt1 = contours[3]
    _, contours,hierarchy = cv2.findContours(img2,2,1)
    cnt2 = contours[3]
    
    ret = cv2.matchShapes(cnt1,cnt2,1,0.0)
    print ret

    ### END SHAPE MATCHING TEST ###'''

    # find the keypoints and descriptors with SIFT
    kp1, des1 = orb.detectAndCompute(img1,None)
    kp2, des2 = orb.detectAndCompute(img2,None)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    # Match descriptors.
    matches = bf.match(des1,des2)
    # Sort them in the order of their distance.
    matches = sorted(matches, key = lambda x:x.distance)
    weight = 1000
    matches_sum = 0
    for match in matches:
        matches_sum += match.distance
    
    avg_dist = matches_sum / len(matches)
    weight = avg_dist
    

    if __name__ == '__main__':
        img3 = cv2.drawMatches(img1,kp1,img2,kp2,matches[:10], None, flags=2)
        #cv2.drawContours(img3, contours, -1, (255, 0, 255), 2)
        print('File: {0}'.format(file_path))
        print('Median distance: {0}'.format(matches[len(matches)/2].distance))
        print('Average distance: {0}'.format(avg_dist))
        plt.imshow(img3),plt.show()

    return weight

def recognize_digit(photo):
    # Initiate SIFT detector
    orb = cv2.ORB_create()      #     TODO:     |
    weights = {}                #  FIX THIS     V
    path = os.path.abspath(__file__)
    path = path.rstrip('iamangrynow.pyc') + 'digit_recognition/'
    weights[0] = compute_weight(path + 'new_zero.png', photo, orb)
    weights[1] = compute_weight(path + 'new_one.png', photo, orb)
    weights[2] = compute_weight(path + 'new_two.png', photo, orb)
    weights[3] = compute_weight(path + 'new_three.png', photo, orb)
    #print(weights.values())
    return(min(weights, key=weights.get))

def detect_marker(photo):
    #Load the dictionary that was used to generate the markers.
    dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
    # Initialize the detector parameters using default values
    parameters =  cv2.aruco.DetectorParameters_create()
    # Detect the markers in the image
    markerCorners, markerIds, rejectedCandidates = cv2.aruco.detectMarkers(photo, dictionary, parameters=parameters)
    #plt.imshow(markerCorners)
    print(rejectedCandidates)

def is_anything_green(image):
    img = np.array(image, dtype=np.uint8)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    is_green = False
    # greenLower = (52, 73, 118)
    # greenUpper = (70, 162, 160)
    greenLower = (52, 60, 66)
    greenUpper = (79, 138, 101)
    green_mask = cv2.inRange(hsv, greenLower, greenUpper)
    #cv2.imshow("mask", green_mask)
    print(sum(sum(green_mask)))
    if (sum(sum(green_mask)) > 8000):
        is_green = True
    
    return is_green



def analyze_frame(photo):
    y = 120-CROPP_DIM/2
    x = 160-CROPP_DIM/2
    photo = cv2.rotate(photo, cv2.ROTATE_90_CLOCKWISE)
    cropped_image = photo[y:y+CROPP_DIM, x:x+CROPP_DIM].copy()
    gray_crop  = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
    ## TODO Check if an Aruco marker is underneath
    #detect_marker(cropped_image)

    binary = cv2.adaptiveThreshold(gray_crop, 
                        255, 
                        cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                        cv2.THRESH_BINARY, 151, 2)
    _, contours,hierarchy = cv2.findContours(binary,2,1)
    max_area = 0
    for cnt in contours:
        cnt_area = cv2.contourArea(cnt)
        if cnt_area > max_area:
            max_area = cnt_area
    print(max_area)



    if max_area > 5000 and is_anything_green(cropped_image):
        digit = recognize_digit(cropped_image)
        print(digit)
        return digit
    else: 
        print('Seems empty')
        return -1
        


if __name__ == '__main__':
    from matplotlib import pyplot as plt
    img = cv2.imread('/home/clover/catkin_ws/src/clover/clover_simulation/src/images_testflight/dps/dps_3_1.png')  
    print analyze_frame(img)
    #print recognize_digit(img)