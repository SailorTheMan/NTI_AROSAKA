# Information: https://clover.coex.tech/en/snippets.html#navigate_wait

import os
import errno
import sys
import math
import rospy
import threading
from clover import srv
from cv_bridge import CvBridge
from std_msgs.msg import String
from std_srvs.srv import Trigger
from clover.srv import SetLEDEffect
from mavros_msgs.srv import SetMode
import cv2
from sensor_msgs.msg import Image
import cv_test
from mavros_msgs.srv import CommandBool
import iamangrynow

###################
# UNCOMMENT FOR PROD
##################

__INVENTARIZATION = True
__DP_DETECTION = True
__DELIVERING = True
__SIM = False

__DOUBLE_CHECK = False

__DEBUG_DP = False
__DEBUG_DPS = [(1, 3, 2), (3, 0, 3)]


SAFE_HEIGHT = 2.5
SPEED = 1.0
CAM_HEIGHT = 280
CAM_WIDTH = 320
TOLERANCE = 20
KX = 0.01
KY = 0.01
DEFAULT_ANGLE = 0
DELIVERY_SPEED = 0.8
INVENT_HEIGHT = 1.0
DP_DETECT_HEIGHT = 2.0

# __INVENTARIZATION = False
# __DP_DETECTION = True
# __DELIVERING = True
# __SIM = True

# __DOUBLE_CHECK = True

# __DEBUG_DP = False
# __DEBUG_DPS = [(1, 1, 0), (3, 1, 2)]


# SAFE_HEIGHT = 2.5
# SPEED = 1.0
# CAM_HEIGHT = 280
# CAM_WIDTH = 320
# TOLERANCE = 20
# KX = 0.01
# KY = 0.01
# DEFAULT_ANGLE = 0
# DELIVERY_SPEED = 0.5
# INVENT_HEIGHT = 1.0
# DP_DETECT_HEIGHT = 2.5

RECT = [0, 0, 0, 0]

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
set_mode = rospy.ServiceProxy('mavros/set_mode', SetMode)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)

def start_dpProcessing():
    os.system('python dp_processing.py')
    print('dp_processing started')

def land_wait():
    land()
    while get_telemetry().armed:
        rospy.sleep(0.2)

def navigate_wait(x=0, y=0, z=0, yaw=DEFAULT_ANGLE, yaw_rate=0, speed=SPEED, 
        frame_id='body', tolerance=0.1, auto_arm=False):

    res = navigate(x=x, y=y, z=z, yaw=yaw, yaw_rate=yaw_rate, speed=speed, 
        frame_id=frame_id, auto_arm=auto_arm)

    if not res.success:
        return res

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            return res
        rospy.sleep(0.2)


def rect_callback(data):
    _, x, y, w, h= str(data).split(' ')
    RECT[0] = int(x.strip('"'))
    RECT[1] = int(y)
    RECT[2] = int(w)
    RECT[3] = int(h.strip('"'))

def take_picture(title):
    rospy.sleep(1)
    img = bridge.imgmsg_to_cv2(rospy.wait_for_message('main_camera/image_raw', Image), 'bgr8')
    cv2.imwrite(title, img)
    return img

def land_dpoint():

    def center_the_rect():
        take_picture('cv_image2.png')
        while RECT[2] == 0 or RECT[3] == 0:
            navigate_wait(x=0.0, y=0.0, z=-0.05, speed=0.05, frame_id='body')
            rospy.sleep(0.2)
            continue
        x_c = RECT[0] + RECT[2] / 2
        y_c = RECT[1] + RECT[3] / 2

        cam_x_c = CAM_WIDTH / 2
        cam_y_c = CAM_HEIGHT / 2

        d_x = x_c - cam_x_c
        d_y = y_c - cam_y_c
        
        navigate(y= -d_x * KX, x= -d_y * KY, z = 0.0, frame_id='body', speed=0.01) 
        first_loop = True
        while not rospy.is_shutdown():
            telem = get_telemetry(frame_id='navigate_target')

            x_c = RECT[0] + RECT[2] / 2
            y_c = RECT[1] + RECT[3] / 2
            d_x = x_c - cam_x_c
            d_y = y_c - cam_y_c

            if (math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < 0.05) or ((math.sqrt((d_x)**2 + (d_y)**2) > 50) and not first_loop):
                center_the_rect()

            if math.sqrt((d_x)**2 + (d_y)**2) > TOLERANCE:
                first_loop = False
                rospy.sleep(0.1)
                continue

            navigate_wait()
            return

    print('trying to land')

    for i in range(2):
        center_the_rect()
        print('centered')
        navigate_wait(x=0.0, y=0.0, z=-0.15, speed=0.3, frame_id='body')
        print('lifted down')
        
    # rospy.sleep(0.05)
    # #land_wait()
    # arming(False)
    # rospy.sleep(5)
    # print('landed')
    
    navigate(z= -0.5, speed= 0.1, frame_id='body')
    old_telem = get_telemetry()
    while True:
        telem = get_telemetry()
        if old_telem.z - telem.z < 0.01:
            if not __SIM:
                arming(False)
            break
        old_telem = telem
        rospy.sleep(0.1)
    rospy.sleep(3)
    print('landed')

def goDetectDP():
    print('detecting dronepoints')
    dps = []
    for i in range(4, -1, -1):
        if i % 2 == 0:
            for j in range(5):
                navigate_wait(x=j * 0.9, y=i * 0.9, z = DP_DETECT_HEIGHT, frame_id='aruco_map')
                rospy.sleep(0.5)
                # REMOVE BEFORE DEPLOY
                #continue
                ######################
                img = take_picture(dps_path + 'dps_' + str(i) + '_' + str(j) + '.png')
                res = iamangrynow.recognize_digit(img)
                if res == 0:
                    dps.append((j, i, 0))
                elif res == 1:
                    dps.append((j, i, 1))
                elif res == 2:
                    dps.append((j, i, 2))
                elif res == 3:
                    dps.append((j, i, 3))

                if __DOUBLE_CHECK:
                    navigate_wait(x=j * 0.9, y=i * 0.9, z = DP_DETECT_HEIGHT-0.5, frame_id='aruco_map')
                    rospy.sleep(0.5)
                    # REMOVE BEFORE DEPLOY
                    #continue
                    ######################
                    img = take_picture(dps_path + 'dps_' + str(i) + '_' + str(j) + '_0.5' + '.png')
                    res = iamangrynow.recognize_digit(img)
        else:
            for j in range(4, -1, -1):
                navigate_wait(x=j * 0.9, y=i * 0.9, z = DP_DETECT_HEIGHT, frame_id='aruco_map')
                rospy.sleep(0.5)
                # REMOVE BEFORE DEPLOY
                #continue
                ######################
                img = take_picture(dps_path + 'dps_' + str(i) + '_' + str(j) + '.png')
                res = iamangrynow.recognize_digit(img)
                if res == 0:
                    dps.append((j, i, 0))
                elif res == 1:
                    dps.append((j, i, 1))
                elif res == 2:
                    dps.append((j, i, 2))
                elif res == 3:
                    dps.append((j, i, 3))

                if __DOUBLE_CHECK:
                    navigate_wait(x=j * 0.9, y=i * 0.9, z = DP_DETECT_HEIGHT - 0.5, frame_id='aruco_map')
                    rospy.sleep(0.5)
                    # REMOVE BEFORE DEPLOY
                    #continue
                    ######################
                    img = take_picture(dps_path + 'dps_' + str(i) + '_' + str(j)+ '0.5' + '.png')
                    res = iamangrynow.recognize_digit(img)
    return dps




invent_path = 'images/invent/'
dps_path = 'images/dps/'

try:
    os.mkdir('images')
except OSError as exc:
    if exc.errno == errno.EEXIST:
        print('image folder already exist')
    else:
        invent_path = ''
        dps_path = ''
        print("!!!Couldn't make directories, I will store all the images in root folder!!!")
        print(exc.message)
try:
    os.mkdir('images/invent')
except OSError as exc:
    if exc.errno == errno.EEXIST:
        print('image/invent folder already exist')
    else:
        invent_path = ''
        print("!!!Couldn't make directories, I will store all the images in root folder!!!")
        print(exc.message)
try:
    os.mkdir('images/dps')
except OSError as exc:
    if exc.errno == errno.EEXIST:
        print('image/dps folder already exist')
    else:
        dps_path = ''
        print("!!!Couldn't make directories, I will store all the images in root folder!!!")
        print(exc.message)

#################################################################
#                           FLIGHT
#################################################################

print('flight started')
bridge = CvBridge()

#################################################################
#                         TAKE OFF
#################################################################

navigate_wait(z=2.0, frame_id='body', auto_arm=True)
print('lifted off')
navigate_wait(x= 0.0, y=0.0, z=2.0, frame_id='aruco_map')

#################################################################
#                      INVENTARIZATION
#################################################################
balance = 0
cargos = [[0, 'products'], [0, 'clothes'], [0, 'fragile packaging'], [0, 'correspondence']]
if __INVENTARIZATION:

    def detect(img):

        res = cv_test.count_cargo(img)
        if res != 0: 
            if res[1] == 'YELLOW':
                cargos[0][0] += 1

            elif res[1] == 'GREEN':
                cargos[1][0] += 1
                
            elif res[1] == 'BLUE':
                cargos[2][0] += 1

            elif res[1] == 'RED':
                cargos[3][0]+= 1



    navigate_wait(x=0 * 0.9, y= 6 * 0.9, z=2.0, frame_id='aruco_map')
    
    title = 0
    ROWS = 2
    for i in range(ROWS):
        for j in range(4):
            if j == 4:
                continue
            navigate_wait(x=j * 0.9 + 0.45, y= (6 - i) * 0.9 , z=INVENT_HEIGHT, frame_id='aruco_map')
            rospy.sleep(0.5)
            img = take_picture(invent_path + 'cv_' + str(title) + '.png')
            title += 1
            detect(img)

            if __DOUBLE_CHECK:
                navigate_wait(x=j * 0.9 + 0.45, y= (6 - i) * 0.9 , z=INVENT_HEIGHT - 0.5, frame_id='aruco_map')
                rospy.sleep(0.5)
                img = take_picture(invent_path + 'cv_' + str(title) + '.png')
                title += 1
                detect(img)

        if i != ROWS - 1:
            for j in range(8, -1, -1):
                navigate_wait(x=j * 0.45, y= (6 - i) * 0.9 - 0.45, z=INVENT_HEIGHT, frame_id='aruco_map')
                rospy.sleep(0.5)
                img = take_picture(invent_path + str(title) + '.png')
                title += 1
                detect(img)

                if __DOUBLE_CHECK:
                    navigate_wait(x=j * 0.45, y= (6 - i) * 0.9 - 0.45, z=INVENT_HEIGHT -0.5, frame_id='aruco_map')
                    rospy.sleep(0.5)
                    img = take_picture(invent_path + str(title) + '.png')
                    title += 1
                    detect(img)
        
    for cargo in cargos:
        balance +=  cargo[0]
    print('Balance ' + str(balance) + ' cargo')
    print('Type 0: ' + str(cargos[0][0]) + ' cargo')
    print('Type 1: ' + str(cargos[1][0]) + ' cargo')
    print('Type 2: ' + str(cargos[2][0]) + ' cargo')
    print('Type 3: ' + str(cargos[3][0]) + ' cargo')



#################################################################
#                  DRONEPOINT DETECTION
#################################################################
dpoints = []
if __DP_DETECTION:
    dpoints = goDetectDP()

if __DEBUG_DP and not dpoints:
    dpoints = __DEBUG_DPS
    print('debug dps initiated')
print(dpoints)
if len(dpoints) > 2 or len(dpoints) < 1 :
    print('not correct dpoints, going back')
    __DELIVERING == False

#################################################################
#                      DELIVERING
#################################################################

delivered = []

if __DELIVERING and dpoints:
    proc_start = threading.Thread(target=start_dpProcessing)
    proc_start.start()
    rospy.sleep(2)
    print('go check topic')
    rospy.sleep(2)
    rospy.Subscriber('maxRect', String, rect_callback)


    for dpoint in dpoints:
        print('going to the dp')

        navigate_wait(x=0.9 * dpoint[0], y=0.9 * dpoint[1], z=2.5, speed=DELIVERY_SPEED, frame_id='aruco_map')
        if __SIM:
            rospy.sleep(2)
            navigate_wait(x=0.9 * dpoint[0], y=0.9 * dpoint[1], z=1.5, speed=DELIVERY_SPEED, frame_id='aruco_map')
            rospy.sleep(2)
            navigate_wait(x=0.9 * dpoint[0], y=0.9 * dpoint[1], z=0.4, speed=DELIVERY_SPEED, frame_id='aruco_map')
        rospy.sleep(3)

        print('ready to land')

        #land_dpoint()

        print('led is on')
        if dpoint[2] == 0:
            set_effect(r=100, g=100, b=0)  # fill strip with yellow color
            print('D' + str(dpoint[2]) + '_delivered ' + str(cargos[0][1]))
            delivered.append((0, cargos[0][1]))

        elif dpoint[2] == 1:
            set_effect(r=0, g=100, b=0)  # fill strip with green color
            print('D' + str(dpoint[2]) + '_delivered ' + str(cargos[1][1]))
            delivered.append((1, cargos[1][1]))
        elif dpoint[2] == 2:
            set_effect(r=0, g=0, b=100)  # fill strip with blue color
            print('D' + str(dpoint[2]) + '_delivered ' + str(cargos[2][1]))
            delivered.append((2, cargos[2][1]))

        elif dpoint[2] == 3:
            set_effect(r=100, g=0, b= 0) # fill strip with red color
            print('D' + str(dpoint[2]) + '_delivered ' + str(cargos[3][1]))
            delivered.append((3, cargos[3][1]))

        rospy.sleep(10)
        # lifted_off = False
        # for i in range(3):
        #     print('trying to lift off')

        #     telem = get_telemetry(frame_id='aruco_map')
        #     print(telem)
            
        #     navigate_wait(z=1.0, frame_id='body', auto_arm=True)

        #     new_telem = get_telemetry(frame_id='aruco_map')
        #     print(new_telem)

        #     if new_telem.z - telem.z > 0.2:
        #         lifted_off = True
        #         break
        #     print("didn't lifted off")
        #     print('retrying...')
        #     rospy.sleep(2)
        # if not lifted_off:
        #     print('i have got some problem with lifting off :((')
        #     print('flight is over')
        #     sys.exit()

        # print('lifted off')
        set_effect(r=0, g=0, b= 0)
        if __SIM:
            navigate_wait(x=0.9 * dpoint[0], y=0.9 * dpoint[1], z=1.0, speed=DELIVERY_SPEED, frame_id='aruco_map')
        navigate_wait(x=0.9 * dpoint[0], y=0.9 * dpoint[1], z=2.0, speed=DELIVERY_SPEED, frame_id='aruco_map')



#################################################################
#                        GOING LAND
#################################################################

navigate_wait(x=0.0, y=0.0, z=2.0, frame_id='aruco_map')
print('target approached')
navigate_wait(x=0.0, y=0.0, z=0.5, frame_id='aruco_map')
land()

for cargo in delivered:
    print('D' + str(cargo[0]) + '_delivered ' + str(cargos[1]))
    balance -= cargos[1]
print('Balance: ' + str(balance) + ' cargo.')
    
print('flight is over')
proc_start.join()




