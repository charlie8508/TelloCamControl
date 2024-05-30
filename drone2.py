from djitellopy import tello
import time
import cv2
import numpy as np
from HandLandmarkModule import handLandmarkDetector, in_circle

################# activate control center view (default=True) ##################
controlCenter = True
################# set up Tello speed ##################
speed = 90

################# set up Flight Control mode from start (default=False) ##################
fControl = False

################# set up Tello ##################
myTello = tello.Tello()
myTello.connect()
print(myTello.get_battery())
myTello.streamon()

################# set up cam and other settings ##################
width = 1200
height = 700
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

BLACK = (0, 0, 0)
myTello.set_speed = 100
ptime = 0
ctime = 0
counter = 0
counterLandTakeoff = 0
detector = handLandmarkDetector()

while True:
    _, img = cap.read()
    img = cv2.resize(img, (width, height))
    
    # Overlay controls on the camera feed
    img2 = img.copy()
    img2 = detector.findHands(img2, draw=False)
    fingerLs = detector.drawFingerPoint(img2)
    
    # Get tello stream
    frame = myTello.get_frame_read().frame
    frame = cv2.resize(frame, (360, 240))
    
    # Get fps
    ctime = time.time()
    fps = 1 / (ctime - ptime)
    ptime = ctime

    # Draw control activation circles
    cv2.putText(img2, str(int(fps)), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, BLACK, 2)
    cv2.circle(img2, (int(width * 0.4), int(height * 0.1)), 25, BLACK, 3)
    cv2.circle(img2, (int(width * 0.6), int(height * 0.1)), 25, BLACK, 3)

    # Check if control mode is activated
    try:
        if in_circle(int(width * 0.4), int(height * 0.1), 25, fingerLs[0]) and in_circle(int(width * 0.6), int(height * 0.1), 25, fingerLs[1]):
            counter += 1
            if counter == 30:
                fControl = not fControl
                print('Control activated', fControl)
        else:
            counter = 0
    except:
        pass

    if fControl:
        cv2.putText(img2, 'CONTROL ACTIVATED', (450, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, BLACK, 3)

        # Left joystick
        cv2.circle(img2, (int(width * 0.3), int(height * 0.45)), 125, BLACK, 15)
        # Right joystick
        cv2.circle(img2, (int(width * 0.7), int(height * 0.45)), 125, BLACK, 15)
        # Land drone circle
        cv2.circle(img2, (int(width * 0.4), int(height * 0.25)), 25, BLACK, 2)
        cv2.putText(img2, 'Land', (int(width * 0.4) - 30, int(height * 0.25) - 40), cv2.FONT_HERSHEY_SIMPLEX, 1, BLACK, 2)
        # Takeoff drone circle
        cv2.circle(img2, (int(width * 0.6), int(height * 0.25)), 25, BLACK, 2)
        cv2.putText(img2, 'Takeoff', (int(width * 0.6) - 40, int(height * 0.25) - 40), cv2.FONT_HERSHEY_SIMPLEX, 1, BLACK, 2)

        # Send landing and takeoff commands
        try:
            if in_circle(int(width * 0.4), int(height * 0.25), 25, fingerLs[0]) or in_circle(int(width * 0.4), int(height * 0.25), 25, fingerLs[1]):
                counterLandTakeoff += 1
                if counterLandTakeoff == 20:
                    myTello.land()
            elif in_circle(int(width * 0.6), int(height * 0.25), 25, fingerLs[0]) or in_circle(int(width * 0.6), int(height * 0.25), 25, fingerLs[1]):
                counterLandTakeoff += 1
                if counterLandTakeoff == 20:
                    myTello.takeoff()
            else:
                counterLandTakeoff = 0
        except:
            pass

        # Send direction/velocity commands
        try:
            if in_circle(int(width * 0.3), int(height * 0.45), 125, fingerLs[0]) and in_circle(int(width * 0.7), int(height * 0.45), 125, fingerLs[1]):
                # Left joystick
                if fingerLs[0][0] > int(width * 0.3):
                    lr = -((int(width * 0.3 - fingerLs[0][0])) / 125)
                else:
                    lr = -(int(width * 0.3 - fingerLs[0][0])) / 125
                if fingerLs[0][1] > int(height * 0.45):
                    fb = (int(height * 0.45 - fingerLs[0][1])) / 125
                else:
                    fb = (int(height * 0.45 - fingerLs[0][1])) / 125

                # Right joystick
                if fingerLs[1][0] > int(width * 0.7):
                    yv = -((int(width * 0.7 - fingerLs[1][0])) / 125)
                else:
                    yv = -(int(width * 0.7 - fingerLs[1][0])) / 125
                if fingerLs[1][1] > int(height * 0.45):
                    ud = (int(height * 0.45 - fingerLs[1][1])) / 125
                else:
                    ud = (int(height * 0.45 - fingerLs[1][1])) / 125

                myTello.send_rc_control(int(lr * speed), int(fb * speed), int(ud * speed), int(yv * speed))
            else:
                myTello.send_rc_control(0, 0, 0, 0)
                cv2.circle(img2, (int(width * 0.3), int(height * 0.45)), 25, BLACK, cv2.FILLED)
                cv2.circle(img2, (int(width * 0.7), int(height * 0.45)), 25, BLACK, cv2.FILLED)
        except:
            cv2.circle(img2, (int(width * 0.3), int(height * 0.45)), 25, BLACK, cv2.FILLED)
            cv2.circle(img2, (int(width * 0.7), int(height * 0.45)), 25, BLACK, cv2.FILLED)
            pass

        # Insert Tello live stream into control center image
        try:
            img2[460:700, 420:780] = frame
        except:
            pass
    else:
        cv2.putText(img2, 'CONTROL DEACTIVATED', (450, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, BLACK, 3)
        cv2.putText(img2, 'To activate Tello hand controller', (340, 300), cv2.FONT_HERSHEY_SIMPLEX, 1, BLACK, 2)
        cv2.putText(img2, 'move both index fingers', (400, 340), cv2.FONT_HERSHEY_SIMPLEX, 1, BLACK, 2)
        cv2.putText(img2, 'into the boxes above', (430, 380), cv2.FONT_HERSHEY_SIMPLEX, 1, BLACK, 2)

        # Insert Tello video stream into control center
        try:
            img2[460:700, 420:780] = frame
        except:
            pass

    cv2.imshow('Control Center', img2)

    if cv2.waitKey(5) & 0xFF == 27:
        myTello.streamoff()
        myTello.land()
        break

cap.release()
