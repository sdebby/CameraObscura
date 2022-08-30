# This detects a face and then track it using the 2 axis servo motors using PIGPIO labrary

# DO NOT FORGET TO RUN PROGRAM FIRST
# sudo pigpiod
# TO STOP
# sudo killall pigpiod

import cv2
import sys
import pigpio
import time #(Can be removed if not needed)

# ** parameters
ScreenResizeFactor = 0.5
# setting video resulution
# ScreenRes (X,Y)
#ScreenRes = 1280,720
ScreenRes = 1920,1080
DeltaCenter = 50

# MotorScreenRatio (X,Y) @ Max motor steps = 2500-500
MotorScreenRatio = ScreenRes[0]/2000 , ScreenRes[1]/2000
print('MotorScreenRatio ('+str(MotorScreenRatio[0])+','+str(MotorScreenRatio[1])+')')
OBJETfound = False

#Setting servo motors
#******
#MOTOR X AXIS 18
#MOTOR Y AXIS 13
ServoGPIOX = 18
ServoGPIOY = 13
pi = pigpio.pi()
#******

faceCscade = cv2.CascadeClassifier('/home/pi/opencv/data/haarcascades/haarcascade_frontalface_alt2.xml')
ProfileCscade = cv2.CascadeClassifier('/home/pi/opencv/data/haarcascades/haarcascade_profileface.xml')

video_capture = cv2.VideoCapture(0) # 0 for a build in camera, can also be a video file or use '' for imporintg a movie
tracker = cv2.TrackerKCF_create()
#tracker = cv2.TrackerCSRT_create()

print ("press CTRL + C to quit")

# Exit if video not opened.
if video_capture is None or not video_capture.isOpened():
	print('Warning: unable to open video source(make shure WEBCAM is on)')
	sys.exit()
if not pi.connected:
    print('Warning: unable to open PIGPIO for servo motors - did you run the program first ? \nrun sudo pigpiod')  
    sys.exit()

#set screen diensions
video_capture.set(3,ScreenRes[0])
video_capture.set(4,ScreenRes[1])
#get screen diensions and center
ScreenCenter = (video_capture.get(3)//2) * ScreenResizeFactor , (video_capture.get(4)//2)* ScreenResizeFactor
ServoPosition = [1500,1000]

print ('Screen center x=' + str(ScreenCenter[0]) + ' y=' + str(ScreenCenter[1]))

#Calculate new position for the camera
def CamNewPosition(CenterBox):
    CenterDelta = (ScreenCenter[0]-CenterBox[0] ) * ScreenResizeFactor, (ScreenCenter[1]-CenterBox[1]) * ScreenResizeFactor
    #print (CenterDelta[0] * MotorScreenRatio[0] * ScreenResizeFactor) 
    #print (CenterDelta[1] * MotorScreenRatio[1] * ScreenResizeFactor)

    #print ('Object detect deltaX ('+ str(CenterDelta[0]) + ') deltaY (' + str(CenterDelta[1])+')')
    print ('Object center box X('+ str(CenterBox[0]) + ') Y(' + str(CenterBox[1])+')')
    MovementStep = 50
    #Moves servo and camera
    if abs(CenterDelta[0]) > DeltaCenter :
        if CenterDelta[0] < 0 and ServoPosition[0] < 2450 :
            print('ServoPosX >')
            #ServoPosition[0] -= CenterDelta[0] * MotorScreenRatio[0] * ScreenResizeFactor
            ServoPosition[0] -= MovementStep
        elif ServoPosition[0] > 550:
            print('ServoPosX <')
            #ServoPosition[0] += CenterDelta[0] * MotorScreenRatio[0] * ScreenResizeFactor
            ServoPosition[0] += MovementStep
    if abs(CenterDelta[1]) > DeltaCenter :
        if CenterDelta[1] > 0 and ServoPosition[1] < 2450:
            print('ServoPosY >')
            #ServoPosition[1] -= CenterDelta[1] * MotorScreenRatio[1] * ScreenResizeFactor
            ServoPosition[1] -= MovementStep
        elif ServoPosition[1] > 550:
            print('ServoPosY <')
            #ServoPosition[1] += CenterDelta[1] * MotorScreenRatio[1] * ScreenResizeFactor
            ServoPosition[1] += MovementStep
    CamMove(ServoPosition[0],ServoPosition[1])

#Move servo motors to absulut value
# Start (500-2500) or stop (0) servo pulses on the GPIO
def CamMove(MoveX,MoveY):
    pi.set_servo_pulsewidth(ServoGPIOX, MoveX)
    pi.set_servo_pulsewidth(ServoGPIOY, MoveY)
    #print (str(ServoPosition[0])+ ','+str(ServoPosition[1]))

#Set motors to center
CamMove(ServoPosition[0],ServoPosition[1])

# video manipulation
def VideoManipul (frame) :
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2XYZ)
    resized = cv2.resize(gray, (0,0), fx=ScreenResizeFactor, fy=ScreenResizeFactor)
    equaHostogram = cv2.equalizeHist(resized)
    #horizontal_img = cv2.flip(resized, 0 )
    #vertical_img = cv2.flip(resized, 1 )
    #both_img = cv2.flip(resized, -1 )
    return (equaHostogram)


#loop until user break
while True:
    try:
        # Start timer
        ret, frame = video_capture.read()
        faces = faceCscade.detectMultiScale(VideoManipul(frame), scaleFactor = 1.3, minNeighbors=5)
        profile = ProfileCscade.detectMultiScale(VideoManipul(frame), scaleFactor = 1.3, minNeighbors=5)
        #print('fps'+ str(video_capture.get(cv2.CAP_PROP_FPS))
        if len(faces) > 0 :
            TRACKING = faces[0,0],faces[0,1],faces[0,2],faces[0,3]
            tracker.init(VideoManipul(frame),(TRACKING[0],TRACKING[1],TRACKING[2],TRACKING[3]))
            print('face found')
            OBJETfound = True
        elif len(profile) > 0 :
            TRACKING = profile[0,0],profile[0,1],profile[0,2],profile[0,3]
            tracker.init(VideoManipul(frame),(TRACKING[0],TRACKING[1],TRACKING[2],TRACKING[3]))
            print('profile found')
            OBJETfound = True
        else:
            OBJETfound = False

        #print (OBJETfound)
        while OBJETfound :
            ok, frame = video_capture.read()
            #print('read '+str(ok)+'  ')
            ret, faces1 = tracker.update(VideoManipul(frame))    
            #print('fps'+ str(video_capture.get(cv2.CAP_PROP_FPS))
            NoMoreTrackingFlag = True
            if ret:
                # Tracking success
                p1 = (int(faces1[0]), int(faces1[1]))
                p2 = (int(faces1[0] + faces1[2]), int(faces1[1] + faces1[3]))
                CenterBox = (p1[0]+p2[0])//2 , (p1[1]+p2[1])//2
                #print ('p1'+ str(p1)+' p2'+str(p2))
                #print ('FACE CENTER X(' + str(CenterBox[0]) + ') , Y(' + str(CenterBox[1]) + ')')
                CamNewPosition(CenterBox) 
                NoMoreTrackingFlag = False               
            else:
                # face is missed and not detected by the tracker
                # moveing camera to last position and breaking 
                if NoMoreTrackingFlag: 
                    print('Object not detected by the tracker!')
                    p1 = (int(TRACKING[0]), int(TRACKING[1]))
                    p2 = (int(TRACKING[0] + TRACKING[2]), int(TRACKING[1] + TRACKING[3]))
                    CenterBox = (p1[0]+p2[0])//2 , (p1[1]+p2[1])//2
                    CamNewPosition(CenterBox)
                OBJETfound = False
            #print (str(p1)+"  "+str(p2))
    except KeyboardInterrupt:
        # break when user interapts
        break
print("\nClosing video & releasing motors")
# When everything is done, release the capture
video_capture.release()
cv2.destroyAllWindows()
#release motors
pi.set_servo_pulsewidth(ServoGPIOX, 0)
pi.set_servo_pulsewidth(ServoGPIOY, 0)
pi.stop()