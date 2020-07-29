import cv2, imutils
import numpy as np
import math, time
import ctypes

SendInput = ctypes.windll.user32.SendInput
PUL = ctypes.POINTER(ctypes.c_ulong)

W=0x11
A=0x1E
S=0x1F
D=0x20

Q=0x10
E=0x12
SPACE=0x39
R=0x13

class KeyBdInput(ctypes.Structure):
    _fields_ = [("wVk", ctypes.c_ushort),
                ("wScan", ctypes.c_ushort),
                ("dwFlags", ctypes.c_ulong),
                ("time", ctypes.c_ulong),
                ("dwExtraInfo", PUL)]

class HardwareInput(ctypes.Structure):
    _fields_ = [("uMsg", ctypes.c_ulong),
                ("wParamL", ctypes.c_short),
                ("wParamH", ctypes.c_ushort)]

class MouseInput(ctypes.Structure):
    _fields_ = [("dx", ctypes.c_long),
                ("dy", ctypes.c_long),
                ("mouseData", ctypes.c_ulong),
                ("dwFlags", ctypes.c_ulong),
                ("time",ctypes.c_ulong),
                ("dwExtraInfo", PUL)]

class Input_I(ctypes.Union):
    _fields_ = [("ki", KeyBdInput),
                 ("mi", MouseInput),
                 ("hi", HardwareInput)]

class Input(ctypes.Structure):
    _fields_ = [("type", ctypes.c_ulong),
                ("ii", Input_I)]

def PressKey(hexKeyCode):
    extra = ctypes.c_ulong(0)
    ii_ = Input_I()
    ii_.ki = KeyBdInput( 0, hexKeyCode, 0x0008, 0, ctypes.pointer(extra) )
    x = Input( ctypes.c_ulong(1), ii_ )
    ctypes.windll.user32.SendInput(1, ctypes.pointer(x), ctypes.sizeof(x))

def ReleaseKey(hexKeyCode):
    extra = ctypes.c_ulong(0)
    ii_ = Input_I()
    ii_.ki = KeyBdInput( 0, hexKeyCode, 0x0008 | 0x0002, 0, ctypes.pointer(extra) )
    x = Input( ctypes.c_ulong(1), ii_ )
    ctypes.windll.user32.SendInput(1, ctypes.pointer(x), ctypes.sizeof(x))

def xAxis(angle):
    
    if angle>20:
        PressKey(A)
        time.sleep(.1)
        ReleaseKey(D)
        time.sleep(.1)
        
    elif angle<-20:
        PressKey(D)
        time.sleep(.1)
        ReleaseKey(A)
        time.sleep(.1)
        
    else:
        print('W is pressed')
        PressKey(W)

def yAxis(speed):
    PressKey(W)
    time.sleep(.1)
    ReleaseKey(S)
    time.sleep(.1)

def Brake():
    PressKey(S)
    time.sleep(.1)
    ReleaseKey(W)
    time.sleep(.1)


cap = cv2.VideoCapture(0)
Dir="-->"

while(1):

    _,img=cap.read()
    im=img
    img=img[150:450,80:500]
    im=cv2.rectangle(im,(500,150),(80,450),(0,255,0),2)

    lower=np.array([0,20,150])
    upper=np.array([20,255,255])

    converted=cv2.cvtColor(img,cv2.COLOR_BGR2HSV) #convert BGR image to HSV image
    skinMask=cv2.inRange(converted,lower,upper) 
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
    
    skinMask=cv2.morphologyEx(skinMask,cv2.MORPH_CLOSE,kernel)
    skinMask = cv2.dilate(skinMask, kernel, iterations = 4)
    skinMask = cv2.GaussianBlur(skinMask, (7,7), 0)
    
    skin=cv2.bitwise_and(img,img,mask=skinMask)
    contours = cv2.findContours(skinMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0]

    try:

        cnt = sorted(contours, key=lambda x: cv2.contourArea(x))
        for ind,ct in enumerate(contours):
            M=cv2.moments(contours[ind])
            area=int(M["m00"])
            
            if area in range(6000,13000):
                m1=cv2.moments(contours[0])
                m2=cv2.moments(contours[1])
                
                x1=int(m1["m10"]/m1["m00"])
                y1=int(m1["m01"]/m1["m00"])
                x2=int(m2["m10"]/m2["m00"])
                y2=int(m2["m01"]/m2["m00"])
                
                slope=math.tan(((y2-y1)/(x2-x1)))*100
                slope=round(slope,2)

                if slope>0:
                    Dir="<--"
                else:
                    Dir="-->"

                distance=math.sqrt(((x2-x1)**2) + ((y2-y1)**2))
                distance=round((distance/300)*100,2)

                if(distance>100):
                    distance=100
                if slope>100:
                    slope=100
                elif slope< -100:
                    slope=-100

                cv2.line(im,(x1,y1),(x2,y2),(100,255,0),5) #plot line between centres of two hands
                cv2.putText(im,"Turning:"+Dir+str(slope)+"deg",(50,50),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2)
                cv2.putText(im,"Acceleration:"+(str(distance)),(50,150),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),2)
                xAxis(slope)
                yAxis(distance)

            else:
                if area>13000 and len(contours)==1:
                    cv2.putText(im,"BRAKE",(50,50),cv2.FONT_HERSHEY_SIMPLEX,1,(0,0,255),4)  
                    Brake()
    except:
        pass

    cv2.imshow('main cam',im)
    cv2.imshow('segment',skin)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
