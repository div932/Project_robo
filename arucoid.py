import cv2
from cv2 import BORDER_TRANSPARENT
from cv2 import BORDER_WRAP
from cv2 import BORDER_CONSTANT
from pyparsing import White
import cv2.aruco as aruco
import math
import numpy as np
L=['Ha.jpg','HaHa.jpg','LMAO.jpg','XD.jpg']
iddict = {}

def arucoid(img):
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    key = getattr(aruco,f"DICT_5X5_250")
    arucodict = aruco.Dictionary_get(key)
    p = aruco.DetectorParameters_create()
    (c,i,r)= cv2.aruco.detectMarkers(img,arucodict,parameters=p)
    return (c,i,r)
def arucoprint(img):
    c,i,r=arucoid(img)
    topleft,topright,bottomright,bottomleft=arucocords(img)
    cx = int((topleft[0]+bottomright[0])/2)
    cy = int((topleft[1]+bottomright[1])/2)
    center = (cx,cy)
    cv2.putText(img,str(i),center,0,1,(0,0,0),1)

def arucocords(img):
    (c,i,r)=arucoid(img)
    if len(c)>0:
        i=i.flatten()
        for (markercorner,markerid) in zip(c,i):
            corner = markercorner.reshape((4,2))
            (topleft,topright,bottomright,bottomleft)=corner
            topleft = (int(topleft[0]),int(topleft[1]))
            topright = (int(topright[0]),int(topright[1]))
            bottomleft = (int(bottomleft[0]),int(bottomleft[1]))
            bottomright = (int(bottomright[0]),int(bottomright[1]))
        return topleft,topright,bottomright,bottomleft

def arucoangle(img):
    topleft,topright,bottomright,bottomleft=arucocords(img)
    cx = int((topleft[0]+bottomright[0])/2)
    cy = int((topleft[1]+bottomright[1])/2)
    px = int((topright[0]+bottomright[0])/2)
    py = int((topright[1]+bottomright[1])/2)
    m=(py-cy)/(px-cx)
    theta = math.atan(m)


    center = (cx,cy)
    cv2.circle(img,topright,5,(0,255,0),-1)
    cv2.circle(img,bottomright,5,(255,0,0),-1)
    cv2.circle(img,(0,0),5,(0,0,255),-1)
    cv2.imshow('ar',img)
    cv2.waitKey(3000)
    return center,(theta*180)/math.pi
def rotate_image(image, angle,center):
    rot_mat = cv2.getRotationMatrix2D(center, angle, 1)
    result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR,borderMode=BORDER_CONSTANT,borderValue=(255,255,255))
    return result
def crp(img):
    #t = np.ones((600,600,3))
    '''cv2.imshow('gdff',t)
    cv2.waitKey(1000)'''
    topleft,topright,bottomright,bottomleft=arucocords(img)
    l=[topleft,topright,bottomright,bottomleft]
    xmax=l[0][0]
    xmin=l[0][0]
    ymax=l[0][1]
    ymin=l[0][1]
    for i in l:
        if i[0]>xmax:
            xmax = i[0]
        if i[0]<xmin:
            xmin = i[0]
        if i[1]>ymax:
            ymax = i[1]
        if i[1]<ymin:
            ymin = i[1]
    print(xmax,xmin,ymax,ymin)
    t = img[ymin:ymax,xmin:xmax]
    return t

y = cv2.imread('new.jpg')
c,i,r = arucoid(y)

print(i)
'''for i in L:
    x = cv2.imread(i)

    center,theta=arucoangle(x)
    f=rotate_image(x,(theta),center)
    df=crp(f)
    cv2.imshow('ar',f)
    cv2.waitKey(3000)
    print(df.shape)'''