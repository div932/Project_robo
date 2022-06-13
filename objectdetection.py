import cv2
import numpy as np
import math

img = cv2.imread('CVtask.jpg')
#img= cv2.resize(img,(0,0),fx=0.5,fy=0.5)
cv2.imshow('vv',img)
cv2.waitKey(1000)
ORANGE_MIN = np.array([0, 40, 90])
ORANGE_MAX = np.array([27, 255, 255])
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
_,thresh = cv2.threshold(gray,230,255,cv2.THRESH_BINARY)
#edged = cv2.Canny(gray, 30, 150)
color={'green':[79,209,146],'orange':[9,127,240],'white':[210,222,228],'black':[0,0,0]}


contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
'''cv2.imshow('vv',thresh)
cv2.waitKey(3000)'''

i = 0
blank = np.zeros(img.shape,np.uint8)
for c in contours:
    peri = cv2.arcLength(c, True)
    approx = cv2.approxPolyDP(c, 0.01 * peri, True)

    if len(approx) == 4:
        
        x,y,w,h=cv2.boundingRect(approx)
        aspectratio = float(w)/h
        if aspectratio >=0.95 and aspectratio<=1.05:
            #print(approx)
            cord = [o[0].tolist() for o in approx]
            xmax=cord[0][0]
            xmin=cord[0][0]
            ymax=cord[0][1]
            ymin=cord[0][1]
            for i in cord:
                if i[0]>xmax:
                    xmax = i[0]
                if i[0]<xmin:
                    xmin = i[0]
                if i[1]>ymax:
                    ymax = i[1]
                if i[1]<ymin:
                    ymin = i[1]
            print(xmax,xmin,ymax,ymin)
            to1 = img[ymin:ymax,xmin:xmax]
            

                

            m1=(int((cord[0][0]+cord[1][0])/2),int((cord[0][1]+cord[1][1])/2))
            c=(int((cord[0][0]+cord[2][0])/2),int((cord[0][1]+cord[2][1])/2))
            if (m1[0]-c[0]) != 0:
                theta = math.atan((m1[1]-c[1])/(m1[0]-c[0]))
            else :
                theta = math.pi/(-2)
            print(theta*180/(math.pi))
            for i in color.keys():
                d = np.array(color[i])
                d.reshape((3,))
                if (d==img[c[1],c[0],:]).any():
                    cv2.putText(img,i,c,cv2.FONT_HERSHEY_SIMPLEX,1,(255,0,0))

            print(img[c[1],c[0],:].shape)
            '''cv2.imshow('ff',t)
            cv2.waitKey(1000)'''
            cv2.circle(img,cord[0],5,(0,0,255),-1)
            cv2.drawContours(img,[approx], -1, (255, 0, 0), 3)
            print(cord)
       

    cv2.imwrite('cropped\\' + str(i) + '_img.jpg', img)


cv2.imwrite('new.jpg',img)
cv2.imshow('vv',img)
cv2.waitKey(0)