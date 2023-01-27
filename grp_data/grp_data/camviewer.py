
import cv2
import numpy as np

def souris(event, x, y, flags, param):
    global lo, hi, color, hsv_px

    if event == cv2.EVENT_MOUSEMOVE:
        # Conversion des trois couleurs RGB sous la souris en HSV
        px = frame[y,x]
        px_array = np.uint8([[px]])
        hsv_px = cv2.cvtColor(px_array,cv2.COLOR_BGR2HSV)

    if event==cv2.EVENT_MBUTTONDBLCLK:
        color=image[y, x][0]

    if event==cv2.EVENT_LBUTTONDOWN:
        if color>0:
            color-=1

    if event==cv2.EVENT_RBUTTONDOWN:
        if color<250:
            color+=5

    lo[0]=color
    hi[0]=color+5

color=0



lo=np.array([0, 150, 70]) #red1
hi=np.array([5, 240, 250])
lo2=np.array([170, 130, 120]) #red2
hi2=np.array([180, 220, 210])
lo3=np.array([0, 0, 0]) #black
hi3=np.array([255, 255, 70])

lo4=np.array([0, 0, 170]) #white
hi4=np.array([255, 60, 255])
lo5=np.array([10, 150, 180]) #orange
hi5=np.array([25, 255, 255])
lo6=np.array([173, 120, 120]) #red for orange bottle
hi6=np.array([178, 180, 185])

color_info=(0, 0, 255)

def trackbar_callback(value):
    global lo3, hi3
    lo3=np.array([0, 0, 0])
    hi3=np.array([255, 255, value])


cap=cv2.VideoCapture(6)
cv2.namedWindow('Camera')
cv2.setMouseCallback('Camera', souris)
# cv2.namedWindow('Trackbar')
# cv2.createTrackbar('Value', 'Trackbar', 55, 255, trackbar_callback)
hsv_px = [0,0,0]

# Creating morphological kernel
kernel = np.ones((3, 3), np.uint8)

while True:
    ret, frame=cap.read()
    image=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask=cv2.inRange(image,lo,hi)
    mask2=cv2.inRange(image,lo2,hi2)
    blackmask=cv2.inRange(image,lo3,hi3)
    whitemask=cv2.inRange(image,lo4,hi4)
    orangemask=cv2.inRange(image,lo5,hi5)
    redmaskfororange=cv2.inRange(image,lo6,hi6)
    redmask=cv2.add(mask, mask2)
    redmask=cv2.erode(redmask,kernel, iterations=1)
    redmask=cv2.dilate(redmask,kernel, iterations=8)
    blackmask=cv2.erode(blackmask,kernel, iterations=2)
    blackmask=cv2.dilate(blackmask,kernel, iterations=8)
    orangemask=cv2.erode(orangemask,kernel, iterations=2)
    orangemask=cv2.dilate(orangemask,kernel, iterations=12)
    whitemask=cv2.erode(whitemask,kernel, iterations=1)
    whitemask=cv2.dilate(whitemask,kernel, iterations=10)
    redmaskfororange=cv2.erode(redmaskfororange,kernel, iterations=0)
    redmaskfororange=cv2.dilate(redmaskfororange,kernel, iterations=20)
    mask=cv2.bitwise_and(redmask, blackmask)
    mask=cv2.bitwise_and(whitemask, mask)

    # mask=cv2.bitwise_and(orangemask, whitemask)
    # mask=cv2.bitwise_and(redmaskfororange, mask)
    mask=cv2.dilate(mask,kernel, iterations=20)
    mask=cv2.erode(mask,kernel, iterations=10)
    image2=cv2.bitwise_and(frame, frame, mask= mask)
    cv2.putText(frame, "Couleur: {:d}".format(color), (10, 30), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)

    # Affichage des composantes HSV sous la souris sur l'image
    pixel_hsv = " ".join(str(values) for values in hsv_px)
    font = cv2.FONT_HERSHEY_SIMPLEX
    # cv2.putText(frame, "px HSV: "+pixel_hsv, (10, 260),
    #            font, 1, (255, 255, 255), 1, cv2.LINE_AA)
    
    elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if len(elements) > 0:
        c=max(elements, key=cv2.contourArea)
        ((x, y), rayon)=cv2.minEnclosingCircle(c)
        if rayon>20 and y > 200:
            cv2.circle(image2, (int(x), int(y)), int(rayon), color_info, 2)
            cv2.circle(frame, (int(x), int(y)), 5, color_info, 10)
            cv2.line(frame, (int(x), int(y)), (int(x)+150, int(y)), color_info, 2)
            cv2.putText(frame, "Objet !!!", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, color_info, 1, cv2.LINE_AA)

    cv2.imshow('Camera', frame)
    # cv2.imshow('procsd', image2)
    cv2.imshow('red mask', redmask)
    cv2.imshow('black mask', blackmask)
    cv2.imshow('white mask', whitemask)
    cv2.imshow('Mask', mask)
    # cv2.imshow('orange mask', orangemask)

    if cv2.waitKey(1)&0xFF==ord('q'):
        break
cap.release()
cv2.destroyAllWindows()