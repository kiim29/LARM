import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import math

class Vision(Node):

    def __init__(self):
        super().__init__('vision')
        self.create_subscription( Image, 'img', self.process_img, 10)
        self.bridge = CvBridge()
        self.processedImgPublisher = self.create_publisher( Image, 'procsd_img', 10)
        self.maskImgPublisher = self.create_publisher( Image, 'mask_img', 10)
        self.colaDetectPublisher = self.create_publisher( Point, 'cola_detect', 10)
        self.timerPublishCola=self.create_timer(1, self.publishCola)
        
        self.point = Point()
        self.canPublish = False
        
        self.color=0

        self.lo=np.array([0, 150, 90])
        self.hi=np.array([5, 220, 250])
        self.lo2=np.array([170, 140, 130])
        self.hi2=np.array([180, 210, 200])
        self.lo3=np.array([0, 0, 0])
        self.hi3=np.array([255, 255, 70])

        self.color_info=(0, 0, 255)

        self.hsv_px = [0,0,0]

        # Creating morphological kernel
        self.kernel = np.ones((3, 3), np.uint8)

    def process_img(self, msgImg) :
        self.notreImage = self.bridge.imgmsg_to_cv2(msgImg, 'bgr8')

        frame=self.notreImage
        image=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(image, self.lo, self.hi)
        mask2=cv2.inRange(image, self.lo2, self.hi2)
        blackmask=cv2.inRange(image, self.lo3, self.hi3)
        redmask=cv2.add(mask, mask2)
        redmask=cv2.erode(redmask, self.kernel, iterations=2)
        redmask=cv2.dilate(redmask, self.kernel, iterations=6)
        blackmask=cv2.erode(blackmask, self.kernel, iterations=2)
        blackmask=cv2.dilate(blackmask, self.kernel, iterations=6)
        mask=cv2.bitwise_and(redmask, blackmask)
        mask=cv2.dilate(mask, self.kernel, iterations=16)
        mask=cv2.erode(mask, self.kernel, iterations=12)
        image2=cv2.bitwise_and(frame, frame, mask= mask)

        elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(elements) > 0:
            c=max(elements, key=cv2.contourArea)
            ((x, y), rayon)=cv2.minEnclosingCircle(c)
            if rayon>10 and y > 200:
                cv2.circle(image2, (int(x), int(y)), int(rayon), self.color_info, 2)
                cv2.circle(frame, (int(x), int(y)), 5, self.color_info, 10)
                cv2.line(frame, (int(x), int(y)), (int(x)+150, int(y)), self.color_info, 2)
                cv2.putText(frame, "Objet !!!", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
                self.canPublish = True
                self.point.x = x
                self.point.y = y

        msg_image = self.bridge.cv2_to_imgmsg( frame,"bgr8" )
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image2fr"
        self.processedImgPublisher.publish(msg_image)

        msg_image = self.bridge.cv2_to_imgmsg( image2,"bgr8" )
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image2fr"
        self.maskImgPublisher.publish(msg_image)


    def publishCola(self) :
        if self.canPublish :
            self.colaDetectPublisher.publish(self.point)
            self.canPublish = False

def main(args=None):
    rclpy.init(args=args)
    vision = Vision()
    rclpy.spin(vision)
    vision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()