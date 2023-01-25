#!/usr/bin/env python3
## Doc: https://dev.intelrealsense.com/docs/python2

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import signal, numpy as np
import cv2, rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge

# Realsense Node:
class Realsense(Node):
    def __init__(self):
        super().__init__('realsense')

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        self.bridge=CvBridge()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper( self.pipeline )
        device = self.config.resolve(pipeline_wrapper).get_device()

        self.sensors= []
        print( f"Connect: { str(device.get_info(rs.camera_info.product_line))}" )
        for s in device.sensors:
            info= s.get_info(rs.camera_info.name)
            print( "Name: " + info )
            self.sensors.append( info )
        
        self.processedImgPublisher = self.create_publisher( Image, 'procsd_img', 10)
        self.maskImgPublisher = self.create_publisher( Image, 'mask_img', 10)
        self.colaDetectPublisher = self.create_publisher( Point, 'cola_detect', 10)
        self.cherryDetectPublisher = self.create_publisher( Point, 'cherry_detect', 10)
        
        self.pointCola = Point()
        self.pointCherry = Point()
        self.canPublishCola = False
        self.canPublishCherry = False
        
        self.lo=np.array([0, 150, 90])#red1
        self.hi=np.array([5, 220, 250])
        self.lo2=np.array([170, 140, 130])#red2
        self.hi2=np.array([180, 210, 200])
        self.lo3=np.array([0, 0, 0])#black
        self.hi3=np.array([255, 255, 45])
        self.lo4=np.array([0, 0, 170]) #white
        self.hi4=np.array([255, 70, 255])
        self.lo5=np.array([4, 150, 180]) #orange
        self.hi5=np.array([12, 255, 255])
        self.lo6=np.array([173, 120, 120]) #red for orange bottle
        self.hi6=np.array([178, 180, 185])
        

        self.color_info=(0, 0, 255)

        self.hsv_px = [0,0,0]

        # Creating morphological kernel
        self.kernel = np.ones((3, 3), np.uint8)

    def connect_imgs(self, fps= 60):
        if ("Stereo Module" not in self.sensors) or ("RGB Camera" not in self.sensors) :
            exit(0)
        
        # prepare publisher:
        self.img_pub= self.create_publisher( Image, "img", 10)
        self.depth_pub= self.create_publisher( Image, "depth", 10)

        # enable stream:
        self.config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, fps)
        self.config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, fps)

        # Start streaming
        self.pipeline.start(self.config)

    def read_imgs(self):
        # Wait for a coherent tuple of frames: depth, color and accel
        frames = self.pipeline.wait_for_frames()
        
        align_to = rs.stream.depth
        align = rs.align(align_to)

        color_frame = frames.first(rs.stream.color)
        depth_frame = frames.first(rs.stream.depth)
        self.depth_frame2 = frames.get_depth_frame()

        aligned_frames =  align.process(frames)
        self.aligned_color_frame = aligned_frames.get_color_frame()
        
        # Convert images to numpy arrays
        self.depth_image = np.asanyarray(depth_frame.get_data())
        self.color_image = np.asanyarray(color_frame.get_data())

    def publish_imgs(self):
        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.convertScaleAbs(self.depth_image, alpha=0.03)

        msg_image = self.bridge.cv2_to_imgmsg( self.color_image,"bgr8" )
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "camera_link"
        self.img_pub.publish(msg_image)

        msg_depth = self.bridge.cv2_to_imgmsg(depth_colormap,"8UC1")
        msg_depth = self.bridge.cv2_to_imgmsg(self.depth_image,)
        msg_depth.header.stamp = msg_image.header.stamp
        msg_depth.header.frame_id = "camera_link"
        self.depth_pub.publish(msg_depth)

        if self.canPublishCola :
            self.colaDetectPublisher.publish(self.pointCola)
            self.canPublishCola = False

        if self.canPublishCherry :
            self.cherryDetectPublisher.publish(self.pointCherry)
            self.canPublishCherry = False
        
        msg_image = self.bridge.cv2_to_imgmsg(self.procsdImg,"bgr8" )
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image2fr"
        self.processedImgPublisher.publish(msg_image)

        msg_image = self.bridge.cv2_to_imgmsg(self.maskk,"bgr8" )
        msg_image.header.stamp = self.get_clock().now().to_msg()
        msg_image.header.frame_id = "image2fr"
        self.maskImgPublisher.publish(msg_image)

    def cola_detect(self) :
        frame=self.color_image
        image=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask=cv2.inRange(image, self.lo, self.hi)
        mask2=cv2.inRange(image, self.lo2, self.hi2)
        blackmask=cv2.inRange(image, self.lo3, self.hi3)
        whitemask=cv2.inRange(image, self.lo4, self.hi4)
        redmask=cv2.add(mask, mask2)
        redmask=cv2.erode(redmask,self.kernel, iterations=1)
        redmask=cv2.dilate(redmask,self.kernel, iterations=8)
        blackmask=cv2.erode(blackmask,self.kernel, iterations=2)
        blackmask=cv2.dilate(blackmask,self.kernel, iterations=8)
        whitemask=cv2.erode(whitemask,self.kernel, iterations=1)
        whitemask=cv2.dilate(whitemask,self.kernel, iterations=10)
        mask=cv2.bitwise_and(redmask, blackmask)
        mask=cv2.bitwise_and(whitemask, mask)
        mask=cv2.dilate(mask, self.kernel, iterations=16)
        mask=cv2.erode(mask, self.kernel, iterations=12)
        image2=cv2.bitwise_and(frame, frame, mask= mask)
        
        # Get the intrinsic parameters
        color_intrin = self.aligned_color_frame.profile.as_video_stream_profile().intrinsics

        elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(elements) > 0:
            c=max(elements, key=cv2.contourArea)
            ((x, y), rayon)=cv2.minEnclosingCircle(c)
            if rayon>10 and y > 200:
                cv2.circle(image2, (int(x), int(y)), int(rayon), self.color_info, 2)
                cv2.circle(frame, (int(x), int(y)), 5, self.color_info, 10)
                cv2.line(frame, (int(x), int(y)), (int(x)+150, int(y)), self.color_info, 2)
                cv2.putText(frame, "Objet Nuka Cola!", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
                
                self.pointCola.x = x
                self.pointCola.y = y
                depth = self.depth_frame2.get_distance(int(x), int(y))

                # self.get_logger().info( f"\ndepth:{depth}" )
                dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
                distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))
                # self.get_logger().info( f"\ndistance:{distance}" )
                
                self.pointCola.z = float(distance)
                if 0.0 < distance <= 1.5:
                    self.canPublishCola = True

    def cherry_detect(self) :
        frame=self.color_image
        image=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        whitemask=cv2.inRange(image,self.lo4,self.hi4)
        orangemask=cv2.inRange(image,self.lo5,self.hi5)
        redmaskfororange=cv2.inRange(image,self.lo6,self.hi6)
        orangemask=cv2.erode(orangemask,self.kernel, iterations=2)
        orangemask=cv2.dilate(orangemask,self.kernel, iterations=12)
        whitemask=cv2.erode(whitemask,self.kernel, iterations=1)
        whitemask=cv2.dilate(whitemask,self.kernel, iterations=10)
        redmaskfororange=cv2.erode(redmaskfororange,self.kernel, iterations=0)
        redmaskfororange=cv2.dilate(redmaskfororange,self.kernel, iterations=20)
        mask=cv2.bitwise_and(orangemask, whitemask)
        mask=cv2.bitwise_and(redmaskfororange, mask)
        mask=cv2.dilate(mask,self.kernel, iterations=20)
        mask=cv2.erode(mask,self.kernel, iterations=10)
        image2=cv2.bitwise_and(frame, frame, mask= mask)
        
        # Get the intrinsic parameters
        color_intrin = self.aligned_color_frame.profile.as_video_stream_profile().intrinsics

        elements=cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        if len(elements) > 0:
            c=max(elements, key=cv2.contourArea)
            ((x, y), rayon)=cv2.minEnclosingCircle(c)
            if rayon>15 and y > 200:
                cv2.circle(image2, (int(x), int(y)), int(rayon), self.color_info, 2)
                cv2.circle(frame, (int(x), int(y)), 5, self.color_info, 10)
                cv2.line(frame, (int(x), int(y)), (int(x)+150, int(y)), self.color_info, 2)
                cv2.putText(frame, "Objet Nuka Cherry!", (int(x)+10, int(y) -10), cv2.FONT_HERSHEY_DUPLEX, 1, self.color_info, 1, cv2.LINE_AA)
                
                self.pointCherry.x = x
                self.pointCherry.y = y
                depth = self.depth_frame2.get_distance(int(x), int(y))

                dx ,dy, dz = rs.rs2_deproject_pixel_to_point(color_intrin, [x,y], depth)
                distance = math.sqrt(((dx)**2) + ((dy)**2) + ((dz)**2))
                
                self.pointCherry.z = float(distance)
                if 0.0 < distance <= 1.5:
                    self.canPublishCherry = True

        self.procsdImg = frame
        self.maskk = image2

# Catch Interuption signal:
isOk= True

def signalInteruption(signum, frame):
    global isOk
    print( "\nCtrl-c pressed" )
    isOk= False

signal.signal(signal.SIGINT, signalInteruption)

# Node processes:
def process_img(args=None):
    rclpy.init(args=args)
    rsNode= Realsense()
    rsNode.connect_imgs()
    while isOk:
        rsNode.read_imgs()
        rsNode.cola_detect()
        rsNode.cherry_detect()
        rsNode.publish_imgs()
        rclpy.spin_once(rsNode, timeout_sec=0.01)
    # Stop streaming
    print("Ending...")
    rsNode.pipeline.stop()
    # Clean end
    rsNode.destroy_node()
    rclpy.shutdown()

# script execution:
if __name__ == '__main__' :
    process_img()
