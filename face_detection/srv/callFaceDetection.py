#!/usr/bin/env python3

# This code was adapted from this project: https://github.com/mauckc/mouth-open

import rospy
import cv2
import os
import dlib
import numpy as np
import pyrealsense2
import tf
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as msg_Image
from face_detection.srv import FaceService, FaceServiceResponse
from snacbot_common.common import *
import rospkg

class Detector:
    def __init__(self):
        rospack = rospkg.RosPack()

        self.bridge = CvBridge() 
        self.service = rospy.Service('Face_Service', FaceService, self.callback)
        self.subImage = rospy.Subscriber("/camera/color/image_raw", msg_Image, self.imageCallback)
        self.subDepth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", msg_Image, self.depthCallback)
        self.s = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.cameraInfoCallback)
        self.rawImage = np.array([])
        self.depthImage = np.array([])
        path = rospack.get_path('face_detection') + "/srv/shape_predictor.dat"
        self.detector = dlib.get_frontal_face_detector()
        self.predictor = dlib.shape_predictor(path)
        self._intrinsics = pyrealsense2.intrinsics()
        self.tf_listener = tf.TransformListener()
        print("we good")
        
    def depthCallback(self, depth_msg):
        try:
            self.depthImage = np.array(self.bridge.imgmsg_to_cv2(depth_msg, "passthrough"))
        except CvBridgeError as e:
            print(e)

    def imageCallback(self, image_msg):
        try:
            self.rawImage = np.array(self.bridge.imgmsg_to_cv2(image_msg, "passthrough"))
        except CvBridgeError as e:
            print(e)


    def callback(self, request):
        # with self.semaphore:
        print("Received")
        gray = cv2.cvtColor(self.rawImage, code=cv2.COLOR_BGR2GRAY)
        faces = self.detector(gray)
        xCenter, yCenter = 0, 0
        for face in faces:
            landmarks = self.predictor(image=gray, box=face)
            ef_x2, ef_x1 = landmarks.part(57).x, landmarks.part(51).x
            ef_y2, ef_y1 = landmarks.part(57).y, landmarks.part(51).y
            # cv2.circle(img=self.rawImage, center=(ef_x2,ef_y2), radius=3, color=(0,255, 0), thickness=-1)
            # cv2.circle(img=self.rawImage, center=(ef_x1,ef_y1), radius=3, color=(0,255, 0), thickness=-1)
            EF = (ef_x2 - ef_x1) * (ef_x2 - ef_x1) + (ef_y2 - ef_y1) * (ef_y2 - ef_y1)
            ab_x2, ab_x1 = landmarks.part(54).x, landmarks.part(48).x
            ab_y2, ab_y1 = landmarks.part(54).y, landmarks.part(48).y
            # cv2.circle(img=self.rawImage, center=(ab_x2,ab_y2), radius=3, color=(0,255, 0), thickness=-1)
            # cv2.circle(img=self.rawImage, center=(ab_x1,ab_y1), radius=3, color=(0,255, 0), thickness=-1)
            AB = (ab_x2 - ab_x1) * (ab_x2 - ab_x1) + (ab_y2 - ab_y1) * (ab_y2 - ab_y1)
            if EF / AB >= 0.25:      
                xCenter = int((ef_x1 + ef_x2 + ab_x1 + ab_x2) / 4)          
                yCenter = int((ef_y1 + ef_y2 + ab_y1 + ab_y2) / 4)
        # cv2.circle(img=self.rawImage, center=(int(xCenter),int(yCenter)), radius=3, color=(255,0, 0), thickness=-1)
        # cv2.imwrite("face.jpg", cv2.cvtColor(self.rawImage,  cv2.COLOR_BGR2RGB))
        x, y, z = convert_depth_to_phys_coord_using_realsense(xCenter, yCenter,  self.depthImage[yCenter][xCenter]/1000, self._intrinsics)
        print(x, y , z)
        #find world frame
        optical_frame_pose = construct_pose(x, y, z, 0, 0, 0, "camera_aligned_depth_to_color_frame")

        self.tf_listener.waitForTransform("camera_aligned_depth_to_color_frame", "world", rospy.Time.now(), rospy.Duration(1.0))
        world_pose = self.tf_listener.transformPose("world", optical_frame_pose)
       
        quat =  tf.transformations.quaternion_from_euler(0, 0, 0)
        world_pose.pose.orientation.x = quat[0]
        world_pose.pose.orientation.y= quat[1]
        world_pose.pose.orientation.z = quat[2]
        world_pose.pose.orientation.w = quat[3]
        return FaceServiceResponse(world_pose)

    
    def cameraInfoCallback(self, cameraInfo):
        self._intrinsics = pyrealsense2.intrinsics()
        self._intrinsics.width = cameraInfo.width
        self._intrinsics.height = cameraInfo.height
        self._intrinsics.ppx = cameraInfo.K[2]
        self._intrinsics.ppy = cameraInfo.K[5]
        self._intrinsics.fx = cameraInfo.K[0]
        self._intrinsics.fy = cameraInfo.K[4]
        #_intrinsics.model = cameraInfo.distortion_model
        self._intrinsics.model  = pyrealsense2.distortion.none 
        self._intrinsics.coeffs = [i for i in cameraInfo.D]  
        self.s.unregister()
                    
                
def main():
    rospy.init_node('face_detection', anonymous=True)
    detec = Detector()
    rospy.spin()
    
if __name__ == '__main__':
    main()
