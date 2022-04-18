#!/usr/bin/env python3

import rospy
import os

import cv2
import torch
import numpy as np
import pyrealsense2
import tf
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image as msg_Image
from snacknet.srv import SnackGrabService, SnackGrabServiceResponse
from snacbot_common.common import *
import rospkg


class Object_Detector:
    def __init__(self, device):
        rospack = rospkg.RosPack()

        print("SnackNet Created")
        self.bridge = CvBridge()
        self.device = device
        path = rospack.get_path('snacknet') + '/srv/yolov5'
        snacknet_path = rospack.get_path('snacknet') + '/srv/snacnetV2.pt'
        self.model = torch.hub.load(path, 'custom', snacknet_path, source='local').eval().to(device)
        print("Finished loading SnackNet, lets get this bread")
        self.depth = np.array([])
        self.image = np.array([])
        self.service = rospy.Service('Snack_Grab', SnackGrabService, self.handle_snack_grab)
        self.subImage = rospy.Subscriber("/camera/color/image_raw", msg_Image, self.callback)
        self.subDepth = rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", msg_Image, self.depthCallBack)
        self.s = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.cameraInfoCallback)
        self._intrinsics = pyrealsense2.intrinsics()
        self.tf_listener = tf.TransformListener()
    
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
    
    def depthCallBack(self, depth_msg):
        try:
            self.depth = np.array(self.bridge.imgmsg_to_cv2(depth_msg, "passthrough"))
        except CvBridgeError as e:
            print(e)
            
    def callback(self, image_msg):
        try:
            self.image = np.array(self.bridge.imgmsg_to_cv2(image_msg, "passthrough"))
        except CvBridgeError as e:
            print(e)
        

    def handle_snack_grab(self, request):
        output = self.model.forward(self.image)
        if len(output.xyxy) > 0 and len(output.xyxy[0]) > 0:
            index = torch.argmax(output.xyxy[0], dim=0)[4]  
            chosen_object = output.xyxy[0][index]
            bounding_box = [int(chosen_object[0]), 
                            int(chosen_object[1]),
                            int(chosen_object[2]),
                            int(chosen_object[3])]
            if chosen_object[4] > 0.5:
                result = self.pose_estimate(bounding_box)
                #image frame     
                print(result)
                x, y, z = convert_depth_to_phys_coord_using_realsense(result[1][0], result[1][1],  result[0]/1000, self._intrinsics)
                print(x, y , z)
                #find world frame
                optical_frame_pose = construct_pose(x, y, z, 0, 0, 0, "camera_aligned_depth_to_color_frame")

                self.tf_listener.waitForTransform("camera_aligned_depth_to_color_frame", "world", rospy.Time.now(), rospy.Duration(1.0))
                world_pose = self.tf_listener.transformPose("world", optical_frame_pose)
                ang = wrap_pi(result[2] - 1.57 if result[2] > 0 else result[2] + 1.57)
                print("ANGLE = ", ang)
                quat =  tf.transformations.quaternion_from_euler(ang, 1.57, 0)
                world_pose.pose.orientation.x = quat[0]
                world_pose.pose.orientation.y= quat[1]
                world_pose.pose.orientation.z = quat[2]
                world_pose.pose.orientation.w = quat[3]
                #world_pose = self.construct_pose(0,0,0,0,0,0,"camera_optical_frame")
                return SnackGrabServiceResponse(world_pose)
        return construct_pose(0,0,0,0,0,0,"world")
            
    def dist_to_plane(self,x,y,z,A,B,C):
        return abs(-A*x-B*y+z-C)/np.sqrt(pow(A,2)+pow(B,2)+1)

    def pose_estimate(self, bounding_box):
        #Extract valuable info from the bounding box input
        x1=bounding_box[0]
        y1=bounding_box[1]
        x2=bounding_box[2]
        y2=bounding_box[3]
        
        #Crop the array by the bounding box
        depth_arr = self.depth[y1:y2,x1:x2]
        
        #Find nonzero-valued coordinate pairs (x,y,depth_arr[y,x]) and generate a plane        
        shape = depth_arr.shape
        y_lim = shape[1]
        x_lim = shape[0]
        
        #Values at the edges of the image. Keep in mind: y axis is along the top edge of the image, x axis along left edge
        top_edge = depth_arr[0,:]
        left_edge = depth_arr[:,0]
        bottom_edge = depth_arr[shape[0]-1,:]
        right_edge = depth_arr[:,shape[1]-1]
        
        #Create a set of coordinates to fit a plane to
        plane_coords = set()
        thresh = 20 # If the value of the edge cell is over this threshold, its coordinate will be counted on the plane.
        for y in range(y_lim):
            if top_edge[y] > thresh:
                plane_coords.add((0,y,top_edge[y]))
            if bottom_edge[y] > thresh:
                plane_coords.add((x_lim - 1,y,bottom_edge[y]))
        for x in range(x_lim):
            if left_edge[x] > thresh:
                plane_coords.add((x,0,left_edge[x]))
            if right_edge[x] > thresh:
                plane_coords.add((x,y_lim - 1,right_edge[x]))
        
        #Fit a plane to the set of points on the edge of the bounding box
        A = np.matrix(list(plane_coords))
        A[:,2]=1
        B = np.matrix(np.array(list(plane_coords))[:,2]).T
        X = (A.T*A).I*A.T*B
        
        #ax+by+c=z
        a = X[0,0]
        b = X[1,0]
        c = X[2,0]
        
        #Create the object mask
        bool_array = np.full((x_lim, y_lim), False)
        height_thresh = 10
        for x in range(x_lim):
            for y in range(y_lim):
                dist = self.dist_to_plane(x,y,depth_arr[x,y],a,b,c)
                if depth_arr[x,y] > thresh and dist > height_thresh:
                    bool_array[x,y] = True
                    
        #Get coordinates of nonzero elements in bool array
        p_x,p_y=np.nonzero(bool_array)
        
        #center of mass
        c_x = (x2-x1)/2
        c_y = (y2-y1)/2
        
        #arrays of points
        p_x = p_x - np.mean(p_x)
        p_y = p_y - np.mean(p_y)
        coords = np.vstack([p_x, p_y])
        
        #Find a vector in the direction of the object length wise using covariance matrix and eigenvectors/values
        cov = np.cov(coords)
        evals, evecs = np.linalg.eig(cov)
        sort_indices = np.argsort(evals)[::-1]
        y_v1, x_v1 = evecs[:, sort_indices[0]]
        
        #Return: depth of plane at (c_x,x_y), coord of object center relative to image top right corner , image orientation
        depth_of_plane_at_obj = a*c_x+b*c_y+c
        center_of_obj_in_image_coords = (x1+int(np.round(c_x)), y1+int(np.round(c_y)))
        theta_of_obj_to_image_x = np.arctan2(y_v1, x_v1)
        return (depth_of_plane_at_obj, center_of_obj_in_image_coords, theta_of_obj_to_image_x)

def main():
    torch.cuda.empty_cache()
    rospy.init_node('Snack_Node', anonymous=True)
    device = None
    if torch.cuda.is_available():
        device = torch.device('cuda:0')
        print("Using GPU!")
    else:
        device = torch.device('cpu')
        print("Warning: Could not find GPU")
    od = Object_Detector(device)
    rospy.spin()

if __name__ == '__main__':
    main()
