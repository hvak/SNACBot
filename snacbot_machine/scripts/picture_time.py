import rospy
from sensor_msgs.msg import Image 
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

bridge = CvBridge()

rgb_count = 0
depth_count = 0

def rgb_callback(msg):
    global rgb_count
    print("Received an rgb image!")
    cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    cv2.imwrite('img/rgb' + str(rgb_count) + ".jpeg", cv2_img)
    rgb_count += 1

def depth_callback(msg):
    global depth_count
    print("Received a depth image!")
    # cv2_img = bridge.imgmsg_to_cv2(msg, msg.encoding)
    # cv2.imwrite('img/depth' + str(depth_count) + ".jpeg", cv2_img)
    data = np.asarray(msg.data)
    np.save('img/depth' + str(depth_count) + ".np", data)
    depth_count += 1



if __name__ == "__main__":
    rospy.init_node("cheeto_node")
    rate = rospy.Rate(0.5)
    rospy.Subscriber("/camera/color/image_raw", Image, rgb_callback)
    rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, depth_callback)

    while not rospy.is_shutdown():
        rate.sleep()