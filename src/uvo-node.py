#!/usr/bin/python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rosbag
import rospkg
import yaml
#import matplotlib.pyplot as plt

class uvonode: 
    def match_features(self, img, kp, desc):
        if (self.last_kp is None or self.last_desc is None or len(self.last_kp)==0 or len(self.last_desc)==0):
            return 
        
        # Match descriptors
        matches = self.matcher.match(self.last_desc, desc)

        # Sort them in the order of their distance
        matches = sorted(matches, key = lambda x:x.distance)

        # Show best 10 matches
        img3 = cv2.drawMatches(self.last_img, self.last_kp, img, kp,matches[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
 
        cv2.imshow("img3",img3)
        cv2.waitKey(3)


    def process_image(self, cv_image):
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        keypoints, descriptors = self.orb.detectAndCompute(gray_image, None)

        # Draw the keypoints on the image
        img_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, None, color=(0, 255, 0), flags=0)

        # Show the image with detected features
        # cv2.imshow("Feature Detection", img_with_keypoints)
        # cv2.waitKey(1)

        self.match_features(cv_image, keypoints, descriptors)

        # Save information about previous frame
        self.last_kp = keypoints
        self.last_desc = descriptors
        self.last_img = img_with_keypoints
        

    def callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
            self.process_image(cv_image)
            #self.plot_ground_truth()


        except Exception as e:
            rospy.logerr("Error processing image: %s", e)

    def __init__ (self):
        # Initiate ORB (Oriented FAST and Rotated BRIEF) detector
        self.orb = cv2.ORB_create()

        # Create CvBridge objeect for transforming ROS messages to cv2 image  
        self.bridge = CvBridge()

        # Create BFMatcher object for matching features
        # For binary string based descriptors like ORB, cv.NORM_HAMMING should be used
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        # Keypoint and features of last image
        self.last_kp = None
        self.last_desc = None
        self.last_img = None

        # Camera intrinsic matrix 
        package_path = rospkg.RosPack().get_path('uvo')  

        with open(package_path + '/config/archaeo_camera_calib.yaml', 'r') as file:
            cam = yaml.load(file, Loader=yaml.FullLoader)['cam0']

        fx = cam['intrinsics'][0]
        fy = cam['intrinsics'][1]
        cx = cam['intrinsics'][2]
        cy = cam['intrinsics'][3]
 
        self.K = np.array([[fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]])

        # Topic to recieve images
        self.imageTopic = rospy.get_param("/image_topic", "/camera/image_raw")

        self.sub = rospy.Subscriber(self.imageTopic, Image, self.callback)
 


if __name__ == "__main__":
    rospy.init_node('uvo_node')
    node = uvonode()
    rospy.spin()

    # Release resources when done 
    cv2.destroyAllWindows()


