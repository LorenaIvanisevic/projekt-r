#!/usr/bin/python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose
import rosbag
import rospkg
import yaml
import matplotlib.pyplot as plt

class uvonode: 
    def match_features(self, img, kp, desc):

        if (self.last_img is None):
            return None, None
        
        # Match descriptors
        matches = self.matcher.match(self.last_desc, desc)

        # Sort them in the order of their distance
        matches = sorted(matches, key = lambda x:x.distance)


        img3 = cv2.drawMatches(self.last_img,self.last_kp,img,kp,matches[:3],None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
        cv2.imshow("Matched keypoints",img3)


        # Return best 3 matched keypoints
        pts1 = np.float32([self.last_kp[m.queryIdx].pt for m in matches[:3]])
        pts2 = np.float32([kp[m.trainIdx].pt for m in matches[:3]])

        return pts1, pts2



    def process_image(self, cv_image):

        # Process only every nth image
        if self.count % self.rate != 0: 
            return
        
        keypoints, descriptors = self.orb.detectAndCompute(cv_image, None)

        pts1, pts2 = self.match_features(cv_image, keypoints, descriptors)


        if (pts1 is not None and pts2 is not None and len(pts1)==3 and len(pts2)==3):

            img_1 = self.last_img
            img_2 = cv_image

            M = cv2.getAffineTransform(pts1, pts2)
            print ("Affine matrix:\n", M)
            
            # Get inverse affine matrix
            #M_inv = np.linalg.inv( np.insert(M, 2,[0,0,1], axis=0) )
       
            cv2.imshow("Translated image", cv2.warpAffine(img_1, M, (img_1.shape[1], img_1.shape[0])))

            cv2.waitKey(3000)      

        # Save information about previous frame
        if (self.last_img is None or (len(pts1)==3 and len(pts2)==3)):
            self.last_kp = keypoints
            self.last_desc = descriptors
            self.last_img = cv_image
        

    def callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            self.process_image(cv_image)
            self.count+=1        

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
            self.cam = yaml.load(file, Loader=yaml.FullLoader)['cam0']

        fx = self.cam['intrinsics'][0]
        fy = self.cam['intrinsics'][1]
        cx = self.cam['intrinsics'][2]
        cy = self.cam['intrinsics'][3]
 
        self.K = np.array([[fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]])

        # Topic to recieve images
        self.imageTopic = rospy.get_param("/image_topic", "/camera/image_raw")

        self.sub = rospy.Subscriber(self.imageTopic, Image, self.callback)

        self.count = 0
        self.rate = 10

 


if __name__ == "__main__":
    rospy.init_node('uvo_node')
    node = uvonode()
    rospy.spin()

    # Release resources when done 
    cv2.destroyAllWindows()


