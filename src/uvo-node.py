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

        # Return best 3 matched keypoints
        pts1 = np.float32([self.last_kp[m.queryIdx].pt for m in matches[:3]])
        pts2 = np.float32([kp[m.trainIdx].pt for m in matches[:3]])

        return pts1, pts2



    def process_image(self, cv_image):
        # Process only every nth image
        if self.count % self.pano_rate != 0: 
            return

        
        keypoints, descriptors = self.orb.detectAndCompute(cv_image, None)

        # Draw the keypoints on the image
        # img_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, None, color=(0, 255, 0), flags=0)
        # cv2.imshow("img", img_with_keypoints)
        # cv2.waitKey(3)

        pts1, pts2 = self.match_features(cv_image, keypoints, descriptors)


        if (pts1 is not None and pts2 is not None):

            img_1 = self.last_img
            img_2 = cv_image

            M = cv2.getAffineTransform(pts1, pts2)
            print ("Affine matrix:\n", M)
            
            # get inverse
            #M_inv = np.linalg.inv( np.insert(M, 2,[0,0,1], axis=0) )

            if self.M is not None: 
                #self.M = self.M @ M
                self.M = M
                print()
            else:
                self.M = M
            

            #warp_dst = cv2.warpAffine(img_1, self.M, (img_1.shape[1], img_1.shape[0]))

            ## transform 2nd image
           # img_2 = cv2.warpAffine(img_2, self.M[:2], (img_2.shape[1], img_2.shape[0]))

            #result = cv2.addWeighted(self.pano, 0.5, inverse_transformed_image, 0.5, 0)
            
            #transformed_pano =  cv2.warpAffine(self.pano, M[:2], (self.pano.shape[1], self.pano.shape[0]))
            
            #result = cv2.addWeighted(transformed_pano, 0.5, img_2 , 0.5, 0)
            #img_1[:img_2.shape[0], :img_2.shape[1]] = img_2
            
            #self.pano = img_1

            cv2.imshow("Translated image", cv2.warpAffine(img_2, self.M, (img_2.shape[1], img_2.shape[0])))

            cv2.waitKey(1000)      

        
    
        # Save information about previous frame
        self.last_kp = keypoints
        self.last_desc = descriptors
        self.last_img = cv_image
        self.pano_rate = 10
        self.M = None

        

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

        self.imgs = []
        self.count = 0
        self.pano_rate = 10
        self.M = None
        self.pano = None

 


if __name__ == "__main__":
    rospy.init_node('uvo_node')
    node = uvonode()
    rospy.spin()

    # Release resources when done 
    cv2.destroyAllWindows()


