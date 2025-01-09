#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import rosbag

class uvonode: 

    def process_image(self, cv_image):
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Feature detection using ORB (Oriented FAST and Rotated BRIEF)
        orb = cv2.ORB_create()
        keypoints, descriptors = orb.detectAndCompute(gray_image, None)

        # Draw the keypoints on the image
        img_with_keypoints = cv2.drawKeypoints(cv_image, keypoints, None, color=(0, 255, 0), flags=0)

        # Show the image with detected features
        cv2.imshow("Feature Detection", img_with_keypoints)
        cv2.waitKey(1)

    def callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV format

            #cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            self.process_image(cv_image)

        except Exception as e:
            rospy.logerr("Error processing image: %s", e)

    def __init__ (self):
        self.bridge = CvBridge()
        self.bag = None

        self.imageTopic = rospy.get_param("/image_topic", "/camera/image_raw")
        self.sub = rospy.Subscriber(self.imageTopic, Image, self.callback)
 


if __name__ == "__main__":
    rospy.init_node('uvo_node')
    node = uvonode()
    rospy.spin()

    # ? Release resources when done 
    cv2.destroyAllWindows()

