
#!/usr/bin/env python3


import roslib
#roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist

class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/robot/camera1/image_raw', Image, self.callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        print("init")

    def callback(self, data):
        try:
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            print("image recieved")
        except CvBridgeError as e:
            print(e)
            return

        lower_blue = np.array([100,150,0])
        upper_blue = np.array([140,255,255])
       #Converting the frame from RGB format to HSV format (I read that HSV corresponds better to how humans see colour and is better than RGB in general for image processing)
        frame_in_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #Getting the height, width and depth of the frame
        height, width, depth = np.shape(frame_in_hsv)

    #Cropping the frame because the bounding rectangle tilts and gets too large on curves
        cropped_frame = frame_in_hsv[int(height*0.7):,:,:]

    #Creating a binary mask in order to identify only the parts of the image that are in the blue range specified
        binary_mask = cv2.inRange(cropped_frame, lower_blue, upper_blue)

    #Finding contours on the binary mask
        contours, hierarchy = cv2.findContours(binary_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #Filtering contours above a certain area so that other insignificant contours do not get identified (other side of the tape)
        filtered_contours = [contour for contour in contours if cv2.contourArea(contour) > 1]

        print(filtered_contours)
    #This is to draw the circle on the midpoint of the bounding rectangle of the contour identified in the cropped frame
        for contour in filtered_contours:
            x, y, w, h = cv2.boundingRect(contour)
            print("postcon")
            centre_x =  int(x+w / 2)
            centre_y =  int((height*0.9)+y+h / 2)

            linear_velocity = 0.4  # Adjust the value as needed
            angular_velocity = 0.004 * (-centre_x + width / 2)
        
    # Create Twist message and publish
            twist_msg = Twist()
            twist_msg.linear.x = linear_velocity
            twist_msg.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist_msg)
            print("abcd")

def main():
    rospy.init_node('image_converter', anonymous=True)
    print("node initiallized")
    ic = image_converter()
    print("pubsub init")

    rate = rospy.Rate(10)  # 10 Hz, adjust as needed

    while not rospy.is_shutdown():
        try:
            rate.sleep()
        except KeyboardInterrupt:
            print("Shutting down")
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


