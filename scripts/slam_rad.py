#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
import sys
import cv2
import numpy as np
import time
import tf

class slam_picture():
    def __init__(self):
	# calib x, y, theta
	self.init = 1     
	self.initx = 0.0
	self.inity = 0.0
	self.inittheta = 0.0

	self.count = 0
	self.theta = 0.0
	self.x = 0.0
	self.y = 0.0
        self.node_name = "slam_picture"
	self.image = cv2.imread("/home/ubuntu/hackathon/hack_2.jpg")

        rospy.init_node(self.node_name)

        # What we do during shutdown
#        rospy.on_shutdown(self.cleanup)

        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name
        cv2.namedWindow(self.cv_window_name)#, cv2.WND_PROP_FULLSCREEN)
	#cv2.setWindowProperty(self.cv_window_name, cv2.WND_PROP_FULLSCREEN, 1)

        # Subscribe to slam_out_pose and set callback
        self.image_sub = rospy.Subscriber("/slam_out_pose", PoseStamped, self.slam_callback)

        rospy.loginfo("Waiting for image topics...")

	def cleanup(self):
		print "Shutting down vision node."
		cv2.destroyAllWindows()   

    def warpImage(self, image, cx, cy, tx, ty, angle):
        rot_mat = cv2.getRotationMatrix2D((cx, cy), angle, 1.0)
	rot_mat[0,2] = rot_mat[0,2] + tx;
   	rot_mat[1,2] = rot_mat[1,2] + ty;
        result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
        return result

    def slam_callback(self, data):
	self.count = (self.count+1)%10
	if (self.count == 0):
		if self.init == 1:
			self.initx = data.pose.position.x
			self.inity = data.pose.position.y
			quaternion = ( data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)
			self.inittheta = (euler[2]*180)/3.14
			self.init = 0

    		self.x     =  data.pose.position.x
    		self.y     =  data.pose.position.y
    		quaternion = ( data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
        	euler = tf.transformations.euler_from_quaternion(quaternion)
        	self.theta = (euler[2]*180.0)/3.14

    		rospy.loginfo(rospy.get_caller_id() + "I heard %f %f %f", self.x, self.y, self.theta)
	
		frame = np.array(self.image, dtype=np.uint8)
	
		map_origin_x = 47.415
		map_origin_y = 25.6
		pix_per_m = 20
		cx = int((map_origin_x + self.x)*pix_per_m) 
		cy = int((map_origin_y - self.y)*pix_per_m)
		rospy.loginfo("\n %d  %d", cx, cy)
		frame = self.warpImage(frame, cx, cy,int(20*(self.initx-self.x)),int(20*(self.inity-self.y)), (self.inittheta-self.theta) )
	        # Display the image.
		#subN = 100
		#imgx = int(100) #input corresponding slam value here
		#imgy = int(100) #input corresponding slam value here
		#subframe = cv2.getRectSubPix(frame, (int(subN*1.66666),subN),(imgx,imgy))
		cv2.imshow(self.node_name, frame)
        
       		# Process any keyboard commands
       		self.keystroke = cv2.waitKey(5)
       		if 32 <= self.keystroke and self.keystroke < 128:
			cc = chr(self.keystroke).lower()
        		if cc == 'q':
        		# The user has press the q key, so exit
        			rospy.signal_shutdown("User hit q key to quit.")


def main(args):       
    try:
        slam_picture()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
