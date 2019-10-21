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
	self.init =    1                      #calib x, y and theta
	self.initx = 0.0
	self.inity = 0.0
	self.inittheta = 0.0
	self.count = 0
	self.theta = 0.0
	self.x = 0.0
	self.y = 0.0
        self.node_name = "slam_picture"
	self.image = cv2.imread("/home/ubuntu/hackathon/grid.png")
#	self.image = cv2.imread("/home/ubuntu/hackathon/plan2.png")
#	self.image = cv2.imread("/home/ubuntu/hackathon/floorplan417scale.png")
	self.frame = np.array(self.image, dtype=np.uint8)

#	self.image = cv2.imread("/home/ubuntu/hackathon/minions.jpg")

        rospy.init_node(self.node_name)

        # What we do during shutdown
#        rospy.on_shutdown(self.cleanup)

        # Create the OpenCV display window for the RGB image
        self.cv_window_name = self.node_name
        cv2.namedWindow(self.cv_window_name, cv2.WND_PROP_FULLSCREEN)
	cv2.setWindowProperty(self.cv_window_name, cv2.WND_PROP_FULLSCREEN, 1)

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
	self.count = self.count+1

	subN = 50
	projN = 1280
	projM = 720
	projR = float(projN)/float(projM)
	imScale = 1
	subDim = float(projM)/float(imScale)
	res = 0.02702 #0.05
#	res = 0.012002 #0.05
	offx = -47.415398
	offy = -25.599998

	if ((self.count%4) == 0):
		if self.init == 1:
			self.initx = data.pose.position.x
			self.inity = data.pose.position.y
			quaternion = ( data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
			euler = tf.transformations.euler_from_quaternion(quaternion)
			self.inittheta = (euler[2]*180)/3.14
			self.init = 0

    		slamx     =  data.pose.position.x - self.initx
    		slamy     =  data.pose.position.y - self.inity
    		slamq = ( data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w)
        	slame = tf.transformations.euler_from_quaternion(slamq)
        	slamth = (slame[2]*180.0)/3.14 - self.inittheta

#    		rospy.loginfo(rospy.get_caller_id() + "I heard %f %f %f", slamx, slamy, slamth)
		frame = self.frame	
		(imM, imN, imD) = frame.shape

	#	map_origin_x = 25.6
	#	map_origin_y = 25.6
	#	pix_per_m = 20
	#	cx = int((map_origin_x + self.x)*pix_per_m) 
	#	cy = int((map_origin_y - self.y)*pix_per_m)
	#	rospy.loginfo("\n %d  %d", cx, cy)
	#	frame = self.warpImage(frame, cx, cy,int(20*(self.initx-self.x)),int(20*(self.inity+self.y)), (self.inittheta-self.theta) )
		
#		pgmx = (slamx - offx)/res
#		pgmy = (slamy - offy)/res

#		print("slam: %f %f %f" % (slamx, slamy, slamth))

		imx = (slamx/res)*imScale
		imy = (slamy/res)*imScale + imM/2.0
		imth = slamth

		print("img pos: %f %f %f" % (imx, imy, imth))

		#imgx = pgmx*imScale
		#imgy = pgmy*imScale

		#print("Img warp: %d %d %f" % (imgx, imgy, slamth))
	
		frame = self.warpImage(frame, imx,imy, 0,0,-1*slamth)

#		print("Size: %d %d  Pos: %d %d" % (projR*subDim,subDim,slamx/res+(offN), slamy/res+(offM))) 
#		subframe = cv2.getRectSubPix(frame, (int(projR*subDim),int(subDim)), (int(slamx/res+(offN)), int(slamy/res+(offM)))) 

	        # Display the image.
		#subN = 100
		#imgx = int(100) #input corresponding slam value here
		#imgy = int(100) #input corresponding slam value here
#		subframe = cv2.getRectSubPix(frame, (int(subN*projR), int(subN)),(0,100))
		subframe = cv2.getRectSubPix(frame, (int(subN*projR), int(subN)),(int(imy),int(-1*imx)))
		cv2.imshow(self.node_name, subframe)
        
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
