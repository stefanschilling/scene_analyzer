#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('cob_people_detection_msgs')
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv
import cv2
import os
import sys
import numpy

from std_msgs.msg import String
from cob_people_detection_msgs.msg import ColorDepthImageArray
#import cob_people_detection_msgs

def callback(data):
    global datei
    global face_det
    if datei != data.data:
        print "start of new set, resetting detection counter"
        face_det = 0
        datei = data.data
        rospy.loginfo(rospy.get_caller_id()+"New Set: %s",data.data)
        datei = data.data.rsplit("/",1)
        print datei[1]
def callback2(data):
    global face_det
    global filename
    global datei
    #rospy.loginfo(rospy.get_caller_id()+"CDIA data:" + str(len(data.head_detections)))
    if len(data.head_detections)==1 and len(data.head_detections[0].face_detections) == 1:
        face_det+=1
        print face_det
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(data.head_detections[0].color_image,"rgb8")
        depth = bridge.imgmsg_to_cv2(data.head_detections[0].depth_image,"32FC3")
        #cv2.imshow("image",image)
        cv2.imshow("depth",depth)
        cv2.waitKey()
        print depth.shape
        set_name = datei[1]
        depth_path="/home/stefan/rgbd_db_heads/"+set_name+"/"+filename+"_d.xml"
        img_path="/home/stefan/rgbd_db_heads/"+set_name+"/"+filename+"_c.bmp"
        #path+"/"+str(dir)+"/"+os.path.splitext(file)[0]+".xml"
        depth_slice1 = depth[:,:,0].astype(numpy.float32)
        depth_slice2 = depth[:,:,1].astype(numpy.float32)
        depth_slice3 = depth[:,:,2].astype(numpy.float32)

        for(r,c),value in numpy.ndenumerate(depth_slice1):
            if depth_slice1[r,c]==0:
                depth_slice1[r,c]=numpy.nan
        for(r,c),value in numpy.ndenumerate(depth_slice2):
            if depth_slice2[r,c]==0:
                depth_slice2[r,c]=numpy.nan
        print filename
        #depth_merged = cv2.merge(depth_slice1, depth_slice2, depth_slice3)
        #cv_mat=cv.fromarray(depth_merged)
        #cv.Save(depth_path,cv_mat,"depth")
        #cv2.imwrite(img_path,image)
        #print depth.cols()
        #cv2.imshow("HEAD",image)
        #cv2.waitKey(0)
        
def callback3(data):
    global filename
    filename = data.data
    
def listener():
    global file_name
    # in ROS, nodes are unique named. If two nodes with the same
    # node are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaenously.
    rospy.init_node('set_subscriber', anonymous=True)
    rospy.Subscriber("set_path", String, callback)
    rospy.Subscriber("file_name", String, callback3)
    rospy.Subscriber("/cob_people_detection/face_detector/face_positions", ColorDepthImageArray, callback2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    global face_det
    global datei
    face_det = 0
    datei = "bla"
    listener()
