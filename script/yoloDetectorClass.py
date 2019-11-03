#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 Cesar Sinchiguano <cesarsinchiguano@hotmail.es>
#
# Distributed under terms of the MIT license.
import sys
sys.path.insert(0, '/home/yumi/darkflow/')

import glob
import cv2
from darkflow.net.build import TFNet
import numpy as np
import time
import rospy
from std_msgs.msg import String,Bool
from yolo_detector.msg import BoundingBox
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from cv_bridge import CvBridge, CvBridgeError
# initialize cv bridge
bridge = CvBridge()

options = {
    'model': '/home/casch/darkflow/cfg/tiny-yolo-voc-2c.cfg',
    'load': 9800,
    'threshold': 0.2,
    'gpu': 0.0
}

class YoloClass(object):
    """UniversalRobot"""
    def __init__(self):
        super(YoloClass, self).__init__()
        # In ROS, nodes are uniquely named.
        rospy.Subscriber('/message_topic', String, self.messageControlCallback)
        rospy.Subscriber('/signal_request_cpp', Bool, self.signalRequestCppCallback)
        rospy.Subscriber('/camera_smartek/image_topic', Image, self.smartekCallback)
        #rospy.Subscriber('/yolo_inference/detectionDone', Bool, self.signalStopCppCallback)



        self.pub_bounding_box = rospy.Publisher('/yolo_inference/bounding_box',
                                                BoundingBox,
                                                queue_size=10)
        self.pub_signalControl = rospy.Publisher('/yolo_inference/detectionDone',
                                                Bool,
                                                queue_size=10)

        self.smartekImage=None
        self.signalRequestCpp=False
        self.messageControl=None
        self.signalStopCpp=False

        self.tmpBox=BoundingBox()
        self.detectionDone=Bool(False)
        self.counter=0
        #self.pub_bounding_box.publish(self.tmpBox)

        self.tmpBoundingBox=BoundingBox()
        self.frameBoundingBox=None

    def smartekCallback(self,data):
        try:
          self.smartekImage = bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
          print(e)
    def messageControlCallback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + "message:  %s", data.data)
        self.messageControl=data.data
        #pass
    def signalRequestCppCallback(self,data):
        #rospy.loginfo(rospy.get_caller_id() + "signalControl:  %s", str(data.data))
        self.signalRequestCpp=data.data
    # def signalStopCppCallback(self,data):
    #     #rospy.loginfo(rospy.get_caller_id() + "signalControl:  %s", str(data.data))
    #     self.signalStopCpp=data.data
    # def signalStopCppRetrieval(self):
    #     return self.signalStopCpp
    def messageControlRetrieval(self):
        return self.messageControl
    def signalRequestCppRetrieval(self):
        return self.signalRequestCpp
    def imageRetrieval(self):
        return self.smartekImage

    def do_encapsulationBoundingBox(self,results):
        self.tmpBoundingBox.confidence=results[0]['confidence']
        self.tmpBoundingBox.topleft_x=results[0]['topleft']['x']
        self.tmpBoundingBox.topleft_y=results[0]['topleft']['y']
        self.tmpBoundingBox.bottomright_x=results[0]['bottomright']['x']
        self.tmpBoundingBox.bottomright_y=results[0]['bottomright']['y']
        self.tmpBoundingBox.Class=results[0]['label']

    def do_bounding_boxes(self,color,results,frame):
        tl = (results[0]['topleft']['x'], results[0]['topleft']['y'])
        br = (results[0]['bottomright']['x'], results[0]['bottomright']['y'])
        label = results[0]['label']
        confidence = results[0]['confidence']
        text = '{}: {:.0f}%'.format(label, confidence * 100)
        self.frameBoundingBox = cv2.rectangle(frame, tl, br, color, 5)
        self.frameBoundingBox= cv2.putText(frame, text, tl, cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2)
        #cv2.imshow('frame', frameBoundingBox)
