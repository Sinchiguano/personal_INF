#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 Cesar Sinchiguano <cesarsinchiguano@hotmail.es>
#
# Distributed under terms of the MIT license.

"""
rostopic pub -1 /signal_request_cpp std_msgs/Bool 0
"""
from yoloDetectorClass import YoloClass
from yoloDetectorClass import *
from yolo_detector.srv import AddTwoInts,AddTwoIntsResponse
from yolo_detector.srv import CheckForObjects,CheckForObjectsResponse
from std_msgs.msg import String,Bool
from yolo_detector.msg import BoundingBox
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from PIL import Image
import rospy



import numpy as np
import matplotlib.pyplot as plt
from PIL import Image

colors = [tuple(255 * np.random.rand(3)) for _ in range(10)]
width=640
height=480
tfnet = TFNet(options)
yoloInstance=YoloClass()

def handle_detect_bounding(req):
    #if req.data:
    stime = time.time()
    frame=Image.open('/home/casch/ws_moveit/src/yolo_detector/imageTest/tmpImage.png')
    img=frame.resize((width, height), Image.NEAREST)
    img.save('/home/casch/ws_moveit/src/yolo_detector/imageTest/imgYolo.png')
    frame=cv2.imread('/home/casch/ws_moveit/src/yolo_detector/imageTest/imgYolo.png')
    results = tfnet.return_predict(frame)
    yoloInstance.detectionDone.data=True
    #---------------------------------------------------------------
    yoloInstance.do_encapsulationBoundingBox(results)
    yoloInstance.do_bounding_boxes(colors[0],results,frame)
    yoloInstance.counter+=1
    cv2.imwrite('/home/casch/ws_moveit/src/yolo_detector/imageResult/imageResult.png', yoloInstance.frameBoundingBox)
    print('FPS: {:.1f} \t Counter: {}'.format((1 / (time.time() - stime)),yoloInstance.counter))
    # plt.imshow(yoloInstance.frameBoundingBox)
    # plt.draw()
    # plt.pause(3)
    # plt.close()
    return CheckForObjectsResponse(yoloInstance.detectionDone.data,yoloInstance.tmpBoundingBox)

def mainServer():
    rospy.init_node('mainServer')
    sYolo = rospy.Service('detect_bounding_box',CheckForObjects, handle_detect_bounding)
    print "YoloDetector ready!"
    rospy.spin()

if __name__ == "__main__":
    mainServer()
