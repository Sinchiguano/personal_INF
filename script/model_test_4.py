#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 Cesar Sinchiguano <cesarsinchiguano@hotmail.es>
#
# Distributed under terms of the MIT license.

"""

"""
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
signalRequestCpp=True
messageControl=None


def do_bounding_boxes(color,result,frame):
    tl = (result['topleft']['x'], result['topleft']['y'])
    br = (result['bottomright']['x'], result['bottomright']['y'])
    label = result['label']
    confidence = result['confidence']
    text = '{}: {:.0f}%'.format(label, confidence * 100)
    frame = cv2.rectangle(frame, tl, br, color, 5)
    frame = cv2.putText(frame, text, tl, cv2.FONT_HERSHEY_COMPLEX, 1, (0, 0, 0), 2)
    cv2.imshow('frame', frame)
    return frame

def callback_messageControl(data):
    #rospy.loginfo(rospy.get_caller_id() + "message:  %s", data.data)
    global messageControl
    messageControl=data.data
    #pass
def callback_signalRequestCpp(data):
    #rospy.loginfo(rospy.get_caller_id() + "signalControl:  %s", str(data.data))
    global signalRequestCpp
    signalRequestCpp=data.data
    #pass
def retrieve_messageControl():
    global messageControl
    return messageControl
def retrieve_signalRequestCpp():
    global signalRequestCpp
    return signalRequestCpp

def do_encapsulationBoundingBox(results):
    tmpBoundingBox=BoundingBox()
    tmpBoundingBox.confidence=results[0]['confidence']
    tmpBoundingBox.topleft_x=results[0]['topleft']['x']
    tmpBoundingBox.topleft_y=results[0]['topleft']['y']
    tmpBoundingBox.bottomright_x=results[0]['bottomright']['x']
    tmpBoundingBox.bottomright_y=results[0]['bottomright']['y']
    tmpBoundingBox.Class=results[0]['label']
    return tmpBoundingBox

smartekImage=None
def smartekCallback(data):
    global smartekImage
    try:
        smartekImage = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        print(e)

def retrieve_image():
    global smartekImage
    return smartekImage

def main():

    tfnet = TFNet(options)
    colors = [tuple(255 * np.random.rand(3)) for _ in range(10)]

    rospy.init_node('yolo_detector', anonymous=True)
    rospy.Subscriber('/message_topic', String, callback_messageControl)
    rospy.Subscriber('/signal_request_cpp', Bool, callback_signalRequestCpp)
    rospy.Subscriber('/camera_smartek/image_topic', Image, smartekCallback)

    pub_bounding_box = rospy.Publisher('/yolo_inference/bounding_box', BoundingBox, queue_size=10)
    pub_signalControl = rospy.Publisher('/yolo_inference/detectionDone', Bool, queue_size=10)

    tmpBox=BoundingBox()
    pub_bounding_box.publish(tmpBox)
    detectionDone=Bool(False)
    counter=0
    while (True):
        # Capture 2D-data
        frame=retrieve_image()

        if frame is None:
            print('no Frame!!!')
            continue
        else:
            print('there is frame to work with')
        # if retrieve_signalRequestCpp():
        #     print('Working on the request of c++!')
        #     stime = time.time()
        #     try:
        #         frame=cv2.imread('/home/casch/ws_moveit/src/yolo_detector/script/tmp_img/testImage_copy.png')
        #     except:
        #         print('no image frame available!')
        #         continue
        #
        #     results = tfnet.return_predict(frame)
        #     #do encapsulation and publish after that
        #     tmpBox=do_encapsulationBoundingBox(results)
        #     pub_bounding_box.publish(tmpBox)
        #     detectionDone=True
        #     pub_signalControl.publish(detectionDone)
        #     for color, result in zip(colors, results):
        #         frame_=do_bounding_boxes(color,result,frame)
        #     #time.sleep(0.3)
        #     counter+=1
        #     cv2.imwrite('result/tmp'+'.png', frame_)
        #     print('FPS: {:.1f} \t Counter: {}'.format((1 / (time.time() - stime)),counter))
        # else:
        #     detectionDone=False
        #     print('Waiting for a request from c++!')
        # pub_bounding_box.publish(tmpBox)
        # pub_signalControl.publish(detectionDone)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        cv2.imshow('frame', frame)

    capture.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()
