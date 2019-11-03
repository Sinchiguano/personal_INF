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

options = {
    'model': '/home/casch/darkflow/cfg/tiny-yolo-voc-2c.cfg',
    'load': 9800,
    'threshold': 0.2,
    'gpu': 0.0
}
signalControl=True
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
    rospy.loginfo(rospy.get_caller_id() + "message:  %s", data.data)
    global messageControl
    messageControl=data.data
    #pass
def callback_signalControl(data):
    rospy.loginfo(rospy.get_caller_id() + "signalControl:  %s", str(data.data))
    global signalControl
    signalControl=data.data
    #pass
def retrieve_messageControl():
    global messageControl
    return messageControl
def retrieve_signalControl():
    global signalControl
    return signalControl

def main():
    tfnet = TFNet(options)
    colors = [tuple(255 * np.random.rand(3)) for _ in range(10)]
    #img_ = [ cv2.imread(img)for img in glob.glob('/home/casch/darkflow/testing/testing2/*.png')]

    rospy.init_node('yolo_detector', anonymous=True)
    rospy.Subscriber('/message_topic', String, callback_messageControl)
    rospy.Subscriber('/signal_topic', Bool, callback_signalControl)
    #boundingBoxesPublisher_ = nodeHandle_.advertise<darknet_ros_msgs::BoundingBoxes>(
    # boundingBoxesTopicName, boundingBoxesQueueSize, boundingBoxesLatch);

    counter=0

    while (True):
        stime = time.time()
        # print(retrieve_signalControl())
        # print(retrieve_messageControl())
        try:
            frame=cv2.imread('/home/casch/ws_moveit/src/yolo_detector/script/tmp_img/133.png')
            signalControl_=retrieve_signalControl()
        except:
            print('no image available!')

        if signalControl:
            results = tfnet.return_predict(frame)
            print('result\n {}'.format(results))
            #print(type(results))
            #print('result\n {}'.format((results['topleft']['y']['x'])))
            #print('result\n {}'.format(results['bottomright']))
            for color, result in zip(colors, results):
                frame_=do_bounding_boxes(color,result,frame)
            time.sleep(0.3)
            counter+=1
            if cv2.waitKey(33) == ord('a'):
                break
            #cv2.imwrite('result/tmp'+str(counter)+'.png', frame_)
            cv2.imwrite('result/tmp'+'.png', frame_)
            print('FPS: {:.1f} \t Counter: {}'.format((1 / (time.time() - stime)),counter))
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    capture.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()
