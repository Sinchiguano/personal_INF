#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 Cesar Sinchiguano <cesarsinchiguano@hotmail.es>
#
# Distributed under terms of the MIT license.

"""

"""

from yolo_detector.srv import AddTwoInts,AddTwoIntsResponse
from yolo_detector.srv import CheckForObjects,CheckForObjectsResponse
import rospy
from std_msgs.msg import String,Bool
from yolo_detector.msg import BoundingBox
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

tmpBox=BoundingBox()

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def handle_detect_bounding(req):
    global tmpBox
    print("it seems to work! tmp2 {}".format(req.trick))
    return CheckForObjectsResponse(tmpBox)

def mainServer():
    rospy.init_node('mainServer')
    sAdd = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints)
    sYolo = rospy.Service('detect_bounding_box',CheckForObjects, handle_detect_bounding)
    print(tmpBox)
    print "Server ready."

    rospy.spin()

if __name__ == "__main__":
    mainServer()






# from yoloDetectorClass import YoloClass
# from yoloDetectorClass import *
# from PIL import Image
#
# '''
# rostopic pub -1 /signal_request_cpp std_msgs/Bool 0
# '''
#
# def main():
#     rospy.init_node('yoloDetectorNode', anonymous=True)
#
#     tfnet = TFNet(options)
#     colors = [tuple(255 * np.random.rand(3)) for _ in range(10)]
#
#     rate = rospy.Rate(10)
#     width=640
#     height=480
#     while not rospy.is_shutdown():
#
#         try:
#             frame=Image.open('/home/casch/ws_moveit/src/yolo_detector/imageTest/tmpImage.png')
#             img=frame.resize((width, height), Image.NEAREST)
#             img.save('/home/casch/ws_moveit/src/yolo_detector/imageTest/imgYolo.png')
#             frame=cv2.imread('/home/casch/ws_moveit/src/yolo_detector/imageTest/imgYolo.png')
#         except:
#             rospy.loginfo('No frame available.')
#             continue
#
#         if frame is None:
#             rospy.loginfo('No valid frame.')
#             time.sleep(0.5)
#             continue
#         else:
#             print('-------------------')
#             if yoloInstance.signalRequestCppRetrieval():
#
#                 try:
#                     stime = time.time()
#                     results = tfnet.return_predict(frame)
#                 except Exception as ex:
#                     print('Detection failed, new attend!',ex)
#                     continue
#                 yoloInstance.do_encapsulationBoundingBox(results)
#                 yoloInstance.detectionDone.data=True
#
#                 yoloInstance.pub_bounding_box.publish(yoloInstance.tmpBoundingBox)
#                 yoloInstance.pub_signalControl.publish(yoloInstance.detectionDone)
#
#                 yoloInstance.do_bounding_boxes(colors[0],results,frame)
#                 yoloInstance.counter+=1
#
#                 cv2.imwrite('/home/casch/ws_moveit/src/yolo_detector/imageResult.png', yoloInstance.frameBoundingBox)
#                 print('FPS: {:.1f} \t Counter: {}'.format((1 / (time.time() - stime)),yoloInstance.counter))
#                 frame=yoloInstance.frameBoundingBox
#
#             command=cv2.waitKey(1) & 0xFF
#             cv2.imshow('frame',frame)
#             yoloInstance.pub_bounding_box.publish(yoloInstance.tmpBoundingBox)
#             yoloInstance.pub_signalControl.publish(yoloInstance.detectionDone)
#             rospy.loginfo('Detection done: {}'.format(yoloInstance.detectionDone.data))
#             rospy.loginfo('BoundingBox data:\n{}'.format(yoloInstance.tmpBoundingBox))
#             yoloInstance.detectionDone.data=False
#
#
#
#         rate.sleep()
#     cv2.destroyAllWindows()
#
# if __name__ == '__main__':
#     yoloInstance=YoloClass()
#     main()
