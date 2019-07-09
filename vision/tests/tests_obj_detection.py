#!/usr/local/bin/python3

import unittest
import cv2
import os
import sys
import numpy as np

sys.path.insert(0, os.getcwd() + '/src')

from tiny_yolo_v3 import TinyYoloV3
from yolo_v3 import YoloV3
from ssdlite_mobilenet import SSDlite
from std_msgs.msg import Time
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from obj_detection import ObjectDetector, ImageWithTransform


def find_object(detections, label, xmin, ymin, xmax, ymax, tolerance=.03):
    for detection in detections:
        if detection.class_label == label and \
                within_tolerance(detection.xmin, xmin, tolerance) and \
                within_tolerance(detection.ymin, ymin, tolerance) and \
                within_tolerance(detection.xmax, xmax, tolerance) and \
                within_tolerance(detection.ymax, ymax, tolerance):
            return True
    return False


def within_tolerance(val, expected_val, tolerance):
    return abs(val - expected_val) <= tolerance


class SSDTest(unittest.TestCase):
    def setUp(self):
        self.image = cv2.imread("tests/dog_yolo.jpg", 1)
        self.matrix4x4 = np.eye(4)
        self.snap_time = Time(0)
        self.cmp_img_msg = CompressedImage()

    def tearDown(self):
        pass

    def testObjDetectionNode(self):
        o = ObjectDetector(SSDlite())
        cmp_img = CompressedImage(data=np.array(cv2.imencode(".jpg", self.image)[1]).tostring())
        cmp_img.header.stamp = self.snap_time
        matrix4x4 = np.eye(4)
        iwt = ImageWithTransform(compressedImage=cmp_img,
                                 cameraInfo=CameraInfo(),
                                 flat_transform=np.reshape(matrix4x4, 16))

        o.call_model(iwt)
        self.assertTrue(len(o.database.observations["dog"]) == 1)

    def testSSDlite(self):
        self._model_test(SSDlite())

    def testYolo(self):
        self._model_test(YoloV3())

    def testTinyYolo(self):
        self._model_test(TinyYoloV3(), tolerance_factor=2.5)

    def testMarkers(self):
        o = ObjectDetector(SSDlite())
        cmp_img = CompressedImage(data=np.array(cv2.imencode(".jpg", self.image)[1]).tostring())
        cmp_img.header.stamp = self.snap_time
        matrix4x4 = np.eye(4)
        iwt = ImageWithTransform(compressedImage=cmp_img,
                                 cameraInfo=CameraInfo(),
                                 flat_transform=np.reshape(matrix4x4, 16))

        o.call_model(iwt)
        markers = o.get_markers(5)
        self.assertTrue(len(markers) == 3)


    def _model_test(self, model, tolerance_factor=1.0):
        obj_detections = model.process_image(self.image, self.cmp_img_msg.header)
        print(obj_detections)
        self.assertTrue(find_object(obj_detections, "dog", 0.16, 0.33, 0.41, 0.9, .05*tolerance_factor))
        self.assertTrue(find_object(obj_detections, "car", 0.60, 0.14, 0.90, 0.30, 0.03*tolerance_factor) or
                        find_object(obj_detections, "truck", 0.60, 0.14, 0.90, 0.30, 0.03*tolerance_factor))
        self.assertTrue(find_object(obj_detections, "bicycle", 0.16, 0.20, 0.74, 0.73, 0.04*tolerance_factor))
        self.assertFalse(find_object(obj_detections, "person", 0.17, 0.20, 0.74, 0.75, 0.03*tolerance_factor))
        model.cleanup()

if __name__ == '__main__':
    unittest.main()
