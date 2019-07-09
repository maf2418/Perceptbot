#!/usr/bin/env python3

import sys
from model_loader import InferenceModel, Stopwatch
from coco_labels import COCO_LABELS #list with mappings of COCO label strings
from perceptbot_camera.msg import ObjDetection
from ssd_helper import *

def label(index):
    if index>len(COCO_LABELS):
        print("TOO HIGH INDEX " + str(index))
    return COCO_LABELS[index]


class SSDlite(InferenceModel):
    def __init__(self):
        super().__init__(PATH + 'ssdlite_mobilenet_v2_coco')
        self.size = (300, 300)
        self.MIN_SCORE = 0.5
        self.is_busy=False

    def process_image(self, frame, header):
        self.is_busy = True
        outs = self.infer(frame)
        outs = outs[self.out_blob]
        self.is_busy = False
        detections = [(label(int(values[1])), values[2:]) for values in outs[0][0] if values[2] > self.MIN_SCORE]
        objects = []
        for detection in detections:
            objects.append(ObjDetection(header=header,
                           class_label = detection[0],
                           confidence = detection[1][0],
                           xmin = detection[1][1],
                           ymin = detection[1][2],
                           xmax = detection[1][3],
                           ymax = detection[1][4]))

        objects = filter_overlapping_boxes(objects)
        return objects
