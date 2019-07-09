
import numpy as np, math
from model_loader import InferenceModel
from ssd_helper import *

PATH = "/home/pi/OpenVino/Models/"

#m_input_size = 416

yolo_scale_13 = 13
yolo_scale_26 = 26
yolo_scale_52 = 52

classes = 80
coords = 4
num = 3
anchors = [10,13,16,30,33,23,30,61,62,45,59,119,116,90,156,198,373,326]
anchor_offsets = {yolo_scale_13: 12, yolo_scale_26: 6, yolo_scale_52: 0}


LABELS = ("person", "bicycle", "car", "motorbike", "aeroplane",
          "bus", "train", "truck", "boat", "traffic light",
          "fire hydrant", "stop sign", "parking meter", "bench", "bird",
          "cat", "dog", "horse", "sheep", "cow",
          "elephant", "bear", "zebra", "giraffe", "backpack",
          "umbrella", "handbag", "tie", "suitcase", "frisbee",
          "skis", "snowboard", "sports ball", "kite", "baseball bat",
          "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
          "wine glass", "cup", "fork", "knife", "spoon",
          "bowl", "banana", "apple", "sandwich", "orange",
          "broccoli", "carrot", "hot dog", "pizza", "donut",
          "cake", "chair", "sofa", "pottedplant", "bed",
          "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
          "remote", "keyboard", "cell phone", "microwave", "oven",
          "toaster", "sink", "refrigerator", "book", "clock",
          "vase", "scissors", "teddy bear", "hair drier", "toothbrush")


def EntryIndex(side, lcoords, lclasses, location, entry):
    n = int(location / (side * side))
    loc = location % (side * side)
    return int(n * side * side * (lcoords + lclasses + 1) + entry * side * side + loc)


class YoloV3(InferenceModel):
    def __init__(self):
        super().__init__(PATH + 'frozen_yolo_v3')
        self.size = (416, 416)
        self.MIN_SCORE = 0.65
        self.is_busy = False

    def _ParseYOLOV3Output(self, blob, objects):
        out_blob_h = blob.shape[2]
        out_blob_w = blob.shape[3]
        side = out_blob_h

        anchor_offset = anchor_offsets[side]

        side_square = side * side
        output_blob = blob.flatten()

        for i in range(side_square):
            row = int(i / side)
            col = int(i % side)
            for n in range(num):
                obj_index = EntryIndex(side, coords, classes, n * side * side + i, coords)
                box_index = EntryIndex(side, coords, classes, n * side * side + i, 0)
                scale = output_blob[obj_index]
                if scale < self.MIN_SCORE:
                    continue
                x = (col + output_blob[box_index + 0 * side_square]) / float(side)
                y = (row + output_blob[box_index + 1 * side_square]) / float(side)
                height = math.exp(output_blob[box_index + 3 * side_square]) * anchors[anchor_offset + 2 * n + 1] / float(self.size[1])
                width = math.exp(output_blob[box_index + 2 * side_square]) * anchors[anchor_offset + 2 * n] /float(self.size[0])
                for j in range(classes):
                    class_index = EntryIndex(side, coords, classes, n * side_square + i, coords + 1 + j)
                    prob = scale * output_blob[class_index]
                    if prob < self.MIN_SCORE:
                        continue
                    obj = obj_detection_msg(x, y, height, width, LABELS[j], prob)
                    objects.append(obj)

    def process_image(self, image, header):
        self.is_busy = True
        outputs = self.infer(image)
        self.is_busy = False
        objects = []

        for output in outputs.values():
            self._ParseYOLOV3Output(output, objects)
        objects = filter_overlapping_boxes(objects)
        for obj in objects:
            obj.header = header
        return objects
