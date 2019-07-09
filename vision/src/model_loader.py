#!/usr/bin/env python

import sys
import os
import cv2
import numpy as np
from time import time
from openvino.inference_engine import IENetwork, IEPlugin

#path to the plugin "libmyriadPlugin.so"
PLUGINPATH = "/home/pi/OpenVino/inference_engine_vpu_arm/deployment_tools/inference_engine/lib/raspbian_9/armv7l/"


class InferenceModel:
    plugin = None #static plugin as only using one device

    def __init__(self, filename):
        self.load_plugin(filename)

    def load_plugin(self, filename):
        if not InferenceModel.plugin:
            InferenceModel.plugin = IEPlugin(device="MYRIAD", plugin_dirs=PLUGINPATH)
        net = IENetwork(model=filename + '.xml', weights=filename + '.bin')

        self.input_blob = next(iter(net.inputs))
        self.out_blob = next(iter(net.outputs))

        self.shape = net.inputs[self.input_blob].shape
        net.batch_size = 1

        # Loading model to the plugin
        self.exec_net = InferenceModel.plugin.load(network=net)
        del net
        print("Inference model '" + filename + "'loaded.")

    def infer(self, frame):
        return self.infer_on_plugin(frame)

    def infer_on_plugin(self, image):
        n, c, h, w = self.shape
        if image.shape[:-1] != (h, w):
            image = cv2.resize(image, (w, h))
        image = image.transpose((2, 0, 1))  # Change data layout from HWC to CHW
        return self.exec_net.infer(inputs={self.input_blob: [image]})

    def cleanup(self):
        del self.exec_net
        

class Stopwatch:
    def __init__(self):
        self.startTime = None
        self.lapTime = None

    def start(self):
        self.startTime = time()
        self.lapTime = self.startTime

    def lap(self):
        newTime = time()
        print("Lap time: " + str(newTime - self.lapTime) + \
              ", Total: "  + str(newTime - self.startTime))
        self.lapTime = newTime
