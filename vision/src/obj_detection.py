#!/usr/bin/env python3

import sys
import rospy
import cv2
import numpy as np

from geometry import CameraObj
from ssdlocalization import Observation, Database, vector_to_msg
from perceptbot_camera.msg import ObjDetection, ImageWithTransform
from image_geometry import PinholeCameraModel as Camera
from model_loader import Stopwatch
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Pose
from time import sleep

from ssdlite_mobilenet import SSDlite
from yolo_v3 import YoloV3
from tiny_yolo_v3 import TinyYoloV3

# dictionary of model classes available to object detector for lookup with launch param
# pass in string name when loading module to use appropriate SSD model
models = {"yolo": YoloV3, "tiny_yolo": TinyYoloV3, "ssdlite": SSDlite}


class ObjectDetector:
    def __init__(self, model):
        self.database = Database()
        self.stopwatch = Stopwatch()
        self.stopwatch.start()
        self.model = model
        self.stopwatch.lap()
        self.pub = None
        self.broadcast_markers = False
        self.vis_pub = None
        rospy.init_node('obj_detection', anonymous=True)

    def call_model(self, iwt):
        if self.model.is_busy:
            return
        img_msg = iwt.compressedImage
        is_stable = iwt.is_stable
        camera_transform = np.reshape(np.array(iwt.flat_transform), (4, 4))
        str_msg = img_msg.data
        buf = np.ndarray(shape=(1, len(str_msg)), dtype=np.uint8, buffer=img_msg.data)
        image = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)

        obj_detections = self.model.process_image(image, img_msg.header)
        if obj_detections:
            camera_obj = CameraObj(camera_transform, img_msg.header.stamp, image)
            camera_obj.set_camera_fov(110, 88)  #THIS HARD CODE NEEDS TO BE DONE AS VARIABLE
            self.publish_messages(obj_detections)
            obs = [Observation.from_ros_msg(camera_obj, obj) for obj in obj_detections]
            self.show_observations(obs, is_stable)
            if is_stable:
                self.database.match_observations(obs)

    def publish_messages(self, obj_detections):
        if self.pub:
            for obj in obj_detections:
                self.pub.publish(obj)

    # turn on broadcasts if being used in ROS context
    def activate_ROS_communication(self, broadcast_markers=True, refresh_duration=5):
        self.pub = rospy.Publisher('/obj_detection', ObjDetection, queue_size=50)
        self.listen()
        self.broadcast_markers = broadcast_markers
        if broadcast_markers:
            self.vis_pub = rospy.Publisher('/visualization_marker_array', MarkerArray, queue_size=1)
            self.show_matches(refresh_duration)
        else:
            rospy.spin()

    def listen(self):
        # buffer may need to be very large to overcome OS queue and ensure only latest
        # message is used...
        rospy.Subscriber('/iwt_camera_stream', ImageWithTransform,
                         callback=self.call_model, queue_size=1, buff_size=2000000)

    #publish RVIZ markers for matches
    def show_matches(self, refresh_duration=5):
        lifetime = rospy.Duration(refresh_duration + 10)
        while not rospy.is_shutdown():
            time = rospy.Time.now()
            #  HACK FOR NOW TO NOT SHOW PEOPLE
            markers = [self.make_match_marker(key, match, time, lifetime)
                       for key, keyList in self.database.matches.items()
                       for match in keyList
                       if key != "person"]
            if markers:
                self.vis_pub.publish(MarkerArray(markers))
                self.vis_pub.publish(MarkerArray(self.annotate_matches(markers)))
            sleep(refresh_duration)

    #publish RVIZ markers for observations
    def show_observations(self, observations, is_stable):
        if self.broadcast_markers and observations:
            time = rospy.Time.now()
            lifetime = rospy.Duration(30 if is_stable else 2) #unstable observations only shown briefly
            markers = [self.make_observation_marker(obs, time, lifetime, is_stable) for obs in observations]
            self.vis_pub.publish(MarkerArray(markers))
            self.vis_pub.publish(MarkerArray(self.annotate_observations(markers)))

    # add text markers for each match
    @staticmethod
    def annotate_matches(marker_list):
        for marker in marker_list:
            marker.ns = "text-" + marker.ns
            marker.type = 9
            pose = Pose()
            pose.position.x = marker.pose.position.x
            pose.position.y = marker.pose.position.y
            pose.position.z = marker.pose.position.z + marker.scale.z + 0.1
            marker.pose = pose
            marker.scale.z = 0.15
        return marker_list

    # add text markers for observations
    @staticmethod
    def annotate_observations(marker_list):
        for marker in marker_list:
            marker.ns = "text-" + marker.ns
            marker.type = 9
            marker.pose = Pose()
            marker.pose.position.x = marker.points[1].x
            marker.pose.position.y = marker.points[1].y
            marker.pose.position.z = marker.points[1].z
            marker.scale.z = 0.1
        return marker_list

    # construct a RVIZ marker from a match
    @staticmethod
    def make_match_marker(key, match, time, lifetime):
        marker = Marker(ns="Match:" + key, id=match.id, type=3,
                        text=key, pose=match.pose, lifetime=lifetime)
        marker.header.frame_id = "map"
        marker.header.stamp = time
        marker.scale.z = match.estimated_height * 0.5
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.color.a = 1.0
        marker.color.r = 0.25
        marker.color.g = 0.75
        marker.color.b = 0.25
        marker.colors = [marker.color]
        return marker

    #construct a RVIZ marker from an observation
    @staticmethod
    def make_observation_marker(observation, time, lifetime, is_stable):
        marker = Marker(ns="Observe:" + observation.class_label, id=observation.id, type=0,
                        text=observation.class_label, lifetime=lifetime)
        marker.id = observation.id + (0 if is_stable else 1000) # reuse namespace at different range
        marker.points = [vector_to_msg(observation.position(), Point()),
                         vector_to_msg(observation.position() + observation.bearing, Point())]
        marker.header.frame_id = "map"
        marker.header.stamp = time
        marker.scale.x = 0.01
        marker.scale.y = 0.03
        marker.color.a = 1.0
        if is_stable:
            marker.color.r = 0.25
            marker.color.g = 0.25
            marker.color.b = 0.75
        else:
            marker.color.r = 0.75
            marker.color.g = 0.25
            marker.color.b = 0.25
        marker.colors = [marker.color]
        return marker

# model name can be based in as the first parameter
if __name__ == '__main__':
    if len(sys.argv) < 2:
        model = YoloV3() # default model
    else:
        model_class = models.get(sys.argv[1], YoloV3)
        model = model_class()
    o = ObjectDetector(model)
    o.activate_ROS_communication()
