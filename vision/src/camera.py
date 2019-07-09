#!/usr/bin/env python

import roslib
import rospy
import cv2
import sys
import io
from math import tan, radians
import numpy as np
from tf import TransformListener
from tf import Exception as tfException
from tf import LookupException as tfLookupException
from tf import ExtrapolationException as tfExtrapolationException
from sensor_msgs.msg import Image, CompressedImage, CameraInfo, RegionOfInterest
from perceptbot_camera.msg import ImageWithTransform
from picamera import PiCamera
from picamera import array as piCamArray
from cv_bridge import CvBridge
from time import sleep

# from calibration for wideangle
DIM=(640, 480)
K=np.array([[277.54574493204046, 0.0, 318.56117282234317], [0.0, 271.8445501808666, 215.66105351793217], [0.0, 0.0, 1.0]])
D=np.array([[0.16995693543353735], [-0.1534627898190762], [0.2587674960012616], [-0.15526437812826197]])
# set UNDISTORT to True to use the calibration
UNDISTORT = False
ROTATE = 0
SLAM_LAG = 3  # seconds to wait for slam updates before considering transform stable


def is_similar_transform(tr1, tr2):
    return np.max(np.abs(tr1 - tr2)) < 0.05


class CameraController:
    def __init__(self, horizontal_fov=110, vertical_fov=88,
                 camera_frame="camera_link", world_frame="map"):
        self.iwt_pub = rospy.Publisher("iwt_camera_stream", ImageWithTransform, queue_size=1)
        self.image_pub = rospy.Publisher("image_color", Image, queue_size=1)
        self.info_pub = rospy.Publisher("camera_info", CameraInfo, queue_size=10)
        self.tfl = TransformListener()
        self.bridge = CvBridge()
        self.resolution = (640, 480)
        self.power_on = False
        self.camera_info = None
        self.camera_frame = camera_frame
        self.world_frame = world_frame
        self.set_camera_info(horizontal_fov, vertical_fov)
        self.buffer_img = None
        self.anchor_time = rospy.Time(0)
        self.matrix4x4 = np.eye(4)
        self.has_sent_stable = False
        self.skip_count = 5

    def set_camera_info(self, horizontal_fov, vertical_fov):
        rect_matrix = [0.0 for i in range(9)]
        f_x = 1 / tan(radians(horizontal_fov) * 0.5)
        f_y = 1 / tan(radians(vertical_fov) * 0.5)
        c_x, c_y = 0.0, 0.0
        intr_matrix = [f_x, 0.0, c_x,
                       0.0, f_y, c_y,
                       0.0, 0.0, 1.0]
        proj_matrix = [f_x, 0.0, c_x, 0.0,
                       0.0, f_y, c_y, 0.0,
                       0.0, 0.0, 1.0, 0.0]
        self.camera_info = CameraInfo(height=self.resolution[1], width=self.resolution[1],
                                      K=intr_matrix, R=rect_matrix, P=proj_matrix)
        self.camera_info.header.frame_id = self.camera_frame

    # uses endless iterator to generate stream and process, which allows
    # us to use picamera continuous and sequence modes yet still publish
    # each frame
    def _stream_to_ros(self, camera, size):
        with piCamArray.PiRGBArray(camera, size=size) as stream:
            while self.power_on and not rospy.is_shutdown():
                yield stream
                snap_time = rospy.Time.now()  # - rospy.Duration(0.025) # delay to handle sync error
                image = self.undistort(stream.array) if UNDISTORT else stream.array 
                self.camera_info.header.stamp = snap_time
                self.publish_standard_camera(image, snap_time)
                # image with camera world transform for localization
                if self.skip_count < 1:
                    self.publish_image_with_stable_transform(image, snap_time)
                    self.skip_count = 5
                self.skip_count -= 1
                stream.truncate(0)

    # saves 100 images to file, used for calibration routine
    def _stream_raw_to_file(self, camera, size):
        with piCamArray.PiRGBArray(camera, size=size) as stream:
            frame_num = 0
            while frame_num < 100:
                yield stream
                print("Frame", frame_num)
                snap_time = rospy.Time.now()  # - rospy.Duration(0.025) # delay to handle sync error
                image = stream.array
                self.camera_info.header.stamp = snap_time
                self.publish_standard_camera(image, snap_time)

                frame_num += 1
                self.output = io.open('calibration/calibrate%02d.jpg' % frame_num, 'wb')
                self.output.write(cv2.imencode(".jpg", image)[1])
                self.output.close()
                stream.truncate(0)
            self.turn_off()

    # publishes on ROS the standard image as per any camera controller
    def publish_standard_camera(self, image, snap_time):
        self.info_pub.publish(self.camera_info)
        # standard camera image ros broadcast
        rosmsg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        rosmsg.header.stamp = snap_time
        rosmsg.header.frame_id = self.camera_frame
        self.image_pub.publish(rosmsg)

    # publishes on ROS the transform stamped image, noting if stable or not
    def publish_image_with_stable_transform(self, image, snap_time):
        try:
            self.camera_info.header.stamp = rospy.Time(0)
            matrix4x4 = self.tfl.asMatrix(self.world_frame, self.camera_info.header)
            is_similar = is_similar_transform(matrix4x4, self.matrix4x4)
            is_stable_for_time = is_similar and (snap_time - self.anchor_time).to_sec() > SLAM_LAG

            if not is_similar:
                self.matrix4x4 = matrix4x4
                self.anchor_time = snap_time
                self.has_sent_stable = False
            self.camera_info.header.stamp = snap_time
            cmp_img = CompressedImage(data=np.array(cv2.imencode(".jpg", image)[1]).tostring())
            cmp_img.header.stamp = self.anchor_time
            msg = ImageWithTransform(compressedImage=cmp_img,
                                     cameraInfo=self.camera_info,
                                     flat_transform=np.reshape(matrix4x4, 16))

            if not self.has_sent_stable and is_stable_for_time:
                msg.is_stable = True
                self.has_sent_stable = True
            else:
                msg.is_stable = False
            self.iwt_pub.publish(msg)

        except tfException:
            print("Camera can not find transform for ", self.world_frame," or ", self.camera_info.header.frame_id)

    @staticmethod
    def make_camera_obj(framerate=30):
        camera = PiCamera(sensor_mode=4, resolution='VGA', framerate=framerate)
        camera.rotation = ROTATE
        camera.start_preview()
        sleep(2)  # allow camera to warm up
        camera.stop_preview()
        return camera

    def turn_on(self):
        self.power_on = True
        with self.make_camera_obj(framerate=30) as camera:
            while self.power_on and not rospy.is_shutdown():
                camera.capture_sequence(self._stream_to_ros(camera, self.resolution), format='bgr',
                                        use_video_port=True)

    # tells camera to run the calibration routine
    def calibrate(self):
        self.power_on = True
        with self.make_camera_obj(framerate=5) as camera:
            for count in range(5):
                print("starting in " + str(5 - count) + "...")
                sleep(1)
            while self.power_on and not rospy.is_shutdown():
                camera.capture_sequence(self._stream_raw_to_file(camera, self.resolution), format='bgr',
                                        use_video_port=True)

    def turn_off(self):
        self.power_on = False

    #applies the distortion matrix through opencv to rectify fisheye images
    def undistort(self, img):
        h,w = img.shape[:2]
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
        return cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)


# run with parameter calibrate to generate calibration images in subfolder
if __name__ == '__main__':
    rospy.init_node("CameraNode")
    if len(sys.argv) > 1 and sys.argv[1] == "calibrate":
        print("Calibrating camera")
        CameraController().calibrate()
    else:
        print("Starting camera in ROS broadcast mode")
        CameraController().turn_on()
