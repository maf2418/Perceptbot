This package includes three related elements of the robot logic, all written in python for easy customization by students without requiring lengthy recompiles of ROS on Raspbian.
1. A camera controller class
2. A class and API for loading and using neural networks with OpenVino on the Neural Compute Stick (NCS) 2
3. An object localization algorithm which combines object bounding box detections with SLAM updates to localize objects on the map.

The camera class is in camera.py.  The user can optionally specify a distortion matrix for use with a fisheye or other lens on the robot.  The controller streams images on two ROS channels: (a) the standard topic for easy interaction with other ROS libraries, and (b) a custom ImageWithTransform message which includes the camera transform matrix as an unrolled float array.
In order to accomodate lag from SLAM running on a Pi, the ImageWithTransform message includes an "is_stable" boolean flag which is set to True if the calculated position has not changed for a few seconds (default parameter = 3).  We found we could only rely on an accurate camera transform if the robot had paused, as the off-the-shelf SLAM algorithm took some time to catch up and used a timestamp at the end of calculcation, not the beginning.
The camera can be run from the command line in calibrate mode, which takes 100 pictures over 20 seconds into the calibration folder, while a standard calibration chessboard is moved by the user.  This can be used with the calibration script to generate a distortion matrix through opencv.

The NCS interface - the InferenceModel base class loads the NCS plugin and the binary models and xml files created by OpenVino when deploying a neural network.  We have included a few example implementations of this class, such as Yolo Version 3, tiny Yolo and SSD-Lite based on mobilnet.  These are each individual python files which include the class labels and code to interpret the tensor outputs of each inference model.  Note that the actual model binaries must be downloaded from either the OpenVino site or converted with the OpenVino tools from the original authors' repositories.

The ObjectDetection class loads one of the above models (can be specified as a parameter) and listens to the ImageWithTransform.  Images are fed to the model and object detection messages are published over ROS.  These messages can be triggers for AI or other routines students can develop.  Visualization markers are published for RVIZ showing the sighting.

The SSDLocalization class listens for stable object detection messages and triangulates the global map position from multiple sightings by projecting the bounding boxes, checking for intersections and comparing the projected object heights before accepting the pairing as a valid match. 

