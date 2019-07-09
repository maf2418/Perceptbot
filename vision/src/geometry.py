# library of helper functions and the CameraObj class

import numpy as np
import math

SMALLNUM = 0.000001


def make_vector3(x=0.0, y=0.0, z=0.0):
    return np.array([[x], [y], [z]])

# object for geometric information of a camera only
class CameraObj:
    # this inverse transform changes from ROS x fwd to camera definition z fwd.
    base_transform = np.array([[0.0, -1.0, 0.0, 0.0],
                               [0.0, 0.0, 1.0, 0.0],
                               [-1.0, 0.0, 0.0, 0.0],
                               [0.0, 0.0, 0.0, 1.0]])

    fwd_vector = make_vector3(1.0, 0.0, 0.0)
    fov_y = math.radians(40)
    fov_x = math.radians(60)
    scale_x = math.tan(math.radians(60) * 0.5)
    scale_y = math.tan(math.radians(40) * 0.5)
    cx = 0
    cy = 0

    def set_camera_fov(self, fov_x, fov_y, cx=0, cy=0):
        self.fov_x = math.radians(fov_x)
        self.fov_y = math.radians(fov_y)
        self.scale_x = math.tan(self.fov_x * 0.5)
        self.scale_y = math.tan(self.fov_y * 0.5)
        self.cx = cx
        self.cy = cy
        self.recalculate_matrix()

    def __init__(self, transform, time_stamp, image=None, max_range=40):
        self.transform = transform
        self.time_stamp = time_stamp
        self.image=image
        self.max_range = max_range
        self.recalculate_matrix()

    def recalculate_matrix(self):
        self.perspective = np.zeros((3, 4))
        self.perspective[0][0] = 1 / self.scale_x
        self.perspective[1][1] = 1 / self.scale_y
        self.perspective[0][2] = self.cx
        self.perspective[1][2] = self.cy

        self.perspective[2][2] = -1.0
        self.perspective = self.perspective.dot(self.base_transform)
        self.world_to_focal_plane = self.perspective.dot(inverse_transform(self.transform))

    def forward(self):
        return self.transform[0:3, 0:1]

    def position(self):
        return self.transform[0:3, 3:4]

    # projects point without adjusting for z(w) value. dividing by homogeneous term puts in -1/1 scale
    def project_point_to_screen_space(self, point):
        return np.reshape(self.world_to_focal_plane[:, 0:3].dot(point[:, 0]) +
                          self.world_to_focal_plane[:, 3], (3, 1))
        
    def project_point_to_screen_space_flat(self, point):
        screen_space = self.project_point_to_screen_space(point)
        return make_vector2(screen_space[0, 0] / screen_space[2][0], screen_space[1, 0] / screen_space[2][0])


    @staticmethod
    def ndc_to_screen_space(x, y):
        return 2 * x - 1.0, 2 * y - 1.0

    @staticmethod
    def screen_space_to_ndc(point):
        return 0.5 * point[0] + 0.5, 0.5 * point[1] + 0.5

    def get_bearing(self, x_screen, y_screen):
        ray_ros = make_vector3(1.0, -x_screen * self.scale_x, y_screen * self.scale_y)
        normalize(ray_ros)
        return self.transform[0:3, 0:3].dot(ray_ros)


def make_vector2(x=0.0, y=0.0):
    return np.array([[x], [y]])


def normalize(vector):
    vector /= math.sqrt(dot(vector, vector))


def make_point3(x=0.0, y=0.0, z=0.0):
    return np.array([[x], [y], [z]])


def make_rotation(yaw=0.0):
    m = np.eye(3)
    c = math.cos(yaw)
    s = math.sin(yaw)
    m[0,0] = c
    m[1,1] = c
    m[0,1] = -s
    m[1,0] = s
    return m


def make_transform(position=make_point3(), yaw=0.0):
    t_ret = new_transform()
    t_ret[0:3, 0:3] = make_rotation(yaw)
    t_ret[0:3, 3] = position[:, 0]
    return t_ret


def new_transform():
    return np.eye(4)


def dot(vector1, vector2):
    return np.dot(vector1.transpose(), vector2)


def inverse_transform(transform):
    t_ret = new_transform()
    t_ret[0:3, 0:3] = np.transpose(transform[0:3, 0:3])
    t_ret[0:3, 3] = - t_ret[0:3, 0:3].dot(transform[0:3, 3])
    return t_ret

def average(a, b):
    return 0.5 * (a + b)
