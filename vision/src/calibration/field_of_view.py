import math
import cv2
import numpy as np
DIM=(640, 480)
K=np.array([[313.2329986134891, 0.0, 296.3147762321292], [0.0, 315.9551836180246, 192.89521545734337], [0.0, 0.0, 1.0]])
D=np.array([[0.08777449620893239], [-0.6489979129167768], [0.8486079561579886], [-0.4482729269106574]])



fov_x = 2 * math.atan(368/(2 * 313.2329986134891)) * 180 / math.pi
fov_y = 2 * math.atan(276/(2 * 315.9551836180246)) * 180 / math.pi

print((fov_x, fov_y))
P = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D, DIM, K, np.eye(3))
print(P)

fov_x = 2 * math.atan(368/(2 * P[0][0])) * 180 / math.pi
fov_y = 2 * math.atan(276/(2 * P[1][1])) * 180 / math.pi

print((fov_x, fov_y))

