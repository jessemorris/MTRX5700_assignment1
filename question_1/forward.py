import numpy as np
import math
from visual_kinematics import *

dh_params = np.array([[0, 0, 0, 0],
                      [ 0.163, 0, 0, -0.5 * math.pi ],
                      [ 0.007, 0.25 * math.pi, -0.425, 0],
                      [ 0, -0.5 * math.pi, 0.392, 0],
                      [ 0.127, 0.5 * math.pi,  0, 0.5 * math.pi],
                      [  0.1, 0, 0, -0.5 * math.pi],
                      [ 0.1, 0, 0, 0]])

robot = Robot(dh_params)

# theta = np.array([0., -0.5 * math.pi, 0 , -0.5*math.pi, 0., 0., 0.])
theta = np.array([0, 0, 0 , 0, 0., 0., 0.])
f = robot.forward(theta)

robot.show()

