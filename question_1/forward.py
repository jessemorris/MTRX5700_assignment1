import numpy as np
import math
from visual_kinematics import *


dh_params = np.array([[ 0.163, 0, 0, -0.5 * math.pi ],
                      [ 0.007, 0.25 * math.pi, -0.425, 0],
                      [ 0, -0.5 * math.pi, 0.392, 0],
                      [ 0.127, 0,  0, -0.5 * math.pi],
                      [  0.1, 0 ,0, 0.5 * math.pi],
                      [ 0.1, 0, 0, 0]])


robot = Robot(dh_params)

# Uncomment one of the following to select a viewing state

###### Part c, straight up position ########
# theta = np.array([0., 0.25 * math.pi, -0.5*math.pi, -0.5*math.pi , 0., 0])

######### Initial state position ##########
# theta = np.array([0, 0, 0 , 0, 0, 0])

########## 0 state position ###############
# theta = np.array([0, 0.75*math.pi, -0.5*math.pi, 0 ,0, 0])

f = robot.forward(theta)

robot.show()

