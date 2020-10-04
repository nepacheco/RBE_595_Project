# Script for testing

import numpy as np
import math
from math import pi
from src.Pose import Pose
from src.Grasp import Grasp

x = Pose(1, 2, 3, pi/2, 0, 0)

t = Pose.makeTranformfromPose(x)
print(t)