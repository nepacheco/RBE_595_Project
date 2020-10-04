# Script for testing

import numpy as np
import math
from math import pi
from src.Shapes.Shape import Shape
from src.Pose import Pose
from src.Grasp import Grasp

x = Pose(0, 0, 0, pi/2, 0, 0)
shape = Shape(x)

print(shape.transformation)

p = np.array([0, 1, 0])
s = shape.applyTransform(p)
print(s)