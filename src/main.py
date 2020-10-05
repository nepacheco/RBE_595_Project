# Script for testing

import numpy as np
import matplotlib.pyplot as plt
from stl import mesh
from mpl_toolkits import mplot3d

from math import pi
from src.Shapes.Box import Box
from src.Shapes.Sphere import Sphere
from src.Pose import Pose
from src.Grasp import Grasp

p0 = Pose(0, 0, 3, pi/2, 0, 0)
m = Sphere(1, p0)

ax = m.makeMesh()
# ax.quiver(1,1,1,1,1,1)

