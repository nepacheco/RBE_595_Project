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

p0 = Pose(0, 0, 0, pi/2, pi/6, pi/3)
m = Box(1,3,2, p0)

m.makeMesh()

