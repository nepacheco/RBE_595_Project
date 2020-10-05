# Script for testing

import numpy as np
import matplotlib.pyplot as plt
from stl import mesh
from mpl_toolkits import mplot3d

from math import pi
from src.Shapes.Box import *
from src.Pose import Pose
from src.Grasp import Grasp

x = Pose(0, 0, 0, pi/2, pi/6, pi/3)
box = Box(1, 2, 3, x)

box.makeMesh()

