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
box = Box(1,2,3, x)

# mesh = mesh.Mesh.from_file('STLs/cube.STL')
#
# # Create a new plot
# figure = plt.figure()
# axes = mplot3d.Axes3D(figure)
#
# vecs = mesh.vectors
# length = 3
# width = 2
# height = 4
# numPts = vecs.shape[0]
# # for pts in range(numPts):
# vecs[:, :, 0] = vecs[:, :, 0] * length
# vecs[:, :, 1] = vecs[:, :, 1] * width
# vecs[:, :, 2] = vecs[:, :, 2] * height
#
# # Load the STL files and add the vectors to the plot
# axes.add_collection3d(mplot3d.art3d.Poly3DCollection(mesh.vectors, edgecolor='k'))
#
# scale = mesh.points.flatten()
# axes.auto_scale_xyz(scale, scale, scale)
# axes.set_xlabel('X (mm)')
# axes.set_ylabel('Y (mm)')
# axes.set_zlabel('Z (mm)')
box.makeMesh()
# plt.show()

