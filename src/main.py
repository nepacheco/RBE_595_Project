# Script for testing

import numpy as np
import math
import matplotlib.pyplot as plt
from stl import mesh
from mpl_toolkits import mplot3d

from math import pi
from src.Pose import Pose
from src.Grasp import Grasp
from src.Shapes.Cylinder import Cylinder
from src.Shapes.Cone import Cone
from src.Shapes.Box import Box
from src.Shapes.Sphere import Sphere


def testCylinderGraspLocations():
    cylinder = Cylinder(Pose(0,0,5,0,0,0),10,1)
    grasps = cylinder.planGrasps([1,2,1,1])
    print(grasps)

def testConeGraspLocations():
    cone = Cone(Pose(0,0,5,0,0,0),10,5)
    grasps = cone.planGrasps([1,1,1,1])
    print(grasps)

# testCylinderGraspLocations()
# testConeGraspLocations()

p0 = Pose(0, 0, 0, 0, 0, 0)
m = Sphere(p0, 1)
ax = m.makeMesh()

grasps = m.planGrasps([5, 5, 5, 5], 1)
print(grasps)
m.visualizeGrasps(ax, grasps)
plt.show()


