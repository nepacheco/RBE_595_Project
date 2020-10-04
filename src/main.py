# Script for testing

import numpy as np
from src.Shapes.Cylinder import Cylinder
import math
from math import pi
from src.Pose import Pose
from src.Grasp import Grasp

#



def testCylinderGraspLocations():
    cylinder = Cylinder(Pose(0,0,0,0,0,0),10,1)
    grasps = cylinder.planGrasps([1,1,1,1])
    print(grasps)

testCylinderGraspLocations()