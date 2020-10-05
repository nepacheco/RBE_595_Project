#!/usr/bin/python

from src.Pose import Pose
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from stl import mesh
from mpl_toolkits import mplot3d



class Shape:

    def __init__(self, originPose = Pose()):
        self.originPose = originPose    # position of centroid of the object with orientation
        self.transformation = Pose.makeTranformfromPose(originPose)

        # params
        self.ptsPerM = 10

    def planGrasps(self, graspParams):
        return

    def visualizeGrasp(self, graspList):
        return

    def applyTransform(self, p):
        """
        multiply the transformation matrix of the shape by the given position to transform the position into the new frame
        :param p: [np.array] (x, y, z) position vector
        :return: [np.array] (x', y', z') transformed position vector
        """
        # make vertical if not already
        # if p.size[0] != 3:
        vec = np.vstack(np.append(p, [1]))

        newvec = self.transformation.dot(vec)
        return newvec[0:3]

    def makeMesh(self):
        """
        abstract method
        creates the mesh grid for a shape a box for the 3d plot
        :return: mesh [x, y, z]
        """
        return

    def visualizeObject(self):
        return
