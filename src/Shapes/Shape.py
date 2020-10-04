#!/usr/bin/python

from src.Pose import Pose
import numpy as np
import matplotlib.pyplot as plt



class Shape:

    def __init__(self, originPose = Pose()):
        self.originPose = originPose
        self.transformation = Pose.makeTranformfromPose(originPose)

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
        vec = np.vstack(np.append(p, [1]))  # add 1 to end and make vertical
        newvec = self.transformation.dot(vec)
        return newvec[0:3]


    def makeMesh(self):
        return

    def visualizeObject(self):

        return
