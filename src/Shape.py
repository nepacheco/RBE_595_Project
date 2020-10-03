#!/usr/bin/python

from .Pose import Pose


class Shape:

    def __init__(self):
        self.originPose = Pose()

    def planGrasps(self, graspParams):
        return

    def visualizeGrasp(self, graspList):
        return

    def visualizeObject(self):
        return
