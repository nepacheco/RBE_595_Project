#!/usr/bin/python

import numpy as np
from scipy.spatial.transform import Rotation as R


class Pose:

    def __init__(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
        self.x = x
        self.y = y
        self.z = z

        # in radians
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

    @staticmethod
    def makeTranformfromPose(p):
        """
        given a pose object, create the transformation matrix
        :param pose: [Pose] pose object
        :return: [np.matix 4x4] transformation matrix
        """

        T = np.zeros([4, 4])     # init empty transformation matrix

        eulers = np.array([p.roll, p.pitch, p.yaw])     # euler array

        rot = R.from_euler('xyz', eulers)       # create rotation matrix from euler angles

        # assemble transformation matrix
        T[0:3, 0:3] = rot.as_matrix()
        T[0, 3] = p.x
        T[1, 3] = p.y
        T[2, 3] = p.z
        T[3, 3] = 1

        return T

    @staticmethod
    def makePoseFromTransform(T):
        """
        Converts transformation matrix to a Pose with xyz euler angles. Assumes the passed in transform is a valid
        transformation matrix
        :param T: [np.matrix 4x4] Transformation Matrix
        :return: [Pose] Pose object
        """
        r = R.from_matrix(T[0:3, 0:3])
        euler_vec = r.as_euler('xyz')
        pose = Pose(T[0][3], T[1][3], T[2][3], euler_vec[0], euler_vec[1], euler_vec[2])
        return pose

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return 'Pose(' + str(self.x) + ', ' + str(self.y) + ', ' + str(self.z) +\
               ', ' + str(self.roll) + ', ' + str(self.pitch) + ', ' + str(self.yaw) + ')'