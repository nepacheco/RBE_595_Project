#!/usr/bin/python

import numpy as np
from scipy.spatial.transform import Rotation as R


class Pose:

    def __init__(self,x=0,y=0,z=0,roll=0,pitch=0,yaw=0):
        self.x = x
        self.y = y
        self.z = z
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

        T = np.zeros([4, 4])     # empty transformation matrix

        eulers = np.array([p.roll, p.pitch, p.yaw])     # euler angles

        rot = R.from_euler('xyz', eulers)

        # assemble transformation matrix
        T[0:3, 0:3] = rot.as_matrix()
        T[0, 3] = p.x
        T[1, 3] = p.y
        T[2, 3] = p.z
        T[3, 3] = 1

        return T




