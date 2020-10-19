#! /usr/bin/env python

import rospy
from shape_node.msg import shapeArray
import numpy as np
import math
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

from math import atan2, asin
from Pose import Pose
from Grasp import Grasp
from Shapes.Cylinder import Cylinder
from Shapes.Cone import Cone
from Shapes.Box import Box
from Shapes.Sphere import Sphere
from stl import mesh




def testCylinderGraspLocations():
    cylinder = Cylinder(Pose(0,0,5,0,0,0),10,1)
    grasps = cylinder.planGrasps([1,2,1,1])
    print(grasps)


def testConeGraspLocations():
    cone = Cone(Pose(0,0,5,0,0,0),10,5)
    grasps = cone.planGrasps([1,1,1,1])
    print(grasps)


def makeCoffeeMug():
    pc = Pose(0, 0, 0, 0, 0, 0)
    pb = Pose(.6, 0, 0, 0, 0, 0)

    box = Box(pb, .75, .2, .5)
    cylinder = Cylinder(pc, 1, 1)

    axes = cylinder.combinePrimatives([box])
    graspBox = box.planGrasps([1, 5, 5, 1])
    graspCylinder = cylinder.planGrasps([5,5,5,5])
    box.visualizeGrasps(axes, graspBox)
    cylinder.visualizeGrasps(axes, graspCylinder)
    plt.show()


def makeBeaker():
    pc = Pose(0, 0, 0, 0, 0, 0)
    pcy = Pose(0, 0, 1, 0, 0, 0)

    cone = Cone(pc, 1.5, .25)
    cylinder = Cylinder(pcy, .5, .1)

    axes = cone.combinePrimatives([cylinder])
    graspCone = cone.planGrasps([1, 1, 1, 1])
    graspCylinder = cylinder.planGrasps([1, 1, 1, 1])
    cone.visualizeGrasps(axes, graspCone)
    cylinder.visualizeGrasps(axes, graspCylinder)
    plt.show()


def makePlane():
    pb1 = Pose(0, 0, 0, 0, 0, 0)
    pb2 = Pose(0, 0, 0, 0, 0, 0)

    box1 = Box(pb1, .3, 0.25, 3)
    box2 = Box(pb2, .15, 3, .6)

    axes = box1.combinePrimatives([box2])
    graspBox1 = box1.planGrasps([1, 1, 1, 1])
    graspBox2 = box2.planGrasps([1, 1, 1, 1])
    box1.visualizeGrasps(axes, graspBox1)
    box2.visualizeGrasps(axes, graspBox2)
    plt.show()


def makePhone():
    pb1 = Pose(0, 0, 0, 0, 0, 0)
    pb2 = Pose(.5, 0, 1.5, 0, 0, 0)
    pb3 = Pose(.5, 0, -1.5, 0, 0, 0)

    box1 = Box(pb1, 4, .75, .5)
    box2 = Box(pb2, 1, .75, .5)
    box3 = Box(pb3, 1, .75, .5)
    axes = box1.combinePrimatives([box2, box3])

    graspBox1 = box1.planGrasps([1, 1, 1, 1])
    graspBox2 = box2.planGrasps([1, 1, 1, 1])
    graspBox3 = box3.planGrasps([1, 1, 1, 1])
    box1.visualizeGrasps(axes, graspBox1)
    box2.visualizeGrasps(axes, graspBox2)
    box3.visualizeGrasps(axes, graspBox3)
    plt.show()


def makeBox(p0, parameters):
    m = Box(p0, parameters(4), parameters(5), parameters(6))
    ax = m.makeMesh()
    grasps = m.planGrasps([parameters(0), parameters(1), parameters(2), parameters(3)])
    m.visualizeGrasps(ax, grasps)
    plt.show()


def makeCylinder(p0, parameters):
    print(p0)
    print(parameters)
    m = Cylinder(p0, parameters[4], parameters[7])
    ax = m.makeMesh()
    grasps = m.planGrasps([parameters[0], parameters[1], parameters[2], parameters[3]])
    m.visualizeGrasps(ax, grasps)
    plt.show()


def makeSphere(p0, parameters):
    m = Sphere(p0, parameters(7))
    ax = m.makeMesh()
    grasps = m.planGrasps([parameters(0), parameters(1), parameters(2), parameters(3)])
    m.visualizeGrasps(ax, grasps)
    plt.show()


def makeCone(p0, parameters):
    m = Cone(p0, parameters(4), parameters(7))
    ax = m.makeMesh()
    grasps = m.planGrasps([parameters(0), parameters(1), parameters(2), parameters(3)])
    m.visualizeGrasps(ax, grasps)
    plt.show()


switch = {
            "Box":      makeBox,
            "Cylinder": makeCylinder,
            "Sphere":   makeSphere,
            "Cone":     makeCone
         }


# Parameters has 8 values [[4 build parameters], h, w, l, r]
def callback(data):
    ori = data.pose.orientation
    Euler = quatToEuler([ori.x, ori.y, ori.z, ori.w])
    p0 = Pose(data.pose.position.x, data.pose.position.y, data.pose.position.z, Euler[0], Euler[1], Euler[2])
    param = data.parameters
    switch[data.shapetype.data](p0,param)


def quatToEuler(quaternion):
    q0 = quaternion[0]
    q1 = quaternion[1]
    q2 = quaternion[2]
    q3 = quaternion[3]

    wx = atan2(2*((q0*q1)+(q2*q3)), 1-(2*(q1**2+q2**2)))
    wy = asin(2*(q0*q2-q3*q1))
    wz = atan2(2*((q0*q3)+(q1*q2)), 1-(2*(q2**2+q3**2)))
    return [wx, wy, wz]


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/ShapeArray", shapeArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
