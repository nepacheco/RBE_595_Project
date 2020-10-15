# Script for testing
# !/usr/bin/env python

import rospy
from std_msgs.msg import String
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


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


def listener():
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("ShapeArray", shapeArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

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


if __name__ == '__main__':
    listener()