#!/usr/bin/python
import numpy

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
        self.grasps = []

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
        imports respective stl for the shape, scales, then transforms by pose
        :return: mesh [x, y, z]
        """
        return

    def combinePrimatives(self, shapeList):
        """
        combines the stl of two primitives
        :param shapeList: a list of primitives to merge with the first
        :return: the axes of the new stl
        """
        # Create a new plot
        figure = plt.figure()
        axes = mplot3d.Axes3D(figure)

        # Combine shape primitives
        shape1Mesh = self.generateMesh()
        combined = mesh.Mesh(shape1Mesh.data)
        for shape in shapeList:
            shapeMesh = shape.generateMesh()
            combined = mesh.Mesh(numpy.concatenate([combined.data, shapeMesh.data]))

        # Load the STL files and add the vectors to the plot
        axes.add_collection3d(mplot3d.art3d.Poly3DCollection(combined.vectors, edgecolor='k'))

        scale = combined.points.flatten()
        axes.auto_scale_xyz(scale, scale, scale)
        axes.set_xlabel('X (mm)')
        axes.set_ylabel('Y (mm)')
        axes.set_zlabel('Z (mm)')

        return axes

    def visualizeGrasps(self, ax, grasps):
        """
        adds the grasp vectors to the plot
        each grasp will plot their x and z vector
        :param ax: axes object
        :return:
        """
        numGrasps = len(grasps)
        poses = (g.pose for g in grasps)   # get list of poses of grasps

        # quiver list consisting of: x, y, z, u, v, w, color
        numAxis = 2
        q = np.zeros([numAxis*numGrasps, 6])
        c = np.zeros([numAxis*numGrasps, 6])

        for i, p in enumerate(poses):
            T = Pose.makeTranformfromPose(p)

            # add z axis to plot
            Tz = T[0:3, 2]
            q[i * numAxis] = np.array([p.x, p.y, p.z, Tz[0], Tz[1], Tz[2]])
            c[i * numAxis] = (0)

            # add z axis to plot
            if numAxis >= 2:
                Tx = T[0:3, 0]
                q[i * numAxis + 1] = np.array([p.x, p.y, p.z, Tx[0], Tx[1], Tx[2]])
                c[i * numAxis + 1] = (.33)

            # add y axis to plot
            if numAxis >= 3:
                Ty = T[0:3, 1]
                q[i * numAxis + 2] = np.array([p.x, p.y, p.z, Ty[0], Ty[1], Ty[2]])
                c[i * numAxis + 2] = (.66)

        # norm = plt.Normalize()
        # norm.autoscale(c)

        # colormap = plt.colormaps()
        # cm = plt.cm.jet(c)
        # print(colormap[c]) 
        # print(cm)
        ax.quiver(q[:,0], q[:,1], q[:,2], q[:,3], q[:,4], q[:,5], color='r', length=0.5)
        # axis('equal')

        return
