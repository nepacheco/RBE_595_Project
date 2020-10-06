import copy
from src.Shapes.Shape import Shape
from ..Pose import Pose
from src.Grasp import Grasp
from src.Shapes.Shape import *
from math import *
from scipy.spatial.transform import Rotation as R
import numpy as np


class Cylinder(Shape):

    def __init__(self, pose=Pose(), height=0, radius=0):
        super().__init__(pose)
        self.height = height
        self.radius = radius

    # This function was formed assuming
    # Z
    # |
    # |_____X
    # and Y going into the plane
    def planGrasps(self, graspParams, surfaceOffset=1):
        """
        Create each grasp assuming the origin of the shape is the global origin, and then multiply the grasp Pose by the
        transformation matrix to put the grasp location in the global frame
        :param graspParams: [array 1x4] Array for the 4 grasp parameters;
        0. # of parallel planes
        1. # of divisions of 360 degrees
        2. # of grasp rotations
        3. # of 180 degree rotations
        :return: [array of Grasp Objects] List of grasp objects
        """
        parallelPlanes = graspParams[0]  # this number should be odd
        divisionsOf360 = graspParams[1]
        graspRotations = graspParams[2]  # value should not be 3
        grasp180Rotations = graspParams[3]

        graspList = []
        # Side Grasps
        sideGrasps = self.getSideGrasps(graspParams,surfaceOffset)
        graspList = graspList + sideGrasps

        # End Grasps
        endGrasps = self.getEndGrasps(graspParams,surfaceOffset)
        graspList = graspList + endGrasps

        return graspList

    def getSideGrasps(self, graspParams, surfaceOffset=0.1):
        """
        Returns the Side Grasps for a cylinder object
        :param surfaceOffset: [double] Distance to start the grasp away from the surface
        :param graspParams: [array 1x4] Grasp parameters associated with creating grasps
        :return: [list<Grasp>] A list of grasps
        """
        parallelPlanes = graspParams[0]  # this number should be odd
        divisionsOf360 = graspParams[1]
        grasp180Rotations = graspParams[3]

        graspList = []
        # Side Grasps
        if parallelPlanes > 1:  # set the initial height at the bottom of cylinder
            graspTranslation = -self.height / 2
        else:  # set initial height at center of mass
            graspTranslation = 0

        for i in range(parallelPlanes):
            # Translation along the objects central axis (z axis) for height placement of grasp
            translationMatrix = Pose.makeTranformfromPose(Pose(0, 0, graspTranslation, 0, 0, 0))
            parallelMatrix = np.matmul(Pose.makeTranformfromPose(self.originPose), translationMatrix)

            # increment GraspTranslation and set grasp rotation
            if parallelPlanes > 1:  # don't divide by 0
                graspTranslation += self.height / (parallelPlanes - 1)
            graspRotation = 0
            for j in range(divisionsOf360):
                # Rotation about the objects central axis (z axis) for rotation placement of grasp
                rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, graspRotation))
                divisionsMatrix = np.matmul(parallelMatrix, rotZMatrix)

                # increment grasp rotation and set wrist rotation
                graspRotation += 2 * np.pi / divisionsOf360
                wristRotation = 0
                for k in range(2*grasp180Rotations):
                    # These three transformation matrices move the frame of the grasp outside the object.
                    # Additional it guarantees that the z axis of the frame points towards the object (approach vector)
                    # and that the x axis is perpendicular to the central axis and the approach vector to achieve the
                    # thumb vector
                    transXMatrix = Pose.makeTranformfromPose(Pose(self.radius + surfaceOffset, 0, 0, 0, 0, 0))
                    rotYMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, -np.pi / 2, 0))
                    rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, -np.pi / 2))

                    # multiplication of the transformation matrices
                    graspMatrix = np.matmul(divisionsMatrix, transXMatrix)
                    graspMatrix = np.matmul(graspMatrix, rotYMatrix)
                    graspMatrix = np.matmul(graspMatrix, rotZMatrix)

                    # Rotate about the approach vector (our z axis) if we need to
                    rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, wristRotation))
                    graspMatrix = np.matmul(graspMatrix, rotZMatrix)
                    wristRotation += np.pi/2

                    # Add object to grasp list as a Pose not a matrix
                    graspList.append(Grasp('cylindrical', Pose.makePoseFromTransform(graspMatrix)))
            return graspList


    def getEndGrasps(self,graspParams, surfaceOffset=0.1):
        """
        Generate the grasps for grabbing a cylinder by either end
        :param graspParams:[array 1x4] Grasp parameters associated with creating grasps
        :param surfaceOffset:[double] Distance to start the grasp away from the surface
        :return: [list<Grasp>] A list of grasps
        """
        graspList = []
        graspRotations = graspParams[2]
        # There are two ends and we just need to invert the z axis for one of them
        for i in range(2):
            # rotation about the y axis only if i == 1
            rotYMatrix = Pose.makeTranformfromPose(Pose(0,0,0,0,i*np.pi,0))
            interMatrix = np.matmul(Pose.makeTranformfromPose(self.originPose),rotYMatrix)
            # translate along negative Z axis to bring the grasp out of the object
            transZMatrix = Pose.makeTranformfromPose(Pose(0,0,-(self.height/2 + surfaceOffset),0,0,0))
            endMatrix = np.matmul(interMatrix,transZMatrix)

            rotation = 0
            for j in range(graspRotations):
                # Rotate about the z axis for plan the set of grasps
                rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, rotation))
                graspMatrix = np.matmul(endMatrix, rotZMatrix)

                # increment rotation for next loop
                rotation += 2*np.pi/graspRotations

                # Add Grasp to list
                graspList.append(Grasp('spherical', Pose.makePoseFromTransform(graspMatrix)))
        return graspList


    def makeMesh(self):
        """
        creates the mesh grid for a box for the 3d plot by:
         1) importing unit shape stl
         2) translating stl to origin and scale by the respective properties
         3) transform all points by shape pose
        :return: axes
        """

        # display figure and get axes
        sphere = mesh.Mesh.from_file('STLs/cylinder.STL')

        # Create a new plot
        figure = plt.figure()
        ax = mplot3d.Axes3D(figure)

        # move to origin scale points by dimensions
        vecs = sphere.vectors
        vecs[:, :, 0] = (vecs[:, :, 0] - 0.5) * self.radius
        vecs[:, :, 1] = (vecs[:, :, 1] - 0.5) * self.radius
        vecs[:, :, 2] = (vecs[:, :, 2] - 0.5) * self.height

        # tranform each point based to based on orientation
        for f in range(vecs.shape[0]):
            for v in range(vecs.shape[1]):
                p = vecs[f, v, :]
                transformed_p = self.applyTransform(p)
                vecs[f, v, :] = np.hstack(transformed_p)

        # Load the STL files and add the vectors to the plot
        ax.add_collection3d(mplot3d.art3d.Poly3DCollection(sphere.vectors, edgecolor='k'))

        # scale plot and add labels
        scale = sphere.points.flatten()
        ax.auto_scale_xyz(scale, scale, scale)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')

        ax.quiver(0, 0, 0, 1, 1, 1, length=.1)

        return ax