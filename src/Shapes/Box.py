import copy
import numpy as np
from src.Shapes.Shape import *
from src.Grasp import Grasp
from src.Pose import Pose


class Box(Shape):

    def __init__(self, pose=Pose(), h=0, w=0, l=0):
        super().__init__(pose)
        self.height = h
        self.width = w
        self.length = l

    # This function was formed assuming
    # Z
    # |
    # |_____X
    # and Y going into the plane
    def planGrasps(self, graspParams, surfaceOffset = 100):
        """
         Create each grasp assuming the origin of the shape is the global origin, and then multiply the grasp Pose by
         the transformation matrix to put the grasp location in the global frame
         :param graspParams: [array 1x4] Array for the 4 grasp parameters;
         0. # of parallel planes
         1. # of divisions of 360 degrees
         2. # of grasp rotations
         3. # of 180 degree rotations
         :param surfaceOffset: [double] Distance to start the grasp away from the surface
         :return: [array of Grasp Objects] List of grasp objects
         """
        numberCubeSides = 6
        parallelPlanes = graspParams[0]  # Has to at least one and must be odd
        grasp180Rotations = graspParams[3]  # has to be a 1 or a 2

        graspList = []

        for i in range(numberCubeSides):
            # Rotate the originPose to generate grasps for all the sides of the cube
            # Change the lengths of the sides to properly line up with the new origin
            if i == 0:
                rotOriginMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, 0))
                xAxisValue = self.length
                yAxisValue = self.width
                zAxisValue = self.height
            if i == 1:
                rotOriginMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, np.pi / 2))
                xAxisValue = self.width
                yAxisValue = self.length
                zAxisValue = self.height
            if i == 2:
                rotOriginMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, np.pi))
                xAxisValue = self.length
                yAxisValue = self.width
                zAxisValue = self.height
            if i == 3:
                rotOriginMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, -np.pi / 2))
                xAxisValue = self.width
                yAxisValue = self.length
                zAxisValue = self.height
            if i == 4:
                rotOriginMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, np.pi / 2, 0, 0))
                xAxisValue = self.width
                yAxisValue = self.height
                zAxisValue = self.length
            if i == 5:
                rotOriginMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, -np.pi / 2, 0, 0))
                xAxisValue = self.width
                yAxisValue = self.height
                zAxisValue = self.length

            originTransform = np.matmul(rotOriginMatrix, Pose.makeTranformfromPose(self.originPose))

            # Generate parallel matrix representing the grasper's frame
            translationMatrix = Pose.makeTranformfromPose(Pose(xAxisValue / 2, 0, -zAxisValue / 2, 0, 0, 0))
            parallelMatrix = np.matmul(originTransform, translationMatrix)

            for j in range(1, parallelPlanes + 1):  # Vertical and Horizontal
                for k in range(1, parallelPlanes + 1):
                    # The following translation matrices are used to move the frame
                    # horizontally and vertically along the side of the cube
                    transXMatrix = Pose.makeTranformfromPose(
                        Pose(-k * (xAxisValue / (parallelPlanes + 1)), 0, 0, 0, 0, 0))
                    transZMatrix = Pose.makeTranformfromPose(
                        Pose(0, 0, j * (zAxisValue / (parallelPlanes + 1)), 0, 0, 0))

                    # Compute the resulting matrix
                    divisionsMatrix = np.matmul(parallelMatrix, transXMatrix)
                    divisionsMatrix = np.matmul(divisionsMatrix, transZMatrix)

                    for l in range(grasp180Rotations * 2):
                        # The following two transforms move the frame out of the object and rotates
                        # the z axis into the object and the x axis perpendicular to the plane it will be grasping
                        transYMatrix = Pose.makeTranformfromPose(Pose(0, -(yAxisValue / 2) - surfaceOffset, 0, 0, 0, 0))
                        rotXMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, -np.pi / 2, 0, 0))

                        graspMatrix = np.matmul(divisionsMatrix, transYMatrix)
                        graspMatrix = np.matmul(graspMatrix, rotXMatrix)

                        # Rotate the thumb corresponding to the grasp180 rotation parameter
                        rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, l * (-np.pi / 2)))
                        graspMatrix = np.matmul(graspMatrix, rotZMatrix)

                        # Create grasp and add it to the list of generated grasps
                        graspList.append(Grasp('cylindrical', Pose.makePoseFromTransform(graspMatrix)))

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
        box = mesh.Mesh.from_file('STLs/cube.STL')

        # Create a new plot
        figure = plt.figure()
        axes = mplot3d.Axes3D(figure)

        # move to origin scale points by dimensions
        vecs = box.vectors
        vecs[:, :, 0] = (vecs[:, :, 0] - 0.5) * self.length
        vecs[:, :, 1] = (vecs[:, :, 1] - 0.5) * self.width
        vecs[:, :, 2] = (vecs[:, :, 2] - 0.5) * self.height

        # tranform each point based to based on orientation
        for f in range(vecs.shape[0]):
            for v in range(vecs.shape[1]):
                p = vecs[f, v, :]
                transformed_p = self.applyTransform(p)
                vecs[f, v, :] = np.hstack(transformed_p)

        # Load the STL files and add the vectors to the plot
        axes.add_collection3d(mplot3d.art3d.Poly3DCollection(box.vectors, edgecolor='k'))

        # scale plot and add labels
        scale = box.points.flatten()
        axes.auto_scale_xyz(scale, scale, scale)
        axes.set_xlabel('X (mm)')
        axes.set_ylabel('Y (mm)')
        axes.set_zlabel('Z (mm)')

        return axes
