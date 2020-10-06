from .Shape import Shape
from copy import deepcopy
from src.Shapes.Shape import *
from math import *
from src.Grasp import Grasp


class Sphere(Shape):
    def __init__(self, pose=Pose(), radius=0):
        super().__init__(pose)
        self.radius = radius

    # This function was formed assuming
    # Z
    # |
    # |_____X
    # and Y going into the plane
    def planGrasps(self, graspParams, surfaceOffset=100):
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
        graspList = []
        divisionsOf360 = graspParams[1]
        graspRotations = graspParams[2]  # Should not be a multiple of 3

        theta = (2 * np.pi) / divisionsOf360
        phi = (2 * pi) / graspRotations
        outerRadius = self.radius + surfaceOffset
        for i in range(divisionsOf360):
            azimuthMatrix = Pose.makeTranformfromPose(Pose(outerRadius * cos(i * theta),
                                                           outerRadius * sin(i * theta), 0,
                                                           0, 0, i * theta))
            parallelMatrix = np.matmul(Pose.makeTranformfromPose(self.originPose), azimuthMatrix)

            for j in range(divisionsOf360):
                elevationMatrix = Pose.makeTranformfromPose(Pose(cos(j * theta), 0,
                                                                 sin(j * theta), 0,
                                                                 j * theta, 0))

                parallelMatrix = np.matmul(parallelMatrix, elevationMatrix)

                # Offset by 1 so there is no division by 0
                for k in range(1, graspRotations + 1):
                    # The following two transforms align the z axis to the center of the object and rotates the
                    # x axis around the direction of motion a specified amount of times
                    rotYMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, -np.pi / 2, 0))
                    rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, (2 * np.pi) / k))

                    graspMatrix = np.matmul(parallelMatrix, rotYMatrix)
                    graspMatrix = np.matmul(rotZMatrix, graspMatrix)
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
        sphere = mesh.Mesh.from_file('STLs/sphere.STL')

        # Create a new plot
        figure = plt.figure()
        ax = mplot3d.Axes3D(figure)

        # move to origin scale points by dimensions
        vecs = sphere.vectors
        vecs[:, :, 0] = (vecs[:, :, 0] - 0.5) * self.radius
        vecs[:, :, 1] = (vecs[:, :, 1] - 0.5) * self.radius
        vecs[:, :, 2] = (vecs[:, :, 2] - 0.5) * self.radius

        # tranform each point based to based on orientation
        for f in range(vecs.shape[0]):
            for v in range(vecs.shape[1]):
                p = vecs[f, v, :]
                transformed_p = self.applyTransform(p)
                vecs[f, v, :] = np.hstack(transformed_p)

        # Load the STL files and add the vectors to the plot
        ax.add_collection3d(mplot3d.art3d.Poly3DCollection(sphere.vectors, edgecolor='w'))

        # scale plot and add labels
        scale = sphere.points.flatten()
        ax.auto_scale_xyz(scale, scale, scale)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')

        return ax
