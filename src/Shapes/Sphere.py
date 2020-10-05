from .Shape import Shape
from copy import deepcopy
from src.Shapes.Shape import *
from math import *
from src.Grasp import Grasp


class Sphere(Shape):
    def __init__(self, radius=0, pose=Pose()):
        super().__init__(pose)
        self.radius = radius

    # This function was formed assuming
    # Z
    # |
    # |_____X
    # and Y going into the plane
    def planGrasps(self, graspParams):
        totalGrasps = []
        divisions = graspParams[1]
        graspRotations = graspParams[2]
        palmDistance = 5

        #plan starting pose
        InitialPose = deepcopy(self.originPose)
        InitialPose.x += self.radius + palmDistance

        azimuth = 0
        theta = (2*pi)/divisions
        phi = (2 * pi) / graspRotations
        while azimuth < divisions:
            approachPose = Pose()
            approachPose.x = -1
            radius = self.radius + palmDistance
            InitialPoseH = deepcopy(self.originPose)
            InitialPoseH.x = radius * cos(azimuth*theta)
            InitialPoseH.y = radius * sin(azimuth*theta)
            InitialPoseH.yaw += azimuth*theta

            rotations = 0
            while rotations < graspRotations:
                thumbPoseH = Pose()
                thumbPoseH.y = 1
                thumbPoseH.roll = rotations*phi
                currentGH = Grasp('spherical',InitialPoseH, approachPose, thumbPoseH)
                totalGrasps.append(currentGH)

            elevation = 0
            while elevation < divisions:
                InitialPoseV = deepcopy(InitialPoseH)
                InitialPoseV.x = radius * cos(elevation*theta)
                InitialPoseV.z = radius * sin(elevation*theta)
                InitialPoseV.pitch += elevation*theta
                rotations = 0
                while rotations < graspRotations:
                    thumbPoseV = Pose()
                    thumbPoseV.y = 1
                    thumbPoseV.roll = rotations * phi
                    currentGV = Grasp('spherical', InitialPoseV, approachPose, thumbPoseV)
                    totalGrasps.append(currentGV)
                elevation += 1
            azimuth += 1
        return totalGrasps


    def makeMesh(self):
        """
        creates the mesh grid for a box for the 3d plot by:
         1) creating the object in a 2D plane at the origin
         2) extruding by the z to get height
         3) subtract from z the offset for the centroid
         3) transforming to the object frame
        :return: coords [np.array] (x, y, z)
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
        ax.add_collection3d(mplot3d.art3d.Poly3DCollection(sphere.vectors))

        # scale plot and add labels
        scale = sphere.points.flatten()
        ax.auto_scale_xyz(scale, scale, scale)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')

        ax.quiver(0, 0, 0, 1, 1, 1, length=.1)

        return ax
