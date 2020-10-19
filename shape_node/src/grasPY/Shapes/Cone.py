import copy
from Shapes.Shape import Shape
from Pose import Pose
from Grasp import Grasp
from scipy.spatial.transform import Rotation as R
import numpy as np
import math
from Shapes.Shape import *

class Cone(Shape):

    def __init__(self, pose=Pose(), height=0, radius=0):
        super().__init__(pose)
        self.height = height
        self.radius = radius

    # This function was formed assuming
    # Z
    # |
    # |_____X
    # and Y going into the plane
    def planGrasps(self, graspParams,surfaceOffset = 1):
        """
        Create each grasp assuming the origin of the shape is the global origin, and then multiply the grasp Pose by the
        transformation matrix to put the grasp location in the global frame
        :param graspParams: [array 1x4] Array for the 4 grasp parameters;
        0. # of paraellel planes
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
        theta = math.atan(self.height/self.radius)
        # hypotenuse of height and radius of cone
        hypot = np.sqrt((self.height**2 + self.radius**2))

        # Side Grasp
        sideGrasps = self.getSideGrasps(graspParams,surfaceOffset);
        graspList += sideGrasps

        # End Grasp
        endGrasps = self.getEndGrasps(graspParams,surfaceOffset)
        graspList += endGrasps

        # Edge Grasp
        edgeGrasps = self.getEdgeGrasps(graspParams,surfaceOffset)
        graspList += edgeGrasps

        return graspList

    def getSideGrasps(self, graspParams, surfaceOffset=0.1):
        """
        Generate the grasps for grabbing a cone from its side
        :param graspParams:[array 1x4] Grasp parameters associated with creating grasps
        :param surfaceOffset:[double] Distance to start the grasp away from the surface
        :return: [list<Grasp>] A list of grasps
        """
        parallelPlanes = graspParams[0]  # this number should be odd
        divisionsOf360 = graspParams[1]
        grasp180Rotations = graspParams[3]

        graspList = []
        theta = math.atan(self.height / self.radius)
        # hypotenuse of height and radius of cone
        hypot = np.sqrt((self.height ** 2 + self.radius ** 2))

        # Side Grasp
        rotation = 0
        for i in range(divisionsOf360):
            # Rotation about the central axis of the object for divisions of 360
            rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, rotation))
            divisionsMatrix = np.matmul(Pose.makeTranformfromPose(self.originPose), rotZMatrix)

            rotation += 2 * np.pi / divisionsOf360
            if parallelPlanes > 1:  # divide hypotenuse by number of parallel planes and start at bottom of cone
                planarTranslation = -hypot / 4  # this is because center of mass is 1/4 of the height not half
            else:
                planarTranslation = hypot / 4  # make sure the location doesn't move center of object hypotenuse
            for j in range(parallelPlanes):
                # First translate along the x axis to reach the slope
                # center of mass is 1/4 of the height for a cone; using similar triangles to find distance to translate
                transXMatrix = Pose.makeTranformfromPose(Pose(3 / 4 * self.radius, 0, 0, 0, 0, 0))
                slopeMatrix = np.matmul(divisionsMatrix, transXMatrix)
                # Rotate about y axis by pi/2-theta to have the x axis perpendicular to slope
                rotYMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, -(np.pi/2 - theta), 0))
                slopeMatrix = np.matmul(slopeMatrix, rotYMatrix)

                # Align Z axis to be approach vector into cone
                rotYMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, -np.pi/2, 0))
                slopeMatrix = np.matmul(slopeMatrix, rotYMatrix)
                # Align X axis to be perpendicular to z axis and central axis
                rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, -np.pi / 2))
                slopeMatrix = np.matmul(slopeMatrix, rotZMatrix)
                # Have frame leave surface of object
                transZMatrix = Pose.makeTranformfromPose(Pose(0, 0, -surfaceOffset, 0, 0, 0))
                slopeMatrix = np.matmul(slopeMatrix, transZMatrix)

                # Perform the parallel plane translation
                transZMatrix = Pose.makeTranformfromPose(Pose(0, planarTranslation, 0, 0, 0, 0))
                parallelPlaneMatrix = np.matmul(slopeMatrix, transZMatrix)

                if parallelPlanes > 1:
                    planarTranslation += hypot / (parallelPlanes - 1)
                wristRotation = 0
                for k in range(2*grasp180Rotations):
                    # Rotate about the approach vector which should now be the z axis
                    rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, wristRotation))
                    graspMatrix = np.matmul(parallelPlaneMatrix, rotZMatrix)
                    wristRotation += np.pi/2

                    # Add to list
                    graspList.append(Grasp('cylindrical', Pose.makePoseFromTransform(graspMatrix)))

        return graspList

    def getEndGrasps(self,graspParams,surfaceOffset=0.1):
        """
        Generate the grasps for grabbing a cone from the bottom or top end
        :param graspParams:[array 1x4] Grasp parameters associated with creating grasps
        :param surfaceOffset:[double] Distance to start the grasp away from the surface
        :return: [list<Grasp>] A list of grasps
        """
        graspRotations = graspParams[2]
        graspList = []
        for i in range(2):
            # rotation about the y axis only if i == 1
            rotYMatrix = Pose.makeTranformfromPose(Pose(0,0,0,0,i*np.pi,0))
            interMatrix = np.matmul(Pose.makeTranformfromPose(self.originPose),rotYMatrix)
            # translate along negative Z axis to bring the grasp out of the object
            transZMatrix = Pose.makeTranformfromPose(Pose(0, 0, -abs(i*self.height - self.height / 4) - surfaceOffset, 0, 0, 0))
            endMatrix = np.matmul(interMatrix, transZMatrix)
            wristRotation = 0
            for j in range(graspRotations):
                # Rotate about the z axis for plan the set of grasps
                rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, wristRotation))
                graspMatrix = np.matmul(endMatrix, rotZMatrix)

                # increment rotation for next loop
                wristRotation += 2 * np.pi / graspRotations

                # Add Grasp to list
                graspList.append(Grasp('spherical', Pose.makePoseFromTransform(graspMatrix)))
        return graspList

    def getEdgeGrasps(self,graspParams,surfaceOffset=0.1):
        """
        Generating grasps for grabbing a cone by its bottom edge
        :param graspParams:[array 1x4] Grasp parameters associated with creating grasps
        :param surfaceOffset:[double] Distance to start the grasp away from the surface
        :return: [list<Grasp>] A list of grasps
        """
        divisionsOf360 = graspParams[1]
        grasp180Rotations = graspParams[3]

        graspList = []
        theta = math.atan(self.height / self.radius)
        # hypotenuse of height and radius of cone
        hypot = np.sqrt((self.height ** 2 + self.radius ** 2))

        # translate along negative Z axis to bring the grasp out of the bottom surface of object
        transZMatrix = Pose.makeTranformfromPose(Pose(0, 0, -(self.height / 4), 0, 0, 0))
        endMatrix = np.matmul(Pose.makeTranformfromPose(self.originPose), transZMatrix)
        rotation = 0
        for i in range(divisionsOf360):
            # Rotation about the central axis of the object for divisions of 360
            rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, rotation))
            divisionsMatrix = np.matmul(endMatrix, rotZMatrix)
            # Translate to outer radius of cone
            transXMatrix = Pose.makeTranformfromPose(Pose(self.radius, 0, 0, 0, 0, 0))
            edgeMatrix = np.matmul(divisionsMatrix, transXMatrix)
            # Fix approach vector
            rotYMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, -(np.pi/2 - theta), 0))
            edgeMatrix = np.matmul(edgeMatrix, rotYMatrix)
            # Align x vector
            rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, -np.pi / 2))
            edgeMatrix = np.matmul(edgeMatrix, rotZMatrix)
            # move surface offset away from object
            transZMatrix = Pose.makeTranformfromPose(Pose(0, 0, -surfaceOffset, 0, 0, 0))
            edgeMatrix = np.matmul(edgeMatrix, transZMatrix)
            
            rotation = rotation + 2 * np.pi / divisionsOf360
            wristRotation = 0
            for j in range(2*grasp180Rotations):
                # Rotate about the z axis for plan the set of grasps
                rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, wristRotation))
                graspMatrix = np.matmul(edgeMatrix, rotZMatrix)

                # increment rotation for next loop
                wristRotation += np.pi/2

                # Add Grasp to list
                graspList.append(Grasp('cylindrical', Pose.makePoseFromTransform(graspMatrix)))
        return graspList

    def generateMesh(self):
        """
        generates the mesh grid for a cone for the 3d plot by:
        1) importing unit shape stl
        2) moves the origin of the stl
        :return: cone STL
        """
        # display figure and get axes
        cone = mesh.Mesh.from_file('STLs/cone.STL')

        # move to origin scale points by dimensions
        vecs = cone.vectors
        vecs[:, :, 0] = (vecs[:, :, 0] - 1) * self.radius
        vecs[:, :, 1] = (vecs[:, :, 1] - 1) * self.radius
        vecs[:, :, 2] = (vecs[:, :, 2] - 0.25) * self.height

        # tranform each point based to based on orientation
        for f in range(vecs.shape[0]):
            for v in range(vecs.shape[1]):
                p = vecs[f, v, :]
                transformed_p = self.applyTransform(p)
                vecs[f, v, :] = np.hstack(transformed_p)

        return cone


    def makeMesh(self):
        """
        creates the mesh grid for a cone for the 3d plot by:
         1) generating unit shape stl
         2) translating stl to origin and scale by the respective properties
         3) transform all points by shape pose
        :return: axes
        """
        # Generate Mesh
        cone = self.generateMesh()

        # Create a new plot
        figure = plt.figure()
        ax = mplot3d.Axes3D(figure)


        # Load the STL files and add the vectors to the plot
        ax.add_collection3d(mplot3d.art3d.Poly3DCollection(cone.vectors, edgecolor='k'))

        # scale plot and add labels
        scale = cone.points.flatten()
        ax.auto_scale_xyz(scale, scale, scale)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')

        return ax
