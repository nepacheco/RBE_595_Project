import copy
from src.Shapes.Shape import Shape
from ..Pose import Pose
from src.Grasp import Grasp
from scipy.spatial.transform import Rotation as R
import numpy as np
import math


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
    def planGrasps(self, graspParams):
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
        surfaceOffset = 0.1
        parallelPlanes = graspParams[0]  # this number should be odd
        divisionsOf360 = graspParams[1]
        graspRotations = graspParams[2]  # value should not be 3
        grasp180Rotations = graspParams[3]

        graspList = []
        theta = math.atan(self.height/self.radius)
        # hypotenuse of height and radius of cone
        hypot = np.sqrt((self.height**2 + self.radius**2))

        # Side Grasp
        rotation = 0
        for i in range(divisionsOf360):
            # Rotation about the central axis of the object for divisions of 360
            rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, rotation))
            divisionsMatrix = np.matmul(Pose.makeTranformfromPose(self.originPose), rotZMatrix)

            rotation += 2*np.pi/divisionsOf360
            if parallelPlanes > 1:  # divide hypotenuse by number of parallel planes and start at bottom of cone
                planarTranslation = -hypot/4  # this is because center of mass is 1/4 of the height not half
            else:
                planarTranslation = hypot/4  # make sure the location doesn't move center of object hypotenuse
            for j in range(parallelPlanes):
                # First translate along the x axis to reach the slope
                # center of mass is 1/4 of the height for a cone; using similar triangles to find distance to translate
                transXMatrix = Pose.makeTranformfromPose(Pose(3/4*self.radius,0,0,0,0,0))
                slopeMatrix = np.matmul(divisionsMatrix,transXMatrix)
                # Rotate about y axis by pi-theta to have the x axis perpendicular to slope
                rotYMatrix = Pose.makeTranformfromPose(Pose(0,0,0,0,-(np.pi-theta),0))
                slopeMatrix = np.matmul(slopeMatrix,rotYMatrix)

                # Align Z axis to be approach vector into cone
                rotYMatrix = Pose.makeTranformfromPose(Pose(0,0,0,0,-np.pi,0))
                slopeMatrix = np.matmul(slopeMatrix,rotYMatrix)
                # Align X axis to be perpendicular to z axis and central axis
                rotZMatrix = Pose.makeTranformfromPose(Pose(0,0,0,0,0,-np.pi/2))
                slopeMatrix = np.matmul(slopeMatrix, rotZMatrix)
                # Have frame leave surface of object
                transZMatrix = Pose.makeTranformfromPose(Pose(0,0,-surfaceOffset,0,0,0))
                slopeMatrix = np.matmul(slopeMatrix,transZMatrix)

                # Perform the parallel plane translation
                transZMatrix = Pose.makeTranformfromPose(Pose(0,0,planarTranslation,0,0,0))
                parallelPlaneMatrix = np.matmul(slopeMatrix,transZMatrix)

                if parallelPlanes > 1:
                    planarTranslation += hypot/(parallelPlanes-1)
                wristRotation = 0
                for k in range(grasp180Rotations):
                    # Rotate about the approach vector which should now be the z axis
                    rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, wristRotation))
                    graspMatrix = np.matmul(parallelPlaneMatrix,rotZMatrix)
                    wristRotation += np.pi

                    # Add to list
                    graspList.append(Grasp('cylindrical', Pose.makePoseFromTransform(graspMatrix)))
                    pass

        # End Grasp
        for i in range(2):
            # rotation about the y axis only if i == 1
            rotYMatrix = Pose.makeTranformfromPose(Pose(0,0,0,0,i*np.pi,0))
            interMatrix = np.matmul(Pose.makeTranformfromPose(self.originPose),rotYMatrix)
            # translate along negative Z axis to bring the grasp out of the object
            transZMatrix = Pose.makeTranformfromPose(Pose(0, 0, -abs(i - self.height / 4 + surfaceOffset), 0, 0, 0))
            endMatrix = np.matmul(Pose.makeTranformfromPose(self.originPose), transZMatrix)
            wristRotation = 0
            for j in range(graspRotations):
                # Rotate about the z axis for plan the set of grasps
                rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, wristRotation))
                graspMatrix = np.matmul(endMatrix, rotZMatrix)

                # increment rotation for next loop
                wristRotation += 2 * np.pi / graspRotations

                # Add Grasp to list
                graspList.append(Grasp('spherical', Pose.makePoseFromTransform(graspMatrix)))


        # Edge Grasp
        # translate along negative Z axis to bring the grasp out of the bottom surface of object
        transZMatrix = Pose.makeTranformfromPose(Pose(0, 0, -(self.height / 4), 0, 0, 0))
        endMatrix = np.matmul(Pose.makeTranformfromPose(self.originPose), transZMatrix)
        rotation = 0
        for i in range(divisionsOf360):
            # Rotation about the central axis of the object for divisions of 360
            rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, rotation))
            divisionsMatrix = np.matmul(endMatrix, rotZMatrix)
            # Translate to outer radius of cone
            transXMatrix = Pose.makeTranformfromPose(Pose(self.radius,0,0,0,0,0))
            edgeMatrix = np.matmul(divisionsMatrix, transXMatrix)
            # Fix approach vector
            rotYMatrix = Pose.makeTranformfromPose(Pose(0,0,0,0,-(np.pi-theta),0))
            edgeMatrix = np.matmul(edgeMatrix,rotYMatrix)
            # Align x vector
            rotZMatrix = Pose.makeTranformfromPose(Pose(0,0,0,0,0,-np.pi/2))
            edgeMatrix = np.matmul(edgeMatrix, rotYMatrix)
            # move surface offset away from object
            transZMatrix = Pose.makeTranformfromPose(Pose(0,0,-surfaceOffset,0,0,0));
            edgeMatrix = np.matmul(edgeMatrix, transZMatrix)

            wristRotation += 2*np.pi/divisionsOf360
            graspRotation = 0
            for j in range(grasp180Rotations):
                # Rotate about the z axis for plan the set of grasps
                rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, wristRotation))
                graspMatrix = np.matmul(edgeMatrix, rotZMatrix)

                # increment rotation for next loop
                rotation += np.pi

                # Add Grasp to list
                graspList.append(Grasp('cylindrical', Pose.makePoseFromTransform(graspMatrix)))


        return graspList