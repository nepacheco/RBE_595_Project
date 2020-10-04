import copy
from src.Shapes.Shape import Shape
from ..Pose import Pose
from src.Grasp import Grasp
import numpy as np



class Box(Shape):

    def __init__(self):
        super().__init__()
        self.height = 0
        self.radius = 0

    # This function was formed assuming
    # Z
    # |
    # |_____X
    # and Y going into the plane
    def planGrasps(self, graspParams):
        '''
        Create each grasp assuming the origin of the shape is the global origin, and then multiply the grasp Pose by the
        transformation matrix to put the grasp location in the global frame
        :param graspParams:
        :return:
        '''
        surfaceOffset = 0.1
        parallelPlanes = graspParams[0] # this number should be odd
        divisionsOf360 = graspParams[1]
        graspRotations = graspParams[2] # value should not be 3
        grasp180Rotations = graspParams[3]

        graspList = []
        # Side Grasps
        graspTranslation = -self.height/2
        for i in range(parallelPlanes):
            # Translation along the objects central axis (z axis) for height placement of grasp
            translationMatrix = Pose.makeTranformfromPose(Pose(0,0,graspTranslation,0,0,0))
            parallelPose = np.matmul(Pose.makeTranformfromPose(self.originPose),translationMatrix)

            #increment GraspTranslation and set grasp rotation
            graspTranslation += self.height/parallelPlanes
            graspRotation = 0
            for j in range(divisionsOf360):
                # Rotation about the objects central axis (z axis) for rotation placement of grasp
                rotZMatrix = Pose.makeTranformfromPose(Pose(0,0,0,0,0,graspRotation))
                divisionsPose = np.matmul(parallelPose,rotZMatrix)

                # increment grasp rotation and set wrist rotation
                graspRotation += 2 * np.pi / divisionsOf360
                wristRotation = 0
                for k in range(grasp180Rotations):
                    # These three transformation matrices move the frame of the grasp outside the object.
                    # Additional it guarantees that the z axis of the frame points towards the object (approach vector)
                    # and that the x axis is perpendicular to the central axis and the approach vector to achieve the
                    # thumb vector
                    transXMatrix = Pose.makeTranformfromPose(Pose(self.radius+surfaceOffset,0,0,0,0,0,0))
                    rotYMatrix = Pose.makeTranformfromPose(Pose(0,0,0,0,-np.pi/2,0))
                    rotZMatrix = Pose.makeTranformfromPose(Pose(0,0,0,0,0,np.pi/2))

                    # multiplication of the transformation matrices
                    graspPose = np.matmul(divisionsPose,transXMatrix)
                    graspPose = np.matmul(graspPose,rotYMatrix)
                    graspPose = np.matmul(graspPose,rotZMatrix)

                    # Rotate about the approach vector (our z axis) if we need to
                    rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, wristRotation))
                    graspPose = np.matmul(graspPose, rotZMatrix)
                    wristRotation += np.pi

                    # Add object to grasp list
                    graspList.append(Grasp('cylindrical', graspPose))

        # End Grasps
        # There are two ends and we just need to invert the z axis for one of them
        for i in range(2):
            # rotation about the y axis only if i == 1
            rotYMatrix = Pose.makeTranformfromPose(Pose(0,0,0,0,i*np.pi,0))
            rotationPose = np.matmul(Pose.makeTranformfromPose(self.originPose),rotYMatrix)
            # translate along negative Z axis to bring the grasp out of the object
            transZMatrix = Pose.makeTranformfromPose(Pose(0,0,-self.height/2 + surfaceOffset,0,0,0))
            translationPose = np.matmul(rotationPose,transZMatrix)

            rotation = 0
            for j in range(graspRotations):
                # Rotate about the z axis for plan the set of grasps
                rotZMatrix = Pose.makeTranformfromPose(Pose(0, 0, 0, 0, 0, rotation))
                graspPose = np.matmul(translationPose, rotZMatrix)

                # increment rotation for next loop
                rotation += 2*np.pi/graspRotations

                # Add Grasp to list
                graspList.append(Grasp('spherical', graspPose))

        return graspList


