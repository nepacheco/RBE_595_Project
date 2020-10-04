import copy
from src.Shapes.Shape import Shape
from ..Pose import Pose
from src.Grasp import Grasp
import numpy



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
        graspRotations = graspParams[2]
        grasp180Rotations = graspParams[3]

        graspList = []
        # Side Grasps
        initTranslation = -self.height/2
        initRotation = 0

        graspPose = self.originPose
        for i in range(parallelPlanes):
            # TODO increase height of position
            graspPose = self.originPose
            for j in range(divisionsOf360):
                # TODO rotate about z axis by increment
                for k in range(grasp180Rotations):
                    #TODO rotate about 180 about the approach vector axis
                    graspList.append(Grasp('cylindrical', graspPose))





