from .Pose import Pose

class Grasp:

    def __init__(self, graspType = 'spherical', pose = Pose(), approachVector = Pose(), thumbOrientation = Pose()):
        self.graspType = graspType
        self.pose = pose
        self.approachVector = approachVector
        self.thumbOrientation = thumbOrientation
        