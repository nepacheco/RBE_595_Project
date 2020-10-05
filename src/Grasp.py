from .Pose import Pose

class Grasp:

    def __init__(self, graspType = 'spherical', pose = Pose(), approachVector = Pose(), thumbOrientation = Pose()):
        self.graspType = graspType
        self.pose = pose
        self.approachVector = approachVector
        self.thumbOrientation = thumbOrientation

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return 'Grasp( type = '+self.graspType+', Pose'+str(self.pose)+')'