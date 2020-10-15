from .Pose import Pose

class Grasp:

    def __init__(self, graspType = 'spherical', pose = Pose()):
        self.graspType = graspType
        self.pose = pose

    def __str__(self):
        return self.__repr__()

    def __repr__(self):
        return 'Grasp( type = '+self.graspType+', Pose'+str(self.pose)+')'