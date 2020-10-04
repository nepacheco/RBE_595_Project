from .Shape import Shape
from copy import deepCopy
class Sphere(Shape):
    def __init__(self):
        super().__init__()
        self.radius = 0

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
        InitialPose = deepCopy(self.originPose)
        InitialPose.x += self.radius + palmDistance

        azimuth = 0
        theta = (2*pi)/divisions
        phi = (2 * pi) / graspRotations
        while azimuth < divisions:
            approachPose = Pose()
            approachPose.x = -1
            radius = self.radius + palmDistance
            InitialPoseH = deepCopy(self.originPose)
            InitialPoseH.x = radius * cos(azimuth*theta)
            InitialPoseH.y = radius * sin(azimuth*theta)
            InitialPoseH.yaw += azimuth*theta

            rotations = 0
            while rotations < graspRotations:
                thumbPoseH = Pose()
                thumbPoseH.y = 1
                thumbPoseH.roll = rotations*phi
                currentGH=grasp('spherical',InitialPoseH, approachPose, thumbPoseH)
                totalGrasps.append(currentGH)

            elevation = 0
            while elevation < divisions:
                InitialPoseV = deepCopy(InitialPoseH)
                InitialPoseV.x = radius * cos(elevation*theta)
                InitialPoseV.z = radius * sin(elevation*theta)
                InitialPoseV.pitch += elevation*theta
                rotations = 0
                while rotations < graspRotations:
                    thumbPoseV = Pose()
                    thumbPoseV.y = 1
                    thumbPoseV.roll = rotations * phi
                    currentGV = grasp('spherical', InitialPoseV, approachPose, thumbPoseV)
                    totalGrasps.append(currentGV)
                elevation += 1
            azimuth += 1
        return totalGrasps
