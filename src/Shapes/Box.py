import copy
from src.Shapes.Shape import Shape
from src.Grasp import Grasp
from src.Pose import Pose



class Box(Shape):

    def __init__(self):
        super().__init__()
        self.height = 0
        self.width = 0
        self.length = 0

    # This function was formed assuming
    # Z
    # |
    # |_____X
    # and Y going into the plane
    def planGrasps(self, graspParams):
        palmDistance = 5
        numberCubeSides = 6
        xEdgeDistance = self.length / 2
        yEdgeDistance = self.width / 2
        zEdgeDistance = self.height / 2
        parallelPlanes = graspParams[0]  # Has to at least be 1
        numberRotations = graspParams[3]
        listOfGrasps = []

        index = 0

        # The sides of the box are labeled like a dice
        while index < numberCubeSides:
            numGraspPerSide = 1
            while numGraspPerSide <= parallelPlanes:
                palmPoseV = Pose()
                palmPoseH = Pose()
                approachPose = Pose()
                thumbPoseV = Pose()
                thumbPoseH = Pose()

                palmPoseV.x = self.originPose.x
                palmPoseV.y = self.originPose.y
                palmPoseV.z = self.originPose.z
                palmPoseV.roll = self.originPose.roll
                palmPoseV.pitch = self.originPose.pitch
                palmPoseV.yaw = self.originPose.yaw

                palmPoseH.x = self.originPose.x
                palmPoseH.y = self.originPose.y
                palmPoseH.z = self.originPose.z
                palmPoseH.roll = self.originPose.roll
                palmPoseH.pitch = self.originPose.pitch
                palmPoseH.yaw = self.originPose.yaw

                approachPose.roll = self.originPose.roll
                approachPose.pitch = self.originPose.pitch
                approachPose.yaw = self.originPose.yaw

                thumbPoseV.roll = self.originPose.roll
                thumbPoseV.pitch = self.originPose.pitch
                thumbPoseV.yaw = self.originPose.yaw

                thumbPoseH.roll = self.originPose.roll
                thumbPoseH.pitch = self.originPose.pitch
                thumbPoseH.yaw = self.originPose.yaw

                if index == 1:
                    palmPoseV.y = self.originPose.y - yEdgeDistance - palmDistance
                    palmPoseH.y = self.originPose.y - yEdgeDistance - palmDistance
                    palmPoseV.z = (self.originPose.z - zEdgeDistance) + (
                                numGraspPerSide * (self.height / (parallelPlanes + 1)))
                    palmPoseH.x = (self.originPose.x - xEdgeDistance) + (
                                numGraspPerSide * (self.length / (parallelPlanes + 1)))
                    approachPose.y = 1
                    thumbPoseV.x = 1
                    thumbPoseH.z = 1

                if index == 2:
                    palmPoseV.z = self.originPose.z - zEdgeDistance - palmDistance
                    palmPoseH.z = self.originPose.z - zEdgeDistance - palmDistance
                    palmPoseV.y = (self.originPose.y - yEdgeDistance) + (
                                numGraspPerSide * (self.width / (parallelPlanes + 1)))
                    palmPoseH.x = (self.originPose.x - xEdgeDistance) + (
                                numGraspPerSide * (self.length / (parallelPlanes + 1)))
                    approachPose.z = 1
                    thumbPoseV.x = 1
                    thumbPoseH.y = 1

                if index == 3:
                    palmPoseV.x = self.originPose.x + xEdgeDistance + palmDistance
                    palmPoseH.x = self.originPose.x + xEdgeDistance + palmDistance
                    palmPoseV.z = (self.originPose.z - zEdgeDistance) + (
                                numGraspPerSide * (self.height / (parallelPlanes + 1)))
                    palmPoseH.y = (self.originPose.y - yEdgeDistance) + (
                                numGraspPerSide * (self.width / (parallelPlanes + 1)))
                    approachPose.x = -1
                    thumbPoseV.y = 1
                    thumbPoseH.z = 1

                if index == 4:
                    palmPoseV.x = self.originPose.x - xEdgeDistance - palmDistance
                    palmPoseH.x = self.originPose.x - xEdgeDistance - palmDistance
                    palmPoseV.z = (self.originPose.z - zEdgeDistance) + (
                                numGraspPerSide * (self.height / (parallelPlanes + 1)))
                    palmPoseH.y = (self.originPose.y - yEdgeDistance) + (
                                numGraspPerSide * (self.width / (parallelPlanes + 1)))
                    approachPose.x = 1
                    thumbPoseV.y = 1
                    thumbPoseH.z = 1

                if index == 5:
                    palmPoseV.z = self.originPose.z + zEdgeDistance + palmDistance
                    palmPoseH.z = self.originPose.z + zEdgeDistance + palmDistance
                    palmPoseV.y = (self.originPose.y - yEdgeDistance) + (
                                numGraspPerSide * (self.width / (parallelPlanes + 1)))
                    palmPoseH.x = (self.originPose.x - xEdgeDistance) + (
                                numGraspPerSide * (self.length / (parallelPlanes + 1)))
                    approachPose.z = -1
                    thumbPoseV.x = 1
                    thumbPoseH.y = 1

                if index == 6:
                    palmPoseV.y = self.originPose.y + yEdgeDistance + palmDistance
                    palmPoseH.y = self.originPose.y + yEdgeDistance + palmDistance
                    palmPoseV.z = (self.originPose.z - zEdgeDistance) + (
                                numGraspPerSide * (self.height / (parallelPlanes + 1)))
                    palmPoseH.x = (self.originPose.x - xEdgeDistance) + (
                                numGraspPerSide * (self.length / (parallelPlanes + 1)))
                    approachPose.y = -1
                    thumbPoseV.x = 1
                    thumbPoseH.z = 1

                currentGraspV = Grasp('cylindrical', palmPoseV, approachPose, thumbPoseV)
                currentGraspH = Grasp('cylindrical', palmPoseH, approachPose, thumbPoseH)
                if numberRotations == 2:
                    rCurrentGraspV = copy.deepcopy(currentGraspV)
                    rCurrentGraspH = copy.deepcopy(currentGraspH)
                    if currentGraspV.thumbOrientation.x == 1:
                        rCurrentGraspV.thumbOrientation.x = -1
                    if currentGraspV.thumbOrientation.y == 1:
                        rCurrentGraspV.thumbOrientation.y = -1
                    if currentGraspV.thumbOrientation.z == 1:
                        rCurrentGraspV.thumbOrientation.z = -1
                    if currentGraspH.thumbOrientation.x == 1:
                        rCurrentGraspH.thumbOrientation.x = -1
                    if currentGraspH.thumbOrientation.y == 1:
                        rCurrentGraspH.thumbOrientation.y = -1
                    if currentGraspH.thumbOrientation.z == 1:
                        rCurrentGraspH.thumbOrientation.z = -1
                    listOfGrasps.append(rCurrentGraspV)
                    listOfGrasps.append(rCurrentGraspH)
                listOfGrasps.append(currentGraspV)
                listOfGrasps.append(currentGraspH)
                numGraspPerSide += 1

            index += 1

        return listOfGrasps


