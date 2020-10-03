from .Shape import Shape

class Sphere(Shape):
    def __init__(self):
        super().__init__()
        self.radius = 0

    def planGrasps(self, graspParams):
        return
