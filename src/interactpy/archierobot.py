import openravepy


class ArchieRobot(openravepy.Robot):
    def __init__(self):
        # Set up ikfast for the robot. May want to add this to a
        # Manipulator class init method instead.
        self.ikmodel = (
            openravepy.databases.inversekinematics.InverseKinematicsModel(
                self, iktype=openravepy.IkParameterization.Type.Transform6D))
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()
