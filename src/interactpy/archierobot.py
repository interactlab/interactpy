import openravepy


class ArchieRobot(openravepy.Robot):
    def __init__(self):
        openravepy.Robot.__init__(self, name='Archie')
