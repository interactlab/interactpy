import openravepy
from prpy import bind_subclass, Cloned
from prpy.planning import SnapPlanner, BiRRTPlanner
from prpy.base.robot import Robot
from archiemanip import ArchieManipulator


class ArchieRobot(Robot):
    def __init__(self):
        Robot.__init__(self, robot_name='archie')
        # Set up ikfast for the robot. May want to add this to a
        # Manipulator class init method instead.
        self.ikmodel = (
            openravepy.databases.inversekinematics.InverseKinematicsModel(
                self, iktype=openravepy.IkParameterization.Type.Transform6D))
        if not self.ikmodel.load():
            self.ikmodel.autogenerate()

        self.arm = self.GetManipulator('j2s7s300')
        bind_subclass(self.arm, ArchieManipulator)
        self.planner = SnapPlanner()
        self.arm.sim_controller = self.AttachController(
            name=self.arm.GetName(),
            args='IdealController',
            dof_indices=self.arm.GetArmIndices(),
            affine_dofs=0,
            simulated=True)
        self.SetActiveDOFs(self.arm.GetIndices())

    def CloneBindings(self, parent):
        super(ArchieRobot, self).CloneBindings(parent)
        self.arm = Cloned(parent.arm)
        self.planner = parent.planner
