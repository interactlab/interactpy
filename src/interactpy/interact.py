import openravepy
from prpy.bind import bind_subclass
from archierobot import ArchieRobot


def initialize():
    '''Load and configure the Archie robot. Returns robot and environment.'''
    env = openravepy.Environment()
    env.SetViewer('qtcoin')
    # Assumes the robot files are located in the urdf folder of the
    # kinova_description package in the catkin workspace.
    urdf_uri = 'package://kinova_description/urdf/j2s7s300_standalone.urdf'
    srdf_uri = 'package://kinova_description/urdf/jaco7dof_standalonev1.srdf'
    or_urdf = openravepy.RaveCreateModule(env, 'urdf')
    robot_name = or_urdf.SendCommand(
        'load {:s} {:s}'.format(urdf_uri, srdf_uri))
    robot = env.GetRobot(robot_name)
    bind_subclass(robot, ArchieRobot)
    # Put robot in natural starting config.
    # TODO(allanzhou): Decide if this is really necessary.
    robot.SetDOFValues([0, 3, 0, 2, 0, 4, 0, 0, 0, 0])
    return env, robot
