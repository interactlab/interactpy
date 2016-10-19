import numpy as np
import openravepy

# TODO(allanzhou): All the stuff in here should eventually be
# moved into the appropriate files.


class PlanningError(Exception):
    pass


class SimpleManipulation:
    def __init__(self, env, robot, ik_name='NloptIK'):
        self.env = env
        self.robot = robot
        self.eff = self.robot.GetActiveManipulator()
        self.iksolver = openravepy.RaveCreateIkSolver(env, ik_name)
        self.eff.SetIKSolver(self.iksolver)
        self.planner = SimplePlanner(self.env)

    def FindIKSolution(self, goal):
        return self.eff.FindIKSolution(
                goal, openravepy.IkFilterOptions.CheckEnvCollisions)

    def ExecuteTrajectory(self, traj):
        self.robot.GetController().SetPath(traj)

    def MoveToGoal(self, goal, execute=False):
        """
        Move manipulator to goal end effector transform, or return the
        trajectory if execute is False.

        @param goal the goal end effector transform
        @return traj a trajectory from current configuration to specified goal
        """

        config = self.robot.GetDOFValues()
        indices = self.eff.GetArmIndices()
        soln = self.FindIKSolution(goal)
        np.put(config, indices, soln)
        traj = self.planner.PlanToConfiguration(self.robot, config)
        self.planner.SmoothTrajectory(traj, self.robot)
        if execute:
            self.ExecuteTrajectory(traj)
        return traj


class SimplePlanner:
    def __init__(self, env, planner='birrt'):
        self.env = env
        self.planner = openravepy.RaveCreatePlanner(self.env, planner)
        self.smoother = openravepy.RaveCreatePlanner(self.env, 'ParabolicSmoother')

    def PlanToConfiguration(self, robot, goal, time_traj=True, **kw_args):
        """
        Plan to a desired configuration with OpenRAVE.
        Adds timing data if time_traj is true.
        @param robot the robot whose active DOFs will be used
        @param goal the desired robot joint configuration
        @param time_traj declares whether the trajectory should have timestamps
        @return traj a trajectory from current configuration to specified goal
        """

        traj = self._Plan(robot, goal, **kw_args)
        if time_traj:
            with self.env:
                openravepy.planningutils.RetimeActiveDOFTrajectory(traj, robot)
        return traj

    def SmoothTrajectory(self, traj, robot):
        with self.env:
            planner_params = openravepy.Planner.PlannerParameters()
            planner_params.SetRobotActiveJoints(robot)
            self.smoother.InitPlan(robot, planner_params)
            self.smoother.PlanPath(traj)

    def GenerateRetimedTrajectory(self, robot, traj, f=lambda t: t, num_wp_new=None, factor=1):
        '''Retimes input trajectory for robot according to the given function f.

        Here f maps new trajectory time to source trajectory time, where inputs and
        outputs are proportions (ie, they are in the range [0, 1]).

        @param robot the robot
        @param traj the source trajectory
        @param f the function mapping source trajectory time to output traj time
        @param num_wp_new the number of waypoints in output traj
        @return new_traj the retimed trajectory
        '''
        num_wp_src = traj.GetNumWaypoints()
        num_wp_new = num_wp_src if not num_wp_new else num_wp_new
        dur, spec = traj.GetDuration(), traj.GetConfigurationSpecification()
        # Make sure that input function does not change total trajectory duration
        assert abs(f(1) - 1.0) < .01
        assert abs(f(0) - 0.0) < .01

        new_traj = openravepy.RaveCreateTrajectory(robot.GetEnv(), '')
        new_traj.Init(spec)

        t_interval = factor * (dur / (num_wp_new - 1))
        beginning, end = traj.GetWaypoint(0), traj.GetWaypoint(num_wp_src - 1)
        # The start waypoint remains the same
        new_traj.Insert(0, beginning)
        for i in range(1, num_wp_new-1):
            wp = traj.Sample(max(dur * f(float(i) / num_wp_new), 0))
            spec.InsertDeltaTime(wp, t_interval)
            new_traj.Insert(i, wp)
        # End waypoint remains the same
        spec.InsertDeltaTime(end, t_interval)
        new_traj.Insert(num_wp_new - 1, end)
        # Retime velocities
        retime_result = openravepy.planningutils.RetimeTrajectory(new_traj, hastimestamps=True)
        return new_traj

    def _Plan(self, robot, goals, maxiter=500, continue_planner=False,
              or_args=None, **kw_args):

        # Get rid of default postprocessing
        extraParams =  '<_postprocessing planner=""><_nmaxiterations>0</_nmaxiterations></_postprocessing>'
        # Maximum planner iterations
        extraParams += '<_nmaxiterations>{:d}</_nmaxiterations>'.format(maxiter)

        if or_args is not None:
            for key, value in or_args.iteritems():
                extraParams += '<{k:s}>{v:s}</{k:s}>'.format(k=str(key),
                                                             v=str(value))

        params = openravepy.Planner.PlannerParameters()
        params.SetRobotActiveJoints(robot)
        params.SetGoalConfig(goals)
        params.SetExtraParameters(extraParams)

        traj = openravepy.RaveCreateTrajectory(self.env, 'GenericTrajectory')

        try:
            self.env.Lock()

            # Plan.
            if (not continue_planner) or not self.setup:
                self.planner.InitPlan(robot, params)
                self.setup = True

            status = self.planner.PlanPath(traj, releasegil=True)
            if status not in [openravepy.PlannerStatus.HasSolution,
                              openravepy.PlannerStatus.InterruptedWithSolution]:
                raise PlanningError('Planner returned with status {:s}.'
                                    .format(str(status)))
        except Exception as e:
            raise PlanningError('Planning failed with error: {:s}'.format(e))
        finally:
            self.env.Unlock()

        return traj

def LinearInterpolation(start_config, goal_config, num_steps, robot):
    """
    Return a path of num_steps configurations between start_config and
    goal_config. The steps are equally spaced and linear in configuration space.
    @param start_config the configuration at which the trajectory starts
    @param goal_config the configuration at which the trajectory ends
    @param num_steps the number of intermediate configurations in the trajectory
    @param robot the robot which is to perform this trajectory.
    """
    diff_config = robot.SubtractActiveDOFValues(goal_config, start_config)
    step = diff_config / num_steps
    waypoints =  [list(start_config)]
    last_config = list(start_config)
    for i in range(num_steps):
        last_config += step
        waypoints.append(list(last_config))
    return waypoints

def MakeIntoTrajectoryWaypoints(configs, total_time, joint_values_offset=0,
                                joint_velocities_offset=10, delta_time_offset=20,
                                is_waypoint_offset=21):
    """
    Take a list of configurations, and make into one list that could be the data
    in OpenRAVE TrajectoryBase waypoints. 
    @param configs a list of configurations
    @param total_time how much time we want the trajectory to take
    @param joint_values_offset the offset in the configurations specification
        where the joint values in a waypoint start
    @param joint_velocities_offset the offset in the configurations specification
        where the joint velocities in a waypoint start
    @param delta_time_offset the offset in the configurations specification
        where the delta_time in a waypoint is.
    @param is_waypoint_offset the offset in the configurations specification
        where the iswaypoint field in a waypoint is.
    The default offset values are set to fit to a waypoint of a normal Jaco
    trajectory.
    """
    num_waypoints = len(configs)
    num_dofs = len(configs[0])
    # A waypoint has joint values, joint velocities, deltatime and iswaypoint
    waypoint_length = num_dofs*2 + 2
    time_per_waypoint = total_time*1.0 / num_waypoints
    waypoints = []
    for i in range(num_waypoints):
        waypoint = np.zeros(waypoint_length)
        waypoint[joint_values_offset:joint_values_offset + num_dofs] =\
            configs[i]
        waypoint[delta_time_offset] = time_per_waypoint
        waypoint[joint_velocities_offset:joint_velocities_offset + num_dofs] =\
            np.subtract(configs[i],configs[i-1]) / (time_per_waypoint*1.0)
        waypoint[is_waypoint_offset] = 1
        waypoints.extend(waypoint)
        if i == 1:
            print waypoint
    return waypoints
