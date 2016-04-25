import numpy
import openravepy

class PlanningError(Exception):
    pass

class SimpleManipulation:
    def __init__(self, env, robot):
        self.env = env
        self.robot = robot
        self.eff = self.robot.GetActiveManipulator()
        self.iksolver = openravepy.RaveCreateIkSolver(env, 'NloptIK')
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
        numpy.put(config, indices, soln)
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
