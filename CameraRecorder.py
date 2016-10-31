import os
import numpy as np
from scipy.interpolate import interp1d 
from scipy.misc import imsave
import openravepy as orp
import GeometryUtils
import TrajectoryUtils
from Action import Action
from OrScreenshotter import OrScreenshotter

class CameraRecorder:
    """
    A class that records frames of a robot performing various actions, and dumps
    them into a directory (the frames are pictures, but they can be strung
    together to make videos).
    Has the advantage that the camera can also move in a trajectory.
    The main functionality is in the CaptureFrames function. However, you must
    call functions that set the robot and camera actions before calling
    CaptureFrames.
    """
    def __init__(self, env, robot, time_interval=0.041, actions=[],
                 video_size=[640, 480],
                 camera_intrinsics=[640, 640, 320, 240]):
        """
        @param env the Openrave Environment we are capturing
        @param robot the robot that will be moving during the camera moving
        @param time_interval the length of the time interval between frames, in
               seconds (eg, 24 frames per second => 0.041 ms per frame, and the
               time_interval should be 0.041 for 24fps video)
        @param actions is a list of actions that the robot will follow.
            The actions are in the order the robot should execute them in.
        """
        self.env = env
        self.robot = robot
        self.time_interval = time_interval
        self.positions_trajectory = None
        self.orientations_trajectory = None
        self.actions = actions
        self.video_size = video_size
        self.camera_intrinsics = camera_intrinsics
        total_duration = 0
        
        for action in self.actions:
            if action.name == "traj":
                total_duration += action.trajectory.GetDuration()
            elif action.name == "wait":
                total_duration += action.time
        self.total_duration = total_duration
        self.num_intervals = total_duration*1.0 / self.time_interval
        print "num intervals", self.num_intervals

    def InsertAction(self, index, name, robot=None, trajectory=None, time=0,\
                     kinbody=None, transform=None):
        action = Action(name, robot=robot, trajectory=trajectory, time=time,
                        kinbody=kinbody, transform=transform)
        self.actions.insert(index, action)
        if action.name == "traj":
            self.total_duration += action.trajectory.GetDuration()
        elif action.name == "wait":
            self.total_duration += action.time
        
    def InsertActions(self, new_actions, index):
        """
        Insert actions beofre the actionat index index. To append actions to the
        end, run
        camerarecorder.InsertActions(new_actions, camerarecorder.GetNumActions())
        @param new_actions a list of actions to insert
        @param index the index to insert them in
        """
        assert index < self.GetNumActions(),\
            "Index must be less than self.GetNumActions()"
        augmented_actions = []
        augmented_actions.extend(self.actions[0:index])
        augmented_actions.extend(new_actions)
        augmented_actions.extend(self.actions[index:])
        self.actions = augmented_actions
        for action in new_actions:
            if action.name == "traj":
                self.total_duration += action.trajectory.GetDuration()
            elif action.name == "wait":
                self.total_duration += action.time

    def GetNumActions(self):
        """
        Return the number of actions that the CameraRecorder is set to record.
        """
        return len(self.actions)

    def SteadyCameraPosition(self, camera_position):
        """
        Set the camera position which the camera will be in for the whole video.
        """
        num_total_intervals = self.total_duration / self.time_interval
        positions_trajectory = np.zeros((num_total_intervals, 3))
        positions_trajectory[:] = camera_position
        self.positions_trajectory = positions_trajectory
        
    def MovingCameraPositions(self, points, evenly_spaced_axis=0,
                         time_before=0, time_after=0):
        """
        Calculate a trajectory from a series of points, and returns elements in
        the trajectory evenly spaced along the evenly_spaced_axis.
        @param points are 3-D points in space along which we want the camera to
            move.
        @param evenly_spaced_axis is the axis along which we will take even
            intervals
        @param time_before and time_after is how long we wish the camera stays in
            the starting or the ending position. We assume the length of the whole
            trajectory, including time_before and time_after, will be as long as
            self.total_duration.
        """
        num_total_intervals = self.total_duration / self.time_interval
        num_before_intervals = time_before / self.time_interval
        num_after_intervals = time_after / self.time_interval
        num_moving_intervals = \
            (self.total_duration - time_before - time_after) / self.time_interval
        moving_trajectory = np.zeros((num_moving_intervals, 3))
        evenly_spaced_points = np.linspace(points[0,evenly_spaced_axis],
                                           points[-1,evenly_spaced_axis],\
                                           num=num_moving_intervals)
        moving_trajectory[:,evenly_spaced_axis] = evenly_spaced_points
        interpolation_axes = [0, 1, 2]
        interpolation_axes.remove(evenly_spaced_axis)
        for axis in interpolation_axes:
            print evenly_spaced_axis, axis
            x_points, y_points = points[:,evenly_spaced_axis],\
                                 points[:,axis]
            sorted_x, sorted_y = zip(*sorted(zip(x_points, y_points)))
            print sorted_x, sorted_y
            f = interp1d(sorted_x, sorted_y)
            calculated_points = f(evenly_spaced_points)
            moving_trajectory[:, axis] = calculated_points

        positions_trajectory = np.zeros((num_total_intervals, 3))
        positions_trajectory[
            num_before_intervals:num_before_intervals+num_moving_intervals,:] =\
            moving_trajectory
        if time_before > 0:
            positions_trajectory[0:num_before_intervals,:] =\
                positions_trajectory[num_before_intervals,:]
        if time_after > 0:
            positions_trajectory[num_before_intervals + num_moving_intervals:,:]\
            = positions_trajectory[num_before_intervals + num_moving_intervals,:]
        self.positions_trajectory = positions_trajectory

    def SteadyCameraOrientation(self, steady_orientation):
        """
        Set the camera orientations in the trajectory to a constant.
        """
        assert self.positions_trajectory is not None, \
            "Must call MovingCameraPositions or SteadyCameraPosition before\
            SteadyCameraOrientation"
        orientations = np.zeros((self.positions_trajectory.shape[0], 3))
        orientations[:,:] = steady_orientation
        self.orientations_trajectory = orientations
    
    def CaptureFrames(self, directory, can_getcameraimage=False):
        """
        Capture the actions specified with the camera movements specified, and
        put all of the captured frames into directory.
        @param directory the directory in which to store the frames.
        @param can_getcameraimage if we can use env.GetViewer().GetCameraImage()

        """
        assert self.positions_trajectory is not None and\
               self.orientations_trajectory is not None,\
               "Must call SteadyCameraPosition/MovingCameraPositions and \
               SteadyCameraOrientation before running CaptureFrames"
        assert self.actions is not None and self.GetNumActions() > 0\
            and self.total_duration > 0,\
            "Please set at least one timed action for the robot to perform"
        total_intervals = int(self.total_duration / self.time_interval)
        if not os.path.isdir(directory):
            os.makedirs(directory)
        last_config = self._get_starting_config()
        if not can_getcameraimage:
            screenshotter = OrScreenshotter(self.env)
        # A queue of actions, and what duration of that action has already been
        # performed.
        actions_queue = [action for action in self.actions]
        frame_num = 0
        current_action_progress = 0
        while len(actions_queue) > 0:
            curr_action = actions_queue[0]
            # Check if this is an auxiliarry action, or if it is an action that has
            # less than self.time_interval time left to perform.
            while curr_action.is_aux_action() or \
                  current_action_progress > curr_action.get_duration() -\
                                            self.time_interval:
                if curr_action.is_aux_action():
                    self.perform_aux_action(curr_action)
                actions_queue.pop(0)
                current_action_progress = 0
                curr_action = actions_queue[0]
            # Now we have a timed action, which we know has at least one time 
            # interval left of time.
            if curr_action.name == "wait":
                capture_config = last_config
            elif curr_action.name == "traj":
                capture_config = TrajectoryUtils.SampleDofValues(
                    curr_action.trajectory, current_action_progress)
            self.robot.SetDOFValues(capture_config)
            camera_transform = np.zeros((4, 4))
            camera_transform =\
                GeometryUtils.SetTranslation(camera_transform,
                    self.positions_trajectory[frame_num])
            camera_transform =\
                GeometryUtils.SetRotation(camera_transform,
                    self.orientations_trajectory[frame_num])
            if can_getcameraimage:
                frame = self.env.GetViewer().\
                    GetCameraImage(self.video_size[0],
                                   self.video_size[1],
                                   self.env.GetViewer().GetCameraTransform(),
                                   self.camera_intrinsics)
                imsave("{}/frame{}".format(directory, frame_num), frame)
            else:
                self.env.GetViewer().SetCamera(camera_transform)
                screenshotter.capture("{}/frame{}.png".format(directory, frame_num))
            last_config = capture_config
            current_action_progress += self.time_interval
            frame_num += 1

    def perform_aux_action(self, action):
        assert action.is_aux_action(),\
            "{} is not an auxilliary action".format(action.name)
        if action.name == "grab":
            self.robot.Grab(action.kinbody)
        if action.name == "release":
            self.robot.Release(action.kinbody)
        if action.name =="settransform":
            action.kinbody.SetTransform(action.transform)

    def _get_starting_config(self):
        for action in self.actions:
            if action.name == "traj":
                return action.trajectory.GetWaypoint(0)

class Action:
    """
    An action that the robot should perform. Can have one of four types:
    "traj", with the argument list consisting of the trajectory to execute
    "wait", with the argument list consisting of the duration of the wait
    "grab", with the argument list consisting of the object to grab
    "release", with the argument list consisting of the object to release
    "settransform", with the argument list consisting of the object to move,
        and the transform it is being set to.
    We define the terminology that "traj" and "wait" are timed actions (as
        they take time), and the rest are auxilliary actions.

    The Action is the format in which the CameraRecorder takes robot actions it
    should record.
    """
    def __init__(self, name, robot=None, trajectory=None, time=0, kinbody=None,
                 transform=None):
        assert name in ["traj", "wait", "grab", "release", "settransform"],\
            "please specify a valid action name"
        self.name = name
        if name == "traj":
            assert trajectory is not None, "trajectory argument must be non\
                                            none for traj actions"
            self.trajectory = trajectory
        if name == "wait":
            assert time > 0, "wait times must be positive"
            self.time = time
        if name == "grab":
            assert kinbody is not None, "kinbody argument must be non-none for\
                                         grab actions"
            self.kinbody = kinbody
        if name == "release":
            assert kinbody is not None, "kinbody argument must be non-none for\
                                         release actions"
            self.kinbody = kinbody
        if name == "settransform":
            assert kinbody is not None and transform is not None,\
                "kinbody and transform arguments must be non-none for settransform"
            self.kinbody = kinbody
            self.transform = transform

    def is_aux_action(self):
        return self.name == "grab" or\
               self.name == "release" or\
               self.name == "settransform"

    def get_duration(self):
        if self.is_aux_action():
            return -1
        if self.name == "traj":
            return self.trajectory.GetDuration()
        if self.name == "wait":
            return self.time

