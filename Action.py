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

