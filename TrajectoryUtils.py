import numpy as np
import openravepy as orp

"""
Utilities that deal with a trajectory.

Perhaps in the future these functions will be methods in a class we make to extend
the OpenRAVE Trajectory class.
"""
def SampleDofValues(trajectory, time, 
                    joint_value_group_name_prefix="joint_values"):
    assert time < trajectory.GetDuration(),\
        "Trajectory must be sampled at a time before it ends"
    spec = trajectory.GetConfigurationSpecification()
    joint_value_group = spec.GetGroupFromName(joint_value_group_name_prefix)
    return trajectory.Sample(time, joint_value_group)

