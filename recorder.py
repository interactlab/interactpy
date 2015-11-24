def TakeSnapshot(viewer, path=None, show_figures=True, width=1920, height=1080, fx=640, fy=640):
    '''
    Copied from https://github.com/personalrobotics/prpy/src/prpy/util.py
    '''
    if isinstance(viewer, openravepy.Environment):
        viewer = viewer.GetViewer()

    viewer.SendCommand('SetFiguresInCamera {0:d}'.format(show_figures))
    image = viewer.GetCameraImage(width, height, viewer.GetCameraTransform(),
                                  [ fx, fy, width/2, height/2 ])

    if path is not None:
        scipy.misc.imsave(path, image)

    return image

def TakeRecording(traj, robot, framerate=24, path='traj_recording', width=1920, height=1080, fx=640, fy=640):
    viewer = robot.GetEnv().GetViewer()
    # Not every robot DOF is necessarily part of the active manipulator of the trajectory,
    # so we must figure out which indices are specified in the trajectory
    armIndices = robot.GetActiveManipulator().GetArmIndices()
    joint_offset = traj.GetConfigurationSpecification().GetGroupFromName('joint_values').offset
    # Sample trajectory, set configuration, and capture image for each frame
    for frame in range(framerate * (int(traj.GetDuration()) + 1)):
        configuration = robot.GetActiveDOFValues()
        sample = traj.Sample(float(frame) / 24)
        for i, dof_index in enumerate(armIndices):
             configuration[dof_index] = sample[joint_offset + i]
        robot.SetActiveDOFValues(configuration)
        TakeSnapshot(viewer, path + str(frame), show_figures=True, width, height, fx, fy)
