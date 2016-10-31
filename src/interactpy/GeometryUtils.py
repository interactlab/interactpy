import numpy as np
import openravepy as orp

# Return a transform that signifies the position position, and an identity rotation.
def MatrixFromPosition(position):
    return np.array([[1, 0, 0, position[0]],
                     [0, 1, 0, position[1]],
                     [0, 0, 1, position[2]],
                     [0, 0, 0, 1]])

# Set the translation of the transform transform to be the position position.
# Keep the rotation part of the transform the same.
def SetTranslation(transform, position):
    translated = transform.copy()
    translated[0,3] = position[0]
    translated[1,3] = position[1]
    translated[2,3] = position[2]
    return translated

# Set the rotation of transform to be the three values in rotation_angles.
# Rotation angles is three angles, in radians, that refer to the x, y and z axes, 
# respectively.
def SetRotation(transform, rotation_angles):
    rotation =  orp.matrixFromAxisAngle(rotation_angles)
    rotated = transform.copy()
    rotated[0:3, 0:3] = rotation[0:3, 0:3]
    return rotated

def SetRotationAxisToZero(transform, rotation_axis):
    """
    Takes the transform, and returns the same transform but where the axis
    rotation_axis has rotation 0
    """
    assert rotation_axis >= 0 and rotation_axis < 3,\
        "rotation axis must be in between 0 and 2, is %d" % rotation_axis
    axis_angle = orp.AxisAngleFromMatrix(transform)
    axis_angle[rotation_axis] = 0
    return SetRotation(transform, axis_angle)

def GetPositionCloseToKinbody(kinbody, position, distance=0.08):
    """
    Return a length-3  xyz  position close to the edge of the box around kinbody.
    Position is a length-3 vector containing elements from {0, 1, -1}, which says
    which side of the object we want to be on (for example, [-1, 0, 0] means on the
    left, and [0, 1, 0] means on top). The distance parameter is how far away from 
    the edge of the box we want to be.
    """
    kinbody_aabb = kinbody.ComputeAABB()
    kinbody_position = kinbody_aabb.pos()
    kinbody_extents = kinbody_aabb.extents()
    return \
    [kinbody_position[0] + (kinbody_extents[0] + distance)*position[0],
    kinbody_position[1] + (kinbody_extents[1] + distance)*position[1],
    kinbody_position[2] + (kinbody_extents[2] + distance)*position[2]]

def cube_k_away(point, k):
    """
    A generator that yields all of the points that make up the faces of a cube
    that is k away from a center point at point.
    Yields the points in a slightly eclectic order, but gets all of them.
    @param point the point at which the cube is centered
    @param k the length of an edge of the cube, from midpoint to its end. So, the
        cube has size (2k)^3
    """
    for i in np.arange(-k, k):
        for j in np.arange(-k, k):
            # If we assume the cube is centered at zero, this inner statement
            # gives us the point (i, j) on each of the six squares that are the
            # sides of the cubes
            yield [point[0] - k, point[1] + i, point[2] + j]
            yield [point[0] + k, point[1] + i, point[2] + j]

            yield [point[0] + i, point[1] - k, point[2] + j]
            yield [point[0] + i, point[1] + k, point[2] + j]

            yield [point[0] + i, point[1] + j, point[2] - k]
            yield [point[0] + i, point[1] + j, point[2] + k]

def GridSearchFeasibleTransform(angles,
                                end_eff,
                                center_point=None,
                                extent=0.2,
                                granularity=60,
                                ik_options=0,
                                end_eff_transform=None):
    """
    Searches for a feasible transform which puts the end effector at the specified
    rotation angles, without moving too far from the current position.
    Feasible means that the end effector's IK Solver returns a non-null solution
    for that transform.
    @param angles the angles at which the end effector has to be rotated
    @param end_eff the end effector we are trying to find a feasible transform.
    @parm extent how far out of the current position shouold we search. We search
        a grid in a cube around the current position.
    @granularity how many points, in one dimension the grid (overall, there will
        be granularity^3 points in the grid search).
    @ik_options the options to pass into end_eff.FindIKSolution()
    """
    assert len(angles) == 3,\
    "angles must be a length-3 array of x, y, z rotation in radians"
    assert end_eff.GetIkSolver() is not None,\
        "Must set an IkSolver for the end effector"
    transform = end_eff.GetTransform()
    if center_point is None:
        center_point = transform[0:3,3]
    candidate = SetRotation(transform, angles)
    candidate = SetTranslation(candidate, center_point)
    ik_soln = end_eff.FindIKSolution(candidate, ik_options)
    if ik_soln is not None:
        return candidate, ik_soln
    # How large one step in the grid will be
    interval = extent*1.0 / granularity
    for i in range(1, granularity):
        for point in cube_k_away(center_point, i*interval):
            candidate = SetTranslation(candidate, point)
            ik_soln = end_eff.FindIKSolution(candidate, ik_options)
            if ik_soln is not None:
                return candidate, ik_soln
    return None, None
