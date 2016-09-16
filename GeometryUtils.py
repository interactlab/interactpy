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
    print rotation.shape
    rotated = transform.copy()
    rotated[0:3, 0:3] = rotation[0:3, 0:3]
    return rotated

# Takes the transform, and returns the same transform but where the axis
# rotation_axis has rotation 0
def SetRotationAxisToZero(transform, rotation_axis):
    assert rotation_axis >= 0 and rotation_axis < 3,\
        "rotation axis must be in between 0 and 2, is %d" % rotation_axis
    axis_angle = orp.AxisAngleFromMatrix(transform)
    axis_angle[rotation_axis] = 0
    return SetRotation(transform, axis_angle)

# Return a transform of a position close to the edge of the box around kinbody.
# Position is a length-3 vector containing elements from {0, 1, -1}, which says
# which side of the object we want to be on (for example, [-1, 0, 0] means on the
# left, and [0, 1, 0] means on top). The distance parameter is how far away from 
# the edge of the box we want to be.
def GetPositionCloseToKinbody(kinbody, position, distance=0.08):
    kinbody_aabb = kinbody.ComputeAABB()
    kinbody_position = kinbody_aabb.pos()
    kinbody_extents = kinbody_aabb.extents()
    return \
    [kinbody_position[0] + (kinbody_extents[0] + distance)*position[0],
    kinbody_position[1] + (kinbody_extents[1] + distance)*position[1],
    kinbody_position[2] + (kinbody_extents[2] + distance)*position[2]]
