************************************
Transforms: Position and Orientation
************************************

The position of objects can be expressed in Cartesian world space coordinates
(x, y, z). The orientation (or rotation) of objects can be expressed using
quaternions (x, y, z, w), euler angles (yaw, pitch, roll) or 3x3 matrices.
PyBullet provides a few helper functions to convert between quaternions,
euler angles and 3x3 matrices. In additions there are some functions to
multiply and invert transforms.
