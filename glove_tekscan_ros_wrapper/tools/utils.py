def quaternion_to_R(quat, matrix):
    """Convert a quaternion into rotation matrix form.

    @param quat:    The quaternion.
    @type quat:     numpy 4D, rank-1 array
    @param matrix:  A 3D matrix to convert to a rotation matrix.
    @type matrix:   numpy 3D, rank-2 array
    """

    # Repetitive calculations.
    q4_2 = quat[3]**2
    q12 = quat[0] * quat[1]
    q13 = quat[0] * quat[2]
    q14 = quat[0] * quat[3]
    q23 = quat[1] * quat[2]
    q24 = quat[1] * quat[3]
    q34 = quat[2] * quat[3]

    # The diagonal.
    matrix[0, 0] = 2.0 * (quat[0]**2 + q4_2) - 1.0
    matrix[1, 1] = 2.0 * (quat[1]**2 + q4_2) - 1.0
    matrix[2, 2] = 2.0 * (quat[2]**2 + q4_2) - 1.0

    # Off-diagonal.
    matrix[0, 1] = 2.0 * (q12 - q34)
    matrix[0, 2] = 2.0 * (q13 + q24)
    matrix[1, 2] = 2.0 * (q23 - q14)

    matrix[1, 0] = 2.0 * (q12 + q34)
    matrix[2, 0] = 2.0 * (q13 - q24)
    matrix[2, 1] = 2.0 * (q23 + q14)
