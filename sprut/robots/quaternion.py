import tensorflow as tf

def R(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    #axis = np.asarray(axis)
    axis = axis / tf.norm(axis) # math.sqrt(np.dot(axis, axis))
    a = tf.math.cos(theta / 2.0)
    b, c, d = -axis * tf.math.sin(theta / 2.0)
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    return [[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]]

if __name__ == "__main__":
    print("# TF session creation started")
    sess = tf.InteractiveSession()
    print("# TF session creation done")
    try:
        sess.run(tf.compat.v1.global_variables_initializer())

        v = tf.constant([3., 5., 0.])
        axis = tf.constant([4., 4., 1.])
        theta = tf.constant(1.2) 

        print("axis=%s" % axis.eval())
        print("theta=%s" % theta.eval())
        #rm = R(axis, theta)

        #print(np.dot(rotation_matrix(axis, theta), v)) 
        # [ 2.74911638  4.77180932  1.91629719]
    except Exception as ex:
        print("# Closing TF session on exception started")
        sess.close()
        print("# Closing TF session on exception done")
        raise(ex)