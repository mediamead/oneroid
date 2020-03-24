import tensorflow as tf

# https://stackoverflow.com/questions/6802577/rotation-of-3d-vector

def R(axis, theta):
    """
    Return the rotation matrix associated with counterclockwise rotation about
    the given axis by theta radians.
    """
    axis = axis / tf.norm(axis)
    a = tf.math.cos(theta / 2.0)
    abc = -axis * tf.math.sin(theta / 2.0)
    b, c, d = abc[0], abc[1], abc[2] 
    aa, bb, cc, dd = a * a, b * b, c * c, d * d
    bc, ad, ac, ab, bd, cd = b * c, a * d, a * c, a * b, b * d, c * d
    rot_matrix = [[aa + bb - cc - dd, 2 * (bc + ad), 2 * (bd - ac)],
                     [2 * (bc - ad), aa + cc - bb - dd, 2 * (cd + ab)],
                     [2 * (bd + ac), 2 * (cd - ab), aa + dd - bb - cc]]
    return tf.squeeze(rot_matrix, axis=2) # FIXME

if __name__ == "__main__":
    print("# TF session creation ...")
    sess = tf.compat.v1.InteractiveSession()
    print("# TF session creation done")
    try:
        sess.run(tf.compat.v1.global_variables_initializer())

        v = tf.expand_dims(tf.constant([3., 5., 0.]), 1)
        axis = tf.expand_dims(tf.constant([4., 4., 1.]), 1)
        theta = tf.constant(1.2) 

        print("v=%s" % v.eval())
        print("axis=%s" % axis.eval())
        print("theta=%s" % theta.eval())
        rot_matrix = R(axis, theta)
        print("R=%s" % sess.run(rot_matrix))

        #print(np.dot(rotation_matrix(axis, theta), v)) 
        v1 = sess.run(tf.matmul(rot_matrix, v))
        print("v1=%s" % v1)
        # [ 2.74911638  4.77180932  1.91629719]
    except Exception as ex:
        print("# Closing TF session on exception ...")
        sess.close()
        print("# Closing TF session on exception done")
        raise(ex)