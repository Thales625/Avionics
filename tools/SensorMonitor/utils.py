import numpy as np

def quaternion_multiply(q, r):
	x0, y0, z0, w0 = q
	x1, y1, z1, w1 = r
	return np.array([
		w0*x1 + x0*w1 + y0*z1 - z0*y1,
		w0*y1 - x0*z1 + y0*w1 + z0*x1,
		w0*z1 + x0*y1 - y0*x1 + z0*w1,
		w0*w1 - x0*x1 - y0*y1 - z0*z1
	])

def rotate_vector_by_quaternion(v, q):
	v_quat = np.array([v[0], v[1], v[2], 0.0])
	q_conj = np.array([-q[0], -q[1], -q[2], q[3]])
	rotated = quaternion_multiply(quaternion_multiply(q, v_quat), q_conj)
	return rotated[:3]

def integrate_quaternion(q, omega, dt):
	delta_q = np.array([
		0.5 * omega[0] * dt,
		0.5 * omega[1] * dt,
		0.5 * omega[2] * dt,
		1.0
	])

	q_new = quaternion_multiply(q, delta_q)
	
	return q_new / np.linalg.norm(q_new)

def quaternion_to_rotation_matrix(q):
	x, y, z, w = q
	R = np.array([
		[1-2*(y**2+z**2), 2*(x*y-z*w),   2*(x*z+y*w)],
		[2*(x*y+z*w),     1-2*(x**2+z**2), 2*(y*z-x*w)],
		[2*(x*z-y*w),     2*(y*z+x*w),   1-2*(x**2+y**2)]
	])
	return R

def quaternion_from_vector_alignment(v_from, v_to):
	v_from = v_from / np.linalg.norm(v_from)
	v_to = v_to / np.linalg.norm(v_to)
	cross = np.cross(v_from, v_to)
	dot = np.dot(v_from, v_to)
	if dot < -0.999999:
		ortho = np.array([1, 0, 0]) if abs(v_from[0]) < 0.9 else np.array([0, 1, 0])
		axis = np.cross(v_from, ortho)
		axis /= np.linalg.norm(axis)
		return np.array([axis[0], axis[1], axis[2], 0.0])
	s = np.sqrt((1 + dot) * 2)
	q = np.array([
		cross[0] / s,
		cross[1] / s,
		cross[2] / s,
		0.5 * s
	])
	return q / np.linalg.norm(q)

def slerp(q1, q2, t):
    cos_theta = np.dot(q1, q2)
    if cos_theta < 0.0:
        q2 = -q2
        cos_theta = -cos_theta

    if cos_theta > 0.9995:
        result = q1 + t * (q2 - q1)
        return result / np.linalg.norm(result)

    theta_0 = np.arccos(cos_theta)
    sin_theta_0 = np.sin(theta_0)
    theta = theta_0 * t
    sin_theta = np.sin(theta)

    s0 = np.cos(theta) - cos_theta * sin_theta / sin_theta_0
    s1 = sin_theta / sin_theta_0

    return (s0 * q1 + s1 * q2)
