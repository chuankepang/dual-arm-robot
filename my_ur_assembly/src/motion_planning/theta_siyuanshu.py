import numpy as np
from scipy.spatial.transform import Rotation as R

R_qr_w = np.array([[np.cos(theta), -np.sin(theta), 0],[np.sin(theta), np.cos(theta), [0, 0, 1]]])
R_qr_g = np.array([[0, 0, 1], [0, 1, 0],[-1, 0, 0]])
R_g_w = np.dot(np.transpose(R_qr_g),R_qr_w)
rotation = R.from_matrix(R_g_w)
Q = rotation.as_quat()
print(Q)