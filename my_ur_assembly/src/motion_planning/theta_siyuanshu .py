#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from scipy.spatial.transform import Rotation as R

R_qr_w = R.from_quat(rot).as_matrix()
R_qr_g = np.array([[0, 0, 1], [0, 1, 0],[-1, 0, 0]])
R_g_w = np.dot(R_qr_w, R_qr_g)
Q = R.from_matrix(R_g_w).as_quat()
print(Q)