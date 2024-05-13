#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import unittest

import numpy as np
np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

from bs_imu import *








class TestBsImu(unittest.TestCase):

    def test_rpy_to_matrix(self):
        rpy_input = np.deg2rad([90,0,90])
        mat = rpy_to_matrix(rpy_input)
        rpy_output = np.rad2deg(matrix_to_rpy(mat))
        print(rpy_output)
        print(mat)




        



if __name__ == '__main__':
    unittest.main()
