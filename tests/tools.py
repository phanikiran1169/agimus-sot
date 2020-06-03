from __future__ import print_function

import unittest
import numpy as np
from agimus_sot.tools import entityExists, assertEntityDoesNotExist, matrixHomoProduct, \
        matrixHomoInverse, entityIfMatrixHomo, plugMatrixHomo, se3ToTuple

class TestAgimusTools(unittest.TestCase):

    def test_assertion(self):
        name = "test_assertion_entity"
        self.assertFalse(entityExists(name))
        mhp = matrixHomoProduct(name, None)
        self.assertRaises(AssertionError, assertEntityDoesNotExist, name)
        self.assertRaises(AssertionError, matrixHomoProduct, name, check=True)
        self.assertRaises(AssertionError, matrixHomoInverse, name, check=True)
        self.assertRaises(AssertionError, entityIfMatrixHomo, name, None, None, None, check=True)

    def test_matrix_homo(self):
        name_a = "test_homo_entity_a"
        name_b = "test_homo_entity_b"
        self.assertFalse(entityExists(name_a))
        self.assertFalse(entityExists(name_b))

        import pinocchio
        Id = pinocchio.SE3.Identity()
        a = matrixHomoProduct(name_a, None, Id)
        b = matrixHomoInverse(name_b, a.sout)
        plugMatrixHomo(b.sout, a.sin0)

    def test_if_entity(self):
        name = "test_if_entity"
        self.assertFalse(entityExists(name))

        import pinocchio
        Id = pinocchio.SE3.Identity()
        M0 = pinocchio.SE3.Random()
        if_ = entityIfMatrixHomo(name,
                condition = None, 
                value_then = Id,
                value_else = M0)

        if_.condition.value = 1
        if_.out.recompute(0)
        self.assertEqual(if_.out.value, se3ToTuple(Id))

        if_.condition.value = 0
        if_.out.recompute(1)
        np.testing.assert_almost_equal(np.array(if_.out.value), M0.homogeneous)

if __name__ == '__main__':
    unittest.main()
