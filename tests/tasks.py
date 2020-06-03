from __future__ import print_function

import unittest
from agimus_sot.task import OpFrame

class TestAgimusTasks(unittest.TestCase):

    def test_opframe(self):
        gripper = self.gripper
        self.assertTrue(gripper.controllable)
        self.assertEqual(gripper.fullName, "tiago/gripper")
        self.assertEqual(gripper.fullLink, "tiago/wrist_ft_tool_link")

        handle = self.handle
        self.assertFalse(handle.controllable)
        self.assertEqual(handle.fullName, "object/handle")
        self.assertEqual(handle.fullLink, "object/base_link")

    def test_task1(self):
        from agimus_sot.task import PreGrasp
        task = PreGrasp(self.gripper, self.handle)
        task.makeTasks(self.robot, True, True)
        self.assertEqual(task.task.name, PreGrasp.sep.join([ PreGrasp.name_prefix,
            self.gripper.name, self.handle.fullName ]) + "_task")

    def test_task2(self):
        from agimus_sot.task import PreGrasp
        task = PreGrasp(self.gripper, self.handle,
                (self.gripper2, self.handle2))
        task.makeTasks(self.robot, True, True, True)
        self.assertEqual(task.task.name, PreGrasp.sep.join([ PreGrasp.name_prefix,
            self.gripper.name, self.handle.fullName,
            "relative", self.gripper2.name, self.handle2.fullName,
            ]) + "_task")

    def test_task3(self):
        from agimus_sot.task import PreGrasp
        task = PreGrasp(self.toolGripper, self.handle,
                (self.gripper, self.handle2))
        task.makeTasks(self.robot, True, True, True)
        self.assertEqual(task.task.name, PreGrasp.sep.join([ PreGrasp.name_prefix,
            self.toolGripper.fullName, self.handle.fullName,
            "based", self.gripper.name, self.handle2.fullName,
            ]) + "_task")

    def test_task4(self):
        from agimus_sot.task import PreGrasp
        task = PreGrasp(self.toolGripper, self.handle,
                (self.gripper, self.toolHandle))
        task.makeTasks(self.robot, True, True, True)
        self.assertEqual(task.task.name, PreGrasp.sep.join([ PreGrasp.name_prefix,
            self.handle.fullName, self.toolGripper.fullName,
            "based", self.gripper.name, self.toolHandle.fullName,
            ]) + "_task")

    @classmethod
    def setUpClass(cls):
        from dynamic_graph.sot.tiago.steel.robot import TiagoSteel
        cls.robot = TiagoSteel("tiago", with_wheels=True)
        if not hasattr(cls.robot, "camera_frame"):
            cls.robot.camera_frame = "xtion_optical_frame"

        g_srdf = {
                "robot": cls.robot.name,
                "name": "gripper",
                'link': 'wrist_ft_tool_link',

                'clearance': 0.09,
                'joints': ('hand_thumb_joint', 'hand_index_joint', 'hand_mrl_joint'),
                'position': (0.13, 0.038, 0.0, 0.0, 0.0, 0.0, 1.0),
                }
        g2_srdf = {
                "robot": cls.robot.name,
                "name": "gripper2",
                'link': 'arm_7_link',

                'clearance': 0.09,
                'joints': (),
                'position': (0.13, 0.038, 0.0, 0.0, 0.0, 0.0, 1.0),
                }

        oh_srdf = {
                "robot": "object",
                "name": "handle",
                'link': 'base_link',
                'clearance': 0.01,
                'position': (0.13, 0.038, 0.0, 0.0, 0.0, 0.0, 1.0),
                }
        oh2_srdf = {
                "robot": "object",
                "name": "handle2",
                'link': 'base_link',
                'clearance': 0.01,
                'position': (0.13, 0.038, 0.0, 0.0, 0.0, 0.0, 1.0),
                }
        th_srdf = {
                "robot": "tool",
                "name": "handle",
                'link': 'base_link',
                'clearance': 0.01,
                'position': (0.13, 0.038, 0.0, 0.0, 0.0, 0.0, 1.0),
                }
        tg_srdf = {
                "robot": "tool",
                "name": "gripper",
                'link': 'base_link',
                'joints': (),
                'clearance': 0.01,
                'position': (0.13, 0.038, 0.0, 0.0, 0.0, 0.0, 1.0),
                }

        cls.gripper = OpFrame(g_srdf,
                modelName = cls.robot.name,
                model = cls.robot.pinocchioModel,
                enabled = True)
        cls.gripper2 = OpFrame(g2_srdf,
                modelName = cls.robot.name,
                model = cls.robot.pinocchioModel,
                enabled = True)

        cls.handle = OpFrame(oh_srdf,
                modelName = cls.robot.name,
                model = cls.robot.pinocchioModel,
                enabled = True)
        cls.handle2 = OpFrame(oh2_srdf,
                modelName = cls.robot.name,
                model = cls.robot.pinocchioModel,
                enabled = True)
        cls.toolGripper = OpFrame(tg_srdf,
                modelName = cls.robot.name,
                model = cls.robot.pinocchioModel,
                enabled = True)
        cls.toolHandle = OpFrame(th_srdf,
                modelName = cls.robot.name,
                model = cls.robot.pinocchioModel,
                enabled = True)

if __name__ == '__main__':
    unittest.main()
