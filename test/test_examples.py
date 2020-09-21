#!/usr/bin/env python
PKG = 'trackjoint'
import subprocess
import unittest
import roslib

roslib.load_manifest(PKG)  # This line is not needed with Catkin.

class TestExamples(unittest.TestCase):

    def run_cmd(self, *cmd):
        rosrun_cmd = ["rosrun", "trackjoint"] + list(cmd)
        print("Command: %s" % ' '.join(rosrun_cmd))
        try:
            p = subprocess.Popen(rosrun_cmd)
            outs, errs = p.communicate()
        except Exception as e:
            self.fail("Command failed: %s" % str(e))
        self.assertEqual(p.returncode, 0)

    def test_simple_example(self):
        self.run_cmd("simple_example")

    def test_streaming_example(self):
        self.run_cmd("streaming_example")

    def test_three_dof_examples(self):
        self.run_cmd("three_dof_examples")

if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_examples', TestExamples)
