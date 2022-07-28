"""
This file tests the output of executable sot-test-log compiled from
tests/sot-test-log.cc.
"""
import unittest
import subprocess
import csv


class TestLog(unittest.TestCase):
    """
    Logs are saved in binary format in file "build/tests/test.log-vstate.log",
    then converted in ascii in file "test.log-vstate-ascii.log" and then
    compared to the expected values.
    """

    def test_log(self):
        subprocess.run("./sot-test-log")
        res = subprocess.run(
            ["../roscontrol-sot-parse-log", "test.log-vstate.log"],
            capture_output=True,
            text=True,
        )
        with open("./test.log-vstate-ascii.log", "w") as f:
            f.write(res.stdout)
        with open("./test.log-vstate-ascii.log", "r") as f:
            r = csv.reader(f, delimiter=" ")
            for i, line in enumerate(r):
                self.assertEqual(len(line), 9)
                data = list(map(float, line[:8]))
                # latest data between 200 and 250
                if i < 50:
                    self.assertEqual(data[2], i + 200)
                    self.assertEqual(data[3], i + 201)
                    self.assertEqual(data[4], i + 202)
                    self.assertEqual(data[5], i + 203)
                    self.assertEqual(data[6], i + 204)
                    self.assertEqual(data[7], i + 205)
                # data between 150 and 200
                else:
                    self.assertEqual(data[2], i + 100)
                    self.assertEqual(data[3], i + 101)
                    self.assertEqual(data[4], i + 102)
                    self.assertEqual(data[5], i + 103)
                    self.assertEqual(data[6], i + 104)
                    self.assertEqual(data[7], i + 105)


if __name__ == "__main__":
    unittest.main()
