"""Классификация пролёта по показаниям основного и мелкого шагов CaseTable.cpp."""
import unittest

from case_table import MEASURES


class PassThroughTest(unittest.TestCase):
    def test_fine_step_outcomes(self):
        is_pass_through = dict(MEASURES)["пролёт"]
        for passed, missed, expected in [(0, 1, False), (1, 0, True), (1, 1, True)]:
            with self.subTest(passed=passed, missed=missed):
                self.assertEqual(is_pass_through({"passed": passed, "missed": missed}), expected)


if __name__ == "__main__":
    unittest.main()
