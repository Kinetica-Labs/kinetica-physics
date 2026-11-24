"""Классификация пролёта по показаниям основного и мелкого шагов CaseTable.cpp."""
import json
import sys
import tempfile
import unittest
from itertools import product
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import case_table
from case_table import FEATURES, KEY, MEASURES, REGIMES, SIZES, check_cells, measure


class PassThroughTest(unittest.TestCase):
    def test_fine_step_outcomes(self):
        is_pass_through = dict(MEASURES)["пролёт"]
        for passed, missed, expected in [(0, 1, False), (1, 0, True), (1, 1, True)]:
            with self.subTest(passed=passed, missed=missed):
                self.assertEqual(is_pass_through({"passed": passed, "missed": missed}), expected)


class CompleteTableTest(unittest.TestCase):
    def setUp(self):
        variants = [(name, element) for name, elements in FEATURES.items() for element in elements]
        self.cells = [dict(zip(KEY, (*a, *b, size, regime)), trials=4, bad=0)
                      for a, b, size, regime in product(variants, variants, SIZES, REGIMES)]

    def test_complete_grid_and_physical_failure(self):
        self.assertEqual(len(self.cells), 7425)
        check_cells(self.cells, 0)
        self.cells[0]["bad"] = 1
        check_cells(self.cells, 1)
        with self.assertRaisesRegex(ValueError, "код прибора"):
            check_cells(self.cells, 0)

    def test_one_row_missing_or_repeated(self):
        for rows in ([], self.cells[:-1], self.cells[:-1] + [self.cells[0]]):
            with self.subTest(count=len(rows)), self.assertRaises(ValueError):
                check_cells(rows, 0)

    def test_replaced_condition_and_incomplete_trials(self):
        for replacement in ({"regime": "неизвестный"}, {"trials": 3}):
            with self.subTest(replacement=replacement), self.assertRaises(ValueError):
                check_cells([dict(self.cells[0], **replacement), *self.cells[1:]], 0)

    def test_stopped_process(self):
        with patch("case_table.subprocess.run") as run:
            run.return_value.returncode = 2
            with self.assertRaisesRegex(RuntimeError, "кодом 2"):
                measure()

    def test_truncated_run_cannot_pass_or_update(self):
        def truncated(args, **options):
            Path(args[2]).write_text(json.dumps(self.cells[0]) + "\n", encoding="utf-8")
            return SimpleNamespace(returncode=0)

        with tempfile.TemporaryDirectory() as directory:
            baseline, history = Path(directory) / "baseline.json", Path(directory) / "history.jsonl"
            baseline.write_bytes(b'{"existing":1}\n')
            with patch.multiple(case_table, BASELINE=baseline, HISTORY=history), patch("case_table.subprocess.run", truncated):
                for flags in (["--check"], ["--update", "sample", "--worse"]):
                    with self.subTest(flags=flags), patch.object(sys, "argv", ["case_table.py", *flags]):
                        with self.assertRaisesRegex(ValueError, "пропущено клеток"):
                            case_table.main()
            self.assertEqual(baseline.read_bytes(), b'{"existing":1}\n')
            self.assertFalse(history.exists())


if __name__ == "__main__":
    unittest.main()
