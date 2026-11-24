"""Сетка нагрузок и маркировка двух решателей в отчёте о захвате."""
import csv
import io
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from unittest.mock import patch

from studies import grip_accuracy


class GripAccuracyTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.path = Path(directory.name) / "grip.csv"

    def samples(self, engine, error=0.1):
        # Оракул входов опыта независим от GRID в анализаторе CSV.
        loads = [(0.5, 10, f, 0) for f in (0, 8, 9.8, 9.9, 10, 10.1, 10.2, 12, -12)]
        loads += [(0, 10, 10, 0), (0.5, 0, 10, 0), (0.5, 10, 10, 1)]
        return [dict(engine=engine, hz=hz, mu=mu, jaw_N=jaw, load_N=load, open=opened,
                     complete=1, max_path_error_mm=error, max_left_force_error_N=0,
                     max_right_force_error_N=0)
                for hz in (60, 120, 240) for mu, jaw, load, opened in loads]

    def write(self, rows):
        with self.path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)

    def shared_report(self, candidates):
        rows = list(candidates)
        for peer in ("box3d-shared8", "rp3d-f32-shared8", "rp3d-f64-shared8"):
            rows.extend(self.samples(peer, 0.2))
        self.write(rows)
        output = io.StringIO()
        with patch.object(sys, "argv", ["grip_accuracy.py", str(self.path), "--shared-clock"]), redirect_stdout(output):
            grip_accuracy.main()
        return list(csv.DictReader(io.StringIO(output.getvalue())))

    def test_replaced_case_in_one_engine(self):
        rows = self.samples("kinetica-shared8-coupled")
        next(row for row in rows if row["hz"] == 60 and row["mu"] == 0)["hz"] = 90
        self.write(rows + self.samples("kinetica-shared8-sequential"))
        with self.assertRaisesRegex(ValueError, "incomplete grip grid"):
            grip_accuracy.read_cases(self.path)

    def test_duplicate_in_one_mode(self):
        rows = self.samples("kinetica-shared8-coupled")
        self.write(rows + [rows[0]])
        with self.assertRaisesRegex(ValueError, "Duplicate observation"):
            grip_accuracy.read_cases(self.path)

    def test_both_modes_keep_their_own_errors(self):
        sequential = "kinetica-shared8-sequential"
        coupled = "kinetica-shared8-coupled"
        results = self.shared_report(self.samples(sequential, 0.3) + self.samples(coupled, 0.1))
        self.assertEqual(len(results), 72)
        self.assertEqual({(r["engine"], r["outcome"]) for r in results},
                         {(sequential, "loss"), (coupled, "win")})

    def test_legacy_label_is_preserved(self):
        results = self.shared_report(self.samples("kinetica-shared8"))
        self.assertEqual(len(results), 36)
        self.assertEqual({r["engine"] for r in results}, {"kinetica-shared8"})


if __name__ == "__main__":
    unittest.main()
