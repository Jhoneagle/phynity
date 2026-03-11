import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SCRIPT = REPO_ROOT / "tools" / "performance_regression_check.py"


class PerformanceRegressionCheckOrphanTests(unittest.TestCase):
    def _write_json(self, path: Path, payload: dict) -> None:
        path.write_text(json.dumps(payload), encoding="utf-8")

    def _run_checker(self, golden_dir: Path, strict_orphans: bool) -> subprocess.CompletedProcess:
        cmd = [
            sys.executable,
            str(SCRIPT),
            "--golden-dir",
            str(golden_dir),
            "--threshold",
            "0.05",
        ]
        if strict_orphans:
            cmd.append("--strict-orphans")
        return subprocess.run(cmd, capture_output=True, text=True, check=False)

    def test_orphan_current_warns_in_non_strict_mode(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            golden_dir = Path(tmp_dir)

            benchmark = {"scenario": "solver", "milliseconds": 10.0}
            self._write_json(golden_dir / "solver.json", benchmark)
            self._write_json(golden_dir / "solver.current.json", benchmark)

            orphan = {"scenario": "orphan_bench", "milliseconds": 9.0}
            self._write_json(golden_dir / "orphan_bench.current.json", orphan)

            result = self._run_checker(golden_dir, strict_orphans=False)

            self.assertEqual(result.returncode, 0, msg=result.stdout + result.stderr)
            self.assertIn("WARNING: ORPHAN_CURRENT:", result.stdout)
            self.assertIn("Performance regression check passed", result.stdout)

    def test_orphan_current_fails_in_strict_mode(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            golden_dir = Path(tmp_dir)

            benchmark = {"scenario": "solver", "milliseconds": 10.0}
            self._write_json(golden_dir / "solver.json", benchmark)
            self._write_json(golden_dir / "solver.current.json", benchmark)

            orphan = {"scenario": "orphan_bench", "milliseconds": 9.0}
            self._write_json(golden_dir / "orphan_bench.current.json", orphan)

            result = self._run_checker(golden_dir, strict_orphans=True)

            self.assertEqual(result.returncode, 1, msg=result.stdout + result.stderr)
            self.assertIn("ERROR: ORPHAN_CURRENT:", result.stdout)
            self.assertIn("Performance regression check failed", result.stdout)

    def test_memory_regression_triggers_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            golden_dir = Path(tmp_dir)

            golden = {"scenario": "solver", "milliseconds": 10.0, "peak_rss_kb": 1024}
            current = {"scenario": "solver", "milliseconds": 10.0, "peak_rss_kb": 3000}
            thresholds = {
                "default_threshold_percent": 5.0,
                "tiers": {"mid-tier": {"threshold_percent": 5.0, "memory_threshold_kb": 2048}},
                "benchmarks": {"solver": {"tier": "mid-tier"}},
            }

            self._write_json(golden_dir / "solver.json", golden)
            self._write_json(golden_dir / "solver.current.json", current)
            self._write_json(golden_dir / "thresholds.json", thresholds)

            result = self._run_checker(golden_dir, strict_orphans=False)

            self.assertEqual(result.returncode, 1, msg=result.stdout + result.stderr)
            self.assertIn("MEMORY_REGRESSION:", result.stdout)

    def test_memory_within_threshold_reports_ok_memory(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            golden_dir = Path(tmp_dir)

            golden = {"scenario": "solver", "milliseconds": 10.0, "peak_rss_kb": 1024}
            current = {"scenario": "solver", "milliseconds": 10.0, "peak_rss_kb": 1500}
            thresholds = {
                "default_threshold_percent": 5.0,
                "tiers": {"mid-tier": {"threshold_percent": 5.0, "memory_threshold_kb": 2048}},
                "benchmarks": {"solver": {"tier": "mid-tier"}},
            }

            self._write_json(golden_dir / "solver.json", golden)
            self._write_json(golden_dir / "solver.current.json", current)
            self._write_json(golden_dir / "thresholds.json", thresholds)

            result = self._run_checker(golden_dir, strict_orphans=False)

            self.assertEqual(result.returncode, 0, msg=result.stdout + result.stderr)
            self.assertIn("OK_MEMORY:", result.stdout)

    def test_cv_regression_triggers_failure(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            golden_dir = Path(tmp_dir)

            # CV = 20 / 100 * 100 = 20.0% (should fail against 10%)
            golden = {"scenario": "solver", "milliseconds": 100.0, "mean_ms": 100.0, "stddev_ms": 5.0}
            current = {"scenario": "solver", "milliseconds": 100.0, "mean_ms": 100.0, "stddev_ms": 20.0}
            thresholds = {
                "default_threshold_percent": 5.0,
                "tiers": {"mid-tier": {"threshold_percent": 5.0, "cv_threshold_percent": 10.0}},
                "benchmarks": {"solver": {"tier": "mid-tier"}},
            }

            self._write_json(golden_dir / "solver.json", golden)
            self._write_json(golden_dir / "solver.current.json", current)
            self._write_json(golden_dir / "thresholds.json", thresholds)

            result = self._run_checker(golden_dir, strict_orphans=False)

            self.assertEqual(result.returncode, 1, msg=result.stdout + result.stderr)
            self.assertIn("CV_REGRESSION:", result.stdout)

    def test_cv_within_threshold_reports_ok_cv(self) -> None:
        with tempfile.TemporaryDirectory() as tmp_dir:
            golden_dir = Path(tmp_dir)

            # CV = 5 / 100 * 100 = 5.0% (should pass against 10%)
            golden = {"scenario": "solver", "milliseconds": 100.0, "mean_ms": 100.0, "stddev_ms": 5.0}
            current = {"scenario": "solver", "milliseconds": 100.0, "mean_ms": 100.0, "stddev_ms": 5.0}
            thresholds = {
                "default_threshold_percent": 5.0,
                "tiers": {"mid-tier": {"threshold_percent": 5.0, "cv_threshold_percent": 10.0}},
                "benchmarks": {"solver": {"tier": "mid-tier"}},
            }

            self._write_json(golden_dir / "solver.json", golden)
            self._write_json(golden_dir / "solver.current.json", current)
            self._write_json(golden_dir / "thresholds.json", thresholds)

            result = self._run_checker(golden_dir, strict_orphans=False)

            self.assertEqual(result.returncode, 0, msg=result.stdout + result.stderr)
            self.assertIn("OK_CV:", result.stdout)


if __name__ == "__main__":
    unittest.main()
