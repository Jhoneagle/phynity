#!/usr/bin/env python3
import argparse
import json
from pathlib import Path


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def load_threshold_config(path: Path | None) -> dict:
    if path is None or not path.exists():
        return {}
    return load_json(path)


def extract_metric_ms(data: dict) -> tuple[float, str]:
    """Extract the appropriate metric from benchmark data.
    
    Returns (value_ms, metric_type) where metric_type is 'median', 'mean', or 'total'.
    Prefers median (robust) over mean over total (single sample).
    """
    if "median_ms" in data and data["median_ms"] > 0.0:
        return float(data["median_ms"]), "median"
    elif "mean_ms" in data and data["mean_ms"] > 0.0:
        return float(data["mean_ms"]), "mean"
    else:
        return float(data.get("milliseconds", 0.0)), "total"


def main() -> int:
    parser = argparse.ArgumentParser(description="Check performance regression against golden baselines")
    parser.add_argument("--threshold", type=float, default=0.05,
                        help="Allowed regression threshold (default: 0.05 = 5%%)")
    parser.add_argument("--golden-dir", type=str, default=None,
                        help="Path to golden performance directory")
    parser.add_argument("--threshold-config", type=str, default=None,
                        help="Optional path to threshold config JSON")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    golden_dir = Path(args.golden_dir) if args.golden_dir else root / "tests" / "golden_outputs" / "performance"
    threshold_config_path = Path(args.threshold_config) if args.threshold_config else golden_dir / "thresholds.json"
    threshold_config = load_threshold_config(threshold_config_path)

    if not golden_dir.exists():
        print(f"Golden directory not found: {golden_dir}")
        return 2

    golden_files = sorted(
        p for p in golden_dir.glob("*.json")
        if not p.name.endswith(".current.json") and p.name != "thresholds.json"
    )
    if not golden_files:
        print(f"No golden files found in {golden_dir}")
        return 2

    failures = 0
    for golden_path in golden_files:
        current_path = golden_path.with_suffix(".current.json")
        if not current_path.exists():
            print(f"Missing current file: {current_path}")
            failures += 1
            continue

        golden = load_json(golden_path)
        current = load_json(current_path)
        benchmark_name = str(current.get("scenario") or golden.get("scenario") or golden_path.stem)

        golden_ms, golden_metric = extract_metric_ms(golden)
        current_ms, current_metric = extract_metric_ms(current)
        threshold_percent = resolve_threshold_percent(threshold_config, benchmark_name, args.threshold * 100.0)

        if golden_ms <= 0.0:
            print(f"Invalid golden milliseconds in {golden_path}")
            failures += 1
            continue

        ratio = current_ms / golden_ms
        allowed = 1.0 + threshold_percent / 100.0

        if ratio > allowed:
            print(f"REGRESSION: {golden_path.name} current={current_ms:.3f}ms({current_metric}) baseline={golden_ms:.3f}ms({golden_metric}) (x{ratio:.2f}, threshold={threshold_percent:.2f}%)")
            failures += 1
        else:
            print(f"OK: {golden_path.name} current={current_ms:.3f}ms({current_metric}) baseline={golden_ms:.3f}ms({golden_metric}) (x{ratio:.2f}, threshold={threshold_percent:.2f}%)")

    if failures:
        print(f"Performance regression check failed: {failures} issue(s)")
        return 1

    print("Performance regression check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
