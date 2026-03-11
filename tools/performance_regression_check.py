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


def resolve_threshold_percent(config: dict, benchmark_name: str, fallback_percent: float) -> float:
    """Resolve threshold by benchmark -> tier -> default, falling back to CLI value."""
    if not config:
        return float(fallback_percent)

    benchmarks = config.get("benchmarks", {})
    tiers = config.get("tiers", {})

    benchmark_cfg = benchmarks.get(benchmark_name, {})
    if "threshold_percent" in benchmark_cfg:
        return float(benchmark_cfg["threshold_percent"])

    tier_name = benchmark_cfg.get("tier")
    if tier_name:
        tier_cfg = tiers.get(tier_name, {})
        if "threshold_percent" in tier_cfg:
            return float(tier_cfg["threshold_percent"])

    if "default_threshold_percent" in config:
        return float(config["default_threshold_percent"])

    return float(fallback_percent)


def resolve_memory_threshold_kb(config: dict, benchmark_name: str) -> float | None:
    if not config:
        return None

    benchmarks = config.get("benchmarks", {})
    tiers = config.get("tiers", {})

    benchmark_cfg = benchmarks.get(benchmark_name, {})
    if "memory_threshold_kb" in benchmark_cfg:
        return float(benchmark_cfg["memory_threshold_kb"])

    tier_name = benchmark_cfg.get("tier")
    if tier_name:
        tier_cfg = tiers.get(tier_name, {})
        if "memory_threshold_kb" in tier_cfg:
            return float(tier_cfg["memory_threshold_kb"])

    return None


def resolve_cv_threshold_percent(config: dict, benchmark_name: str) -> float | None:
    if not config:
        return None

    benchmarks = config.get("benchmarks", {})
    tiers = config.get("tiers", {})

    benchmark_cfg = benchmarks.get(benchmark_name, {})
    if "cv_threshold_percent" in benchmark_cfg:
        return float(benchmark_cfg["cv_threshold_percent"])

    tier_name = benchmark_cfg.get("tier")
    if tier_name:
        tier_cfg = tiers.get(tier_name, {})
        if "cv_threshold_percent" in tier_cfg:
            return float(tier_cfg["cv_threshold_percent"])

    return None


def compute_cv_percent(data: dict) -> float | None:
    mean_ms = float(data.get("mean_ms", 0.0) or 0.0)
    stddev_ms = float(data.get("stddev_ms", 0.0) or 0.0)
    if mean_ms <= 0.0 or stddev_ms < 0.0:
        return None
    return (stddev_ms / mean_ms) * 100.0


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


def matching_golden_path(current_path: Path) -> Path:
    base_name = current_path.name[: -len(".current.json")]
    return current_path.with_name(base_name + ".json")


def main() -> int:
    parser = argparse.ArgumentParser(description="Check performance regression against golden baselines")
    parser.add_argument("--threshold", type=float, default=0.05,
                        help="Allowed regression threshold (default: 0.05 = 5%%)")
    parser.add_argument("--golden-dir", type=str, default=None,
                        help="Path to golden performance directory")
    parser.add_argument("--threshold-config", type=str, default=None,
                        help="Optional path to threshold config JSON")
    parser.add_argument("--strict-orphans", action="store_true",
                        help="Fail if .current.json files exist without matching golden .json files")
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
    current_files = sorted(golden_dir.glob("*.current.json"))

    orphan_current_files = []
    for current_path in current_files:
        golden_path = matching_golden_path(current_path)
        if not golden_path.exists():
            orphan_current_files.append((current_path, golden_path))

    failures = 0
    for current_path, golden_path in orphan_current_files:
        level = "ERROR" if args.strict_orphans else "WARNING"
        print(f"{level}: ORPHAN_CURRENT: {current_path.name} has no matching baseline {golden_path.name}")
        if args.strict_orphans:
            failures += 1

    if not golden_files and not current_files:
        print(f"No golden files found in {golden_dir}")
        return 2

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
        memory_threshold_kb = resolve_memory_threshold_kb(threshold_config, benchmark_name)
        cv_threshold_percent = resolve_cv_threshold_percent(threshold_config, benchmark_name)

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

        if memory_threshold_kb is not None and "peak_rss_kb" in current:
            current_peak_rss_kb = float(current.get("peak_rss_kb", 0.0) or 0.0)
            if current_peak_rss_kb > memory_threshold_kb:
                print(
                    f"MEMORY_REGRESSION: {golden_path.name} peak_rss_kb={current_peak_rss_kb:.0f} "
                    f"threshold={memory_threshold_kb:.0f}"
                )
                failures += 1
            else:
                print(
                    f"OK_MEMORY: {golden_path.name} peak_rss_kb={current_peak_rss_kb:.0f} "
                    f"threshold={memory_threshold_kb:.0f}"
                )

        if cv_threshold_percent is not None:
            current_cv = compute_cv_percent(current)
            if current_cv is not None:
                if current_cv > cv_threshold_percent:
                    print(
                        f"CV_REGRESSION: {golden_path.name} cv_percent={current_cv:.2f} "
                        f"threshold={cv_threshold_percent:.2f}"
                    )
                    failures += 1
                else:
                    print(
                        f"OK_CV: {golden_path.name} cv_percent={current_cv:.2f} "
                        f"threshold={cv_threshold_percent:.2f}"
                    )

    if failures:
        print(f"Performance regression check failed: {failures} issue(s)")
        return 1

    print("Performance regression check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
