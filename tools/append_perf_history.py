#!/usr/bin/env python3
import argparse
import csv
import json
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable

HEADER = [
    "schema_version",
    "recorded_at_utc",
    "git_sha",
    "git_ref",
    "workflow_run_id",
    "os",
    "arch",
    "compiler",
    "build_type",
    "benchmark_suite",
    "benchmark_name",
    "frames",
    "dt_seconds",
    "milliseconds_total",
    "milliseconds_per_frame",
    "golden_milliseconds_per_frame",
    "regression_percent",
    "threshold_percent",
    "pass_fail",
    "machine_tag",
]


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def load_threshold_config(path: Path | None) -> dict:
    if path is None or not path.exists():
        return {}
    return load_json(path)


def resolve_threshold_percent(config: dict, benchmark_name: str, default_percent: float) -> float:
    bench_config = config.get("benchmarks", {}).get(benchmark_name, {})
    if "threshold_percent" in bench_config:
        return float(bench_config["threshold_percent"])
    if "default_threshold_percent" in config:
        return float(config["default_threshold_percent"])
    return default_percent


def find_benchmark_pairs(perf_dir: Path) -> Iterable[tuple[Path, Path]]:
    for golden_path in sorted(perf_dir.glob("*.json")):
        if golden_path.name.endswith(".current.json"):
            continue
        current_path = golden_path.with_suffix(".current.json")
        if current_path.exists():
            yield golden_path, current_path


def ensure_header(csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    if not csv_path.exists() or csv_path.stat().st_size == 0:
        with csv_path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.writer(handle)
            writer.writerow(HEADER)


def safe_div(numerator: float, denominator: float) -> float:
    if denominator == 0.0:
        return 0.0
    return numerator / denominator


def main() -> int:
    parser = argparse.ArgumentParser(description="Append performance benchmark rows to CSV history")
    parser.add_argument("--history-csv", required=True, help="Path to perf history CSV")
    parser.add_argument("--perf-dir", required=True, help="Directory containing *.json and *.current.json files")
    parser.add_argument("--schema-version", type=int, default=1)
    parser.add_argument("--git-sha", required=True)
    parser.add_argument("--git-ref", required=True)
    parser.add_argument("--workflow-run-id", required=True)
    parser.add_argument("--os", required=True)
    parser.add_argument("--arch", required=True)
    parser.add_argument("--compiler", required=True)
    parser.add_argument("--build-type", required=True)
    parser.add_argument("--benchmark-suite", default="performance")
    parser.add_argument("--dt-seconds", type=float, default=0.0166667)
    parser.add_argument("--threshold-percent", type=float, default=5.0)
    parser.add_argument("--threshold-config", default="")
    parser.add_argument("--machine-tag", required=True)
    args = parser.parse_args()

    history_csv = Path(args.history_csv)
    perf_dir = Path(args.perf_dir)
    threshold_config_path = Path(args.threshold_config) if args.threshold_config else perf_dir / "thresholds.json"
    threshold_config = load_threshold_config(threshold_config_path)

    if not perf_dir.exists():
        print(f"Performance directory not found: {perf_dir}")
        return 2

    pairs = list(find_benchmark_pairs(perf_dir))
    if not pairs:
        print(f"No benchmark pairs found in {perf_dir}")
        return 2

    ensure_header(history_csv)

    now = datetime.now(tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    appended = 0

    with history_csv.open("a", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        for golden_path, current_path in pairs:
            golden = load_json(golden_path)
            current = load_json(current_path)

            benchmark_name = str(current.get("scenario") or golden.get("scenario") or golden_path.stem)
            threshold_percent = resolve_threshold_percent(threshold_config, benchmark_name, args.threshold_percent)
            frames = int(current.get("iterations", 0) or 0)
            milliseconds_total = float(current.get("milliseconds", 0.0) or 0.0)
            golden_total = float(golden.get("milliseconds", 0.0) or 0.0)

            ms_per_frame = safe_div(milliseconds_total, float(frames)) if frames > 0 else milliseconds_total
            golden_ms_per_frame = safe_div(golden_total, float(frames)) if frames > 0 else golden_total

            if golden_ms_per_frame > 0.0:
                regression_percent = (ms_per_frame - golden_ms_per_frame) / golden_ms_per_frame * 100.0
            else:
                regression_percent = 0.0

            pass_fail = "PASS" if regression_percent <= threshold_percent else "FAIL"

            writer.writerow([
                args.schema_version,
                now,
                args.git_sha,
                args.git_ref,
                args.workflow_run_id,
                args.os,
                args.arch,
                args.compiler,
                args.build_type,
                args.benchmark_suite,
                benchmark_name,
                frames,
                f"{args.dt_seconds:.7f}",
                f"{milliseconds_total:.6f}",
                f"{ms_per_frame:.6f}",
                f"{golden_ms_per_frame:.6f}",
                f"{regression_percent:.2f}",
                f"{threshold_percent:.2f}",
                pass_fail,
                args.machine_tag,
            ])
            appended += 1

    print(f"Appended {appended} row(s) to {history_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
