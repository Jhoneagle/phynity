#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def main() -> int:
    parser = argparse.ArgumentParser(description="Check performance regression against golden baselines")
    parser.add_argument("--threshold", type=float, default=0.05,
                        help="Allowed regression threshold (default: 0.05 = 5%%)")
    parser.add_argument("--golden-dir", type=str, default=None,
                        help="Path to golden performance directory")
    args = parser.parse_args()

    root = Path(__file__).resolve().parents[1]
    golden_dir = Path(args.golden_dir) if args.golden_dir else root / "tests" / "golden_outputs" / "performance"

    if not golden_dir.exists():
        print(f"Golden directory not found: {golden_dir}")
        return 2

    golden_files = sorted(p for p in golden_dir.glob("*.json") if not p.name.endswith(".current.json"))
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

        golden_ms = float(golden.get("milliseconds", 0.0))
        current_ms = float(current.get("milliseconds", 0.0))

        if golden_ms <= 0.0:
            print(f"Invalid golden milliseconds in {golden_path}")
            failures += 1
            continue

        ratio = current_ms / golden_ms
        allowed = 1.0 + args.threshold

        if ratio > allowed:
            print(f"REGRESSION: {golden_path.name} current={current_ms:.3f}ms baseline={golden_ms:.3f}ms (x{ratio:.2f})")
            failures += 1
        else:
            print(f"OK: {golden_path.name} current={current_ms:.3f}ms baseline={golden_ms:.3f}ms (x{ratio:.2f})")

    if failures:
        print(f"Performance regression check failed: {failures} issue(s)")
        return 1

    print("Performance regression check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
