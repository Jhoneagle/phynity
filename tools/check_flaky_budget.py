#!/usr/bin/env python3
import argparse
import csv
from collections import Counter
from datetime import datetime, timedelta, timezone
from pathlib import Path


def parse_timestamp(value: str) -> datetime:
    return datetime.strptime(value, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)


def load_recent_counts(csv_path: Path, days: int) -> Counter[str]:
    cutoff = datetime.now(tz=timezone.utc) - timedelta(days=days)
    counts: Counter[str] = Counter()
    if not csv_path.exists():
        return counts

    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                recorded = parse_timestamp(row["recorded_at_utc"])
            except ValueError:
                continue
            if recorded < cutoff:
                continue
            counts[row["test_name"]] += 1
    return counts


def render_markdown(counts: Counter[str], threshold: int, days: int) -> str:
    lines = [
        "# Flaky Test Budget Summary",
        "",
        f"Window: last {days} day(s)",
        f"Threshold: {threshold} incident(s)",
        "",
    ]
    if not counts:
        lines.append("No flaky incidents recorded in the selected window.")
        return "\n".join(lines) + "\n"

    lines.append("| Test | Incidents | Budget |")
    lines.append("| --- | ---: | --- |")
    for test_name, count in counts.most_common():
        budget = "EXCEEDED" if count >= threshold else "OK"
        lines.append(f"| {test_name} | {count} | {budget} |")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="Check flaky test incident budget over a rolling window")
    parser.add_argument("--history-csv", required=True)
    parser.add_argument("--days", type=int, default=14)
    parser.add_argument("--threshold", type=int, default=3)
    parser.add_argument("--output-markdown", required=True)
    args = parser.parse_args()

    counts = load_recent_counts(Path(args.history_csv), args.days)
    output_path = Path(args.output_markdown)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(render_markdown(counts, args.threshold, args.days), encoding="utf-8")

    exceeded = [test_name for test_name, count in counts.items() if count >= args.threshold]
    if exceeded:
        print("Flaky test budget exceeded:")
        for test_name in sorted(exceeded):
            print(f"  - {test_name}: {counts[test_name]} incidents")
        return 1

    print("Flaky test budget is within limits")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())