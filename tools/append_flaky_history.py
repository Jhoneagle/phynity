#!/usr/bin/env python3
import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path


HEADER = [
    "schema_version",
    "recorded_at_utc",
    "git_sha",
    "git_ref",
    "workflow_run_id",
    "workflow_name",
    "os",
    "report_name",
    "test_name",
    "outcome",
    "attempts",
]


def ensure_header(csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    if not csv_path.exists() or csv_path.stat().st_size == 0:
        with csv_path.open("w", encoding="utf-8", newline="") as handle:
            csv.writer(handle).writerow(HEADER)


def parse_report(path: Path) -> tuple[int, list[str], list[str]]:
    attempts = 1
    flaky: list[str] = []
    remaining: list[str] = []
    section = ""

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("attempts="):
            attempts = int(line.split("=", 1)[1])
            continue
        if line == "flaky_tests:":
            section = "flaky"
            continue
        if line == "remaining_failures:":
            section = "remaining"
            continue
        if line == "<none>":
            continue
        if section == "flaky":
            flaky.append(line)
        elif section == "remaining":
            remaining.append(line)

    return attempts, flaky, remaining


def main() -> int:
    parser = argparse.ArgumentParser(description="Append flaky test incidents from retry reports to CSV history")
    parser.add_argument("--history-csv", required=True)
    parser.add_argument("--reports-root", required=True)
    parser.add_argument("--git-sha", required=True)
    parser.add_argument("--git-ref", required=True)
    parser.add_argument("--workflow-run-id", required=True)
    parser.add_argument("--workflow-name", required=True)
    parser.add_argument("--schema-version", type=int, default=1)
    args = parser.parse_args()

    history_csv = Path(args.history_csv)
    reports_root = Path(args.reports_root)
    ensure_header(history_csv)

    reports = sorted(reports_root.rglob("*.txt"))
    if not reports:
        print(f"No retry reports found under {reports_root}")
        return 0

    now = datetime.now(tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")
    appended = 0
    with history_csv.open("a", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        for report in reports:
            attempts, flaky, remaining = parse_report(report)
            report_name = report.stem
            os_name = report_name.split("-")[-1] if "-" in report_name else "unknown"

            for test_name in flaky:
                writer.writerow([
                    args.schema_version,
                    now,
                    args.git_sha,
                    args.git_ref,
                    args.workflow_run_id,
                    args.workflow_name,
                    os_name,
                    report_name,
                    test_name,
                    "FLAKY_RECOVERED",
                    attempts,
                ])
                appended += 1

            for test_name in remaining:
                writer.writerow([
                    args.schema_version,
                    now,
                    args.git_sha,
                    args.git_ref,
                    args.workflow_run_id,
                    args.workflow_name,
                    os_name,
                    report_name,
                    test_name,
                    "FAILED_AFTER_RETRY",
                    attempts,
                ])
                appended += 1

    print(f"Appended {appended} flaky history row(s) to {history_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())