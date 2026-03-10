#!/usr/bin/env python3
import argparse
import re
import subprocess
from pathlib import Path


def parse_failed_tests(log_path: Path) -> list[str]:
    if not log_path.exists():
        return []

    failed_tests: list[str] = []
    for line in log_path.read_text(encoding="utf-8").splitlines():
        _, _, test_name = line.partition(":")
        test_name = test_name.strip()
        if test_name:
            failed_tests.append(test_name)
    return failed_tests


def build_command(args: argparse.Namespace, regex: str | None = None) -> list[str]:
    command = ["ctest"]

    if args.config:
        command.extend(["-C", args.config])
    if args.output_on_failure:
        command.append("--output-on-failure")
    if args.parallel:
        command.extend(["-j", str(args.parallel)])
    effective_regex = regex if regex is not None else args.regex
    if effective_regex:
        command.extend(["-R", effective_regex])
    if args.label:
        command.extend(["-L", args.label])
    if args.exclude_label:
        command.extend(["-LE", args.exclude_label])

    return command


def write_report(report_path: Path, attempts: int, flaky_tests: list[str], remaining_failures: list[str]) -> None:
    report_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        f"attempts={attempts}",
        f"flaky_test_count={len(flaky_tests)}",
        f"remaining_failure_count={len(remaining_failures)}",
        "",
        "flaky_tests:",
    ]
    lines.extend(flaky_tests or ["<none>"])
    lines.extend(["", "remaining_failures:"])
    lines.extend(remaining_failures or ["<none>"])
    report_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description="Run ctest with bounded retries and flake reporting")
    parser.add_argument("--build-dir", required=True)
    parser.add_argument("--config", default="")
    parser.add_argument("--regex", default="")
    parser.add_argument("--label", default="")
    parser.add_argument("--exclude-label", default="")
    parser.add_argument("--parallel", type=int, default=0)
    parser.add_argument("--max-retries", type=int, default=2)
    parser.add_argument("--report-file", required=True)
    parser.add_argument("--output-on-failure", action="store_true")
    args = parser.parse_args()

    build_dir = Path(args.build_dir)
    last_failed_log = build_dir / "Testing" / "Temporary" / "LastTestsFailed.log"
    flaky_tests: list[str] = []
    attempts = 1

    initial_command = build_command(args)
    print(f"[ctest-retry] initial run: {' '.join(initial_command)}")
    result = subprocess.run(initial_command, cwd=build_dir)
    if result.returncode == 0:
        write_report(Path(args.report_file), attempts, flaky_tests, [])
        print("[ctest-retry] all tests passed on first attempt")
        return 0

    remaining_failures = parse_failed_tests(last_failed_log)
    if not remaining_failures:
        write_report(Path(args.report_file), attempts, flaky_tests, [])
        print("[ctest-retry] ctest failed but no LastTestsFailed.log was produced")
        return result.returncode

    for retry_index in range(1, args.max_retries + 1):
        attempts += 1
        retry_regex = "^(" + "|".join(re.escape(name) for name in remaining_failures) + ")$"
        retry_command = build_command(args, retry_regex)
        print(f"[ctest-retry] retry {retry_index}/{args.max_retries}: {' '.join(retry_command)}")
        retry_result = subprocess.run(retry_command, cwd=build_dir)
        if retry_result.returncode == 0:
            flaky_tests.extend(remaining_failures)
            remaining_failures = []
            break

        next_failures = parse_failed_tests(last_failed_log)
        retried_set = set(remaining_failures)
        next_failed_set = set(next_failures)
        flaky_tests.extend(sorted(retried_set - next_failed_set))
        remaining_failures = next_failures

        if not remaining_failures:
            break

    flaky_tests = sorted(set(flaky_tests))
    write_report(Path(args.report_file), attempts, flaky_tests, remaining_failures)

    if remaining_failures:
        print("[ctest-retry] tests still failing after retries:")
        for test_name in remaining_failures:
            print(f"  - {test_name}")
        return 1

    if flaky_tests:
        print("[ctest-retry] flaky tests recovered on retry:")
        for test_name in flaky_tests:
            print(f"  - {test_name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())