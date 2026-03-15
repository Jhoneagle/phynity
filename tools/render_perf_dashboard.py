#!/usr/bin/env python3
import argparse
import csv
import html
from collections import defaultdict
from pathlib import Path


def load_rows(csv_path: Path) -> list[dict[str, str]]:
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def svg_polyline(values: list[float], width: int = 220, height: int = 64) -> str:
    if not values:
        return "<svg width='220' height='64'></svg>"

    minimum = min(values)
    maximum = max(values)
    span = maximum - minimum
    if span == 0.0:
        span = 1.0

    if len(values) == 1:
        points = f"0,{height // 2} {width},{height // 2}"
    else:
        points = []
        for index, value in enumerate(values):
            x = (width * index) / (len(values) - 1)
            normalized = (value - minimum) / span
            y = height - normalized * (height - 8) - 4
            points.append(f"{x:.1f},{y:.1f}")
        points = " ".join(points)

    return (
        f"<svg width='{width}' height='{height}' viewBox='0 0 {width} {height}' role='img' aria-label='trend'>"
        f"<rect x='0' y='0' width='{width}' height='{height}' rx='10' fill='#f6efe5'></rect>"
        f"<polyline fill='none' stroke='#b85c38' stroke-width='3' points='{points}'></polyline>"
        "</svg>"
    )


def parse_series_value(row: dict[str, str], key: str) -> float | None:
    raw = row.get(key, "")
    if raw in ("", None):
        return None
    try:
        return float(raw)
    except ValueError:
        return None


def build_dashboard(rows: list[dict[str, str]], max_points: int) -> str:
    grouped: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        grouped[(row["benchmark_name"], row["os"])] .append(row)

    sections: list[str] = []
    for (benchmark_name, os_name) in sorted(grouped):
        series = sorted(grouped[(benchmark_name, os_name)], key=lambda row: row["recorded_at_utc"])
        trimmed = series[-max_points:]
        per_frame = [float(row["milliseconds_per_frame"]) for row in trimmed]
        peak_rss_series = [value for value in (parse_series_value(row, "peak_rss_kb") for row in trimmed) if value is not None]
        allocator_delta_series = [
          value for value in (parse_series_value(row, "allocator_delta_bytes") for row in trimmed) if value is not None
        ]
        latest = trimmed[-1]
        regression = float(latest["regression_percent"])
        peak_rss_kb_raw = latest.get("peak_rss_kb", "")
        allocator_delta_raw = latest.get("allocator_delta_bytes", "")
        peak_rss_display = "n/a"
        if peak_rss_kb_raw not in ("", None):
            try:
                peak_rss_display = f"{int(float(peak_rss_kb_raw))} KB"
            except ValueError:
                peak_rss_display = html.escape(str(peak_rss_kb_raw))
        allocator_delta_display = "n/a"
        if allocator_delta_raw not in ("", None):
          try:
            allocator_delta_display = f"{int(float(allocator_delta_raw))} B"
          except ValueError:
            allocator_delta_display = html.escape(str(allocator_delta_raw))
        state_class = "fail" if latest["pass_fail"] == "FAIL" else "pass"
        state_label = html.escape(latest["pass_fail"])

        sections.append(
            "\n".join(
                [
                    "<article class='card'>",
                    f"  <h2>{html.escape(benchmark_name)} <span>{html.escape(os_name)}</span></h2>",
                    f"  <div class='trend'>{svg_polyline(per_frame)}</div>",
                    f"  <div class='trend memory'>{svg_polyline(peak_rss_series, height=50)}</div>",
                    f"  <div class='trend memory'>{svg_polyline(allocator_delta_series, height=50)}</div>",
                    "  <dl>",
                    f"    <div><dt>Latest</dt><dd>{float(latest['milliseconds_per_frame']):.4f} ms/frame</dd></div>",
                    f"    <div><dt>Baseline</dt><dd>{float(latest['golden_milliseconds_per_frame']):.4f} ms/frame</dd></div>",
                    f"    <div><dt>Regression</dt><dd>{regression:.2f}%</dd></div>",
                    f"    <div><dt>Peak RSS</dt><dd>{peak_rss_display}</dd></div>",
                    f"    <div><dt>Allocator Delta</dt><dd>{allocator_delta_display}</dd></div>",
                    f"    <div><dt>Status</dt><dd class='{state_class}'>{state_label}</dd></div>",
                    "  </dl>",
                    f"  <p class='meta'>Latest run: {html.escape(latest['recorded_at_utc'])} on {html.escape(latest['machine_tag'])}</p>",
                    "</article>",
                ]
            )
        )

    cards = "\n".join(sections) if sections else "<p>No performance history available.</p>"
    return f"""<!doctype html>
<html lang='en'>
<head>
  <meta charset='utf-8'>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <title>Phynity Performance Dashboard</title>
  <style>
    :root {{
      --bg: #f3eadf;
      --panel: #fffaf4;
      --ink: #1f1d1a;
      --muted: #6e6257;
      --accent: #b85c38;
      --ok: #2f6b3a;
      --bad: #9b2c2c;
      --border: #dfd1c3;
    }}
    body {{
      margin: 0;
      font-family: Georgia, 'Times New Roman', serif;
      background: radial-gradient(circle at top, #fff7ec, var(--bg));
      color: var(--ink);
    }}
    main {{
      max-width: 1200px;
      margin: 0 auto;
      padding: 32px 20px 48px;
    }}
    h1 {{
      margin: 0 0 8px;
      font-size: 2.4rem;
    }}
    .intro {{
      color: var(--muted);
      max-width: 70ch;
      margin-bottom: 24px;
    }}
    .grid {{
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
      gap: 16px;
    }}
    .card {{
      background: var(--panel);
      border: 1px solid var(--border);
      border-radius: 18px;
      padding: 18px;
      box-shadow: 0 14px 30px rgba(54, 37, 22, 0.08);
    }}
    h2 {{
      display: flex;
      justify-content: space-between;
      gap: 12px;
      margin: 0 0 12px;
      font-size: 1.1rem;
    }}
    .trend.memory svg {{
      height: 50px;
    }}
    h2 span {{
      color: var(--muted);
      font-size: 0.92rem;
      font-weight: normal;
    }}
    dl {{
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 12px;
      margin: 14px 0 0;
    }}
    dl div {{
      background: #fbf3ea;
      border-radius: 12px;
      padding: 10px 12px;
    }}
    dt {{
      font-size: 0.8rem;
      color: var(--muted);
      margin-bottom: 4px;
    }}
    dd {{
      margin: 0;
      font-size: 1rem;
    }}
    .pass {{ color: var(--ok); }}
    .fail {{ color: var(--bad); }}
    .meta {{
      margin: 12px 0 0;
      color: var(--muted);
      font-size: 0.9rem;
    }}
  </style>
</head>
<body>
  <main>
    <h1>Phynity Performance Dashboard</h1>
    <p class='intro'>This page is generated from the append-only performance history CSV. Each card shows the latest result and a small trend line for the most recent benchmark samples.</p>
    <section class='grid'>
{cards}
    </section>
  </main>
</body>
</html>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description="Render a static HTML performance dashboard from perf history CSV")
    parser.add_argument("--history-csv", required=True, help="Path to perf history CSV")
    parser.add_argument("--output", required=True, help="Output HTML file")
    parser.add_argument("--max-points", type=int, default=20, help="Maximum points to render per series")
    args = parser.parse_args()

    history_csv = Path(args.history_csv)
    output = Path(args.output)
    rows = load_rows(history_csv) if history_csv.exists() else []
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(build_dashboard(rows, args.max_points), encoding="utf-8")
    print(f"Wrote dashboard to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())