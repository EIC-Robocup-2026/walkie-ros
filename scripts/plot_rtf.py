#!/usr/bin/env python3
"""
Live terminal plot of Gazebo's real_time_factor -- for debugging sim
performance when running headless (no GUI stats widget available).

Reads the `gz topic -e -t /stats` stream directly (no ROS bridge needed,
works whether or not the sim was launched with `headless:=true`) and draws
a scrolling ASCII chart in the terminal, color-coded around RTF == 1.0.

Usage:
    python3 scripts/plot_rtf.py
    python3 scripts/plot_rtf.py --topic /world/aws_small_house/stats
    python3 scripts/plot_rtf.py --log-csv /tmp/rtf.csv
"""
import argparse
import csv
import shutil
import signal
import subprocess
import sys
import time
from collections import deque

GREEN = "\x1b[32m"
YELLOW = "\x1b[33m"
RED = "\x1b[31m"
DIM = "\x1b[2m"
RESET = "\x1b[0m"
CLEAR = "\x1b[2J\x1b[H"

CHART_ROWS = 14


def color_for(value):
    if value >= 0.95:
        return GREEN
    if value >= 0.7:
        return YELLOW
    return RED


def render_chart(values, width, y_max):
    cols = list(values)[-width:]
    grid = [[" "] * len(cols) for _ in range(CHART_ROWS)]
    colors = [color_for(v) for v in cols]

    for x, v in enumerate(cols):
        filled = max(0, min(CHART_ROWS, round(v / y_max * CHART_ROWS)))
        for y in range(filled):
            grid[CHART_ROWS - 1 - y][x] = "█"

    one_row = CHART_ROWS - 1 - round(1.0 / y_max * CHART_ROWS)
    lines = []
    for row_idx, row in enumerate(grid):
        label = f"{y_max * (CHART_ROWS - row_idx) / CHART_ROWS:5.2f} "
        line_chars = []
        for x, ch in enumerate(row):
            if ch != " ":
                line_chars.append(f"{colors[x]}{ch}{RESET}")
            elif row_idx == one_row:
                line_chars.append(f"{DIM}·{RESET}")
            else:
                line_chars.append(" ")
        lines.append(label + "".join(line_chars))
    lines.append(" " * 6 + "-" * len(cols))
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--topic", default="/stats", help="gz-transport world stats topic (default: /stats)")
    parser.add_argument("--window", type=int, default=200, help="number of samples kept for the chart/stats")
    parser.add_argument("--fps", type=float, default=8.0, help="terminal redraw rate")
    parser.add_argument("--log-csv", default=None, help="optional path to also log sim_time,real_time_factor as CSV")
    args = parser.parse_args()

    proc = subprocess.Popen(
        ["gz", "topic", "-e", "-t", args.topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )

    def shutdown(*_):
        proc.terminate()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    values = deque(maxlen=args.window)
    csv_file = open(args.log_csv, "w", newline="") if args.log_csv else None
    csv_writer = csv.writer(csv_file) if csv_file else None
    if csv_writer:
        csv_writer.writerow(["wall_time", "real_time_factor"])

    last_draw = 0.0
    print(f"Subscribing to gz topic '{args.topic}' ... Ctrl+C to stop.")

    for line in proc.stdout:
        line = line.strip()
        if not line.startswith("real_time_factor:"):
            continue
        try:
            value = float(line.split(":", 1)[1])
        except ValueError:
            continue

        values.append(value)
        if csv_writer:
            csv_writer.writerow([time.time(), value])
            csv_file.flush()

        now = time.time()
        if now - last_draw < 1.0 / args.fps:
            continue
        last_draw = now

        term_width = shutil.get_terminal_size((100, 24)).columns
        chart_width = max(10, term_width - 8)
        y_max = max(1.2, max(values) * 1.1)

        current = values[-1]
        avg = sum(values) / len(values)
        lo, hi = min(values), max(values)

        out = [CLEAR]
        out.append(f"Gazebo real_time_factor  (topic: {args.topic})")
        out.append(render_chart(values, chart_width, y_max))
        out.append("")
        out.append(
            f"current: {color_for(current)}{current:6.3f}{RESET}   "
            f"avg: {avg:6.3f}   min: {lo:6.3f}   max: {hi:6.3f}   "
            f"samples: {len(values)}"
        )
        print("\n".join(out), flush=True)

    proc.wait()
    if csv_file:
        csv_file.close()


if __name__ == "__main__":
    main()
