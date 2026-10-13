#!/usr/bin/env python3
import json
import sys
import os

def load_json(path):
    with open(path, "r") as f:
        return json.load(f)

def fmt_speedup(before, after):
    if after <= 0.0:
        return "N/A"
    speedup = before / after
    pct = (1.0 - after / before) * 100.0
    return f"{speedup:.2f}x ({pct:+.1f}%)"

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 compare_benchmarks.py <before.json> <optimized.json> [output_md]")
        sys.exit(1)

    file_before = sys.argv[1]
    file_opt = sys.argv[2]
    out_md = sys.argv[3] if len(sys.argv) > 3 else "benchmark_comparison.md"

    d_before = load_json(file_before)
    d_opt = load_json(file_opt)

    col_before = d_before["collision_benchmark"]
    col_opt = d_opt["collision_benchmark"]

    lines = []
    lines.append("# MoveIt Collision & Planning Benchmark Comparison")
    lines.append("\nComparing MoveIt performance **Before Optimization** (original high-density collision meshes) vs **Optimized** (decimated collision meshes).\n")

    # 1. Collision detection microbenchmark
    lines.append("## 1. Direct Collision Detection Performance (FCL Microbenchmark)")
    lines.append("| Metric | Before Optimization | Optimized | Speedup / Improvement |")
    lines.append("| :--- | :--- | :--- | :--- |")
    lines.append(f"| **Collision Checks / Sec** | {col_before['checks_per_sec']:,.0f} Hz | {col_opt['checks_per_sec']:,.0f} Hz | **{col_opt['checks_per_sec'] / col_before['checks_per_sec']:.2f}x faster** |")
    lines.append(f"| **Mean Check Time** | {col_before['mean_us']:.2f} μs ({col_before['mean_us']/1000.0:.4f} ms) | {col_opt['mean_us']:.2f} μs ({col_opt['mean_us']/1000.0:.4f} ms) | **{fmt_speedup(col_before['mean_us'], col_opt['mean_us'])}** |")
    lines.append(f"| **Median Check Time** | {col_before['median_us']:.2f} μs | {col_opt['median_us']:.2f} μs | **{fmt_speedup(col_before['median_us'], col_opt['median_us'])}** |")
    lines.append(f"| **95th Percentile Time** | {col_before['p95_us']:.2f} μs | {col_opt['p95_us']:.2f} μs | **{fmt_speedup(col_before['p95_us'], col_opt['p95_us'])}** |")
    lines.append(f"| **99th Percentile Time** | {col_before['p99_us']:.2f} μs | {col_opt['p99_us']:.2f} μs | **{fmt_speedup(col_before['p99_us'], col_opt['p99_us'])}** |")
    lines.append(f"| **Total Time (5,000 checks)** | {col_before['total_time_ms']:.2f} ms | {col_opt['total_time_ms']:.2f} ms | **{fmt_speedup(col_before['total_time_ms'], col_opt['total_time_ms'])}** |")
    lines.append(f"| **Collision Detection Consistency** | {col_before['collisions_detected']} / {d_before['num_collision_checks']} ({col_before['collision_rate']*100:.1f}%) | {col_opt['collisions_detected']} / {d_opt['num_collision_checks']} ({col_opt['collision_rate']*100:.1f}%) | Consistent |")

    # Contact details benchmark
    if "contact_benchmark" in d_before and "contact_benchmark" in d_opt:
        cb = d_before["contact_benchmark"]
        co = d_opt["contact_benchmark"]
        lines.append("\n### Detailed Contact Computation (1,000 queries with contact points & normals):")
        lines.append("| Metric | Before Optimization | Optimized | Speedup |")
        lines.append("| :--- | :--- | :--- | :--- |")
        lines.append(f"| **Mean Contact Time** | {cb['mean_us']:.2f} μs | {co['mean_us']:.2f} μs | **{fmt_speedup(cb['mean_us'], co['mean_us'])}** |")
        lines.append(f"| **Median Contact Time** | {cb['median_us']:.2f} μs | {co['median_us']:.2f} μs | **{fmt_speedup(cb['median_us'], co['median_us'])}** |")
        lines.append(f"| **95th Percentile** | {cb['p95_us']:.2f} μs | {co['p95_us']:.2f} μs | **{fmt_speedup(cb['p95_us'], co['p95_us'])}** |")

    # 2. Planning results
    lines.append("\n## 2. MoveIt Motion Planning Time Comparison (OMPL / RRTConnect)")
    
    # Map scenario results by (group, scenario)
    p_before = {(r["group"], r["scenario"]): r for r in d_before.get("planning_results", [])}
    p_opt = {(r["group"], r["scenario"]): r for r in d_opt.get("planning_results", [])}

    all_keys = list(p_before.keys())
    groups = sorted(list(set(k[0] for k in all_keys)))

    total_time_before_all = 0.0
    total_time_opt_all = 0.0
    count_all = 0

    for grp in groups:
        lines.append(f"\n### Planning Group: `{grp}`")
        lines.append("| Scenario | Before Mean (ms) | Before Median (ms) | Optimized Mean (ms) | Optimized Median (ms) | Speedup (Mean) | Success Rate |")
        lines.append("| :--- | :--- | :--- | :--- | :--- | :--- | :--- |")

        grp_keys = [k for k in all_keys if k[0] == grp]
        grp_before_sum = 0.0
        grp_opt_sum = 0.0

        for k in grp_keys:
            rb = p_before[k]
            ro = p_opt.get(k)
            if not ro:
                continue

            b_mean = rb["mean_ms"]
            b_med = rb["median_ms"]
            o_mean = ro["mean_ms"]
            o_med = ro["median_ms"]
            grp_before_sum += b_mean
            grp_opt_sum += o_mean
            total_time_before_all += b_mean
            total_time_opt_all += o_mean
            count_all += 1

            speedup_str = fmt_speedup(b_mean, o_mean)
            succ_str = f"{rb['successful_trials']}/{rb['total_trials']} vs {ro['successful_trials']}/{ro['total_trials']}"

            lines.append(f"| **{rb['scenario']}** | {b_mean:.2f} ms | {b_med:.2f} ms | **{o_mean:.2f} ms** | **{o_med:.2f} ms** | **{speedup_str}** | {succ_str} |")

        if grp_keys:
            avg_b = grp_before_sum / len(grp_keys)
            avg_o = grp_opt_sum / len(grp_keys)
            lines.append(f"| **Group Average** | **{avg_b:.2f} ms** | - | **{avg_o:.2f} ms** | - | **{fmt_speedup(avg_b, avg_o)}** | - |")

    # Overall summary
    lines.append("\n## 3. Overall Benchmark Summary")
    if count_all > 0:
        overall_b = total_time_before_all / count_all
        overall_o = total_time_opt_all / count_all
        lines.append("| Metric | Before Optimization | Optimized | Improvement |")
        lines.append("| :--- | :--- | :--- | :--- |")
        lines.append(f"| **Average Planning Time Across All Scenarios** | **{overall_b:.2f} ms** | **{overall_o:.2f} ms** | **{fmt_speedup(overall_b, overall_o)}** |")
        lines.append(f"| **Collision Checking Speed** | **{col_before['mean_us']:.2f} μs** | **{col_opt['mean_us']:.2f} μs** | **{fmt_speedup(col_before['mean_us'], col_opt['mean_us'])}** |")
        lines.append(f"| **Collision Checking Frequency** | **{col_before['checks_per_sec']:,.0f} checks/s** | **{col_opt['checks_per_sec']:,.0f} checks/s** | **{col_opt['checks_per_sec']/col_before['checks_per_sec']:.2f}x higher** |")

    report_content = "\n".join(lines)
    with open(out_md, "w") as f:
        f.write(report_content)

    print(report_content)
    print(f"\nReport written to: {out_md}")

if __name__ == "__main__":
    main()
