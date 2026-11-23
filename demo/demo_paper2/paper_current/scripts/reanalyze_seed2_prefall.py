"""Recompare recorded seed2 sessions using pre-fall data and common commands."""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from compare_go2_contact_models import load_session, metrics


def interval_weights(session, stop):
    # Do not let the final selected observation extend past the time boundary.
    return np.maximum(0, np.minimum(session["weights"], stop - session["time_s"]))


def attitude_metrics(session, weights):
    angles = session["rpy_deg"]
    selected = weights > 0
    angles, w = angles[selected], weights[selected]
    if not len(w):
        raise ValueError("No selected observations for attitude metrics")
    mean = np.average(angles, axis=0, weights=w)
    variance = np.average((angles - mean) ** 2, axis=0, weights=w)
    # Remove a separate mean per command bin to reduce differences in commanded posture.
    within_bin_variance = 0.0
    selected_cells = [c for c, valid in zip(session["cells"], selected) if valid]
    for cell in set(selected_cells):
        mask = np.array([c == cell for c in selected_cells])
        center = np.average(angles[mask], axis=0, weights=w[mask])
        within_bin_variance += float(np.sum(w[mask] * np.sum((angles[mask] - center) ** 2, axis=1)))
    result = {
        "mean_roll_deg": float(mean[0]), "mean_pitch_deg": float(mean[1]),
        "roll_std_deg": float(np.sqrt(variance[0])), "pitch_std_deg": float(np.sqrt(variance[1])),
        "demeaned_combined_tilt_rms_deg": float(np.sqrt(variance.sum())),
        "command_demeaned_combined_tilt_rms_deg": float(np.sqrt(within_bin_variance / w.sum())),
    }
    total_squared = float(np.average(np.sum(angles ** 2, axis=1), weights=w))
    assert np.isclose(total_squared, float(np.sum(mean ** 2) + variance.sum()), atol=1e-10)
    return result


def balance_commands(sessions, weights, moving_only=False):
    balanced = [np.zeros_like(w) for w in weights]
    records = []
    for cell in sorted(set(sessions[0]["cells"]) & set(sessions[1]["cells"])):
        if moving_only and all(value == 0 for value in cell):
            continue
        masks = [np.asarray([c == cell for c in s["cells"]]) for s in sessions]
        exposure = [float(w[mask].sum()) for w, mask in zip(weights, masks)]
        if min(exposure) < 1.0:
            continue
        shared = min(exposure)
        for i in range(2):
            balanced[i][masks[i]] = weights[i][masks[i]] * shared / exposure[i]
        cell_metrics = []
        for s, w, mask in zip(sessions, weights, masks):
            cell_metrics.append(metrics(s, np.where(mask, w, 0)))
        records.append({"command_bin": [int(v) for v in cell],
                        "duration_static_s": exposure[0], "duration_smooth_s": exposure[1],
                        "equalized_duration_s": shared,
                        "static": cell_metrics[0], "smooth": cell_metrics[1]})
    return balanced, records


def analyze(sessions, stop, mode):
    weights = [interval_weights(s, stop) for s in sessions]
    bin_records = []
    if mode != "common_time":
        weights, bin_records = balance_commands(sessions, weights, mode == "matched_moving")
    values = [metrics(s, w) for s, w in zip(sessions, weights)]
    for s, w, value in zip(sessions, weights, values):
        value.update(attitude_metrics(s, w))
    return {
        "end_time_exclusive_s": stop, "mode": mode,
        "sample_counts_before_selection": [len(s["weights"]) for s in sessions],
        "selected_sample_counts": [int(np.count_nonzero(w)) for w in weights],
        "excluded_sample_counts": [int(len(w) - np.count_nonzero(w)) for w in weights],
        "effective_exposure_s": [float(w.sum()) for w in weights],
        "static": values[0], "smooth": values[1],
        "smooth_relative_change_pct": {
            key: (100 * (values[1][key] / value - 1)
                  if value and values[1][key] is not None and not key.startswith("mean_") else None)
            for key, value in values[0].items()},
        "command_bins": bin_records,
    }


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--baseline", required=True, type=Path)
    p.add_argument("--smooth", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    args = p.parse_args()
    sessions = [load_session(args.baseline), load_session(args.smooth)]
    definitions = {
        "combined_tilt_rms_deg": "sqrt(mean_time(roll_deg**2 + pitch_deg**2))",
        "tilt_p95_deg": "time-weighted P95 of sqrt(roll_deg**2 + pitch_deg**2)",
        "combined_angular_rate_rms_rad_s": "sqrt(mean_time(wx**2 + wy**2))",
        "target_q_rate_rms_rad_s": "RMS of target_q backward differences across 12 joints",
        "joint_tau_est_rms_Nm": "RMS across time and 12 joints of estimated torque",
        "absolute_mechanical_power_mean_W": "mean_time(sum_j(abs(tau_est_j * dq_j)))",
        "system_electrical_power_mean_W": "mean_time(power_v * power_a), whole-robot electrical power",
        "mean_roll_deg": "time-weighted mean roll; signed posture bias",
        "mean_pitch_deg": "time-weighted mean pitch; signed posture bias",
        "roll_std_deg": "time-weighted roll population standard deviation, not uncertainty across trials",
        "pitch_std_deg": "time-weighted pitch population standard deviation, not uncertainty across trials",
        "demeaned_combined_tilt_rms_deg": "sqrt(var_time(roll) + var_time(pitch)); removes session mean",
        "command_demeaned_combined_tilt_rms_deg": "RMS angular deviations after subtracting separate roll/pitch means within each selected command bin",
    }
    for s in sessions:
        log_path = Path(s["meta"]["log_path"]).name
        source_dir = args.baseline if s is sessions[0] else args.smooth
        with (source_dir / log_path).open(encoding="utf-8") as stream:
            rows = [json.loads(line) for line in stream]
        s["rpy_deg"] = np.rad2deg(np.asarray([row["imu"]["rpy"][:2] for row in rows]))
        # Use combined-axis norms as in the previous figure; old analyzer RMS
        # divided by the number of axes, giving values smaller by sqrt(2).
        s["values"]["combined_tilt_rms_deg"] = s["values"].pop("roll_pitch_rms_deg") * 2
        s["values"]["combined_angular_rate_rms_rad_s"] = s["values"].pop("roll_pitch_rate_rms_rad_s") * 2
    events = np.flatnonzero(sessions[1]["tilt_deg"] >= 20)
    if not len(events):
        raise RuntimeError("No smooth fall event found; review time-window rule")
    onset = float(sessions[1]["time_s"][events[0]])
    results = [analyze(sessions, cutoff, mode)
               for cutoff in (35.0, 38.0, 40.0, 42.0, onset)
               for mode in ("common_time", "matched_commands", "matched_moving")]
    report = {
        "sessions": [s["meta"]["session_id"] for s in sessions],
        "configuration_checks": {k: sessions[0]["meta"].get(k) == sessions[1]["meta"].get(k)
                                 for k in ("config_sha256", "coverage_sha256", "policy_seed", "checkpoint_iteration")},
        "first_smooth_tilt_over_20_deg_s": onset,
        "primary_window": "0 <= t < 40 s (excludes rollover and its immediately preceding onset)",
        "matching_rule": "Signed 0.25-wide command bins per axis; deadband 0.05; >=1 s per session per shared bin; equalize time weight to min exposure",
        "metric_definitions": definitions, "results": results,
        "quality_full_sessions": [s["quality"] for s in sessions],
        "limitations": [
            "One manual session per policy with seed2 only; temporal samples are not independent repeated trials; no p-values.",
            "Command matching balances instantaneous command bins, not actual speed, distance, command histories, external disturbances or surface conditions.",
            "The first 40 seconds is a descriptive sensitivity window selected after observing the fall, not a preregistered test.",
            "The baseline has lower commanded-motion exposure, and smooth battery voltage is lower; report rather than hide these differences.",
            "Lower effort does not establish better energy efficiency or cost of transport without equal achieved motion/distance.",
            "Contact labels are unavailable; no claim about foot slip, airtime or friction realization is supported.",
            "All original logs remain intact; time exclusion and unmatched-command exclusions are counted in this report.",
        ],
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "prefall_reanalysis.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    with (args.output / "prefall_metrics.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["cutoff_s", "mode", "metric", "static", "smooth", "smooth_change_pct", "effective_s"])
        for result in results:
            for metric in definitions:
                writer.writerow([result["end_time_exclusive_s"], result["mode"], metric,
                                 result["static"][metric], result["smooth"][metric],
                                 result["smooth_relative_change_pct"][metric], result["effective_exposure_s"]])
    primary = next(r for r in results if r["end_time_exclusive_s"] == 40.0 and r["mode"] == "matched_commands")
    labels = {
        "combined_tilt_rms_deg": "roll/pitch 合成幅值 RMS（°）",
        "tilt_p95_deg": "roll/pitch 合成幅值 P95（°）",
        "combined_angular_rate_rms_rad_s": "roll/pitch 合成角速度 RMS（rad/s）",
        "target_q_rate_rms_rad_s": "目标关节变化率 RMS（rad/s）",
        "joint_tau_est_rms_Nm": "估计关节力矩 RMS（Nm）",
        "absolute_mechanical_power_mean_W": "机械功率代理（W）",
        "system_electrical_power_mean_W": "整机电功率（W）",
        "mean_roll_deg": "平均roll（°，带符号）",
        "mean_pitch_deg": "平均pitch（°，带符号）",
        "roll_std_deg": "去均值roll标准差（°）",
        "pitch_std_deg": "去均值pitch标准差（°）",
        "demeaned_combined_tilt_rms_deg": "去均值合成摆动RMS（°）",
        "command_demeaned_combined_tilt_rms_deg": "各指令箱内去均值合成摆动RMS（°）",
    }
    lines = ["# Seed2 排除摔倒后的重新比较", "",
             "仅使用本地两次已有记录；原始日志和之前报告保留。", "",
             f"smooth 首次超过20°发生于 {onset:.6f} 秒。主分析取两者前40秒，额外排除约2.34秒失稳前缘；35秒、38秒、42秒、42.34秒为敏感性检查。", "",
             "主表按摇杆三个轴的方向与幅值分箱，并在双方均覆盖至少1秒的分箱内赋相同总时间权重。", "",
             f"匹配后等效时长每种策略 {primary['effective_exposure_s'][0]:.2f} 秒，共{len(primary['command_bins'])}个分箱，占前40秒约44.4%。未覆盖充分的指令不进入匹配结论。", "",
             "| 指标 | 原静摩擦 seed2 | smooth seed2 | smooth相对变化 |", "|---|---:|---:|---:|"]
    for key, label in labels.items():
        change = primary["smooth_relative_change_pct"][key]
        display_change = "不对带符号偏置计算百分比" if change is None else f"{change:+.1f}%"
        lines.append(f"| {label} | {primary['static'][key]:.3f} | {primary['smooth'][key]:.3f} | {display_change} |")
    lines += ["", "分方向结果并不一致：高幅值纯前进时原静摩擦姿态/角速度更小；高幅值纯后退时 smooth 的合成幅值 RMS 更小，角速度接近。不能写成所有方向均优于 smooth。", "",
              "全部匹配分箱的偏航轴均在死区内，现有匹配结果不覆盖转向表现。所谓相同命令只代表同一幅值区间，不是完全一致的时序或实际运动速度。", "",
              "## 时间窗敏感性", "", "| 截止时间 | 等效匹配秒数 | 指令箱数 | 倾角RMS变化 | 去均值摆动RMS变化 | 箱内去均值摆动RMS变化 | 角速度RMS变化 |", "|---|---:|---:|---:|---:|---:|---:|"]
    for result in results:
        if result["mode"] != "matched_commands":
            continue
        change = result["smooth_relative_change_pct"]
        lines.append(f"| {result['end_time_exclusive_s']:.2f}s | {result['effective_exposure_s'][0]:.2f} | {len(result['command_bins'])} | {change['combined_tilt_rms_deg']:+.1f}% | {change['demeaned_combined_tilt_rms_deg']:+.1f}% | {change['command_demeaned_combined_tilt_rms_deg']:+.1f}% | {change['combined_angular_rate_rms_rad_s']:+.1f}% |")
    early_results = [r for r in results if r["mode"] == "matched_commands" and r["end_time_exclusive_s"] <= 40]
    bin_sets = [{tuple(cell["command_bin"]) for cell in r["command_bins"]} for r in early_results]
    assert all(s == bin_sets[0] for s in bin_sets), "Early-window command support changed; review comparisons"
    lines += ["", "35/38/40秒三组匹配使用完全相同的6种指令分箱，等效覆盖时长约14.96/17.10/17.78秒。", "",
              "提前截到35秒（距20°失稳点约7.34秒）仍有去均值摆动+17.3%、角速度+14.7%；38秒为+18.6%/+15.0%；40秒为+20.6%/+14.8%。因此差异不是只出现在失稳前最后几秒。", "",
              "去掉零摇杆分箱后，35/38/40秒去均值摆动仍分别高约16.9%/18.3%/20.1%，角速度高15.4%/15.8%/15.5%，方向保持。", "",
              "去均值只去掉相应区间的固定平均姿态，不去掉动态变化；指令箱内去均值再去除各箱平均姿态，但仍包含命令过渡与瞬态，不能解释为完全相同轨迹下的稳态性能。时间标准差不是跨实验不确定度，不能用来做显著性结论。", "",
              "解释：排除摔倒后，原静摩擦策略在这次可匹配操作范围内仍显示较小的总体姿态波动，但差距比整段比较温和。动作变化率与力矩差距小，不能据此强调显著的动作平滑性或力矩优势。", "",
              "局限：单seed、每个策略一次人工记录，无独立重复和显著性检验；电池电压不同，且缺少可信实际速度/距离，功率差异不能直接解释为能效提升。没有可靠接触标签，不能从这些指标证明减少足端滑移或静摩擦机制。", "",
              "口径：本表姿态RMS使用 sqrt(mean(roll²+pitch²))；与之前按轴平均的RMS相差sqrt(2)，相对比例不变。功率为 sum(abs(tau_est*dq)) 的时间均值代理，不是电机真实输入功率。"]
    (args.output / "analysis_zh.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    for result in results:
        assert all(abs(result["effective_exposure_s"][0] - x) < 1e-8 for x in result["effective_exposure_s"])
        if result["mode"] == "common_time":
            assert all(abs(x - result["end_time_exclusive_s"]) < 1e-8 for x in result["effective_exposure_s"])
    print(json.dumps({"primary_40s_matched": {k:v for k,v in primary.items() if k != "command_bins"},
                      "output": str(args.output)}, indent=2))


if __name__ == "__main__":
    main()
