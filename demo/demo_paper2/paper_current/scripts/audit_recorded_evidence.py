"""Recompute manuscript evidence checks from audit runs and archived raw logs."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import re
from collections import Counter
from pathlib import Path
import numpy as np
import yaml

ROOT=Path(__file__).resolve().parents[1]


def rows(path):
    return list(csv.DictReader(path.open(newline="",encoding="utf-8")))


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit-dir",type=Path,required=True)
    args=parser.parse_args()
    audit=args.audit_dir.resolve()
    results={}
    impact=rows(audit/"impact/impact_audit.csv")
    proposed={r["case"]:r for r in impact if r["variant"]=="shifted"}
    previous={r["case"]:r for r in impact if r["variant"]=="velocity_regularized"}
    old={r["case"]:r for r in rows(ROOT/"res/source_data/experiment1/panel_c_impulse_refined_v6.csv")}
    results["impact"]={
        "matched_case_count":len(proposed),"same_case_names":set(proposed)==set(previous),
        "max_proposed_change_from_magnitude_sum_Ns":max(abs(float(proposed[k]["net_tangential_impulse_Ns"])-float(old[k]["tangential_impulse_Ns"])) for k in old),
        "max_normal_impulse_relative_difference":max(abs(float(previous[k]["normal_impulse_Ns"])/float(proposed[k]["normal_impulse_Ns"])-1) for k in proposed),
        "all_first_episode_pre_power_nonpositive_with_1e8_tolerance":all(int(r["positive_pre_power_intervals"])==0 for r in impact)}
    newa=rows(audit/"impact/panel_a_center_velocity_audited.csv")
    olda=rows(ROOT/"res/source_data/experiment1/panel_a_trajectory_v5.csv")
    changes=[]
    for r in olda:
        candidates=[n for n in newa if n["method"]==r["method"]]
        q=min(candidates,key=lambda n:abs(float(n["time_from_contact_ms"])-float(r["time_from_contact_ms"])))
        if abs(float(q["time_from_contact_ms"])-float(r["time_from_contact_ms"]))<1e-6:
            changes.append(abs(float(q["normal_velocity_m_per_s"])-float(r["normal_velocity_m_per_s"])))
    results["impact"]["panel_a_max_velocity_correction_m_per_s"]=max(changes)
    high=rows(audit/"high_stiffness/normal/high_stiffness_regression.csv")
    archive=rows(ROOT/"res/high_stiffness_release4m_20261020/normal/high_stiffness_regression.csv")
    archived={(float(r["stiffness"]),float(r["friction"])):r for r in archive}
    selected_k={1e7,1e9,1e12,1e15,1e18,1e21,1e24,1e30}
    selected=[r for r in high if float(r["stiffness"]) in selected_k]
    same={float(r["stiffness"]):r for r in selected if float(r["friction"])==0}
    results["high_stiffness"]={
        "all_run_count":len(high),"reported_subset_count":len(selected),
        "max_bounce_absolute_difference_from_archive_m":max(abs(float(r["first_bounce_height"])-float(archived[(float(r["stiffness"]),float(r["friction"]))]["first_bounce_height"])) for r in high),
        "max_reported_bounce_relative_error":max(abs(float(r["first_bounce_height_ratio_incident"])-1) for r in selected),
        "max_reported_contact_time_relative_error":max(float(r["contact_time_relative_error"]) for r in selected),
        "friction_pairs_height_identical":all(float(r["first_bounce_height"])==float(same[float(r["stiffness"])]["first_bounce_height"]) for r in selected),
        "all_reported_tangential_impulses_zero":all(float(r["tangent_impulse_magnitude"])==0 for r in selected),
        "excluded_low_stiffness": [{"k":float(r["stiffness"]),"bounce_relative_error":float(r["first_bounce_height_ratio_incident"])-1,
                                     "contact_time_measured_s":float(r["measured_contact_time"]),"contact_time_reference_s":float(r["analytical_contact_time"])}
                                    for r in high if float(r["stiffness"])<1e7 and float(r["friction"])==0]}
    training=rows(audit/"training/multiseed_seed_summary.csv")
    cohort=[r for r in training if int(r["seed"]) in (2,3,4)]
    config=yaml.safe_load((ROOT/"data/training/seed_2/flat_go2/exp1/config.yaml").read_text())
    results["training"]={"all_completed_seeds":[int(r["seed"]) for r in training],
        "reported_seeds":[int(r["seed"]) for r in cohort],
        "terminal_rewards":{r["seed"]:float(r["reward_last100_mean"]) for r in cohort},
        "recovery_events":sum(int(r["recovered_environment_count"]) for r in cohort),
        "all_reported_iterations_1200":all(int(r["iterations"])==1200 for r in cohort),
        "domain_randomization_config":next(value["domain_rand"] for value in config.values() if isinstance(value,dict) and "domain_rand" in value)}
    # Config file may wrap environment configuration one level under 'env'.
    manifest=json.loads((ROOT/"hardware/sire_seeds_1_4_20260924_manifest.json").read_text())
    checks={}
    for entry in manifest["policies"]:
        seed=entry["seed"]
        checkpoint=ROOT/f"data/training/seed_{seed}/flat_go2/exp1/model_1200.pt"
        jit=ROOT/"hardware/policies/sire_seeds_1_4_20260924"/f"sire_seed_{seed}_model_1200_jit.pt"
        checks[str(seed)]={"checkpoint_matches_manifest":digest(checkpoint)==entry["checkpoint_sha256"],
                           "jit_matches_manifest":digest(jit)==entry["jit_sha256"]}
    results["training"]["policy_hash_checks"]=checks
    recovery_reasons=Counter()
    event_ids=[]
    contact_counts=[]
    for seed in (2,3,4):
        for line in (ROOT/f"data/training/seed_{seed}/stdout.log").read_text(errors="replace").splitlines():
            if "[Sire recovered environment]" not in line:
                continue
            match=re.search(r"reason=(.*?) sim_time=",line)
            recovery_reasons[match.group(1) if match else "unparsed"]+=1
            env=re.search(r"env_id=(\d+)",line)
            if env:
                event_ids.append((seed,int(env.group(1))))
            contact=re.search(r"contacts=\{count=(\d+)",line)
            if contact:
                contact_counts.append(int(contact.group(1)))
    results["training"]["recovery_reason_counts"]=dict(recovery_reasons)
    results["training"]["recovery_reason_categories"]={
        "configured_safety_bounds":sum(n for key,n in recovery_reasons.items() if key=="physics state exceeded configured safety bounds"),
        "integrator_catastrophic_state":sum(n for key,n in recovery_reasons.items() if "Integrator: catastrophic state" in key),
        "other":sum(n for key,n in recovery_reasons.items() if key!="physics state exceeded configured safety bounds" and "Integrator: catastrophic state" not in key)}
    results["training"]["distinct_seed_environment_id_pairs"]=len(set(event_ids))
    results["training"]["recovery_events_with_zero_recorded_contacts"]=sum(n==0 for n in contact_counts)
    hardware=rows(audit/"hardware/hardware_control_cycles.csv")
    subset=[r for r in hardware if int(r["seed"]) in (2,3,4)]
    results["hardware"]={"complete_control_cycles":len(subset),
        "nominal_duration_s":len(subset)*.02,
        "sum_observed_session_time_spans_s":sum(float(r["duration_s"]) for r in rows(audit/"hardware/hardware_seed_summary.csv") if int(r["seed"]) in (2,3,4)),
        "deadline_misses":sum(int(r["deadline_miss"]) for r in subset),
        "max_compute_to_send_ms":max(float(r["compute_to_send_ms"]) for r in subset)}
    fresh=json.loads((audit/"prefall/normal_motion_reanalysis.json").read_text())
    reference=json.loads((ROOT/"analysis/hardware_normal_motion_20261031/normal_motion_reanalysis.json").read_text())
    def primary_result(report):
        if isinstance(report.get("primary"),dict) and "static" in report["primary"]:
            result=report["primary"]
        else:
            result=next(r for r in report["results"] if r["end_time_exclusive_s"]==40 and r["mode"]=="matched_commands")
        assert result["end_time_exclusive_s"]==40 and result["mode"]=="matched_commands"
        return result
    primary=primary_result(fresh)
    archived_primary=primary_result(reference)
    table_metrics=("tilt_p95_deg","command_demeaned_combined_tilt_rms_deg","combined_angular_rate_rms_rad_s","target_q_rate_rms_rad_s","joint_tau_est_rms_Nm")
    sessions_match=fresh["sessions"]==reference["sessions"]
    metric_checks={side:{key:bool(np.isclose(primary[side][key],archived_primary[side][key],rtol=1e-12,atol=1e-12)) for key in table_metrics} for side in ("static","smooth")}
    metrics_reproduced=all(value for checks in metric_checks.values() for value in checks.values())
    assert sessions_match and metrics_reproduced, "Current normal-motion Table IV source or metrics differ from its archived report"
    assert np.allclose(primary["effective_exposure_s"],archived_primary["effective_exposure_s"],rtol=1e-12,atol=1e-12)
    assert [r["command_bin"] for r in primary["command_bins"]]==[r["command_bin"] for r in archived_primary["command_bins"]]
    results["normal_motion"]={"sessions":fresh["sessions"],"session_ids_reproduced":sessions_match,
        "all_five_metrics_reproduced":metrics_reproduced,"metric_checks":metric_checks,
        "selection":fresh.get("selection"),
        "matched_exposure_s":primary["effective_exposure_s"],"matched_bins":len(primary["command_bins"]),
        "reductions_pct":{key:100*(1-primary["static"][key]/primary["smooth"][key]) for key in table_metrics}}
    velocity_path=ROOT/"analysis/seed2_velocity_comparison/seed2_velocity_comparison_full.csv"
    velocity=rows(velocity_path)
    rawpath=next((ROOT/"data/hardware_raw/seed_2").glob("*.jsonl"))
    raw=[json.loads(line) for line in rawpath.read_text().splitlines() if line.strip()]
    assert len(raw)==len(velocity)
    bytime={int(r["monotonic_ns"]):r for r in raw}
    errors={}
    for key,axis in (("command_forward_m_s",0),("command_lateral_m_s",1),("command_yaw_rate_rad_s",2)):
        errors[key]=max(abs(float(r[key])-bytime[int(r["monotonic_ns"])]["command"][axis]) for r in velocity)
    gyro=max(abs(float(r["onboard_imu_yaw_rate_rad_s"])-bytime[int(r["monotonic_ns"])]["imu"]["gyroscope"][2]) for r in velocity)
    smoothing={}
    for rawcol,displaycol in (("onboard_base_velocity_body_forward_m_s","onboard_base_velocity_body_forward_m_s_display_5sample_mean"),
                              ("onboard_base_velocity_body_lateral_m_s","onboard_base_velocity_body_lateral_m_s_display_5sample_mean"),
                              ("onboard_imu_yaw_rate_rad_s","onboard_imu_yaw_rate_rad_s_display_5sample_mean")):
        data=np.array([float(r[rawcol]) for r in velocity])
        smooth=np.convolve(np.pad(data,(2,2),mode="edge"),np.ones(5)/5,mode="valid")
        smoothing[displaycol]=float(np.max(abs(smooth-np.array([float(r[displaycol]) for r in velocity]))))
    results["velocity_response"]={"raw_cycles":len(raw),"figure_cycles":sum(10<=float(r["time_s"])<=40 for r in velocity),
        "max_command_errors":errors,"max_gyro_error_rad_per_s":gyro,"max_display_smoothing_errors":smoothing,
        "linear_velocity_is_independently_measured":False,"contact_labels_are_independent":False}
    (audit/"recorded_evidence_checks.json").write_text(json.dumps(results,indent=2),encoding="utf-8")
    print(json.dumps(results,indent=2))


if __name__=="__main__":
    main()
