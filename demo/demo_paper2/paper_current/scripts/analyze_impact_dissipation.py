"""Evaluate one vector maximum-dissipation residual on full Sire impacts.

Reconstruct the same frozen discrete contact velocity for both methods from
the actual sphere state, contact point, returned force, and accepted interval.
Validate this reconstruction against native Proposed solver traces.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import zipfile
import numpy as np


def write_csv(path, rows):
    with path.open("w",newline="",encoding="utf-8") as stream:
        writer=csv.DictWriter(stream,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def cross_matrix(v):
    x,y,z=v
    return np.array([[0.,-z,y],[z,0.,-x],[-y,x,0.]])


def disk_residual(impulse, velocity, capacity, effective_mass):
    argument=impulse-effective_mass*velocity
    length=np.linalg.norm(argument)
    projected=argument if length<=capacity else capacity*argument/length
    return float(np.linalg.norm(impulse-projected)/capacity)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input",type=Path,required=True,help="Audit directory, or its archived ZIP")
    parser.add_argument("--output-dir",type=Path,required=True)
    args=parser.parse_args()
    out=args.output_dir.resolve()
    out.mkdir(parents=True,exist_ok=True)
    archive=zipfile.ZipFile(args.input) if args.input.is_file() else None
    def read(relative):
        data=archive.read(relative) if archive else (args.input/relative).read_bytes()
        return json.loads(data),hashlib.sha256(data).hexdigest()
    audit,_=read("impact/impact_audit_report.json")
    cases=sorted({r["case"] for r in csv.DictReader(
        (archive.read("impact/impact_audit.csv").decode() if archive else (args.input/"impact/impact_audit.csv").read_text()).splitlines())})
    parameters=audit["parameters"]
    mass,radius,mu=parameters["mass"],parameters["radius"],parameters["friction"]
    inertia=.4*mass*radius**2
    summaries,intervals,hashes,checks=[],[],[],[]
    for case in cases:
        for variant in ("shifted","velocity_regularized"):
            prefix=f"impact/runs/{case}/{variant}"
            raw,sha=read(prefix+"/raw_records.json")
            summary,_=read(prefix+"/summary.json")
            hashes.append({"path":prefix+"/raw_records.json","sha256":sha})
            states=[i for i,p in enumerate(raw["partPq"]) if p is not None]
            trace_by_time={}
            if variant=="shifted":
                if archive:
                    paths=[p for p in archive.namelist() if p.startswith(prefix+"/traces/") and "/frame_" in p and p.endswith(".json")]
                else:
                    paths=[p.relative_to(args.input).as_posix() for p in (args.input/prefix/"traces").rglob("frame_*.json")]
                # Earlier exploratory directories may contain multiple runs;
                # the final raw record corresponds to the latest trace folder.
                latest=max({p.rsplit("/",1)[0] for p in paths})
                for p in paths:
                    if p.rsplit("/",1)[0]!=latest: continue
                    trace,_=read(p)
                    trace_by_time[round(trace["sim_time"],10)]=trace
            selected=[]
            for i,j in zip(states,states[1:]):
                t0,t1=float(raw["timeIndex"][i]),float(raw["timeIndex"][j])
                if not summary["contact_start_s"]-1e-10<=t0<summary["contact_last_positive_force_end_s"]-1e-10: continue
                h=t1-t0
                if h<=0: continue
                force=np.asarray(raw["generalForces"][i][1][:3],dtype=float)
                if force[2]<=1e-8: continue
                p=np.asarray(raw["partPq"][i][1][:3],dtype=float)
                spatial=np.asarray(raw["partVs"][i][1],dtype=float)
                pair=raw["penetrationPairs"][i]
                if not pair or len(pair)!=1: raise RuntimeError("Expected one physical contact point")
                point=np.asarray(pair[0]["p_WC"],dtype=float)
                arm=point-p
                skew=cross_matrix(arm)
                G=np.eye(3)/mass-skew@skew/inertia
                velocity=spatial[:3]+np.cross(spatial[3:],point)
                free_acceleration=np.array([0.,0.,-9.81])+np.cross(spatial[3:],np.cross(spatial[3:],arm))
                impulse=h*force
                post=velocity+h*free_acceleration+G@impulse
                rho=1/(.5*np.trace(G[:2,:2]))
                capacity=mu*impulse[2]
                residual=disk_residual(impulse[:2],post[:2],capacity,rho)
                speed=float(np.linalg.norm(post[:2]))
                sliding_error=float(np.linalg.norm(impulse[:2]+capacity*post[:2]/speed)/capacity) if speed>1e-6 else math.nan
                row={"case":case,"method":variant,"initial_tangential_speed_m_per_s":summary["tangential_speed"],
                     "time_s":t0,"accepted_interval_s":h,"normal_impulse_Ns":float(impulse[2]),
                     "tangential_impulse_x_Ns":float(impulse[0]),"tangential_impulse_y_Ns":float(impulse[1]),
                     "post_tangential_velocity_x_m_per_s":float(post[0]),"post_tangential_velocity_y_m_per_s":float(post[1]),
                     "effective_tangential_mass_kg":rho,"normalized_projection_residual":residual,
                     "sliding_vector_error":sliding_error,
                     "friction_cone_relative_excess":max(0.,float(np.linalg.norm(impulse[:2])/capacity-1)),
                     "post_tangential_power_W":float(force[:2]@post[:2])}
                selected.append(row)
                intervals.append(row)
                if variant=="shifted":
                    trace=trace_by_time.get(round(t0,10))
                    if trace is None: raise RuntimeError("Missing native trace for active interval")
                    f=np.asarray(trace["cResult"])
                    native_G=-np.asarray(trace["invM"]).reshape(3,3)
                    native_post=np.asarray(trace["v0"])-trace["h"]*np.asarray(trace["b"])+trace["h"]*native_G@f
                    checks.append({"case":case,"time_s":t0,
                        "tangential_speed_abs_difference":abs(np.linalg.norm(native_post[:2])-speed),
                        "normal_speed_abs_difference":abs(native_post[2]-post[2]),
                        "power_abs_difference":abs(f[:2]@native_post[:2]-force[:2]@post[:2]),
                        "tangential_curvature_abs_difference":abs(.5*np.trace(native_G[:2,:2])-.5*np.trace(G[:2,:2])),
                        "force_norm_abs_difference":abs(np.linalg.norm(f)-np.linalg.norm(force)),
                        "accepted_interval_abs_difference":abs(trace["h"]-h)})
            if not selected: raise RuntimeError("No active intervals")
            summaries.append({"case":case,"method":variant,"initial_tangential_speed_m_per_s":summary["tangential_speed"],
                "active_intervals":len(selected),"max_projection_residual":max(r["normalized_projection_residual"] for r in selected),
                "rms_projection_residual":float(np.sqrt(np.mean([r["normalized_projection_residual"]**2 for r in selected]))),
                "positive_post_power_intervals":sum(r["post_tangential_power_W"]>1e-8 for r in selected),
                "maximum_cone_excess":max(r["friction_cone_relative_excess"] for r in selected)})
    write_csv(out/"dissipation_intervals.csv",intervals)
    write_csv(out/"dissipation_summary.csv",sorted(summaries,key=lambda r:(r["method"],r["initial_tangential_speed_m_per_s"])))
    write_csv(out/"native_reconstruction_checks.csv",checks)
    validation={key:float(max(r[key] for r in checks)) for key in checks[0] if key not in ("case","time_s")}
    passed=validation["tangential_speed_abs_difference"]<1e-8 and validation["normal_speed_abs_difference"]<1e-8 and validation["power_abs_difference"]<1e-7
    sanity={
        "correct_sliding":disk_residual(np.array([-1.,0.]),np.array([1.,0.]),1.,1.),
        "wrong_sliding_magnitude":disk_residual(np.array([-.5,0.]),np.array([1.,0.]),1.,1.),
        "perpendicular_same_magnitude":disk_residual(np.array([0.,1.]),np.array([1.,0.]),1.,1.),
        "valid_nonzero_sticking":disk_residual(np.array([.4,.3]),np.zeros(2),1.,1.),
        "sticking_cone_violation":disk_residual(np.array([1.2,0.]),np.zeros(2),1.,1.)}
    assert sanity["correct_sliding"]==sanity["valid_nonzero_sticking"]==0
    assert all(sanity[key]>0 for key in ("wrong_sliding_magnitude","perpendicular_same_magnitude","sticking_cone_violation"))
    report={"input":str(args.input.resolve()),"raw_source_hashes":hashes,"parameters":parameters,
        "definition":"norm(lambda_T - project_disk(lambda_T - rho*vhat_T_plus)) / (mu*lambda_N)",
        "rho_definition":"inverse of mean tangential diagonal of the sphere contact inverse-mass operator",
        "velocity_definition":"v_contact + h*(a_free + G*f), same frozen rigid-sphere operator for both methods",
        "first_contact_episode_only":True,"rotation_locked":False,"native_reconstruction_max_errors":validation,
        "native_reconstruction_passed":passed,"native_trace_frames_checked":len(checks),"no_data_smoothing":True}
    report["metric_sanity_checks"]=sanity
    for variant in ("shifted","velocity_regularized"):
        part=[r for r in intervals if r["method"]==variant]
        report[variant]={"cases":sum(r["method"]==variant for r in summaries),"intervals":len(part),
            "max_projection_residual":max(r["normalized_projection_residual"] for r in part),
            "positive_post_power_intervals":sum(r["post_tangential_power_W"]>1e-8 for r in part),
            "max_cone_excess":max(r["friction_cone_relative_excess"] for r in part)}
    (out/"dissipation_report.json").write_text(json.dumps(report,indent=2),encoding="utf-8")
    print(json.dumps({key:value for key,value in report.items() if key!="raw_source_hashes"},indent=2))
    if not passed: raise RuntimeError("Reconstruction does not match native solver traces; do not publish this metric")


if __name__=="__main__":
    main()
