# Segmented Go2 hardware protocol

This directory contains the segmented deployment entry point for small test
areas. Its parser, logging path, and onboard estimator have been tested on the
Go2 computer at `192.168.0.6`. The locomotion protocol itself has not yet been
run on the motors with this entry point.

Copy these files beside the existing robot deployment files:

- `deploy_go2_segmented.py`
- `go2_flat_segmented_v1.yaml`
- existing `deploy_go2.py`
- existing `deploy_go2_paper.py`
- existing `config.yaml` or method-specific deployment YAML
- exported TorchScript policy

Example invocation:

```bash
python3 deploy_go2_segmented.py eth0 \
  --config deploy_sire_1200.yaml \
  --policy policies/sire_seeds_1_4_20260924/sire_seed_1_model_1200_jit.pt \
  --protocol go2_flat_segmented_v1.yaml \
  --segment forward_030 \
  --repeat 1 \
  --policy-seed 1 \
  --checkpoint-iteration 1200 \
  --output-dir segmented_logs \
  --notes "flat indoor floor; start marker A"
```

## Operator sequence

1. Put the robot inside the marked start box with the required orientation and
   keep hold of the physical emergency stop.
2. Start the process. The base controller releases `mcf` and enters its
   zero-torque START wait.
3. Press START. The robot moves to and holds the default pose.
4. Press A once at the base controller's `Holding default position` prompt.
   With the recommended `--foot-contact-source none`, the script skips the
   telemetry preflight. Other contact-source modes run a one-second zero-command
   policy check before continuing.
5. When `Release A to arm the trial` appears, release A fully.
6. When `Hold A to start the selected segment` appears, press and continuously
   hold A. This second press starts the clock and both experiment logs.
7. Keep A held for the complete segment. Releasing A invalidates the trial and
   performs a short command ramp to zero. The trial cannot resume.
8. During the armed wait, experiment, and recovery loops, SELECT requests an
   immediate damping stop. During the inherited zero-torque/default-pose
   initialization, use the physical emergency stop or interrupt the process;
   those inherited loops do not poll SELECT.
9. When the segment completes or is invalidated, both the policy-rate JSONL and
   high-rate `.lowstate.jsonl` close.
10. Release A fully. Recovery is armed only after this release is observed.
11. In recovery mode, hold A and use the joystick to move the robot. Release A
   to command zero velocity. Recovery samples are not written to the experiment
   JSONL.
12. Press SELECT to finish recovery and send a damping command.

Run each segment as a separate process/trial. Reposition the robot before the
next run. Do not concatenate interrupted segments and call them one trial.

## Required review before robot use

- Confirm that Unitree A, START, and SELECT map to the indices in the installed
  SDK and remote firmware.
- Measure clear venue dimensions and shorten command holds to retain physical
  stopping margins. The provided protocol uses `operator_deadman` because the
  earlier robot logs contained no valid `SportModeState`. A physical spotter
  must continuously hold A and release it before the marked stopping boundary.
- Use `boundary_mode: onboard_position` with a measured `distance_limit_m` only
  after confirming that `SportModeState.position` is fresh and meaningful. The
  script refuses that mode without both conditions.
- Set `--max-motor-temp`, `--max-abs-roll`, and `--max-abs-pitch` from the
  approved robot safety limits. They intentionally have no guessed defaults.
- Confirm whether `SportModeState.position` remains available after sport mode
  is released. Segments with a distance limit refuse to start without a fresh
  position estimate.
- Bench-test protocol parsing and logging with motors unloaded before standing
  or locomotion tests.
- Use a physical spotter and an unobstructed emergency-stop path.
- Review the printed telemetry preflight. On the previously logged robot, raw
  force index 2 and all four estimated foot-force channels are unusable, so the
  expected `auto` result is `contact_source=none`. Do not use contact timing or
  slip metrics unless all four channels pass.
- Unitree's public message definition does not label the four foot-force array
  indices. Keep contact labels disabled until the order has been verified on
  the actual robot. Only then use `--confirm-foot-order-fr-fl-rr-rl` if the
  observed order is FR, FL, RR, RL.

## Trial validity

Only metadata with all of the following is eligible for analysis:

- `status == "complete"`
- `eligible_for_analysis == true`
- `exit_reason == "protocol_completed"`
- `samples_dropped == 0`
- `raw_lowstate_samples_dropped == 0`
- no motor loss or safety abort
- the planned segment duration completed

Manual recovery is deliberately excluded from the experiment log. Metadata
records whether recovery mode was entered and its duration.

## First motor test

Run `stand` before any translational or yaw segment, with one person holding the
remote and another watching the terminal:

```bash
cd ~/deploy_sire
python3 deploy_go2_segmented.py eth0 \
  --config deploy_sire_1200.yaml \
  --policy policies/sire_seeds_1_4_20260924/sire_seed_1_model_1200_jit.pt \
  --protocol go2_flat_segmented_v1.yaml \
  --segment stand \
  --repeat 1 \
  --policy-seed 1 \
  --checkpoint-iteration 1200 \
  --foot-contact-source none \
  --output-dir segmented_logs \
  --notes "stand pilot; flat indoor floor"
```

The four current policies are stored as
`policies/sire_seeds_1_4_20260924/sire_seed_N_model_1200_jit.pt`. Set
`--policy-seed N` to the same `N`; the metadata must match the selected file.
Their checkpoint and JIT hashes are recorded in
`sire_seeds_1_4_20260924_manifest.json`. Do not pass
`--confirm-foot-order-fr-fl-rr-rl`: the live probe confirmed one broken raw
force channel and did not establish the array-to-leg order.

`--config` is intentionally required. Always pass `deploy_sire_1200.yaml` for
these four policies. The older generic deployment YAML uses different PD gains
and action clipping and is not training-consistent.

Only after the stand trial completes normally should the same command be rerun
with `--segment forward_030`, `backward_025`, `lateral_left_020`,
`lateral_right_020`, `yaw_ccw_040`, or `yaw_cw_040`. Reposition the robot at the
same start mark before every process.

## Continuous operator collection (local v3, not yet uploaded)

Use `run_operator_collection.sh SEED REPEAT SESSION_ID [NOTES]`. The initial
START/stand-up sequence is retained; once standing, joystick control and full
logging begin directly. A is not required. Neutral sticks command zero.
SELECT requests damping and saves the session as a normal operator finish.

`go2_operator_advisory_v3.yaml` supplies advisory labeling only. A 0.05
command deadband avoids labeling neutral-stick noise; it does not modify the
command. Any larger positive x is forward, regardless of speed. Multiple active
axes receive combined labels such as forward+cw. All samples, brief segments,
transitions and poor tracking remain in the full policy and 500 Hz raw logs.

The console shows current command direction, cumulative pure/mixed class
durations and underrepresented pure directions. The default 3 s per direction
is an accumulated reminder target across short segments, not an acceptance
gate. Seed orders affect reminder priority only. Reaching all targets never
ends recording. Missing directions never invalidate the session.

There is no automatic timeout or automatic coverage finish. The operator ends
the recording with SELECT when the robot is secured for damping. Metadata
separates recording integrity, normal/interrupted termination, and advisory
motion summaries. Fault/interruption logs remain available for offline analysis.

Offline analysis operates on the complete policy JSONL, including older attempts
whose metadata had been marked invalid by the obsolete coverage gate:

```bash
python3 operator_motion_summary.py SESSION.jsonl \
  --config go2_operator_advisory_v3.yaml --seed 1 \
  --output SESSION.motion-summary.json
```

The report contains every contiguous command segment and class totals.
Segments of at least 2 s receive a descriptive long-command flag; shorter
segments remain included. This flag does not claim stable physical response.
Run the onboard estimator on the complete raw log before aligning it to segments;
do not concatenate isolated raw windows and integrate across their time gaps.

Use the complete command/IMU/response timeline with synchronized video for
deployment feasibility. Mixed turns and pure yaw remain distinct. Unequal
operator commands and repeated windows are not independent training seeds.
`extract_operator_coverage.py` is retained only for historical coverage-window
logs; it is not the analysis path for this continuous collection mode.

## Frozen paper protocol

Use `go2_flat_segmented_v1.yaml` for low-speed commissioning. The first seed-1
paper-v1 session is retained as an excluded pilot: it showed yaw under-tracking
and high body/action variation during the intermediate zero-command dwell.
The seed-1 v2 session is also retained as an excluded commissioning run. Its
0.5 s ramps, 1 s holds, and 1 s reversal changed commands before a stable gait
was established. Collect the formal dataset with the frozen
`go2_paper_protocol_v3.yaml`:

| Segment | Command | Duration | Maximum ideal excursion |
|---|---:|---:|---:|
| `paper_v3_stand_08` | zero | 8 s | zero |
| `paper_v3_fore_aft_050` | `+0.5` to `-0.5` m/s | 16 s | 1.75 m |
| `paper_v3_lateral_pair_050` | `+0.5` to `-0.5` m/s | 16 s | 1.75 m |
| `paper_v3_yaw_pair_080` | `+0.8` to `-0.8` rad/s | 16 s | 2.80 rad |

The paired commands have zero ideal net displacement or rotation. V3 uses 1.5 s
start/stop ramps, 2 s holds, a 3 s continuous sign reversal, a 2 s baseline,
and a 4 s post-stop window. Before every standing-recovery stage, the terminal
prints the next segment, peak command, maximum ideal excursion, required clear
direction, and complete phase schedule. Use an unobstructed 3 m by 3 m area and
place the robot toward the appropriate edge based on this preview.

For example, run seed 1's fore-aft trial with:

```bash
python3 deploy_go2_segmented.py eth0 \
  --config deploy_sire_1200.yaml \
  --policy policies/sire_seeds_1_4_20260924/sire_seed_1_model_1200_jit.pt \
  --protocol go2_paper_protocol_v3.yaml \
  --segment paper_v3_fore_aft_050 \
  --repeat 1 \
  --policy-seed 1 \
  --checkpoint-iteration 1200 \
  --foot-contact-source none \
  --output-dir segmented_logs \
  --notes "paper protocol; marked flat indoor floor"
```

On the robot, prefer the checked wrapper for formal acquisition:

```bash
cd ~/deploy_sire
./run_paper_trial.sh 1 paper_v3_fore_aft_050 1 \
  "surface A; start mark A; operator LTJ"
```

The arguments are `SEED SEGMENT REPEAT [NOTES]`. The wrapper rejects an unknown
seed, segment, nonpositive repeat, or missing policy/config/protocol file, then
binds the seed number to the corresponding policy path automatically.

To collect every segment and repeat for one seed with one terminal command, use:

```bash
cd ~/deploy_sire
./run_paper_seed_session.sh 1 3 20260928_seed1 \
  "surface A; start mark A; operator LTJ"
```

The arguments are `SEED REPEATS SESSION_ID [NOTES]`. Between trials, use A and
the joystick to restore the robot position while the same controller and policy
continue holding the robot upright. Release A and press START to arm the next
trial, then follow the release-A/hold-A prompt. Do not press SELECT between
trials: SELECT ends the complete session with damping. Each trial still receives
separate JSONL and metadata files. If a trial is invalid, the session stops;
rerunning the same command resumes the session and skips earlier valid trials in
that session directory.

The same standing-recovery stage is provided once immediately after the initial
START/default-pose transition and before the first trial. Hold A and use the
joystick to align the robot with the marked start pose, release A to stop, then
visually confirm a safe four-foot stance and press START with A released to
begin zero-command policy settling.

The session does not force the joints back to training `default_angles` between
trials. Those angles are the policy action center and simulation reset pose, not
the loaded physical equilibrium. The commissioning data showed that forcing
that target temporarily removed the policy's nonzero support action and caused
a visible head dip before the policy recovered. Instead, the same policy remains
active and converges to its own zero-command standing equilibrium.

After START, the persistent session performs an unlogged pre-trial settling
check before it permits the A press that starts acquisition.
It holds a zero velocity command for at least 2 s and requires the final 0.5 s
to remain within all of the following limits:

- absolute roll and pitch no greater than 0.20 rad;
- body angular-rate norm no greater than 0.25 rad/s;
- joint-velocity RMS no greater than 0.25 rad/s.

If the state does not stabilize within 5 s, the script returns to standing
recovery. Because no valid four-foot force source is available, the operator
must also visually confirm that all feet are planted before pressing START.

Unitree's motor `lost` field behaves as a cumulative counter on the checked
robot. The session stores its startup vector as a baseline and aborts only when
one controlled motor increases by 10 or more during that session. The baseline,
threshold, and any abort snapshot are retained in metadata.

For a quantitative hardware dataset, run all four segments for seeds 1--4 with
at least three valid repeats per seed/segment. Five repeats are preferable if
time permits. Organize acquisition in repeat blocks and vary seed order between
blocks so battery state and motor temperature are not confounded with one seed.
An interrupted or invalid trial is rerun with the same seed, segment, and repeat
number; invalid files remain retained for the audit trail.

## Per-trial processing

Each successful run creates three files sharing one timestamped stem:

- `.metadata.json`: immutable policy/config/protocol hashes and trial validity;
- `.jsonl`: 50 Hz policy, command, target, joint, IMU, timing, and power data;
- `.lowstate.jsonl`: approximately 500 Hz raw motor, IMU, force, tick, and clock
  data.

Run the audit and onboard estimator after every trial:

```bash
python3 audit_go2_telemetry.py segmented_logs/TRIAL.jsonl \
  --output segmented_logs/TRIAL.telemetry-audit.json
python3 estimate_go2_kinematics.py segmented_logs/TRIAL.lowstate.jsonl \
  --output segmented_logs/TRIAL.onboard-kinematics.csv
```

Accept a trial only when metadata says `eligible_for_analysis: true`, both drop
counts are zero, `raw_lowstate_recorder_error` is null,
`logger_close_errors` is empty, the estimator reports `sequence_gaps: 0`, and
the estimator's valid fraction and contact residual are retained for comparison.
The live stationary check produced 100% valid estimates, a median rate near
502 Hz, a mean contact residual of 0.0077 m/s, and about 3.4 mm planar drift in
five seconds. Treat these as a system check, not universal acceptance thresholds
for moving trials.

## Supported measurements

The files are sufficient for policy execution success, joint tracking, action
and target histories, attitude, control/inference timing, estimated motor
torque, power, and short-horizon onboard base velocity/displacement comparisons.
Use the same estimator settings and protocol for every compared policy.

They are not sufficient for calibrated contact-force comparison, reliable
four-foot contact timing, air time, or an independent foot-slip measurement.
The default contact set is inferred from kinematics, and zero stance-foot world
velocity is already an estimator constraint. Describe derived base motion as an
`onboard kinematic estimate`, not measured ground truth.

## Onboard kinematic estimate

Each trial writes a high-rate `TRIAL.lowstate.jsonl` directly from the DDS
`LowState` callback. It contains all 20 motor states, IMU orientation, angular
velocity and acceleration, both four-channel foot-force arrays, robot tick,
host timestamps, and protocol phase. The logger is asynchronous; any dropped
raw sample makes the trial ineligible.

After copying a completed trial from the robot, estimate base motion with only
the recorded onboard sensors:

```bash
python3 estimate_go2_kinematics.py \
  segmented_logs/TRIAL.lowstate.jsonl \
  --output segmented_logs/TRIAL.onboard-kinematics.csv
```

The default `kinematic_lowest` contact selector does not depend on the broken
`foot_force_est` field or an assumed force-array order. It obtains body-frame
foot position and velocity from the Go2 geometry and joint encoders, uses IMU
orientation/angular velocity, and applies stance-foot zero-velocity constraints
to estimate base velocity. It integrates horizontal velocity for relative
displacement and estimates height from stance-foot geometry on a locally flat
floor. The companion `.summary.json` reports valid coverage and contact-fit
residuals.

This is an onboard dead-reckoning estimate. Use it consistently for all methods
to compare short, identical segments. Position accumulates drift, the
kinematic contact selector is heuristic, and stance-foot world velocity is a
constraint of the estimator, so the result is not independent ground truth for
absolute trajectory accuracy or foot slip. Preserve the raw log so contact
selection and estimator thresholds can be reprocessed identically for every
trial.

## Immediate telemetry audit

Before the first experiment, run the read-only probe once while Unitree sport
mode is still active:

```bash
python3 probe_go2_telemetry.py eth0 --mode-label sport --duration 5 \
  --output probe_sport.json
```

Run it again from a second terminal while low-level deployment is active:

```bash
python3 probe_go2_telemetry.py eth0 --mode-label lowlevel --duration 5 \
  --output probe_lowlevel.json
```

If the high-frequency sport topic has no callbacks, repeat with
`--sport-topic rt/lf/sportmodestate`; Unitree publishes different frequency
variants on some platforms/SDK setups.

The probe sends no command and changes no robot mode. Comparing the two files
distinguishes a parser/topic problem from a field that the firmware does not
populate in low-level mode. If `foot_force_est` is zero in this direct probe,
changing the deployment contact threshold will not fix it.

Validate the full raw-log and offline-estimator path without starting a policy:

```bash
python3 capture_go2_lowstate.py eth0 --duration 5 \
  --output diagnostic.lowstate.jsonl
python3 estimate_go2_kinematics.py diagnostic.lowstate.jsonl \
  --output diagnostic.onboard-kinematics.csv
```

The capture utility subscribes only to `LowState`; it publishes no motor
command and does not change the active Unitree service.

After every trial, run:

```bash
python3 audit_go2_telemetry.py segmented_logs/TRIAL.jsonl \
  --output segmented_logs/TRIAL.telemetry-audit.json
```

The audit reports field completeness, foot-force channel health,
`SportModeState` availability, timing, motor loss, and which paper metrics the
log can support. Do not infer all-feet contact, air time, or slip when the audit
marks those metrics unavailable.
