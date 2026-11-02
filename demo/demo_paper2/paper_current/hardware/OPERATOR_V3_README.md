# Continuous manual recording v3

Run one seed/repeat with a new session ID:

```bash
cd ~/deploy_sire
./run_operator_collection.sh 1 1 operator_v3_seed1_repeat1 "surface A; operator LTJ"
```

Press the initial START to leave zero torque and move to the default pose.
After this transition, joystick control and logging start immediately. No A
press or additional START is required. Neutral sticks command zero. SELECT
ends the recording normally and requests damping; secure the robot first.

All policy and raw LowState samples are saved. The console labels command
directions, including mixed motion, and reports accumulated seconds and suggested
directions to supplement. A 0.05 deadband affects labels only, never joystick
commands. The 3 s accumulated reminder target is advisory, not an acceptance
gate. There is no minimum segment duration, coverage stop, or automatic timeout.

Missing directions do not invalidate a recording. Metadata separates normal
operator finish, safety interruption and logging integrity. Motion labels refer
to commanded direction, not verified physical displacement or steady motion.

Files are in `operator_hardware_logs/SESSION_ID/`. Offline processing:

```bash
python3 operator_motion_summary.py PATH_TO_SESSION.jsonl \
  --config go2_operator_advisory_v3.yaml --seed 1 \
  --output session.motion-summary.json
```

This reports every contiguous segment, including short and repeated segments.
Long segments are marked descriptively, never selected by tracking quality.
Estimate onboard motion on the complete raw stream before matching segments.
Training gains, action scales, policy weights and yaw-rate semantics are unchanged.
