# Contact implementation and reproducible dependency setup

## Engine paths

- `ShiftedSpectralAdmmContactSolver` in `src/physics/contact/simple_admm_contact_solver.cpp`
  solves the trajectory-referenced frictional NCP. Its default mode is `single_point`.
- `SimpleAdmmContactSolver` provides the zero-normal-reference ablation.
- `AdmmContactSolver` supplies the shared analytical normal trajectory,
  numerical separation-root evaluation and accepted-interval integration.
- `SmoothLinearContactSolver` and task `go2_smooth_linear` retain the training
  comparator's velocity-regularized friction in the shared ADMM pipeline.
  The impact/incline scripts explicitly select their recorded comparator;
  these names are not interchangeable solver implementations.
- `initial_penetration_policy="eliminate"` records initial detected penetration;
  `retain_bounded` remains available for compatibility.

Additional opt-in HeightField patch/prism modes are development features, not
the paper's one-representative-point-per-geometry-pair experiment mode. Their
geometry generators and tests are included because the current engine builds
them. Do not substitute these modes into the archived paper configuration.

## Published dependency revisions

The contact fixes are committed to the corresponding forks, and Sire pins
those exact revisions through its submodules:

| Dependency | Repository / branch | Pinned revision |
| --- | --- | --- |
| ARIS | [leitianjian/aris](https://github.com/leitianjian/aris), `sire-contact-solver` | `25b37bbaaeb9543a339fbf58dd9afb7c5248a008` |
| COAL/hpp-fcl | [leitianjian/hpp-fcl](https://github.com/leitianjian/hpp-fcl), `devel` | `6f1194fe417dfae8143dc1d5d0737a5b8508f3be` |

ARIS fixes subsystem-local body indexing and clears reused contact
inertia/acceleration arrays before assembly. COAL adds an opt-in HeightField
contact-patch query and distinct surface-witness insertion. The latter changes
the request structure, so rebuild COAL and Sire together; do not mix old DLLs
with the updated headers.

From the repository root, using the project's Python:

```powershell
git submodule sync --recursive
git submodule update --init --recursive
& ./.venv/Scripts/python.exe scripts/prepare_contact_dependencies.py
```

A fresh clone needs no manual patch application. The checker verifies the
pinned commits and per-file SHA256. Rebuild/install ARIS and COAL into your
configured dependency prefixes, then follow
[`python/BUILDING.md`](../python/BUILDING.md) for Sire.

The files in `third_party/patches/` record the changes from the historical
bases for traceability. The checker's optional `--apply` remains available
only for those exact older bases; it is unnecessary at the current pinned
revisions. The checker does not reset files, fetch code, or build/install a
library, and refuses unknown commits or source edits.

## Paper entry point and validation

See [`demo/demo_paper2/paper_current/README.md`](../demo/demo_paper2/paper_current/README.md).
That directory is the sole maintained paper plotting entry point. It includes
experiment/plotting/analysis code and saved configurations. Research inputs,
images, analysis results, full logs and checkpoints remain in the local archive
and are excluded from Git. Restoring these inputs is required to replot the
recorded manuscript results; no data download is implied.

With `BUILD_TEST=ON`, build and run `single_point_contact_test` and
`height_field_witness_test`. The former covers shifted/unshifted spectral ADMM,
initial-depth handling, warm starts and separation roots; the latter checks
HeightField geometry. `test/physics/contact/test_smooth_shared_kernel.py`
additionally exercises the Python-bound velocity-regularized kernel using the
project interpreter. `run.py simulate contact-smoke` is a small complete Sire
scene check; it does not replace a full paper sweep or a training run.

The separation search returns the separating side of a bracket. Its existing
width tolerance is `min(1e-8, h*1e-4)` seconds, with a floating-point floor;
analytical root tests compare to this numerical contract rather than requiring
picosecond accuracy from a 10 ns search. This release documents that contract
and aligns the old assertions without changing the event-search algorithm.
On MSVC, UTF-8 source decoding is explicitly enabled so source comments do not
depend on the Windows system code page.

## CUDA normal trajectory replay

Configure with `-DSIRE_ENABLE_CUDA=ON -DBUILD_TEST=ON` and build the
`cuda_contact_trajectory_test` target. This target evaluates the shared ADMM affine
normal trajectory in batches at supplied times. It uses the same FP64
Padé scaling, pivoted solve, and squaring steps as the CPU implementation.
It also executes the exponential separation scan, candidate confirmation,
and Brent refinement on the GPU. This standalone replay supplies the
spectral rate and frequency computed by the CPU eigensolver so those search
stages can be checked independently.

```sh
ctest --test-dir build/cuda -R '^cuda_contact_trajectory_test$' --output-on-failure
```

Set `SIRE_SOLVER_TRACE_DIR` when running a CPU scene with an ADMM-family contact solver.
New `frame_*.json` files include `normal_trajectory` with the matrix, bias,
initial state, and CPU state at the accepted timestep `h`. Pass one or more
of these frame paths to `cuda_contact_trajectory_test` to replay and compare
them. For traces marked `separation_method: exponential`, it also compares
the computed separating bracket endpoint against the recorded separation
time. The executable batches frames with the same number of contacts.

## CUDA shifted spectral ADMM replay

With the same CMake options, build `cuda_spectral_admm_test` and run it with
`ctest --test-dir <build-dir> -R '^cuda_spectral_admm_test$' --output-on-failure`.
The FP64 CUDA solver accepts one padded batch with a separate active contact
count and timestep per problem. It uses the shifted normal target and the
three residuals from the Sire single-point solver. Inputs may include the
prepared primal and dual warm starts. The kernel also returns the solved dual.

New traces from the CPU shifted single-point solver include a
`shifted_spectral_admm` record. Pass these frame paths to
`cuda_spectral_admm_test` to compare force, dual, and convergence against the
recorded CPU values. Warm-start matching across physical steps is performed
by CPU Sire during trace recording.

The same test exercises `launchShiftedContactStep` on a nonblocking CUDA
stream. It computes the normal spectrum, separation time, accepted timestep,
normal target velocity, and shifted ADMM result from device buffers. Mixed
contact counts share one padded batch. Replay compares the spectrum and
separation with CPU Sire. If the accepted times differ within the search
tolerance, it recomputes the CPU force reference at the GPU accepted time
with the same prepared warm starts.

`launchBodyContactStep` starts one stage earlier, with the two bodies' inverse
inertia blocks and free accelerations in each active contact frame. The GPU
forms Sire's signed relative inertia and acceleration, applies the material
stiffness scaling to the normal system, and runs the same contact solve in
one stream. The input depth is the already-corrected depth of a preserved
pair. New CPU ADMM single-point traces include `body_contact_inputs`. The
replay compares the assembled inertia, acceleration, normal coefficients,
and initial depths and rates before comparing the step and force results.

## CUDA contact history replay

`launchFrameContacts` builds the same contact axes as ARIS from the
detected world point and normal. It uses body spatial velocities to
evaluate the velocity of B relative to A at the contact point and resolves
that velocity in the contact frame.

`launchPrepareContactHistory` keeps penetration baselines and warm starts
in device memory for each environment. It filters single-point detections
in input order and publishes the selected indices, material values, and
initial forces and duals. It matches forces by geometry pair and rotates the
previous world forces into the new contact frames. A dimension-preserving
solve reuses the duals by index, as in CPU Sire.

`launchBodyContactStep` can gather raw body-resolved inputs using the selected
indices. `launchCommitContactHistory` stores the solved forces and duals for
the next event, and sums the contact wrenches for each body in world
coordinates about the world origin. Run the stages on the same CUDA
stream with preallocated buffers. Environments can clear their own history
on reset.

With `SIRE_SOLVER_TRACE_DIR` set, CPU ADMM single-point runs also write
`single_point_contact_history.jsonl`. The file lists every detection event,
including events with no active contacts, and links to the individual solver
frames. Build and run `cuda_contact_history_test` to check independent history
and partial resets across 1,024 environments. Pass the trace directory to
the executable to also replay sequential Go2 events. The live replay
supplies collision witnesses, body velocities, and body-resolved dynamics
from the CPU trace. It compares the GPU contact frames and relative
velocities with CPU Sire before checking the history and solve results.

```sh
ctest --test-dir build/cuda -R '^cuda_contact_history_test$' --output-on-failure
build/cuda/test/cuda_contact_history_test /path/to/cpu/trajectory
```

## CUDA flat-ground collision

`loadFlatGroundModel` reads the initialized Sire collision filter, local
shape poses, dimensions, and material pairs. The Go2 scene has one anchored
ground box and 25 enabled ground pairs. The scene metadata is shared across
environments. `launchFlatGroundCollision` reads the per-environment body poses
on the device and sends compact world witnesses and counts directly to
`launchFrameContacts` on the same stream.

The CUDA query ports the pinned COAL narrow phase for the three shape
pairs used in the scene. Box–sphere uses the analytic distance calculation.
Box–box and box–cylinder use the ported GJK/EPA support functions, simplex
tests, convergence settings, and witness reconstruction. All three
queries use the finite ground box, including its sides and edges. GPU
collision queries use eight CUDA lanes per environment and emit contacts
in geometry-ID order. CPU order follows broad-phase traversal.

Build `cuda_flat_ground_collision_test` with CUDA and tests enabled.
The CTest case checks synthetic collisions across 1,024 environments
against COAL and checks contacts from several Go2 poses against CPU Sire.
With `SIRE_SOLVER_TRACE_DIR` set,
the history file also records body poses. Replay generates collisions
and contact frames from those poses and velocities, then compares the
results with CPU Sire by geometry pair.

```sh
build_dir="$PWD/build/cuda"
ctest --test-dir "$build_dir" -R '^cuda_flat_ground_collision_test$' --output-on-failure
cd demo/demo_python/sirePaperDogRL
"$build_dir/test/cuda_flat_ground_collision_test" go2_rai_foot.xml /path/to/cpu/trajectory
```

See the [CUDA implementation guide](cuda_implementation.md) for the Go2 rollout and training workflow.
