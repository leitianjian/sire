# Contact implementation and reproducible dependency setup

## Engine paths

- `ShiftedSpectralAdmmContactSolver` in `src/physics/contact/simple_admm_contact_solver.cpp`
  solves the trajectory-referenced frictional NCP. Its default mode is `single_point`.
- `SimpleAdmmContactSolver` provides the zero-normal-reference ablation.
- V5 supplies the shared analytical normal trajectory, numerical separation-root
  evaluation and accepted-interval integration pipeline.
- `SmoothLinearContactSolver` and task `go2_smooth_linear` retain the training
  comparator's velocity-regularized friction in the shared V5 pipeline.
  The impact/incline scripts explicitly select their recorded comparator;
  these names are not interchangeable solver implementations.
- `initial_penetration_policy="eliminate"` records initial detected penetration;
  `retain_bounded` remains available for compatibility.

Additional opt-in HeightField patch/prism modes are development features, not
the paper's one-representative-point-per-geometry-pair experiment mode. Their
geometry generators and tests are included because the current engine builds
them. Do not substitute these modes into the archived paper configuration.

## Dependency patches (required before building this revision)

The Sire checkout depends on two changes that are not in the pinned upstream
submodule commits. They are distributed as patches instead of publishing
unreviewed changes to another repository:

| Dependency | Pinned base | Patch |
| --- | --- | --- |
| ARIS | `df71a39310a3b505459ff218f71fecbe92f5b6ce` | `third_party/patches/aris.patch` |
| COAL/hpp-fcl | `797131dd2f7553130b16aacda921a6b466fee606` | `third_party/patches/hpp-fcl.patch` |

The ARIS patch fixes subsystem-local body indexing and clears reused contact
inertia/acceleration arrays before assembly. The COAL patch adds an opt-in
HeightField contact-patch query and distinct-witness insertion. It changes the
request structure, so rebuild COAL and Sire together; do not mix old DLLs with
the patched headers.

From the repository root, using the project's Python:

```powershell
git submodule update --init --recursive
& ./.venv/Scripts/python.exe scripts/prepare_contact_dependencies.py --apply
```

The script checks submodule commits and per-file SHA256, accepts an already
patched checkout, and refuses mixed or unknown edits. It does not reset files,
fetch code, or build/install a library. Rebuild/install ARIS and COAL into your
configured dependency prefixes after applying the patches, then follow
[`python/BUILDING.md`](../python/BUILDING.md) for Sire. Dependency source changes
remain visible as dirty submodules after applying the reproducible patches.

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
