# Sire Python build workflow

## Interpreter rule

On this checkout, build, install, import and run Sire with exactly:

```powershell
D:\code\sire\.venv\Scripts\python.exe
```

The previous import failure occurred because Sire was built with the agent's
Python rather than this venv. `native_build.py` now rejects that interpreter.
Its build directory contains both the Python ABI/platform tag and a hash of the
venv path, so another environment cannot silently reuse this CMake cache.

## First install

Run once, or after changing Python packaging metadata:

```powershell
Set-Location D:\code\sire\python
& D:\code\sire\.venv\Scripts\python.exe -m pip install -e . -v --no-build-isolation --no-deps
```

This also performs an initial native Release build. `--no-build-isolation`
means pip uses tools already installed in the project venv; it does not skip
native compilation. Omit `--no-deps` if Python runtime dependencies have not
been installed yet.

## Fast daily native build

After editing C++ code, bypass pip metadata and run:

```powershell
Set-Location D:\code\sire
& .\.venv\Scripts\python.exe .\python\build_native.py --jobs 8
```

The command builds only target `sire` and its required dependencies, installs
only component `PythonRuntime`, copies only changed runtime files, then starts a
fresh process with the same interpreter to verify ABI, DLL loading and the RL
batch API. Python/XML-only edits require no native rebuild.

The default settings in `python/local.toml` build Release only, without demos,
tests or Tracy. Use fewer jobs if parallel C++ compilation exhausts RAM.

## Separate diagnostic builds

These profiles have independent CMake caches:

```powershell
& .\.venv\Scripts\python.exe .\python\build_native.py --profile debug --jobs 8
& .\.venv\Scripts\python.exe .\python\build_native.py --profile profile --jobs 8
```

Each successful command activates its result for the current Python ABI. Run
the Release command again before training after using Debug or profile.

Clarabel's existing runtime is reused during normal Sire builds. After changing
Clarabel Rust source, force its source build once:

```powershell
$env:BUILD_SIRE_CLARABEL_FORCE_SOURCE_BUILD = 'true'
& .\.venv\Scripts\python.exe .\python\build_native.py --jobs 8
Remove-Item Env:BUILD_SIRE_CLARABEL_FORCE_SOURCE_BUILD
```

For planning/debugging the build wrapper without compiling:

```powershell
& .\.venv\Scripts\python.exe .\python\build_native.py --dry-run
```

## Import verification

```powershell
& .\.venv\Scripts\python.exe -c "import sys, sire; print(sys.executable); print(sire.__file__); print(sire.tracyEnabled); print(sire.SireRLBatchStepper)"
```

Native import failures now preserve the original exception and report the
interpreter. Python 3.8+ on Windows restricts dependent-DLL lookup, so the
package explicitly registers the selected ABI runtime directory before loading
the extension.
