# ChangeLog of Sire project

Tracking the main change of very commit!

## 2025/08/18 mjcf2sire config file

* add support of mjcf file to sire config xml file (python)
* add wht parallel wheel mjcf 2 sire xml config file

## 2024/08/24 Simulation Play/Pause Functionalities

* Merge pull requests of !3.
  * Fix compile error in linux but not shown on MSVC.
  * Add Linux bash scripts to auto install Sire.
  * Add python script to auto build sire and third parties library.
  * Update dockerfile to build sire with new scripts.
* Fix bugs of change double inheritance of geometry to single inheritance in C++ reflection class properties
* Remove `dll_export` in `interface.cpp`
* Simulation Play/Pause Functionalities
  * Remove `cs.start()`, and `simulator.start()` from main.cpp.
  * Add `sim_play` and `sim_reset` command in xml and source code, the reset order is important in `sim_reset`.
  * Add `clear()` function to clear storage in `ContactPairManager`
  * Add `saveInitialModel()` and `resetInitialModel()` method in `PhysicsEngine` to enable physics engine save initial model and reset  when needed.
  * Add `isRunning`, `pause` and `reset` method in `simulation_loop` module.
  * Web: add `SimulationController` cell and `css` to play/pause in web
  * Web: websocket `onclose` `setTimeout` to reconnect and add `onError` to invoke `onClose()`
