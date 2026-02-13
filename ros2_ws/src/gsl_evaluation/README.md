# VGR GSL House01 Wrapper

This package adds a standalone ROS2 launch wrapper to run the non-semantic GSL gas source localization methods on `House01` from `vgr_dataset`.

It does not modify or remove any existing files. Instead, it reuses:

- the prerecorded VGR gas playback data
- the existing GSL action server
- the existing simulated gas and wind sensors
- the existing `BasicSim` and `Nav2` setup pattern used by the current GSL repository

Example:

```bash
ros2 launch vgr_pmfs_house01_env main_simbot_launch.py simulation:=1,3-2,4_fast
```

Supported methods:

- `PMFS`
- `GrGSL`
- `ParticleFilter`
- `Spiral`
- `SurgeCast`
- `SurgeSpiral`

Example with an explicit method:

```bash
ros2 launch vgr_pmfs_house01_env main_simbot_launch.py simulation:=1,3-2,4_fast method:=GrGSL
```

Run the same scenario repeatedly without manually restarting:

```bash
ros2 launch vgr_pmfs_house01_env series_simbot_launch.py runs:=10 scenario:=House02 simulation:=3,5-1_fast method:=PMFS use_rviz:=False
```

The series launch starts one full `main_simbot_launch.py` child launch at a time and waits for it to exit before starting the next run. By default it uses `run_index:=auto`, so result and variance CSV rows continue with the next available run index. To force a fixed sequence, pass `start_run_index:=1`.

Run scenarios from `vgr_dataset/simulations.csv` without typing each house manually:

```bash
ros2 launch vgr_pmfs_house01_env series_simbot_launch.py scenario_set:=first simulation_speed:=fast runs:=3 method:=PMFS use_rviz:=False
```

Useful batch options:

- `scenario_set:=single` keeps the original one-scenario mode.
- `scenario_set:=first` runs the first block of 30 rows in `simulations.csv`.
- `scenario_set:=second` runs the second block of 30 rows in `simulations.csv`.
- `scenario_set:=all` runs both blocks.
- `simulation_speed:=fast`, `simulation_speed:=slow`, or `simulation_speed:=both` appends `_fast`, `_slow`, or both to each CSV simulation name.
- `houses:=House01,House02` or `houses:=1-5` limits the batch to selected houses.
- `house_start:=1 house_end:=10` limits the batch by numeric house range.

Semantic methods are not supported by this wrapper because they require extra ontology, room-mask, and detection inputs that are not present in the plain VGR House01 dataset.

Available simulations:

- `1,3-2,4_fast`
- `1,3-2,4_slow`
- `2,4-1_fast`
- `2,4-1_slow`
