# VGR KDM+V/W Wrapper

This package provides a standalone ROS2 wrapper to run KDM+V/W gas distribution mapping on the VGR dataset.

It is intentionally self-contained:

- it reuses the VGR gas playback and simulated sensor stack
- it carries a local minimal `gdm/` subtree for KDM+V/W instead of importing the existing `gdm` package
- it publishes KDM+V/W gas estimate and uncertainty maps as ROS occupancy grids
- it writes the latest estimate, uncertainty, and observation history to `results/KDM_VW`

Example:

```bash
ros2 launch kdm_vw main_simbot_launch.py simulation:=1,3-2,4_fast
```

Useful launch arguments:

- `scenario:=House01`
- `simulation:=1,3-2,4_fast`
- `use_rviz:=True`
- `use_nav2:=True`
- `use_autonomous_exploration:=true`
- `shutdown_on_complete:=true`
- `shutdown_delay:=2.0`
- `kdm_update_period:=1.0`
- `min_observation_spacing:=0.15`
- `min_observation_period:=0.25`

Published topics inside the robot namespace:

- `KDM_VW/gas_estimate`
- `KDM_VW/gas_uncertainty`

Expected sensor topics inside the robot namespace:

- `PID/Sensor_reading`
- `Anemometer/WindSensor_reading`
- `ground_truth`

When autonomous exploration is enabled, the launch can stop automatically after the
coverage explorer finishes all waypoints:

```bash
ros2 launch kdm_vw main_simbot_launch.py simulation:=1,3-2,4_fast shutdown_on_complete:=true
```

After a run, convert a predicted CSV map to a heatmap image with:

```bash
ros2 run kdm_vw csv_to_heatmap \
  results/KDM_VW/House01_1,3-2,4_fast_gas_estimate.csv \
  --occupancy-yaml /mnt/c/Users/ajamt/Desktop/Dissertation/src/gaden/install/vgr_dataset/share/vgr_dataset/scenarios/House01/occupancy.yaml
```

To export a ground-truth gas slice from the running GADEN player service and save both
CSV and PNG outputs:

```bash
ros2 run kdm_vw export_ground_truth_map \
  --occupancy-yaml /mnt/c/Users/ajamt/Desktop/Dissertation/src/gaden/install/vgr_dataset/share/vgr_dataset/scenarios/House01/occupancy.yaml \
  --z 0.5 \
  --output-csv results/KDM_VW/House01_1,3-2,4_fast_ground_truth.csv
```
