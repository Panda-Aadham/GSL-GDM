# VGR PMFS House01 Wrapper

This package adds a standalone ROS2 launch wrapper to run the `PMFS` gas source localization method on `House01` from `vgr_dataset`.

It does not modify or remove any existing files. Instead, it reuses:

- the prerecorded VGR gas playback data
- the existing GSL `PMFS` action server
- the existing simulated gas and wind sensors
- the existing `BasicSim` and `Nav2` setup pattern used by the current GSL repository

Example:

```bash
ros2 launch vgr_pmfs_house01_env main_simbot_launch.py simulation:=1,3-2,4_fast
```

Available simulations:

- `1,3-2,4_fast`
- `1,3-2,4_slow`
- `2,4-1_fast`
- `2,4-1_slow`
