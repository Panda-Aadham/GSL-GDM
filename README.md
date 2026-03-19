# GSL-GDM

This repository contains several robotics, simulation, and embedded projects collected in one place. The main dissertation-related work is centred around the ROS 2 workspace in `ros2_ws/`, while some other folders are separate side projects.

## Repository structure

### Dissertation-related work

- `ros2_ws/`
  Main ROS 2 workspace used for the dissertation experiments.
  This is the most important folder for simulation, localisation, mapping, and evaluation.

- `ros2_ws/src/gsl_evaluation/`
  ROS 2 package for running gas source localisation (GSL) evaluation experiments on VGR scenarios.
  This package contains the launch files for running methods such as PMFS, GrGSL, ParticleFilter, Spiral, SurgeCast, and SurgeSpiral.

- `ros2_ws/results/`
  Stores experiment outputs.
  This includes:
  - raw per-method scenario CSV files in folders such as `PMFS/`, `GrGSL/`, and `ParticleFilter/`
  - generated summary CSV files in `results/summaries/`
  - generated plots in `results/plots/`

- `ros2_ws/plot_results.py`
  Script for plotting aggregated comparison figures from the raw result folders.

- `ros2_ws/results/generate_summary_files.py`
  Script for generating summary CSV files from the raw result folders.

### Other repository components

- `esp32-firmware/`
  Embedded firmware-related code for ESP32-based work.

- `python_model/`
  Separate project unrelated to the dissertation.
  This folder is used for training a multilayer perceptron (MLP) to predict air quality index values for deployment on an ESP32.

## Main dissertation workflow

The usual dissertation workflow in this repository is:

1. Build the ROS 2 workspace.
2. Launch a GSL evaluation scenario from `gsl_evaluation`.
3. Collect raw CSV outputs under `ros2_ws/results/<Method>/`.
4. Generate summary CSV files.
5. Plot aggregated comparison figures.

## ROS 2 evaluation package

The main package for VGR-based GSL evaluation is:

- `ros2_ws/src/gsl_evaluation/`

See the package README for package-specific launch commands and supported methods:

- `ros2_ws/src/gsl_evaluation/README.md`

Typical launch example:

```bash
cd ros2_ws
source install/setup.bash
ros2 launch gsl_evaluation main_simbot_launch.py simulation:=1,3-2,4_fast method:=PMFS
```

## Results and analysis

Raw result CSV files are stored per method in folders such as:

- `ros2_ws/results/PMFS/`
- `ros2_ws/results/GrGSL/`
- `ros2_ws/results/ParticleFilter/`
- `ros2_ws/results/Spiral/`
- `ros2_ws/results/SurgeCast/`
- `ros2_ws/results/SurgeSpiral/`

Generated outputs are organised as:

- `ros2_ws/results/summaries/`
  Aggregated CSV summary files.
- `ros2_ws/results/plots/`
  Generated comparison charts.

Example usage:

```bash
cd ros2_ws
python results/generate_summary_files.py results
python plot_results.py --results-dir results
```

## Environment

The current GADEN setup is known to work with the following environment:

- WSL 2 - Ubuntu 22.04
- ROS 2 Humble
- CMake 3.22.1
- GCC 10.5.0
- G++ 10.5.0
- NVIDIA Driver 552.22
- CUDA Toolkit 12.9.86

## Notes

- The dissertation work is mainly inside `ros2_ws/`.
- The `python_model/` folder is a separate ESP32 air-quality project and should not be treated as part of the dissertation pipeline.
- For package-specific usage, prefer the README inside the relevant package directory over this top-level overview.
