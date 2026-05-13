# Copilot Instructions

## Project Overview

This project converts ROS bag sensor recordings into [Open Simulation Interface (OSI)](https://opensimulationinterface.github.io/osi-documentation/) protobuf data stored in spec-compliant [MCAP](https://mcap.dev/) multi-trace containers. It is part of the [REHEARSE](https://doi.org/10.1109/IV55156.2024.10588491) project for adverse weather sensor noise models.

## Architecture

There are two entry points:

1. **`config_creator.py`** — Interactive CLI that reads a rosbag and generates a `config.yaml` describing its sensors (type, resolution, mounting position, ROS topic).
2. **`dataset_creator.py`** — Main pipeline that reads a rosbag + config and exports an OSI MCAP trace file. Usage:
   ```
   python3 dataset_creator.py CONFIG targets.yaml save_path bag_location [--include-raw]
   ```

### Sensor Module System (`dataset_creator_OSI/modules/`)

Sensor classes are **dynamically loaded via `importlib`** based on the `sensor_type` field in `config.yaml`. The class name **must exactly match** the filename (e.g., class `Camera` in `Camera.py`, class `Lidar` in `Lidar.py`). This is case-sensitive.

Each sensor class follows the same interface:
- `MESSAGE_TYPE` — class attribute: the protobuf message class (e.g., `SensorData`).
- `__init__(self, config)` — receives sensor config dict from YAML.
- `export(self, msg)` — converts a single ROS message and **returns** a protobuf message (no file I/O).

OSI protobuf imports (`osi3`) are provided by the `asam-osi-utilities` package (installed via pip).

### MCAP Exporter (`dataset_creator_OSI/exporter.py`)

`MCAPExporter` wraps `mcap.writer.Writer` + the `MCAPChannel` helper from `asam-osi-utilities`:
- Writes all sensors to a single `.mcap` file with ASAM OSI spec-compliant metadata and schemas
- zstd compression at level 19 (monkey-patched), 32 MiB chunks
- Optional `--include-raw` flag embeds original ROS messages as non-OSI channels
- Context manager support (`with MCAPExporter(...) as exporter:`)

### Config YAML Structure

Top-level keys are sensor names. Each sensor has `sensor_type`, `topic`, and `mounting_position` (with `x`, `y`, `z`, `roll`, `pitch`, `yaw`). Camera sensors additionally have `image_resolution_horizontal` and `image_resolution_vertical`.

### Utilities (`dataset_creator_OSI/utils/`)

Helper scripts for cuboid projection, image undistortion, and vehicle-to-image coordinate transforms. These are used for visualization and annotation workflows, not by the main export pipeline.

## Key Conventions

- **Output format**: MCAP multi-trace container (`.mcap`) — all sensors in one file.
- **Coordinate conversion**: Lidar and Radar modules convert Cartesian point clouds to spherical coordinates (distance, elevation, azimuth) via `cart2sph()`.
- **Target definitions**: `targets.yaml` maps target names (car, bike, pedestrian, etc.) to physical dimensions. Targets are written as `GroundTruth` messages.
- **Dependencies**: Managed via `requirements.txt`. Key: `asam-osi-utilities` (provides `osi3` + MCAP writing), `zstandard` (for zstd level 19).
- When adding a new sensor type, create a new file in `dataset_creator_OSI/modules/` with a class whose name matches the filename exactly. The class must have a `MESSAGE_TYPE` attribute and an `export(msg)` method that returns a protobuf message.
