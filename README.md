# Dataset Creator OSI

This package converts ROS bag sensor recordings into [ASAM OSI](https://opensimulationinterface.github.io/osi-documentation/) protobuf data stored in spec-compliant [MCAP](https://mcap.dev/) multi-trace container files.

It is part of the [REHEARSE](https://doi.org/10.1109/IV55156.2024.10588491) project for adverse weather sensor noise models.

## Installation

```bash
pip install -r requirements.txt
```

The `asam-osi-utilities` package provides bundled `osi3` protobuf bindings — no separate OSI installation needed.

## Usage

### Step 1: Create sensor config

```bash
python3 config_creator.py path/to/bag.bag
```

This interactive tool reads a rosbag and generates a `config.yaml` describing its sensors.

### Step 2: Convert to MCAP

```bash
python3 dataset_creator.py config.yaml targets.yaml output_dir/ path/to/bag.bag
```

Optional flags:
- `--include-raw` — Embed original ROS messages alongside OSI channels in the MCAP file

### Output

A single `.mcap` file per bag containing:
- One OSI channel per sensor (Camera → `SensorData`, Lidar → `SensorData`, Radar → `SensorData`)
- Optional `GroundTruth` channel for static target data
- Optional raw ROS channels (with `--include-raw`)
- zstd compression (level 19), 32 MiB chunks
- Full ASAM OSI MCAP spec compliance (metadata, schemas, channel metadata)

## Sensor Types

Sensor class names must match filenames exactly (e.g., class `Camera` in `Camera.py`). Classes are loaded dynamically via `importlib` based on the `sensor_type` field in `config.yaml`.

## Config YAML Structure

Top-level keys are sensor names. Each sensor has `sensor_type`, `topic`, and `mounting_position`. Camera sensors additionally have `image_resolution_horizontal` and `image_resolution_vertical`.

Suggestion: check out the [parallel project](https://doi.org/10.5281/zenodo.1146014) for batch processing multiple bags.

# Citation

```
@INPROCEEDINGS{10588491,
  author={Poledna, Yuri and Drechsler, Maikol Funk and Donzella, Valentina and Chan, Pak Hung and Duthon, Pierre and Huber, Werner},
  booktitle={2024 IEEE Intelligent Vehicles Symposium (IV)}, 
  title={REHEARSE: adveRse wEatHEr datAset for sensoRy noiSe modEls}, 
  year={2024},
  volume={},
  number={},
  pages={2451-2457},
  keywords={Point cloud compression;Rain;Laser radar;Noise;Radar;Open systems;Sensor phenomena and characterization},
  doi={10.1109/IV55156.2024.10588491}}

}
```

# Acknoledgment
Co-funded by the European Union. Views and opinions expressed are however those of the author(s) only and do not necessarily reflect those of the European Union or European Climate, Infrastructure and Environment Executive Agency (CINEA). Neither the European Union nor the granting authority can be held responsible for them. Project grant no. 101069576.
UK participants in this project are co-funded by Innovate UK under contract no.10045139. 
Swiss participants in this project are co-funded by the Swiss State Secretariat for Education, Research and Innovation (SERI) under contract no. 22.00123.

