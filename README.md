# Monocular Vision-Based UAV Navigation for Orchard Monitoring

[![Project page](https://img.shields.io/badge/Project-Page-4A6A15?logo=googlechrome&logoColor=white)](https://ritabrata-chakraborty.github.io/Monocular-Vision-Based-UAV-Navigation-for-Orchard-Monitoring/) [![Paper](https://img.shields.io/badge/Paper-PDF-B31B1B?logo=googlescholar&logoColor=white)](docs/paper.pdf) [![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?logo=python&logoColor=white)](requirements.txt) [![Jetson](https://img.shields.io/badge/Jetson-Orin%20NX-76B900?logo=nvidia&logoColor=white)](environment-jetson5.yml) [![License](https://img.shields.io/badge/License-Non--Commercial-555555)](LICENSE.md)

Code for **"Path Planning of Low-Altitude UAV for Tree Canopy Tracking and Orchard Monitoring"** by Kaushal Kishore (CSIR-CEERI) and Ritabrata Chakraborty (BITS Pilani).

A quadcopter flying above an orchard locks onto one treeline and follows it using a single forward-facing camera. YOLOv11 segments the tree canopies, Gaussian Soft-NMS keeps neighbouring rows apart, a Kalman filter keeps the chosen row locked, and a line fitted through the canopy's centre points gives the roll and yaw commands. Everything runs onboard an NVIDIA Jetson Orin NX.

<p align="center"><img src="Demo.gif" alt="Tree Line Follower interface during live deployment" width="640"></p>
<p align="center"><b>Figure 1:</b> The Tree Line Follower Interface During Live Deployment</p>

## How It Works

Each frame goes through five stages:

1. **Segment** canopies with a YOLOv11 instance-segmentation model (`checkpoints/canopy11s640.pt`, 640 × 384 input).
2. **Separate rows** with Gaussian Soft-NMS (σ = 0.5), so treelines that converge near the image edges stay separate detections (`src/post_processing.py`).
3. **Lock the target row** with a Kalman filter. A detection is accepted as the same row only if its IoU with the previous box is at least `TRACK_IOU_THRESHOLD` (`src/tracking.py`).
4. **Extract the navigation line**: morphological opening, keep the largest connected component, then fit a line through the row-wise centre points of the mask (`src/post_processing.py`).
5. **Command the UAV**: the line's horizontal offset sets roll and its angle sets yaw, both exponentially smoothed; pitch is held constant (`src/control.py`).

## Results

<p align="center"><b>Table 1:</b> Instance Segmentation on the Olive-Orchard Dataset (290 Images, Pilani, India)</p>

<div align="center">

| Model | Box mAP@50 | Box mAP@50–95 | Mask mAP@50 | Mask mAP@50–95 |
|:---:|:---:|:---:|:---:|:---:|
| YOLACT++ | 69.69 | 42.58 | 72.63 | 54.36 |
| YolactEdge | 73.17 | 46.52 | 75.06 | 59.51 |
| **YOLOv11** | **95.5** | **78.5** | **96.5** | **71.4** |

</div>

With TensorRT on the Jetson Orin NX, the pipeline runs at 17 FPS. In field trials over the Pilani olive orchard, the quadcopter generated and followed navigation lines autonomously and stayed stable under light wind. Full details are in the [paper](docs/paper.pdf).

## Installation

**Desktop** (Python 3.8+, CUDA GPU recommended):

```bash
git clone https://github.com/Ritabrata-Chakraborty/Monocular-Vision-Based-UAV-Navigation-for-Orchard-Monitoring.git
cd Monocular-Vision-Based-UAV-Navigation-for-Orchard-Monitoring
pip install -r requirements.txt
```

**Jetson Orin** (JetPack 6, Python 3.10):

```bash
conda env create -f environment-jetson5.yml
conda activate tree-line-follower-jetson5
bash scripts/setup_jetson5_env.sh   # PyTorch from NVIDIA wheels, then pip dependencies
```

The trained model ships in `checkpoints/` as `.pt`, `.onnx` and a TensorRT `.engine`.

## Usage

```bash
# Interactive interface on a camera (recommended)
python scripts/run_inference.py --ui --video 0

# Interactive interface on a video file
python scripts/run_inference.py --ui --video path/to/video.mp4

# Headless pipeline on the default sample video (datasets/canopy/DJI_0009.MOV)
python scripts/run_inference.py
```

Options: `--video` (file path or camera index), `--model` (weights, default `checkpoints/canopy11s640.pt`), `--class` (class filter, default 1) and `--ui`.

**Interface controls:** all detected segments are shown until you **click one** to start tracking and control output. **S** stops output (raw feed only), **R** restarts and lets you pick another segment, **Q** quits.

### Training and Export

```bash
# Train on a local dataset
python scripts/train.py --data path/to/data.yaml --epochs 100 --val

# Or download the dataset from Roboflow first
python scripts/train.py --roboflow-key YOUR_KEY --epochs 100

# Export the checkpoint to TensorRT (FP16 by default)
python scripts/export_tensorrt.py
```

`train.py` copies `best.pt` to `checkpoints/canopy11s640.pt` unless `--no-copy` is given; `--export` exports it to TensorRT straight away. `scripts/benchmark_model.py` benchmarks model speed, and `scripts/plot_error_convergence.py` plots offset and angle error convergence for control tuning.

## Configuration

Paths, input size and tracking settings are in `src/config.py`:

<div align="center">

| Setting | Default | Meaning |
|:---:|:---:|:---:|
| `MODEL_CANOPY` | `checkpoints/canopy11s640.pt` | Segmentation weights |
| `DEFAULT_VIDEO` | `datasets/canopy/DJI_0009.MOV` | Video used when `--video` is omitted |
| `IMG_WIDTH`, `IMG_HEIGHT` | 640, 384 | Model input size |
| `TRACK_IOU_THRESHOLD` | 0.6 | Minimum IoU to keep tracking the same segment |

</div>

Control gains (smoothing factors and camera field of view) are arguments of `control_drone` in `src/control.py`.

## Repository Layout

```
├── src/
│   ├── config.py            # Paths, input size, tracking threshold
│   ├── inference.py         # Headless and interactive pipelines
│   ├── tracking.py          # Kalman filter and IoU-gated row lock
│   ├── post_processing.py   # Soft-NMS, mask refinement, line and spline fitting
│   └── control.py           # Roll and yaw commands with smoothing
├── scripts/                 # Inference, training, TensorRT export, benchmarking, Jetson setup
├── checkpoints/             # Trained canopy segmentation model (.pt, .onnx, .engine)
├── datasets/canopy/         # Sample orchard flight videos
├── runs/segment/            # Training and validation outputs
├── docs/                    # Project page and paper PDF
├── requirements.txt
└── environment-jetson5.yml
```

## Citation

```bibtex
@misc{kishore2026orchard,
  title  = {Path Planning of Low-Altitude UAV for Tree Canopy Tracking
            and Orchard Monitoring},
  author = {Kishore, Kaushal and Chakraborty, Ritabrata},
  note   = {Preprint},
  year   = {2026},
  url    = {https://ritabrata-chakraborty.github.io/Monocular-Vision-Based-UAV-Navigation-for-Orchard-Monitoring/}
}
```

## Authors

- **Kaushal Kishore**, CSIR-Central Electronics Engineering Research Institute (CSIR-CEERI), Pilani, India
- **Ritabrata Chakraborty**, Birla Institute of Technology and Science (BITS) Pilani, India (f20220843@pilani.bits-pilani.ac.in)

This work was supported by CSIR-CEERI, Pilani.

## License and Intellectual Property

The code is released for personal, educational and non-commercial use only; see [LICENSE.md](LICENSE.md). Use and modification may also be subject to applicable intellectual property rights. Please acknowledge the authors when you use or refer to this work.
