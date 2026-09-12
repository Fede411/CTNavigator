# CTNavigator

An open-source surgical navigation prototype for orthopaedic oncological surgery, built entirely from low-cost consumer hardware and an open software stack. It localizes a surgical instrument with a stereo pair of Microsoft Kinect v1 sensors and displays it in real time within the patient's CT inside [3D Slicer](https://www.slicer.org/).

This repository contains the code and STL files developed for the Master's Thesis *"Design, Development and Validation of an Open-Source Orthopaedic Oncological Surgical Navigator"* (Universidad Politécnica de Madrid, ETSIT).

> **Status — research prototype.** This system is a proof of concept. Its accuracy (target registration error of ≈9.5 mm) is **not** sufficient for clinical use, and it carries no medical-device certification. It is intended as a reproducible research and development platform, not for use on patients.

---

## How it works

The system has two components that communicate over the [OpenIGTLink](http://openigtlink.org/) protocol:

1. **Tracking backend (C#)** — acquires infrared frames from two Kinect v1 cameras, detects the retroreflective spheres, reconstructs their 3D positions by stereo triangulation, estimates the pose of the tracked instrument and reference marker, and streams the resulting transforms.
2. **Navigation module (Python / 3D Slicer)** — receives those transforms, registers physical space to the CT via paired fiducials, and renders the instrument in real time inside the patient's anatomy.

```
Kinect A ─┐
          ├─► C# tracking backend ──(OpenIGTLink TRANSFORM)──► 3D Slicer module ──► navigation view
Kinect B ─┘
```

---

## Requirements

### Hardware
- **2× Microsoft Kinect v1 (Model 1414)**, mounted as a stereo rig (a 3D-printable casing is provided).
- Each Kinect on its **own USB controller** (two Kinect v1 units saturate a single controller's bandwidth).
- A retroreflective-marker instrument and a reference marker (STL files provided in [`/models`](models)).

### Software
- **Windows** (the Kinect v1 SDK is Windows-only).
- **Kinect for Windows SDK v1.8**.
- **.NET Framework** (matching the backend build).
- **3D Slicer 5.x** with the **SlicerIGT** and **SlicerOpenIGTLink** extensions installed.
- *(Optional, for calibration)* MATLAB with the Stereo Camera Calibrator app, or OpenCV.

---

## Installation

### 1. Tracking backend
A precompiled executable is provided in `/backend`. On first run it creates two folders next to the executable — `output/` (characterization and mapping CSVs) and `calib/` (calibration captures) — so no path configuration is required.

### 2. 3D Slicer module
The navigation module is a Python scripted module (`CTNavigator.py`, `CTNavigator.ui`).

1. In 3D Slicer, go to **Edit → Application Settings → Modules**.
2. Under **Additional module paths**, add the folder containing `CTNavigator.py`.
3. Restart Slicer. The module will appear under the **CTNavigator** category.

The module launches the tracking backend (`KinectTracker.exe`) automatically; keep it in the same folder as `CTNavigator.py`. To use a different location, set the `CTNAVIGATOR_EXE` environment variable to its full path.
---

## Calibration

Example calibration parameters are included so the system runs out of the box, **but calibrating your own rig is practically obligatory** — the extrinsics depend on the exact physical placement of your two cameras.

Two independent routes are supported:
- **MATLAB Stereo Camera Calibrator** (reference).
- **Two-stage OpenCV pipeline** (fully open-source alternative): each camera is calibrated individually over its full field of view, then the stereo extrinsics are solved with the intrinsics held fixed.

Place the resulting parameters in the calibration folder used by the backend (see the absolute-path note above). A 3D-printable checkerboard pattern is provided in [`/models`](models).

---

## Usage

1. Connect and power both Kinects (each on its own USB controller).
2. Run the tracking backend; confirm both cameras stream infrared and the markers are detected.
3. In 3D Slicer, open the **CTNavigator** module and load the patient CT (and segmented model, if available).
4. Open the OpenIGTLink connection to the backend; the connection status is shown in the module.
5. **Register**: click each anatomical landmark on the CT, then capture the same points physically with the instrument tip (one capture per landmark, same order). The fiducial registration error is displayed and colour-coded as feedback.
6. Navigate: the instrument now appears in real time within the patient's imaging. A dedicated navigation window can be opened for the clinical view.

---

## Repository structure

| Path | Contents |
|------|----------|
| `SlicerModule/` | 3D Slicer Python module (`CTNavigator.py`, `CTNavigator.ui`, `Resources/`) and a copy of the tracking backend (`KinectTracker.exe`) |
| `KinectTracker/Kinect-PLUS/` | C# tracking backend source (`.cs`, `.csproj`, `packages.config`) |
| `Hardware/Carrito/` | STL files for the stereo-camera casing and mounts |
| `Hardware/Stereo Calibration/` | Example checkerboard capture sessions used for calibration |

> The `KinectTracker.exe` shipped alongside the module is the compiled backend; the C# sources under `KinectTracker/Kinect-PLUS/` build the same executable. NuGet packages (Emgu.CV, MathNet, etc.) are restored automatically on build.


---

## Known limitations

- **Not for clinical use.** Target registration error ≈9.5 mm, far above what orthopaedic oncological resection requires, and with no CE marking or regulatory approval.
- **Discontinued hardware.** The Kinect v1 is end-of-life and no longer manufactured; long-term reproducibility would benefit from migrating to a currently supported depth camera (e.g. Intel RealSense D455). The tracking pipeline is not tied to the Kinect — it only requires a calibrated stereo pair of infrared cameras.
- **Working volume and pose availability** depend on position and orientation; tracking degrades in peripheral and oblique regions.

---

## Citation

If you use this work, please cite the accompanying Master's Thesis:

> F. Sangineto Cotrina, *Design, Development and Validation of an Open-Source Orthopaedic Oncological Surgical Navigator*, Master's Thesis, Universidad Politécnica de Madrid (ETSIT), 2026.

---

## License

This project is released under the [BSD 3-Clause License](LICENSE). You are free to use, modify and redistribute it, including commercially, provided the copyright notice and disclaimer are retained and the authors' names are not used to endorse derived products without permission.

---

## Acknowledgements

Developed within the collaboration between the UPAM3D unit of *Hospital General Universitario Gregorio Marañón* and the Bioengineering and Telemedicine Group (GBT-UPM). Built on the open-source ecosystem of 3D Slicer, SlicerIGT and OpenIGTLink.
