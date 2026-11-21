# VDF–VDC

VDF–VDC is a robot-world/hand–eye calibration method that explicitly models
robot-side and vision-side deviations and balances their contributions during
optimization. This repository contains a MATLAB calibration example and two
standalone C++ performance benchmarks.

| Directory | Entry point | Purpose |
| --- | --- | --- |
| [`VDF_VDC_MATLAB/`](https://github.com/closedloop-calib/VDF_VDC/tree/main/VDF_VDC_MATLAB) | [`run_real_experiment.m`](VDF_VDC_MATLAB/run_real_experiment.m) | Run VDF–VDC and the Li, Tabb rp1, and Tabb Zc2 baselines on the real-world example. |
| [`VDF_VDC_CPP/`](https://github.com/closedloop-calib/VDF_VDC/tree/main/VDF_VDC_CPP) | [`RunRealExperiment.cpp`](VDF_VDC_CPP/RunRealExperiment.cpp) | Benchmark four VDF–VDC implementations with different normal-equation solvers. |
| [`HALCON_Benchmark/`](https://github.com/closedloop-calib/VDF_VDC/tree/main/HALCON_Benchmark) | [`RunHalconBenchmark.cpp`](HALCON_Benchmark/RunHalconBenchmark.cpp) | Benchmark HALCON nonlinear hand–eye calibration separately. |

## Example data

All three programs use [`hand_eye_calibration_input.txt`](hand_eye_calibration_input.txt)
in the repository root. It contains the processed observations for 80 views
with 88 ChArUco corners per view. The data were collected with an FR3 robot and
an eye-in-hand monocular camera at 4384 × 3288 pixels. The target has 11 × 8
corners at 15 mm spacing.

The [Google Drive folder](https://drive.google.com/drive/folders/13A5Y0QUgKiESrSq64oKzl_8M-GJxd95-?usp=drive_link) contains the 80 original calibration images, their corresponding robot poses (`robotpose.xlsx`), the Python script `prepare_hand_eye_data.py`, and a `README.txt` with setup and usage instructions. The script generates [`hand_eye_calibration_input.txt`](hand_eye_calibration_input.txt) from the images and robot poses. The MATLAB and C++ examples read the processed file included in this repository directly.

## MATLAB calibration

The reference environment is MATLAB R2020b with Optimization Toolbox
(`lsqnonlin`, `optimoptions`) and the quaternion functions `rotm2quat`,
`quat2rotm`, and `quatnorm`. The rotation conversion functions are available through Robotics System
Toolbox, and `quatnorm` is available through Aerospace Toolbox.

Open `VDF_VDC_MATLAB/run_real_experiment.m` and click **Run**. From the
repository root, the equivalent MATLAB command is:

```matlab
run(fullfile('VDF_VDC_MATLAB', 'run_real_experiment.m'))
```

The script reports the VDF–VDC consistency ratio `q` and a table with solver
time in seconds, rotation consistency in degrees, translation consistency in
millimetres, and RMS reprojection error in pixels. The estimated transforms
`boardInBase` and `cameraInTcp` remain in the workspace in the table's method
order. The solver itself is available as `method.VDF_VDC(...)` in the
[`+method`](VDF_VDC_MATLAB/+method/) package.

## VDF–VDC C++ benchmark

1. In Visual Studio 2022, create an **Empty Project** for C++. Add
   `CalibrationData.cpp`, `VDF_VDC.cpp`, and `RunRealExperiment.cpp` from
   `VDF_VDC_CPP/` and keep their headers alongside them.
2. Select **Release / x64** and a C++14 or newer language standard. Set the linker
   subsystem to **Console**. Under **C/C++ → General → Additional Include
   Directories**, add the Eigen directory that contains the `Eigen/` folder.
3. Under **Debugging → Command Arguments**, enter the full path to the shared
   `hand_eye_calibration_input.txt` in double quotes. Build and start the
   **Local Windows Debugger**.

The program prints results for Dense–full, Sparse–full, Dense–Schur, and
Sparse–Schur. It reports calibration time including Li initialization and two
Windows process working-set metrics. Each implementation gets one warm-up run
and 100 timed runs. This project requires Eigen.

## HALCON C++ benchmark

1. Install HALCON 24.05 with its C++ interface and a valid HALCON license.
   Create a separate Visual Studio 2022 C++ **Empty Project** and add
   `HalconCalibrationData.cpp` and `RunHalconBenchmark.cpp` from
   `HALCON_Benchmark/`. Keep `HalconCalibrationData.h` alongside them.
2. Select **Release / x64** with the **Console** subsystem. For a standard
   HALCON installation, add `$(HALCONROOT)\include` and
   `$(HALCONROOT)\include\halconcpp` to the compiler include directories,
   `$(HALCONROOT)\lib\$(HALCONARCH)` to the linker library directories,
   and `halconcpp.lib` to the linker dependencies.
   The HALCON DLLs must be available at runtime. See the
   [HALCON Programmer's Guide](https://www.mvtec.com/fileadmin/Redaktion/mvtec.com/products/halcon/documentation/halcon/programmers_guide.pdf).
3. Enter the same quoted input path under **Debugging → Command Arguments**.
   Build and start the **Local Windows Debugger**.

The program prints the HALCON row. It uses the `nonlinear` and `error_pose` options and reports
the time of the calibration call over 100 timed runs after one warm-up.

## Data format and coordinate convention

| Text section | MATLAB field | Contents |
| --- | --- | --- |
| `KM` | `cameraMatrix` | 3 × 4 camera matrix `[K, zeros(3,1)]`, with intrinsics in pixels |
| `Worldpts` | `worldPoints` | Target points as planar XY coordinates in metres, with Z = 0 |
| `imgPts` | `imagePoints` | Undistorted pixel coordinates, grouped by view |
| `BasePoseIntcpCoords` | `baseInTcp` | One 4 × 4 robot-base-to-TCP transform per view |
| `gridPoseInCameraCoords` | `boardInCamera` | One 4 × 4 board-to-camera transform per view |

Use the same view order in all sections and the same corner order in the world
and image point lists. Pose translations are in metres. The processed robot
poses already account for the 220 mm tool offset.

In the code, transforms act on column homogeneous coordinates. For each view
the closure is `baseInTcp * boardInBase = cameraInTcp * boardInCamera`.
`boardInBase` maps the calibration board into the robot base frame, and
`cameraInTcp` maps the camera into the TCP frame.

## Acknowledgments

The Li and Tabb baselines and several MATLAB utilities are adapted from
[RWHE-Calib](https://github.com/ihtishamaliktk/RWHE-Calib). See
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for attributions and scope
and [`licenses/`](licenses/) for the accompanying license texts.
