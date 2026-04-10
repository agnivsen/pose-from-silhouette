# Globally Optimal Pose from Orthographic Silhouettes

Agniva Sengupta ¹ ², Dilara Kuş ¹ ², Jianning Li ², Stefan Zachow ²  

**¹** Zuse Institute Berlin  
**²** Freie Universität Berlin  


---

## Overview

This repository contains the official MATLAB implementation of:

**Globally Optimal Pose from Orthographic Silhouettes (CVPR 2026)**

(Selected as a **_highlight_** at IEEE/CVF CVPR 2026)

We address the problem of estimating the 3D pose of known objects in ℝ³ from their unoccluded orthographic silhouettes. The method exploits the continuity of silhouette area over rotation space to enable globally optimal pose recovery.

Key contributions include:
- Silhouette signatures modeled as response surfaces of silhouette area
- Efficient branching over the rotation search space
- Auxiliary ellipse-based shape cues for acceleration

The method estimates globally optimal pose from silhouettes alone, without requiring correspondences, and is applicable to general shapes.

---

## Repository Structure

```
├── Data/                  # Input data and precomputed shape signatures  
├── Docs/                  # Supplementary material  
├── PoseEstimation/        # Core algorithms  
├── Utils/                 # Utility functions  
├── runMe.m                # Main demo script  
└── README.md  
```

---

## Getting Started

### Requirements
- MATLAB (_tested on 23.2.0.2459199, R2023b Update 5_)


---

## Running the Demo

1. Clone the repository:
```
git clone https://git.zib.de/asengupta/pose-from-silhouette.git
cd pose-from-silhouette.
```


2. Open MATLAB and navigate to the project root directory.

3. Run the main script:
```
runMe.m
```

---

## What the Demo Does

The script performs the following steps:

1. Loads a 3D point cloud model (pelvic bone)
2. Generates a random 3D pose
3. Simulates an orthographic silhouette
4. Runs the proposed GlOptiPoS algorithm
5. Visualizes:
   - Ground truth pose
   - Estimated pose

Execution time may range from a few seconds to a few minutes depending on computational resources.

---

## Notes

- All required paths are automatically added in `runMe.m`
- The method assumes orthographic projection
- Shape signatures are precomputed and required for inference

### Using Custom Shapes

To use your own data:
- Replace `Data/pointcloud.txt` with your point cloud
- Provide a corresponding shape signature (`ShapeSignature.mat`)
- Code to compute shape signature shall be uploaded shortly

---

## Project Page

Additional details, results, and data are provided in:

https://agnivsen.github.io/pose-from-silhouette/

---

## Supplementary Material

Supplementary material to the main paper is in:

```
Docs/Supplementary.pdf
```

---

## Citation

If you find this work useful, please cite:

```
@article{sengupta2026pfs,
author    = {Sengupta, Agniva and Kuş, Dilara and Li, Jianning and Zachow, Stefan},
title     = {Globally Optimal Pose from Orthographic Silhouettes},
journal   = {Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition},
year      = {2026},
}
```

---


## License

This project is licensed under the  
**GNU AFFERO GENERAL PUBLIC LICENSE, Version 3 (19 November 2007)**.

See the `LICENSE` file for details.

---

## Contact

Agniva Sengupta  
Zuse Institute Berlin / Freie Universität Berlin  

sengupta@zib.de; agniva.sengupta@fu-berlin.de

For questions or issues, please open a GitLab issue or reach out directly.

---


