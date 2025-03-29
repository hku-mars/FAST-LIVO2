## Usage Instructions
The files in this folder follow the naming convention: `xxx_xx_*`

- `xxx`: Scene name (e.g., eee, nya, sbs).
- `xx`: Sequence number (e.g., 01, 02, 03) for the same scene.
- `xxx_xx_prism.txt`: Estimated poses from FAST-LIVO2 converted to the PRISM coordinate system.
- `xxx_xx_gt.txt`: Ground truth trajectories converted to the TUM format.

To compute RMSE between ground truth and estimated trajectories, run:
```bash
evo_ape tum eee_01_gt.txt eee_01_prism.txt -a --plot --plot_mode xyz
```
Results below:
| Scene  | Sequence | RMSE (cm) |
|--------|----------|-----------|
| eee    | 01       | 2.71      |
| eee    | 02       | 2.11      |
| eee    | 03       | 2.61      |
| nya    | 01       | 3.56      |
| nya    | 02       | 3.39      |
| nya    | 03       | 3.52      |
| sbs    | 01       | 2.34      |
| sbs    | 02       | 2.83      |
| sbs    | 03       | 3.11      |

**Note:** The provided files are results (*_prism.txt, *_gt.txt) from my runs. To compare your own FAST-LIVO2 pose outputs with ground truth, you must first run `evaluate_viral.py` to:
- Convert the official [leica_pose.csv](https://github.com/ntu-aris/viral_eval) file to TUM format.
- Transform SLAM trajectories from the **IMU frame** to the **PRISM coordinate system**.