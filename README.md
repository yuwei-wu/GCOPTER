# GCOPTER

This is a revised, pure C++ version of GCOPTER for global trajectory planning and corridor generation.

---

## Build & Run Instructions

```bash
git clone -b noros https://github.com/yuwei-wu/GCOPTER.git
cd GCOPTER/gcopter
mkdir build && cd build
cmake ..
make
cd ..
./build/global_planning_demo input.yaml
```

---

## Input Configuration (input.yaml)

Example `input.yaml`:

```yaml
config_path: "config/global_planning.yaml"
map_path: "config/obs.txt"

start:
  x: -1.0
  y: -1.0
  z: 1.5

goal:
  x: 8.0
  y: 0.0
  z: 1.0
```

* `config_path`: Path to the global planning configuration file.
* `map_path`: Path to the environment map (obstacles), either in `.txt` format or converted from `.pcd`.

---

## Changing Input Maps

You can use either `.txt` files or `.pcd` point cloud files as input maps.

To convert a `.pcd` file to `.txt` format, run:

```bash
python scripts/pcd_to_txt.py --input config/your_map.pcd --output config/obs.txt
```

---

## Visualization

After running the planner, you can visualize the trajectory, corridors, and obstacles using Python scripts.

```bash
cd scripts/
python visualizer.py
```

Ensure you have the required Python packages installed:

```bash
pip install open3d numpy matplotlib scipy plotly
```

---

## Folder Structure

```
.
├── gcopter
│   ├── build/            # Build directory (created after cmake)
│   ├── CMakeLists.txt
│   ├── config            # Configuration files and map files (.yaml, .txt, .pcd)
│   │   ├── global_planning.yaml
│   │   ├── obs.txt
│   │   └── pt100001.pcd
│   ├── include
│   │   └── gcopter/
│   ├── input.yaml        # Example input configuration
│   ├── LICENSE
│   ├── scripts           # Python scripts for visualization and utilities
│   │   ├── corridor_utils.py
│   │   ├── pcd_to_txt.py
│   │   └── visualizer.py
│   ├── src
│   │   └── global_planning.cpp
│   └── trajectory.txt
└── README.md

```





__GCOPTER__ is an efficient and versatile multicopter trajectory optimizer built upon a novel sparse trajectory representation named [__MINCO__](https://arxiv.org/pdf/2103.00190.pdf). __User-defined state-input constraints__ for dynamics involving [__nonlinear drag effects__](https://github.com/ZJU-FAST-Lab/GCOPTER/blob/main/misc/flatness.pdf) are supported.


## About

If our repo helps your academic projects, please cite our paper. Thank you!

__Author__: [Zhepei Wang](https://zhepeiwang.github.io) and [Fei Gao](https://scholar.google.com/citations?hl=en&user=4RObDv0AAAAJ) from [ZJU FAST Lab](http://zju-fast.com).

__Paper__: [Geometrically Constrained Trajectory Optimization for Multicopters](https://arxiv.org/abs/2103.00190), Zhepei Wang, Xin Zhou, Chao Xu, and Fei Gao, <em>[IEEE Transactions on Robotics](https://doi.org/10.1109/TRO.2022.3160022)</em> (__T-RO__), Regular Paper.
```
@article{WANG2022GCOPTER,
    title={Geometrically Constrained Trajectory Optimization for Multicopters}, 
    author={Wang, Zhepei and Zhou, Xin and Xu, Chao and Gao, Fei}, 
    journal={IEEE Transactions on Robotics}, 
    year={2022}, 
    volume={38}, 
    number={5}, 
    pages={3259-3278}, 
    doi={10.1109/TRO.2022.3160022}
}
```

## Powerful Submodules
- [SDLP: Seidel's Algorithm](https://github.com/ZJU-FAST-Lab/SDLP) on Linear-Complexity Linear Programming for Computational Geometry.
- [VertexEnumeration3D](https://github.com/ZJU-FAST-Lab/VertexEnumeration3D): Highly Efficient Vertex Enumeration for 3D Convex Polytopes (Outperforms [cddlib](https://github.com/cddlib/cddlib) in 3D).
- [LBFGS-Lite](https://github.com/ZJU-FAST-Lab/LBFGS-Lite): An Easy-to-Use Header-Only L-BFGS Solver.
