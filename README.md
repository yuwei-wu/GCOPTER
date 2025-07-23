# GCOPTER ROS2

This is a revised version for benchmarking used by Kumar Lab. 

## Usage

### Prerequisites

Before running the code, make sure you have ROS 2 (Humble) installed. You may also need to install some additional dependencies:

```bash
sudo apt update
sudo apt install ros-humble-pcl-conversions ros-humble-pcl-ros
sudo apt install libpcl-dev libompl-dev
```

### Build and Run

```bash
mkdir -p test_ws/src
cd test_ws/src
git clone -b ros2 https://github.com/yuwei-wu/GCOPTER.git
cd ../
colcon build
source install/setup.bash
ros2 launch gcopter global_planning.launch.py
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
