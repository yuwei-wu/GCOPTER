# GCOPTER

This is a revised version for benchmarking used by the Kumar Lab. 

## Branches Overview

* **main**: ROS1 implementation
* **ros2**: ROS2 implementation ([ros2 branch](https://github.com/yuwei-wu/GCOPTER/tree/ros2))
* **noros**: Pure C++ implementation without ROS ([noros branch](https://github.com/yuwei-wu/GCOPTER/tree/noros))

Each branch contains detailed usage instructions specific to its environment.

## Useage

### Prerequisites

Make sure you have ROS1 installed if you are using the `main` branch. Additionally, install the following dependencies:

```bash
sudo apt update
sudo apt install libpcl-dev libompl-dev
```

### Build and Run

```
mkdir -p test_ws/src
cd test_ws/src
git clone https://github.com/yuwei-wu/GCOPTER.git
cd ../
catkin build
source devel/setup.bash
roslaunch gcopter global_planning.launch
```

### ROS Topics Overview

```
/move_base_simple/goal (geometry_msgs/PoseStamped)
    - Uses 2D Nav Goal to send start and end poses
/voxel_map (sensor_msgs/PointCloud2)
    - Publishes a 3D voxel grid map of the environment

/visualizer/body_rate (geometry_msgs/Vector3Stamped)
/visualizer/edge (visualization_msgs/Marker)
/visualizer/mesh (visualization_msgs/Marker)
/visualizer/route (nav_msgs/Path)
/visualizer/speed (std_msgs/Float32)
/visualizer/spheres (visualization_msgs/Marker)
/visualizer/tilt_angle (std_msgs/Float32)
/visualizer/total_thrust (std_msgs/Float32)
/visualizer/trajectory (nav_msgs/Path)
/visualizer/waypoints (visualization_msgs/Marker)
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
