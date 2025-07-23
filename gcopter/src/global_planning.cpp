#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>

struct Config
{
    std::string mapTopic;
    std::string targetTopic;
    double dilateRadius;
    double voxelWidth;
    std::vector<double> mapBound;
    double timeoutRRT;
    double maxVelMag;
    double maxBdrMag;
    double maxTiltAngle;
    double minThrust;
    double maxThrust;
    double vehicleMass;
    double gravAcc;
    double horizDrag;
    double vertDrag;
    double parasDrag;
    double speedEps;
    double weightT;
    std::vector<double> chiVec;
    double smoothingEps;
    int integralIntervs;
    double relCostTol;


    Config(rclcpp::Node &node)
    {
        node.declare_parameter("MapTopic", std::string("/voxel_map"));
        node.declare_parameter("TargetTopic", std::string("/goal_pose"));
        node.declare_parameter("DilateRadius", 0.5);
        node.declare_parameter("VoxelWidth", 0.25);
        node.declare_parameter("MapBound", std::vector<double>{-25.0, 25.0, -25.0, 25.0, 0.0, 5.0});
        node.declare_parameter("TimeoutRRT", 0.02);
        node.declare_parameter("MaxVelMag", 4.0);
        node.declare_parameter("MaxBdrMag", 2.1);
        node.declare_parameter("MaxTiltAngle", 1.05);
        node.declare_parameter("MinThrust", 2.0);
        node.declare_parameter("MaxThrust", 12.0);
        node.declare_parameter("VehicleMass", 0.61);
        node.declare_parameter("GravAcc", 9.8);
        node.declare_parameter("HorizDrag", 0.70);
        node.declare_parameter("VertDrag", 0.80);
        node.declare_parameter("ParasDrag", 0.01);
        node.declare_parameter("SpeedEps", 0.0001);
        node.declare_parameter("WeightT", 20.0);
        node.declare_parameter("ChiVec", std::vector<double>{1.0e4, 1.0e4, 1.0e4, 1.0e4, 1.0e5});
        node.declare_parameter("SmoothingEps", 1.0e-2);
        node.declare_parameter("IntegralIntervs", 16);
        node.declare_parameter("RelCostTol", 1.0e-5);

        node.get_parameter("MapTopic", mapTopic);
        node.get_parameter("TargetTopic", targetTopic);
        node.get_parameter("DilateRadius", dilateRadius);
        node.get_parameter("VoxelWidth", voxelWidth);
        node.get_parameter("MapBound", mapBound);
        node.get_parameter("TimeoutRRT", timeoutRRT);
        node.get_parameter("MaxVelMag", maxVelMag);
        node.get_parameter("MaxBdrMag", maxBdrMag);
        node.get_parameter("MaxTiltAngle", maxTiltAngle);
        node.get_parameter("MinThrust", minThrust);
        node.get_parameter("MaxThrust", maxThrust);
        node.get_parameter("VehicleMass", vehicleMass);
        node.get_parameter("GravAcc", gravAcc);
        node.get_parameter("HorizDrag", horizDrag);
        node.get_parameter("VertDrag", vertDrag);
        node.get_parameter("ParasDrag", parasDrag);
        node.get_parameter("SpeedEps", speedEps);
        node.get_parameter("WeightT", weightT);
        node.get_parameter("ChiVec", chiVec);
        node.get_parameter("SmoothingEps", smoothingEps);
        node.get_parameter("IntegralIntervs", integralIntervs);
        node.get_parameter("RelCostTol", relCostTol);
        
    }
};

class GlobalPlanner : public rclcpp::Node
{
private:
    Config config;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr mapSub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr targetSub;
    Visualizer visualizer;
    voxel_map::VoxelMap voxelMap;
    std::vector<Eigen::Vector3d> startGoal;
    Trajectory<5> traj;
    double trajStamp;
    bool mapInitialized = false;

public:
    GlobalPlanner() : Node("global_planner_node"), config(*this), visualizer(*this)
    {
        Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                            (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                            (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);
        Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);

        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);

        mapSub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            config.mapTopic, 1,
            std::bind(&GlobalPlanner::mapCallBack, this, std::placeholders::_1));

        targetSub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            config.targetTopic, 1,
            std::bind(&GlobalPlanner::targetCallBack, this, std::placeholders::_1));
    }

    void mapCallBack(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!mapInitialized)
        {
            size_t cur = 0;
            size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            for (size_t i = 0; i < total; ++i)
            {
                cur = msg->point_step / sizeof(float) * i;
                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                    continue;

                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0], fdata[cur + 1], fdata[cur + 2]));
            }
            voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));
            mapInitialized = true;
        }
    }

    void targetCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (!mapInitialized)
            return;

        if (startGoal.size() >= 2)
            startGoal.clear();

        double zGoal = config.mapBound[4] + config.dilateRadius +
                       fabs(msg->pose.orientation.z) * (config.mapBound[5] - config.mapBound[4] - 2 * config.dilateRadius);
        Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, zGoal);

        if (voxelMap.query(goal) == 0)
        {
            visualizer.visualizeStartGoal(goal, 0.5, startGoal.size());
            startGoal.emplace_back(goal);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Infeasible Position Selected !!!");
        }

        plan();
    }

    void plan()
    {
        if (startGoal.size() != 2)
            return;

        std::vector<Eigen::Vector3d> route;
        sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0], startGoal[1], voxelMap.getOrigin(), voxelMap.getCorner(),
                                               &voxelMap, 0.01, route);

        std::vector<Eigen::MatrixX4d> hPolys;
        std::vector<Eigen::Vector3d> pc;
        voxelMap.getSurf(pc);
        sfc_gen::convexCover(route, pc, voxelMap.getOrigin(), voxelMap.getCorner(), 7.0, 3.0, hPolys);
        sfc_gen::shortCut(hPolys);

        if (route.size() <= 1)
            return;

        visualizer.visualizePolytope(hPolys);

        Eigen::Matrix3d iniState, finState;
        iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
        finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

        gcopter::GCOPTER_PolytopeSFC gcopter;

        Eigen::VectorXd magnitudeBounds(5), penaltyWeights(5), physicalParams(6);
        magnitudeBounds << config.maxVelMag, config.maxBdrMag, config.maxTiltAngle, config.minThrust, config.maxThrust;
        penaltyWeights << config.chiVec[0], config.chiVec[1], config.chiVec[2], config.chiVec[3], config.chiVec[4];
        physicalParams << config.vehicleMass, config.gravAcc, config.horizDrag, config.vertDrag, config.parasDrag, config.speedEps;

        int quadratureRes = config.integralIntervs;
        traj.clear();

        if (!gcopter.setup(config.weightT, iniState, finState, hPolys, INFINITY, config.smoothingEps, quadratureRes,
                           magnitudeBounds, penaltyWeights, physicalParams))
            return;

        if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
            return;

        if (traj.getPieceNum() > 0)
        {
            trajStamp = this->now().seconds();
            visualizer.visualize(traj, route);
        }
    }

    void process()
    {
        if (traj.getPieceNum() <= 0)
            return;

        double delta = this->now().seconds() - trajStamp;
        if (delta < 0.0 || delta > traj.getTotalDuration())
            return;

        double thr;
        Eigen::Vector4d quat;
        Eigen::Vector3d omg;

        Eigen::VectorXd physicalParams(6);
        physicalParams << config.vehicleMass, config.gravAcc, config.horizDrag, config.vertDrag, config.parasDrag, config.speedEps;

        flatness::FlatnessMap flatmap;
        flatmap.reset(physicalParams(0), physicalParams(1), physicalParams(2), physicalParams(3), physicalParams(4), physicalParams(5));

        flatmap.forward(traj.getVel(delta), traj.getAcc(delta), traj.getJer(delta), 0.0, 0.0, thr, quat, omg);

        double speed = traj.getVel(delta).norm();
        double bodyratemag = omg.norm();
        double tiltangle = acos(1.0 - 2.0 * (quat(1) * quat(1) + quat(2) * quat(2)));

        std_msgs::msg::Float64 speedMsg, thrMsg, tiltMsg, bdrMsg;
        speedMsg.data = speed;
        thrMsg.data = thr;
        tiltMsg.data = tiltangle;
        bdrMsg.data = bodyratemag;

        visualizer.speedPub->publish(speedMsg);
        visualizer.thrPub->publish(thrMsg);
        visualizer.tiltPub->publish(tiltMsg);
        visualizer.bdrPub->publish(bdrMsg);

        visualizer.visualizeSphere(traj.getPos(delta), config.dilateRadius);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GlobalPlanner>();

    rclcpp::Rate rate(1000);
    while (rclcpp::ok())
    {
        node->process();
        rclcpp::spin_some(node);
        rate.sleep();
    }
    rclcpp::shutdown();
    return 0;
}
