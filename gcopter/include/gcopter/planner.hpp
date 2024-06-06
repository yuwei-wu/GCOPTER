#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include "plan_env/sdf_map.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>



namespace gcopter
{


struct Config
{
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
    double dilateRadius;
    double TimeoutRRT;

    Config(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("MaxVelMag", maxVelMag);
        nh_priv.getParam("MaxBdrMag", maxBdrMag);
        nh_priv.getParam("MaxTiltAngle", maxTiltAngle);
        nh_priv.getParam("MinThrust", minThrust);
        nh_priv.getParam("MaxThrust", maxThrust);
        nh_priv.getParam("VehicleMass", vehicleMass);
        nh_priv.getParam("GravAcc", gravAcc);
        nh_priv.getParam("HorizDrag", horizDrag);
        nh_priv.getParam("VertDrag", vertDrag);
        nh_priv.getParam("ParasDrag", parasDrag);
        nh_priv.getParam("SpeedEps", speedEps);
        nh_priv.getParam("WeightT", weightT);
        nh_priv.getParam("ChiVec", chiVec);
        nh_priv.getParam("SmoothingEps", smoothingEps);
        nh_priv.getParam("IntegralIntervs", integralIntervs);
        nh_priv.getParam("RelCostTol", relCostTol);
        nh_priv.getParam("DilateRadius", dilateRadius);
        nh_priv.getParam("TimeoutRRT", TimeoutRRT);
    }
};


class GcopterPlanner
{
public:
    voxel_map::VoxelMap voxelMap;

private:
    Config config;

    Trajectory<5> traj;

    Eigen::VectorXd magnitudeBounds, penaltyWeights, physicalParams;
    int quadratureRes;

    Visualizer visualizer;


public:
    GcopterPlanner(ros::NodeHandle& nh, const std::string& frame_id)
    : config(Config(nh)),
      visualizer(nh, frame_id)
    {

        std::cout << "config.integralIntervs " << config.integralIntervs  << std::endl;
 

        /***set up params***/
        quadratureRes = config.integralIntervs;

        magnitudeBounds.resize(5);
        penaltyWeights.resize(5);
        physicalParams.resize(6);

        magnitudeBounds(0) = config.maxVelMag;
        magnitudeBounds(1) = config.maxBdrMag;
        magnitudeBounds(2) = config.maxTiltAngle;
        magnitudeBounds(3) = config.minThrust;
        magnitudeBounds(4) = config.maxThrust;
        penaltyWeights(0) = (config.chiVec)[0];
        penaltyWeights(1) = (config.chiVec)[1];
        penaltyWeights(2) = (config.chiVec)[2];
        penaltyWeights(3) = (config.chiVec)[3];
        penaltyWeights(4) = (config.chiVec)[4];
        physicalParams(0) = config.vehicleMass;
        physicalParams(1) = config.gravAcc;
        physicalParams(2) = config.horizDrag;
        physicalParams(3) = config.vertDrag;
        physicalParams(4) = config.parasDrag;
        physicalParams(5) = config.speedEps;
    }

    inline Trajectory<5> getTraj()
    {
        return traj;
    }
    

    inline Eigen::VectorXd getPhysicalParams()
    {
        return physicalParams;
    }


    inline void setMap(std::shared_ptr<bg_planner::SDFMap>& map)
    {
        /// step one: convert map
        const Eigen::Vector3i xyz(map->mp_->map_voxel_num_(0),
                                  map->mp_->map_voxel_num_(1),
                                  map->mp_->map_voxel_num_(2));

        const Eigen::Vector3d offset(map->mp_->map_origin_(0), map->mp_->map_origin_(1), map->mp_->map_origin_(2));
        voxelMap = voxel_map::VoxelMap(xyz, offset, map->mp_->resolution_);
        for (int x = map->mp_->box_min_(0); x < map->mp_->box_max_(0); ++x)
            for (int y = map->mp_->box_min_(1); y < map->mp_->box_max_(1); ++y)
                for (int z = map->mp_->box_min_(2); z < map->mp_->box_max_(2); ++z) {
                    if (map->getOccupancy(Eigen::Vector3i(x, y, z)) == bg_planner::SDFMap::OCCUPIED)
                    {
                        Eigen::Vector3i idx(x, y, z);
                        voxelMap.setOccupied(idx);
                    }
                }
        // for (int m = 0; m < map->mp_->map_voxel_num_(2); m++) {
        //     for (int j = 0; j < map->mp_->map_voxel_num_(1); j++) {
        //         for (int i = 0; i < map->mp_->map_voxel_num_(0); i++) {
        //             Eigen::Vector3i idx(i, j, m);
        //             int vl = map->md_->occupancy_buffer_[map.dim.y * map.dim.x * m +  map.dim.x * j + i ] ;
        //             if(vl == 100)
        //             {   
        //                 voxelMap.setOccupied(idx);
        //             }
        //         }
        //     }
        // }
        voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));
    }

    inline bool plan(const Eigen::Matrix3d& init, 
                     const Eigen::Matrix3d& fini, 
                     std::vector<Eigen::Vector3d>& route)
    {

        // route.clear(); #uncomment this line if you want to use RRT
        Eigen::Matrix3d startPVA = init;
        Eigen::Matrix3d endPVA = fini;

        if(route.size() <= 0)
        {
            ROS_ERROR("Got no route, planning with RRT...");
            sfc_gen::planPath<voxel_map::VoxelMap>(startPVA.col(0),
                                                   endPVA.col(0),
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, config.TimeoutRRT,
                                                   route);

            if (route.size() <= 0)
            {
                ROS_ERROR("Failed to plan RRT path!");
                return false;
            }                                    
        }

        auto plan_t1 = ros::Time::now();
        endPVA.col(0) = route.back();

        /* corridor generation */
        std::vector<Eigen::Vector3d> pc;
        voxelMap.getSurf(pc);

        std::vector<Eigen::MatrixX4d> hPolys;
        sfc_gen::getPolyConst(route,
                                pc,
                                voxelMap.getOrigin(),
                                voxelMap.getCorner(),
                                3.0,
                                2.0,
                                hPolys);
        sfc_gen::shortCut(hPolys);

        ROS_INFO("Got %d convex hulls.", (int)hPolys.size());
        ROS_WARN("[GCOPTER] SFC time: %lf", (ros::Time::now()-plan_t1).toSec());

        auto time2 = ros::Time::now();
        gcopter::GCOPTER_PolytopeSFC gcopter;

        traj.clear();

        if (!gcopter.setup(config.weightT,
                            startPVA, endPVA,
                            hPolys, INFINITY,
                            config.smoothingEps,
                            quadratureRes,
                            magnitudeBounds,
                            penaltyWeights,
                            physicalParams))
        {   
            ROS_ERROR("Failed to setup GCOPTER!");
            return false;
        }


        if (std::isinf(gcopter.optimize(traj, config.relCostTol)))
        {
            ROS_ERROR("Failed to optimize GCOPTER!");
            return false;
        }
        ROS_WARN("[GCOPTER] plan time: %lf", (ros::Time::now()-time2).toSec());

        ROS_INFO("[GCOPTER] Got a trajectory with %d pieces.", traj.getPieceNum());

        if (traj.getPieceNum() > 0)
        {
            
            // std::cout << "success! " << std::endl;

            visualizer.visualizeStartGoal(endPVA.col(0), 0.5, 1);
            visualizer.visualize(traj, route);
            visualizer.visualizePolytope(hPolys);

            return true;
        }
        return false;
    }


public:
  typedef std::unique_ptr<GcopterPlanner> Ptr;

};

}   // namespace gcopter_planner
