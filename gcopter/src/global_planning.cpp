#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"


#include <gcopter/planner.hpp>



#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>



class PlannerServer
{
private:

    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::Subscriber targetSub;

    bool mapInitialized;
    std::vector<Eigen::Vector3d> startGoal;

    Trajectory<5> traj;
    double trajStamp;

    gcopter::GcopterPlanner::Ptr gcopter_planner;

    std::string mapTopic, targetTopic;

public:
    PlannerServer(ros::NodeHandle &nh_)
        : nh(nh_),
          mapInitialized(false)
    {
        ros::NodeHandle nh = ros::NodeHandle("~");
        nh.getParam("MapTopic", mapTopic);
        nh.getParam("TargetTopic", targetTopic);
        // nh.getParam("VoxelWidth", voxelWidth);
        // nh.getParam("MapBound", mapBound);
        // const Eigen::Vector3i xyz((mapBound[1] - mapBound[0]) / voxelWidth,
        //                           (mapBound[3] - mapBound[2]) / voxelWidth,
        //                           (mapBound[5] - mapBound[4]) / voxelWidth);

        // const Eigen::Vector3d offset(mapBound[0], mapBound[2], mapBound[4]);
        // voxelMap = voxel_map::VoxelMap(xyz, offset, voxelWidth);

        mapSub = nh.subscribe(mapTopic, 1, &PlannerServer::mapCallBack, this,
                              ros::TransportHints().tcpNoDelay());

        targetSub = nh.subscribe(targetTopic, 1, &PlannerServer::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());

        gcopter_planner.reset(new gcopter::GcopterPlanner(nh, "odom"));
                   
    }

    inline void mapCallBack(const kr_planning_msgs::VoxelMap::ConstPtr &map_resp)
    {
        if (!mapInitialized) //TODO: this needs to be called multiple times depending on receding horizon or not
        {

            gcopter_planner->setMap(*map_resp);
            mapInitialized = true;
        }
    }



    inline void plan()
    {
        if (startGoal.size() == 2)
        {
            Eigen::Matrix3d iniState, finState;
            std::vector<Eigen::Vector3d> route;

            iniState << startGoal.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
            finState << startGoal.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

            std::vector<Eigen::MatrixXd> plys;
            if(gcopter_planner->plan(iniState, finState, route, plys))
            {
                traj = gcopter_planner->getTraj();
            }
 
        }
    }

    inline void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        if (mapInitialized)
        {
            if (startGoal.size() >= 2)
            {
                startGoal.clear();
            }
            const Eigen::Vector3d goal(msg->pose.position.x, msg->pose.position.y, 1.0);


            if (gcopter_planner->voxelMap.query(goal) == 0)
            {
                startGoal.emplace_back(goal);
            }
            else
            {
                ROS_WARN("Infeasible Position Selected !!!\n");
            }

            plan();
        }
        return;
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "global_planning_node");
    ros::NodeHandle nh_;

    PlannerServer planner_server_utils(nh_);

    ros::Rate lr(1000);
    while (ros::ok())
    {
        ros::spinOnce();
        lr.sleep();
    }

    return 0;
}
