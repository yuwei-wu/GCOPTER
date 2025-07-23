#ifndef VISUALIZER_HPP
#define VISUALIZER_HPP

#include "gcopter/trajectory.hpp"
#include "gcopter/quickhull.hpp"
#include "gcopter/geo_utils.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <chrono>
#include <memory>
#include <cmath>
#include <iostream>

// Visualizer for the planner
class Visualizer
{
private:
    rclcpp::Node & node_;  // Node reference

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr routePub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr wayPointsPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr trajectoryPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr meshPub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr edgePub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr spherePub;

public:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr speedPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr thrPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr tiltPub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bdrPub;

public:
    Visualizer(rclcpp::Node & node) : node_(node)
    {
        routePub = node_.create_publisher<visualization_msgs::msg::Marker>("/visualizer/route", 10);
        wayPointsPub = node_.create_publisher<visualization_msgs::msg::Marker>("/visualizer/waypoints", 10);
        trajectoryPub = node_.create_publisher<visualization_msgs::msg::Marker>("/visualizer/trajectory", 10);
        meshPub = node_.create_publisher<visualization_msgs::msg::Marker>("/visualizer/mesh", 10);
        edgePub = node_.create_publisher<visualization_msgs::msg::Marker>("/visualizer/edge", 10);
        spherePub = node_.create_publisher<visualization_msgs::msg::Marker>("/visualizer/spheres", 10);
        speedPub = node_.create_publisher<std_msgs::msg::Float64>("/visualizer/speed", 10);
        thrPub = node_.create_publisher<std_msgs::msg::Float64>("/visualizer/total_thrust", 10);
        tiltPub = node_.create_publisher<std_msgs::msg::Float64>("/visualizer/tilt_angle", 10);
        bdrPub = node_.create_publisher<std_msgs::msg::Float64>("/visualizer/body_rate", 10);
    }

    template <int D>
    inline void visualize(const Trajectory<D> &traj,
                          const std::vector<Eigen::Vector3d> &route)
    {
        using namespace visualization_msgs::msg;

        Marker routeMarker, wayPointsMarker, trajMarker;

        routeMarker.id = 0;
        routeMarker.type = Marker::LINE_LIST;
        routeMarker.header.stamp = node_.now();
        routeMarker.header.frame_id = "odom";
        routeMarker.pose.orientation.w = 1.0;
        routeMarker.action = Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.0;
        routeMarker.color.g = 0.0;
        routeMarker.color.b = 0.0;
        routeMarker.color.a = 1.0;
        routeMarker.scale.x = 0.1;

        wayPointsMarker = routeMarker;
        wayPointsMarker.id = -1;
        wayPointsMarker.type = Marker::SPHERE_LIST;
        wayPointsMarker.ns = "waypoints";
        wayPointsMarker.scale.x = wayPointsMarker.scale.y = wayPointsMarker.scale.z = 0.35;

        trajMarker = routeMarker;
        trajMarker.ns = "trajectory";
        trajMarker.color.r = 0.0;
        trajMarker.color.g = 0.5;
        trajMarker.color.b = 1.0;
        trajMarker.scale.x = 0.3;

        if (!route.empty()) {
            bool first = true;
            Eigen::Vector3d last;
            for (const auto &pt : route) {
                if (first) {
                    last = pt;
                    first = false;
                    continue;
                }
                geometry_msgs::msg::Point p;
                p.x = last(0); p.y = last(1); p.z = last(2);
                routeMarker.points.push_back(p);
                p.x = pt(0); p.y = pt(1); p.z = pt(2);
                routeMarker.points.push_back(p);
                last = pt;
            }
            routePub->publish(routeMarker);
        }

        if (traj.getPieceNum() > 0) {
            Eigen::MatrixXd wps = traj.getPositions();
            for (int i = 0; i < wps.cols(); ++i) {
                geometry_msgs::msg::Point p;
                p.x = wps(0, i); p.y = wps(1, i); p.z = wps(2, i);
                wayPointsMarker.points.push_back(p);
            }
            wayPointsPub->publish(wayPointsMarker);
        }

        if (traj.getPieceNum() > 0) {
            double T = 0.01;
            Eigen::Vector3d lastX = traj.getPos(0.0);
            for (double t = T; t < traj.getTotalDuration(); t += T) {
                Eigen::Vector3d X = traj.getPos(t);
                geometry_msgs::msg::Point p1, p2;
                p1.x = lastX(0); p1.y = lastX(1); p1.z = lastX(2);
                p2.x = X(0); p2.y = X(1); p2.z = X(2);
                trajMarker.points.push_back(p1);
                trajMarker.points.push_back(p2);
                lastX = X;
            }
            trajectoryPub->publish(trajMarker);
        }
    }

    inline void visualizePolytope(const std::vector<Eigen::MatrixX4d> &hPolys)
    {
        Eigen::Matrix3Xd mesh(3, 0), curTris(3, 0), oldTris(3, 0);

        for (const auto &hPoly : hPolys)
        {
            oldTris = mesh;
            Eigen::Matrix<double, 3, -1, Eigen::ColMajor> vPoly;
            geo_utils::enumerateVs(hPoly, vPoly);

            quickhull::QuickHull<double> qh;
            const auto hull = qh.getConvexHull(vPoly.data(), vPoly.cols(), false, true);
            const auto &idxBuffer = hull.getIndexBuffer();
            int hNum = idxBuffer.size() / 3;

            curTris.resize(3, hNum * 3);
            for (int i = 0; i < hNum * 3; ++i)
                curTris.col(i) = vPoly.col(idxBuffer[i]);

            mesh.resize(3, oldTris.cols() + curTris.cols());
            mesh.leftCols(oldTris.cols()) = oldTris;
            mesh.rightCols(curTris.cols()) = curTris;
        }

        using namespace visualization_msgs::msg;

        Marker meshMarker, edgeMarker;
        meshMarker.id = 0;
        meshMarker.header.stamp = node_.now();
        meshMarker.header.frame_id = "odom";
        meshMarker.pose.orientation.w = 1.0;
        meshMarker.action = Marker::ADD;
        meshMarker.type = Marker::TRIANGLE_LIST;
        meshMarker.ns = "mesh";
        meshMarker.color.r = 0.0;
        meshMarker.color.g = 0.0;
        meshMarker.color.b = 1.0;
        meshMarker.color.a = 0.15;
        meshMarker.scale.x = 1.0;
        meshMarker.scale.y = 1.0;
        meshMarker.scale.z = 1.0;

        edgeMarker = meshMarker;
        edgeMarker.type = Marker::LINE_LIST;
        edgeMarker.ns = "edge";
        edgeMarker.color.g = 1.0;
        edgeMarker.color.a = 1.0;
        edgeMarker.scale.x = 0.02;
        

        for (int i = 0; i < mesh.cols(); ++i) {
            geometry_msgs::msg::Point p;
            p.x = mesh(0, i); p.y = mesh(1, i); p.z = mesh(2, i);

            meshMarker.points.push_back(p);
        }

        for (int i = 0; i < mesh.cols() / 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                geometry_msgs::msg::Point p1, p2;
                p1.x = mesh(0, 3*i + j); p1.y = mesh(1, 3*i + j); p1.z = mesh(2, 3*i + j);
                p2.x = mesh(0, 3*i + (j + 1) % 3); p2.y = mesh(1, 3*i + (j + 1) % 3); p2.z = mesh(2, 3*i + (j + 1) % 3);
                edgeMarker.points.push_back(p1);
                edgeMarker.points.push_back(p2);
            }
        }

        meshPub->publish(meshMarker);
        edgePub->publish(edgeMarker);
    }

    inline void visualizeSphere(const Eigen::Vector3d &center, const double &radius)
    {
        using namespace visualization_msgs::msg;

        Marker sphereMarker;
        sphereMarker.id = 0;
        sphereMarker.type = Marker::SPHERE_LIST;
        sphereMarker.header.stamp = node_.now();
        sphereMarker.header.frame_id = "odom";
        sphereMarker.pose.orientation.w = 1.0;
        sphereMarker.action = Marker::ADD;
        sphereMarker.ns = "spheres";
        sphereMarker.color.b = 1.0;
        sphereMarker.color.a = 1.0;
        sphereMarker.scale.x = sphereMarker.scale.y = sphereMarker.scale.z = radius * 2.0;

        geometry_msgs::msg::Point p;
        p.x = center(0); p.y = center(1); p.z = center(2);
        sphereMarker.points.push_back(p);

        spherePub->publish(sphereMarker);
    }

    inline void visualizeStartGoal(const Eigen::Vector3d &center, const double &radius, const int sg)
    {
        using namespace visualization_msgs::msg;

        Marker marker;
        marker.id = sg;
        marker.type = Marker::SPHERE_LIST;
        marker.header.stamp = node_.now();
        marker.header.frame_id = "odom";
        marker.pose.orientation.w = 1.0;
        marker.action = Marker::ADD;
        marker.ns = "StartGoal";
        marker.color.r = 1.0;
        marker.color.a = 1.0;
        marker.scale.x = marker.scale.y = marker.scale.z = radius * 2.0;

        geometry_msgs::msg::Point p;
        p.x = center(0); p.y = center(1); p.z = center(2);
        marker.points.push_back(p);

        spherePub->publish(marker);
    }
};

#endif  // VISUALIZER_HPP