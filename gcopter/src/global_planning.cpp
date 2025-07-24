#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>
#include <yaml-cpp/yaml.h>


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

    Config(const std::string& yamlPath)
    {
        std::cout << "Loading configuration from: " << yamlPath << std::endl;
        YAML::Node config = YAML::LoadFile(yamlPath);

        mapTopic = config["MapTopic"].as<std::string>();
        targetTopic = config["TargetTopic"].as<std::string>();
        dilateRadius = config["DilateRadius"].as<double>();
        voxelWidth = config["VoxelWidth"].as<double>();
        mapBound = config["MapBound"].as<std::vector<double>>();
        timeoutRRT = config["TimeoutRRT"].as<double>();
        maxVelMag = config["MaxVelMag"].as<double>();
        maxBdrMag = config["MaxBdrMag"].as<double>();
        maxTiltAngle = config["MaxTiltAngle"].as<double>();
        minThrust = config["MinThrust"].as<double>();
        maxThrust = config["MaxThrust"].as<double>();
        vehicleMass = config["VehicleMass"].as<double>();
        gravAcc = config["GravAcc"].as<double>();
        horizDrag = config["HorizDrag"].as<double>();
        vertDrag = config["VertDrag"].as<double>();
        parasDrag = config["ParasDrag"].as<double>();
        speedEps = config["SpeedEps"].as<double>();
        weightT = config["WeightT"].as<double>();
        chiVec = config["ChiVec"].as<std::vector<double>>();
        smoothingEps = config["SmoothingEps"].as<double>();
        integralIntervs = config["IntegralIntervs"].as<int>();
        relCostTol = config["RelCostTol"].as<double>();

        std::cout << "Loaded configuration values:\n";
        std::cout << "  MapTopic       : " << mapTopic << "\n";
        std::cout << "  TargetTopic    : " << targetTopic << "\n";
        std::cout << "  DilateRadius   : " << dilateRadius << "\n";
        std::cout << "  VoxelWidth     : " << voxelWidth << "\n";
        std::cout << "  MapBound       : [ ";
        for (auto val : mapBound) std::cout << val << " ";
        std::cout << "]\n";
        std::cout << "  TimeoutRRT     : " << timeoutRRT << "\n";
        std::cout << "  MaxVelMag      : " << maxVelMag << "\n";
        std::cout << "  MaxBdrMag      : " << maxBdrMag << "\n";
        std::cout << "  MaxTiltAngle   : " << maxTiltAngle << "\n";
        std::cout << "  MinThrust      : " << minThrust << "\n";
        std::cout << "  MaxThrust      : " << maxThrust << "\n";
        std::cout << "  VehicleMass    : " << vehicleMass << "\n";
        std::cout << "  GravAcc        : " << gravAcc << "\n";
        std::cout << "  HorizDrag      : " << horizDrag << "\n";
        std::cout << "  VertDrag       : " << vertDrag << "\n";
        std::cout << "  ParasDrag      : " << parasDrag << "\n";
        std::cout << "  SpeedEps       : " << speedEps << "\n";
        std::cout << "  WeightT        : " << weightT << "\n";
        std::cout << "  ChiVec         : [ ";
        for (auto val : chiVec) std::cout << val << " ";
        std::cout << "]\n";
        std::cout << "  SmoothingEps   : " << smoothingEps << "\n";
        std::cout << "  IntegralIntervs: " << integralIntervs << "\n";
        std::cout << "  RelCostTol     : " << relCostTol << "\n";
    }
};
class GlobalPlanner
{
private:
    Config config;

    bool mapInitialized;
    voxel_map::VoxelMap voxelMap;
    std::vector<Eigen::Vector3d> startGoal;

    Trajectory<5> traj;
    double trajStamp;

public:
    GlobalPlanner(const Config &conf)
        : config(conf), mapInitialized(false){
        Eigen::Vector3i xyz((config.mapBound[1] - config.mapBound[0]) / config.voxelWidth,
                            (config.mapBound[3] - config.mapBound[2]) / config.voxelWidth,
                            (config.mapBound[5] - config.mapBound[4]) / config.voxelWidth);

        Eigen::Vector3d offset(config.mapBound[0], config.mapBound[2], config.mapBound[4]);
        voxelMap = voxel_map::VoxelMap(xyz, offset, config.voxelWidth);
    }

    void loadMapFromFile(const std::string &filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Failed to open map file!" << std::endl;
            return;
        }

        double x, y, z;
        while (file >> x >> y >> z) {
            //std::cout << "Loading point: (" << x << ", " << y << ", " << z << ")" << std::endl;
            voxelMap.setOccupied(Eigen::Vector3d(x, y, z));
        }

        voxelMap.dilate(std::ceil(config.dilateRadius / voxelMap.getScale()));
        mapInitialized = true;
    }

    void plan() {
        //compute the computation time
        auto start = std::chrono::steady_clock::now();

        if (startGoal.size() != 2) return;

        std::vector<Eigen::Vector3d> route;
        sfc_gen::planPath<voxel_map::VoxelMap>(startGoal[0], startGoal[1],
                                               voxelMap.getOrigin(), voxelMap.getCorner(),
                                               &voxelMap, 0.01, route);

        std::vector<Eigen::MatrixX4d> hPolys;
        std::vector<Eigen::Vector3d> pc;
        voxelMap.getSurf(pc);

        sfc_gen::convexCover(route, pc, voxelMap.getOrigin(), voxelMap.getCorner(),
                             7.0, 3.0, hPolys);
        sfc_gen::shortCut(hPolys);

        if (route.size() > 1) {

            Eigen::Matrix3d iniState, finState;
            iniState << route.front(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();
            finState << route.back(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero();

            gcopter::GCOPTER_PolytopeSFC gcopter;

            Eigen::VectorXd magnitudeBounds(5), penaltyWeights(5), physicalParams(6);
            magnitudeBounds << config.maxVelMag, config.maxBdrMag, config.maxTiltAngle,
                               config.minThrust, config.maxThrust;
            penaltyWeights << config.chiVec[0], config.chiVec[1], config.chiVec[2],
                              config.chiVec[3], config.chiVec[4];
            physicalParams << config.vehicleMass, config.gravAcc, config.horizDrag,
                              config.vertDrag, config.parasDrag, config.speedEps;

            const int quadratureRes = config.integralIntervs;

            traj.clear();

            if (!gcopter.setup(config.weightT, iniState, finState, hPolys,
                               INFINITY, config.smoothingEps, quadratureRes,
                               magnitudeBounds, penaltyWeights, physicalParams)) {
                return;
            }

            if (std::isinf(gcopter.optimize(traj, config.relCostTol))) {
                return;
            }

            if (traj.getPieceNum() > 0) {
                trajStamp = std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
            }

            std::cout << "Planning completed in "
                      << std::chrono::duration_cast<std::chrono::milliseconds>(
                             std::chrono::steady_clock::now() - start).count()
                      << " ms." << std::endl;

            //print trajectory information
            std::cout << "Trajectory generated with " << traj.getPieceNum() << " pieces." << std::endl;
            std::cout << "Total duration: " << traj.getTotalDuration() << " seconds." << std::endl;
            std::cout << "Total energy: " << traj.getEnergy(5) << std::endl;   
            std::cout << "Trajectory positions:\n" << traj.getPositions() << std::endl;


            //save the trajectory and corridor to file
            std::ofstream trajFile("trajectory.txt");
            if (trajFile.is_open()) {

                // clear previous content
                trajFile.clear();
                trajFile.seekp(0, std::ios::beg);   

                trajFile << "Trajectory Positions:\n" << traj.getPositions() << "\n";
                trajFile << "Durations:\n" << traj.getDurations() << "\n";
                trajFile << "Total Duration: " << traj.getTotalDuration() << "\n";
                trajFile << "Total Energy: " << traj.getEnergy(5) << "\n";

                //also corridor
                trajFile << "Corridor:\n";
                for (const auto &hPoly : hPolys) {
                    trajFile << hPoly << "\n";
                    trajFile << "-----------------\n";
                }
                trajFile << "Start: " << startGoal[0].transpose() << "\n";
                trajFile << "Goal: " << startGoal[1].transpose() << "\n";
                trajFile.close();
                std::cout << "Trajectory saved to trajectory.txt" << std::endl;
            }
        }
    }

    void setTarget(const Eigen::Vector3d &goal) {
        if (!mapInitialized) return;

        if (startGoal.size() >= 2) startGoal.clear();

        if (voxelMap.query(goal) == 0) {
            startGoal.emplace_back(goal);
            plan();
        } else {
            std::cout << "Infeasible Position Selected! goal: " << goal.transpose() << std::endl;
        }
    }


};


int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <input.yaml>\n";
        return 1;
    }

    std::string input_file = argv[1];

    YAML::Node input_config;
    try {
        input_config = YAML::LoadFile(input_file);
    } catch (const YAML::BadFile& e) {
        std::cerr << "Failed to open input file: " << input_file << "\n";
        return 1;
    }

    std::string config_path = input_config["config_path"].as<std::string>();
    std::string map_path = input_config["map_path"].as<std::string>();

    auto start_node = input_config["start"];
    Eigen::Vector3d start(start_node["x"].as<double>(), start_node["y"].as<double>(), start_node["z"].as<double>());

    auto goal_node = input_config["goal"];
    Eigen::Vector3d goal(goal_node["x"].as<double>(), goal_node["y"].as<double>(), goal_node["z"].as<double>());


    std::cout << "Start: " << start.transpose() << "\n";
    std::cout << "Goal: " << goal.transpose() << "\n";

    std::cout << "Config Path: " << config_path << "\n";
    std::cout << "Map Path: " << map_path << "\n";
    

    Config config(config_path);
    GlobalPlanner planner(config);
    planner.loadMapFromFile(map_path);

    planner.setTarget(start);
    planner.setTarget(goal);

    return 0;
}