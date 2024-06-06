/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef SFC_GEN_HPP
#define SFC_GEN_HPP

#include "geo_utils.hpp"
#include "firi.hpp"

#include <ompl/util/Console.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

#include <ompl/base/DiscreteMotionValidator.h>


#include <ompl/control/PathControl.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <deque>
#include <memory>
#include <Eigen/Eigen>

namespace sfc_gen
{
    namespace oc = ompl::control;
    namespace ob = ompl::base;

    template <typename Map>
    inline double planPath(const Eigen::Vector3d &s,
                           const Eigen::Vector3d &g,
                           const Eigen::Vector3d &lb,
                           const Eigen::Vector3d &hb,
                           Map *mapPtr,
                           const double &timeout,
                           const double &goal_tol,
                           std::vector<Eigen::Vector3d> &p)
    {
        auto space(std::make_shared<ompl::base::RealVectorStateSpace>(3));
        p.clear();

        ompl::base::RealVectorBounds bounds(3);
        bounds.setLow(0, 0.0);
        bounds.setHigh(0, hb(0) - lb(0));
        bounds.setLow(1, 0.0);
        bounds.setHigh(1, hb(1) - lb(1));
        bounds.setLow(2, 0.0);
        bounds.setHigh(2, hb(2) - lb(2));
        space->setBounds(bounds);

        auto si(std::make_shared<ompl::base::SpaceInformation>(space));

        si->setStateValidityChecker(
            [&](const ompl::base::State *state)
            {
                const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
                const Eigen::Vector3d position(lb(0) + (*pos)[0],
                                               lb(1) + (*pos)[1],
                                               lb(2) + (*pos)[2]);
                return mapPtr->query(position) == 0;
            });
        si->setup();

        ompl::base::ScopedState<> start(space), goal(space);
        start[0] = s(0) - lb(0);
        start[1] = s(1) - lb(1);
        start[2] = s(2) - lb(2);
        goal[0] = g(0) - lb(0);
        goal[1] = g(1) - lb(1);
        goal[2] = g(2) - lb(2);

        auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
        pdef->setStartAndGoalStates(start, goal, goal_tol);
         
        double dis = (s - g).norm();
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
        obj->setCostThreshold(ob::Cost(dis));
        pdef->setOptimizationObjective(obj);

        auto planner(std::make_shared<ompl::geometric::RRTstar>(si));
        planner->setProblemDefinition(pdef);
        planner->setup();

        ompl::base::PlannerStatus solved;
        solved = planner->ompl::base::Planner::solve(timeout);

        double cost = INFINITY;

        if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
        {
            p.clear();
            const ompl::geometric::PathGeometric path_ =
                ompl::geometric::PathGeometric(
                    dynamic_cast<const ompl::geometric::PathGeometric &>(*pdef->getSolutionPath()));
            for (size_t i = 0; i < path_.getStateCount(); i++)
            {
                const auto state = path_.getState(i)->as<ompl::base::RealVectorStateSpace::StateType>()->values;
                p.emplace_back(lb(0) + state[0], lb(1) + state[1], lb(2) + state[2]);
            }
            cost = pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()).value();
        }

        return cost;
    }



    template <typename Map>
    inline void dyplanPath(const Eigen::Matrix3d &s,
                             const Eigen::Matrix3d &g,
                             const Eigen::Vector3d &lb,
                             const Eigen::Vector3d &hb,
                             Map *mapPtr,
                             const double &timeout,
                             const double &goal_tol,
                             std::vector<Eigen::VectorXd> &p)
    {
        auto space(std::make_shared<ob::RealVectorStateSpace>(6));
        p.clear();
        //std::cout << "goal is " << g << std::endl;
        //ompl::msg::setLogLevel(ompl::msg::LOG_NONE);

        ob::RealVectorBounds bounds(6);
        bounds.setLow(0, 0.0);
        bounds.setHigh(0, hb(0) - lb(0));
        bounds.setLow(1, 0.0);
        bounds.setHigh(1, hb(1) - lb(1));
        bounds.setLow(2, 0.0);
        bounds.setHigh(2, hb(2) - lb(2));

        bounds.setLow(3, -3.0);
        bounds.setHigh(3, 3.0);
        bounds.setLow(4, -3.0);
        bounds.setHigh(4, 3.0);
        bounds.setLow(5, -3.0);
        bounds.setHigh(5, 3.0);

        space->setBounds(bounds);

        // create a control space
        auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 3));
    
        // set the bounds for the control space
        ob::RealVectorBounds cbounds(3);
        cbounds.setLow(0, -3.0);
        cbounds.setHigh(0, 3.0);
        cbounds.setLow(1, -3.0);
        cbounds.setHigh(1, 3.0);
        cbounds.setLow(2, -2.0);
        cbounds.setHigh(2, 2.0);
        cspace->setBounds(cbounds);
  
        // define a simple setup class
        oc::SimpleSetup ss(cspace);

        ss.setStateValidityChecker(
            [&](const ob::State *state)
            {
                const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
                const Eigen::Vector3d position(lb(0) + (*pos)[0],
                                               lb(1) + (*pos)[1],
                                               lb(2) + (*pos)[2]);
                return mapPtr->query(position) == 0;
            });
        // si->setMotionValidator(
        //     [&](const ompl::base::State *state)
        //     {
        //         const auto *pos = state->as<ompl::base::RealVectorStateSpace::StateType>();
        //         const Eigen::Vector3d position(lb(0) + (*pos)[0],
        //                                        lb(1) + (*pos)[1],
        //                                        lb(2) + (*pos)[2]);
        //         return mapPtr->query(position) == 0;
        //     });
        ss.setStatePropagator(
            [&](const ob::State *state, const oc::Control* control,
                const double duration, ob::State *result)
            {
                Eigen::MatrixXd phi_ = Eigen::MatrixXd::Identity(6, 6);
                for (int i = 0; i < 3; ++i) phi_(i, i + 3) = duration;

                const auto* spv0   = state->as<ob::RealVectorStateSpace::StateType>();
                const auto* ctrl = control->as<oc::RealVectorControlSpace::ControlType>();

                Eigen::Matrix<double, 6, 1> pv0, pv1;
                pv0 <<  (*spv0)[0],
                        (*spv0)[1],
                        (*spv0)[2],
                        (*spv0)[3],
                        (*spv0)[4],
                        (*spv0)[5];

                const Eigen::Vector3d um((*ctrl)[0], (*ctrl)[1], (*ctrl)[2]);

                Eigen::Matrix<double, 6, 1> integral;
                integral.head(3) = 0.5 * std::pow(duration, 2) * um;
                integral.tail(3) = duration * um;

                pv1 = phi_ * pv0 + integral;

                auto* spv1 = result->as<ob::RealVectorStateSpace::StateType>();
                (*spv1)[0] = pv1(0);
                (*spv1)[1] = pv1(1);
                (*spv1)[2] = pv1(2);
                (*spv1)[3] = pv1(3);
                (*spv1)[4] = pv1(4);
                (*spv1)[5] = pv1(5);

            });
        ss.getSpaceInformation()->setMinMaxControlDuration(1, 15);
        ss.getSpaceInformation()->setPropagationStepSize(0.1);

        ompl::base::ScopedState<> start(space), goal(space);
        start[0] = s(0, 0) - lb(0);
        start[1] = s(1, 0) - lb(1);
        start[2] = s(2, 0) - lb(2);
        start[3] = s(0, 1);
        start[4] = s(1, 1);
        start[5] = s(2, 1);

        goal[0] = g(0, 0) - lb(0);
        goal[1] = g(1, 0) - lb(1);
        goal[2] = g(2, 0) - lb(2);
        goal[3] = g(0, 1);
        goal[4] = g(1, 1);
        goal[5] = g(2, 1);

    
        ss.setStartAndGoalStates(start, goal, goal_tol);
        ss.setPlanner(std::make_shared<oc::SST>(ss.getSpaceInformation()));
        //ss.setOptimizationObjective(std::make_shared<ompl::base::StateCostIntegralObjective>(ss.getSpaceInformation(), true));
        //ss.print();
        double dis = (s - g).norm();
        ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(ss.getSpaceInformation()));
        obj->setCostThreshold(ob::Cost(dis));
        ss.setOptimizationObjective(obj);
        ss.setup();
        auto solved = ss.solve(timeout);
        std::cout << "solved" << solved << std::endl;

        if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION)
        {
            p.clear();

            oc::PathControl path_ = ss.getSolutionPath();

            for (size_t i = 0; i < path_.getStateCount(); i++)
            {

                const auto   state = path_.getState(i)->as<ob::RealVectorStateSpace::StateType>()->values;
                
                if(i == path_.getStateCount() -1)
                {
                    Eigen::VectorXd wpt(10);
                    wpt << lb(0) + state[0], lb(1) + state[1], lb(2) + state[2],
                        state[3], state[4], state[5], 
                        0.0, 0.0, 0.0, 0.0;
                    p.push_back(wpt);
                }else
                {
                    const auto   acc   = path_.getControl(i)->as<oc::RealVectorControlSpace::ControlType>()->values;
                    const double dur   = path_.getControlDuration(i);

                    Eigen::VectorXd wpt(10);
                    wpt << lb(0) + state[0], lb(1) + state[1], lb(2) + state[2],
                        state[3], state[4], state[5], 
                        acc[0], acc[1], acc[2],
                        dur;
                    p.push_back(wpt);

                }
        
            }
        }

        return;
    }


    inline void convexCover(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const double &progress,
                            const double &range,
                            std::vector<Eigen::MatrixX4d> &hpolys,
                            const double eps = 1.0e-6)
    {
        hpolys.clear();
        const int n = path.size();
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;

        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a, b = path[0];
        std::vector<Eigen::Vector3d> valid_pc;
        std::vector<Eigen::Vector3d> bs;
        valid_pc.reserve(points.size());
        for (int i = 1; i < n;)
        {
            a = b;
            if ((a - path[i]).norm() > progress)
            {
                b = (path[i] - a).normalized() * progress + a;
            }
            else
            {
                b = path[i];
                i++;
            }
            bs.emplace_back(b);

            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + range, highCorner(2));
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - range, lowCorner(2));

            valid_pc.clear();
            for (const Eigen::Vector3d &p : points)
            {
                if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0)
                {
                    valid_pc.emplace_back(p);
                }
            }
            Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());

            firi::firi(bd, pc, a, b, hp);

            if (hpolys.size() != 0)
            {
                const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
                if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                             ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
                {
                    firi::firi(bd, pc, a, a, gap, 1);
                    hpolys.emplace_back(gap);
                }
            }

            hpolys.emplace_back(hp);
        }
    }

    inline bool checkInsidePoly(const Eigen::Vector3d &pt,
                                const Eigen::MatrixX4d &hPoly)
    {

        const Eigen::ArrayXd hNorm = hPoly.leftCols<3>().rowwise().norm();
        Eigen::MatrixX3d A = hPoly.leftCols<3>().array().colwise() / hNorm;
        Eigen::VectorXd  b = -hPoly.rightCols<1>().array() / hNorm;

        for (int i = 0; i < hPoly.rows(); i++){
            double linear_const = A.row(i) * pt - b(i);

            if ( linear_const > 1e-3)
            {
                return false;
            }
        }

        return true;
    }




    inline void getPolyConst(const std::vector<Eigen::Vector3d> &path,
                            const std::vector<Eigen::Vector3d> &points,
                            const Eigen::Vector3d &lowCorner,
                            const Eigen::Vector3d &highCorner,
                            const double &progress,
                            const double &range,
                            std::vector<Eigen::MatrixX4d> &hpolys,
                            const double eps = 1.0e-6)
    {

        hpolys.clear();
        const int n = path.size();
        Eigen::Matrix<double, 6, 4> bd = Eigen::Matrix<double, 6, 4>::Zero();
        bd(0, 0) = 1.0;
        bd(1, 0) = -1.0;
        bd(2, 1) = 1.0;
        bd(3, 1) = -1.0;
        bd(4, 2) = 1.0;
        bd(5, 2) = -1.0;

        Eigen::MatrixX4d hp, gap;
        Eigen::Vector3d a, b = path[1];
        std::vector<Eigen::Vector3d> valid_pc;
        std::vector<Eigen::Vector3d> bs;
        valid_pc.reserve(points.size());

        //int cnt_num = 0;
        a = path[0];
            
        for (int i = 0; i < n; i++)
        {
            if (i > 0)
            {
                if (checkInsidePoly(path[i], hpolys.back()) && 
                    (a - path[i]).norm() <= progress &&
                    i < n-1)
                {
                    //cnt_num ++;
                    continue;
                }
                else
                {
                    b = path[i];
                    //cnt_num = 0;
                }
            
            }

            bs.emplace_back(b);

            bd(0, 3) = -std::min(std::max(a(0), b(0)) + range, highCorner(0));
            bd(1, 3) = +std::max(std::min(a(0), b(0)) - range, lowCorner(0));
            bd(2, 3) = -std::min(std::max(a(1), b(1)) + range, highCorner(1));
            bd(3, 3) = +std::max(std::min(a(1), b(1)) - range, lowCorner(1));
            bd(4, 3) = -std::min(std::max(a(2), b(2)) + 0.8 * range, highCorner(2));
            bd(5, 3) = +std::max(std::min(a(2), b(2)) - 0.5 * range, lowCorner(2));

            valid_pc.clear();
            for (const Eigen::Vector3d &p : points)
            {
                if ((bd.leftCols<3>() * p + bd.rightCols<1>()).maxCoeff() < 0.0)
                {
                    valid_pc.emplace_back(p);
                }
            }
            Eigen::Map<const Eigen::Matrix<double, 3, -1, Eigen::ColMajor>> pc(valid_pc[0].data(), 3, valid_pc.size());

            firi::firi(bd, pc, a, b, hp);

            if (hpolys.size() != 0)
            {
                const Eigen::Vector4d ah(a(0), a(1), a(2), 1.0);
                if (3 <= ((hp * ah).array() > -eps).cast<int>().sum() +
                             ((hpolys.back() * ah).array() > -eps).cast<int>().sum())
                {
                    firi::firi(bd, pc, a, a, gap, 1);
                    hpolys.emplace_back(gap);
                }
            }

            hpolys.emplace_back(hp);

            a = b;
        }



    }



    inline void shortCut(std::vector<Eigen::MatrixX4d> &hpolys)
    {
        std::vector<Eigen::MatrixX4d> htemp = hpolys;
        if (htemp.size() == 1)
        {
            Eigen::MatrixX4d headPoly = htemp.front();
            htemp.insert(htemp.begin(), headPoly);
        }
        hpolys.clear();

        int M = htemp.size();
        Eigen::MatrixX4d hPoly;
        bool overlap;
        std::deque<int> idices;
        idices.push_front(M - 1);
        for (int i = M - 1; i >= 0; i--)
        {
            for (int j = 0; j < i; j++)
            {
                if (j < i - 1)
                {
                    overlap = geo_utils::overlap(htemp[i], htemp[j], 0.01);
                }
                else
                {
                    overlap = true;
                }
                if (overlap)
                {
                    idices.push_front(j);
                    i = j + 1;
                    break;
                }
            }
        }
        for (const auto &ele : idices)
        {
            hpolys.push_back(htemp[ele]);
        }
    }

}

#endif
