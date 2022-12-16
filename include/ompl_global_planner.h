#ifndef OMPL_GLOBAL_PLANNER_H
#define OMPL_GLOBAL_PLANNER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <visualization_msgs/Marker.h>

#include <base_local_planner/costmap_model.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>

#include <memory>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace ompl_global_planner
{
    class OmplGlobalPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        OmplGlobalPlanner();
        ~OmplGlobalPlanner();

        // default functions for plugin:
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan);

        // for visualization
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        void publishGraph();

        // ompl functions:
        bool isStateValid(const ob::State* state);
        double getStateCellCost(const ob::State* state);

    private:
        costmap_2d::Costmap2DROS* costmap_ros_; 
        costmap_2d::Costmap2D* costmap_;

        std::shared_ptr<base_local_planner::WorldModel> world_model_;
        // std::shared_ptr<base_local_planner::CostmapModel> world_model_;

        bool initialized_;

        boost::mutex mutex_;

        // for ompl planner
        og::SimpleSetupPtr ss_;
        ob::StateSpacePtr space_;

        // visualize path
        ros::Publisher plan_pub_;
        ros::Publisher graph_pub_;
    };


    class CostMapObjective : public ob::StateCostIntegralObjective
    {
    public:
        CostMapObjective(OmplGlobalPlanner& op, const ob::SpaceInformationPtr& si)
            : ob::StateCostIntegralObjective(si, true), ompl_planner_(op){}

        virtual ob::Cost stateCost(const ob::State* state) const
        {
            return ob::Cost(ompl_planner_.getStateCellCost(state));
        }

    private:
        OmplGlobalPlanner& ompl_planner_;
    };

}

#endif
