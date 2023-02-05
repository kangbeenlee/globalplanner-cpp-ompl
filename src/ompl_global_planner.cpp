#include <ompl_global_planner.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(ompl_global_planner::OmplGlobalPlanner, nav_core::BaseGlobalPlanner)

namespace ompl_global_planner
{
    OmplGlobalPlanner::OmplGlobalPlanner() : costmap_ros_(NULL), initialized_(false)
    {
        space_ = std::make_shared<ob::RealVectorStateSpace>(2);
    }

    OmplGlobalPlanner::~OmplGlobalPlanner()
    {
    }

    void OmplGlobalPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        if (!initialized_)
        {
            costmap_ros_ = costmap_ros;
            costmap_ = costmap_ros_->getCostmap();

            ros::NodeHandle private_nh("~/" + name);
            plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
            graph_pub_ = private_nh.advertise<visualization_msgs::Marker>("graph", 1);
            world_model_ = std::make_shared<base_local_planner::CostmapModel>(*costmap_);

            // set state space boudary:
            ob::RealVectorBounds bounds(2);
            bounds.setLow(-10);
            bounds.setHigh(10);

            // bounds.setLow(0, costmap_->getOriginX());
            // bounds.setLow(1, costmap_->getOriginY());
            // bounds.setHigh(0, costmap_->getOriginX() + costmap_->getSizeInCellsX() * costmap_->getResolution());
            // bounds.setHigh(0, costmap_->getOriginY() + costmap_->getSizeInCellsX() * costmap_->getResolution());

            ROS_DEBUG("Check boundary size %.2f, %.2f, %.2f, %.2f", costmap_->getOriginX(), costmap_->getOriginY(),
                        costmap_->getOriginX() + costmap_->getSizeInCellsX() * costmap_->getResolution(),
                        costmap_->getOriginY() + costmap_->getSizeInCellsX() * costmap_->getResolution());

            space_->as<ob::RealVectorStateSpace>()->setBounds(bounds);
            ss_ = std::make_shared<og::SimpleSetup>(space_);

            // get space information
            si_ = ss_->getSpaceInformation();

            // set state validity checker
            ss_->setStateValidityChecker(boost::bind(&OmplGlobalPlanner::isStateValid, this, _1));

            // optimize criteria
            ob::OptimizationObjectivePtr cost_objective(std::make_shared<CostMapObjective>(*this, si_));
            ob::OptimizationObjectivePtr length_objective(std::make_shared<ob::PathLengthOptimizationObjective>(si_));
            // ss_->setOptimizationObjective(cost_objective + length_objective);
            // ss_->setOptimizationObjective(cost_objective);
            ss_->setOptimizationObjective(length_objective);

            initialized_ = true;
            ROS_INFO("Ompl global planner initialized!");
        }
        else
        {
            ROS_WARN("This planner has already been initialized... doing nothing");
        }

    }

    bool OmplGlobalPlanner::isStateValid(const ob::State* state)
    {
        double cell_cost = getStateCellCost(state);

        if (cell_cost >= 0 && cell_cost < 30)
        {
            return true;
        }
        return false;
    }

    double OmplGlobalPlanner::getStateCellCost(const ob::State* state)
    {
        auto state2D = state->as<ob::RealVectorStateSpace::StateType>();
        double wx = state2D->values[0];
        double wy = state2D->values[1];

        unsigned int mx, my;
        costmap_->worldToMap(wx, wy, mx, my);

        return costmap_->getCost(mx, my);
    }

    bool OmplGlobalPlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                                    std::vector<geometry_msgs::PoseStamped>& plan)
    {
        boost::mutex::scoped_lock lock(mutex_);

        if (!initialized_)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return false;
        }

        if(goal.header.frame_id != costmap_ros_->getGlobalFrameID())
        {
            ROS_ERROR("This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.", 
            costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
            return false;
        }

        ROS_DEBUG("Got a start: %.2f, %.2f, and a goal: %.2f, %.2f", start.pose.position.x, start.pose.position.y, goal.pose.position.x, goal.pose.position.y);

        // // use below codes when you need yaw of start or goal in SE2StateSpace planning
        // tf::Stamped<tf::Pose> start_tf;
        // tf::Stamped<tf::Pose> goal_tf;
        // // convert PoseStamped message to TF (transform (frame))
        // tf::poseStampedMsgToTF(start, start_tf);
        // tf::poseStampedMsgToTF(goal, goal_tf);

        // double useless_pitch, useless_roll, goal_yaw, start_yaw;
        // start_tf.getBasis().getEulerYPR(start_yaw, useless_pitch, useless_roll);
        // goal_tf.getBasis().getEulerYPR(goal_yaw, useless_pitch, useless_roll);

        ROS_INFO("Try to find global path with OMPL ...");

        ROS_INFO("New goal is received ... OMPL Planner is on intialization ...");

        // rrtx_->initialize();
        goal_x_ = goal.pose.position.x;
        goal_y_ = goal.pose.position.y;

        ob::ScopedState<>start_state(space_);
        start_state[0] = start.pose.position.x;
        start_state[1] = start.pose.position.y;

        ob::ScopedState<> goal_state(space_);
        goal_state[0] = goal.pose.position.x;
        goal_state[1] = goal.pose.position.y;

        ss_->setStartAndGoalStates(start_state, goal_state);

        // select sampling-based planner for global path
        // auto planner(std::make_shared<og::RRT>(si_));
        auto planner(std::make_shared<og::RRTstar>(si_));

        // 0.3 is enough for rrt and 0.8 is enough for rrt_star
        planner->setRange(0.8);
        planner->setGoalBias(0.05);

        ss_->setPlanner(planner);
        ss_->getPlanner()->clear();

        ob::PlannerStatus solved = ss_->solve(0.1);

        // clear the plan, just in case
        plan.clear();
        plan.push_back(start);

        // convert path into ROS messages:
        if (solved)
        {
            ROS_INFO("Successed to find global path with OMPL!");

            og::PathGeometric& geometic_path = ss_->getSolutionPath();
            // path post-processing
            ss_->getPathSimplifier()->simplifyMax(geometic_path);
            ss_->getPathSimplifier()->smoothBSpline(geometic_path);
            geometic_path.interpolate();

            // Conversion loop from states to messages:
            for (unsigned int i = 0; i < geometic_path.getStateCount(); i++)
            {
                auto state = geometic_path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
                double x = state->values[0];
                double y = state->values[1];

                // by below command, we don't have to set message header.frame_id
                geometry_msgs::PoseStamped ps = goal;
                ps.header.stamp = ros::Time::now();
                ps.pose.position.x = x;
                ps.pose.position.y = y;
                plan.push_back(ps);
            }
            plan.push_back(goal);
        }
        else
        {
            ROS_ERROR("Failed to compute global path with OMPL");
        }

        // publish the plan to visualize
        publishPlan(plan);
        publishGraph();
        return solved;
    }

    void OmplGlobalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
    {
        if (!initialized_)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return;
        }

        // create a message for the plan 
        nav_msgs::Path gui_path;
        gui_path.poses.resize(path.size());

        if (!path.empty()) {
            gui_path.header = path[0].header;
        }
        gui_path.poses = path;
        plan_pub_.publish(gui_path);
    }

    void OmplGlobalPlanner::publishGraph()
    {
        if (!initialized_)
        {
            ROS_ERROR("The planner has not been initialized, please call initialize() to use the planner");
            return;
        }

        ob::PlannerData planner_data(ss_->getSpaceInformation());
        ss_->getPlanner()->getPlannerData(planner_data);

        visualization_msgs::Marker edge;
        edge.type = visualization_msgs::Marker::LINE_LIST;
        edge.header.frame_id = costmap_ros_->getGlobalFrameID();
        edge.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point ps, pe;

        for (unsigned int i = 0; i < planner_data.numVertices(); i++)
        {
            // for sampling vertex
            const ob::PlannerDataVertex vertex = planner_data.getVertex(i);
            auto state2D = vertex.getState()->as<ob::RealVectorStateSpace::StateType>();
            double x = state2D->values[0];
            double y = state2D->values[1];

            ps.x = x;
            ps.y = y;

            // find incoming edge (to get parent)
            std::vector<unsigned int> edge_list;
            unsigned int num_edges = planner_data.getIncomingEdges(i, edge_list);
            unsigned int num = 0;
            // unsigned int num_edges = plannerData.getEdges(i, edge_list);

            if (!edge_list.empty())
            {
                unsigned int parent_i = edge_list.front();
                const ob::PlannerDataVertex parent_vertex = planner_data.getVertex(parent_i);
                auto parent_state = parent_vertex.getState()->as<ob::RealVectorStateSpace::StateType>();
                double parent_x = parent_state->values[0];
                double parent_y = parent_state->values[1];

                pe.x = parent_x;
                pe.y = parent_y;


                edge.header.stamp = ros::Time::now();
                edge.id = num++;
                
                edge.pose.orientation.w = 1;
                edge.color.g = 1.0;
                edge.color.a = 1.0;
                edge.scale.x = 0.03;

                edge.points.push_back(ps);
                edge.points.push_back(pe);

                graph_pub_.publish(edge);
            }
        }

    }

} // end namespace global_planner

