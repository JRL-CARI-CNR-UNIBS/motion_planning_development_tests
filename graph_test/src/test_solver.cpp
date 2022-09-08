#include <ros/ros.h>
#include <graph_core/graph/subtree.h>
#include <graph_core/moveit_collision_checker.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <graph_core/graph/graph_display.h>
#include <graph_core/solvers/anytime_rrt.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/birrt.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_solver");
  ros::NodeHandle nh;

  ros::AsyncSpinner aspin(4);
  aspin.start();

  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

  std::string solver_name = "RRT";
  nh.getParam("solver_name",solver_name);

  std::string group_name = "cartesian_arm";
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  if (!ps_client.waitForExistence(ros::Duration(10)))
  {
    ROS_ERROR("unable to connect to /get_planning_scene");
    return 1;
  }

  moveit_msgs::GetPlanningScene ps_srv;
  if (!ps_client.call(ps_srv))
  {
    ROS_ERROR("call to srv not ok");
    return 1;
  }

  if (!planning_scene->setPlanningSceneMsg(ps_srv.response.scene))
  {
    ROS_ERROR("unable to update planning scene");
    return 0;
  }

  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::MoveitCollisionChecker>(planning_scene, group_name);
  pathplan::MetricsPtr metrics=std::make_shared<pathplan::Metrics>();

  std::vector<std::string> joint_names = kinematic_model->getJointModelGroup(group_name)->getActiveJointModelNames();

  unsigned int dof = joint_names.size();
  Eigen::VectorXd lb(dof);
  Eigen::VectorXd ub(dof);

  for (unsigned int idx = 0; idx < dof; idx++)
  {
    const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }
  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(lb, ub, lb, ub);
  pathplan::Display display(planning_scene,group_name);
  ros::Duration(1.0).sleep();
  display.clearMarkers();

  pathplan::TreeSolverPtr solver = std::make_shared<pathplan::RRT>(metrics,checker,sampler);

  if(solver_name == "RRTConnect")
    solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);
  else if(solver_name == "AnytimeRRT")
    solver = std::make_shared<pathplan::AnytimeRRT>(metrics,checker,sampler);
  else if(solver_name == "RRTStar")
    solver = std::make_shared<pathplan::RRTStar>(metrics,checker,sampler);

  Eigen::Vector3d start_conf;
  Eigen::Vector3d goal_conf;
  start_conf = {0.0,0.0,0.0};
  goal_conf = {0.8,0.8,0.8};

  pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);
  pathplan::NodePtr goal_node  = std::make_shared<pathplan::Node>(goal_conf);

  pathplan::PathPtr solution;
  ROS_INFO_STREAM("Computing a path using "<<solver_name);
  bool success = solver->computePath(start_node, goal_node, nh, solution);

  if(success)
  {
    ROS_INFO_STREAM("Path found!\n"<<*solution);
    display.displayTree(solution->getTree());

    display.changeConnectionSize();
    display.changeNodeSize();
    display.displayPathAndWaypoints(solution,"pathplan",{0,0,1,1});
  }
  else
    ROS_INFO("No path found!");

  return 0;
}
