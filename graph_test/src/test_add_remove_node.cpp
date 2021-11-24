#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/solvers/rrt_connect.h>
#include <graph_core/solvers/birrt.h>
#include <graph_core/solvers/rrt_star.h>
#include <graph_core/solvers/path_solver.h>
#include <graph_core/tube_informed_sampler.h>
#include <graph_core/parallel_moveit_collision_checker.h>
#include <graph_core/graph/subtree.h>
#include <graph_core/moveit_collision_checker.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <graph_core/graph/graph_display.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node_test_add_remove_node");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  // ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  std::string group_name = "cartesian_arm";
  std::string last_link = "end_effector";
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(group_name);
  std::vector<std::string> joint_names = joint_model_group->getActiveJointModelNames();

  unsigned int dof = joint_names.size();
  Eigen::VectorXd lb(dof);
  Eigen::VectorXd ub(dof);

  for (unsigned int idx = 0; idx < dof; idx++)
  {
    const robot_model::VariableBounds& bounds = kinematic_model->getVariableBounds(joint_names.at(idx));  //bounds dei joints definito in urdf e file joints limit
    if (bounds.position_bounded_)
    {
      lb(idx) = bounds.min_position_;
      ub(idx) = bounds.max_position_;
    }
  }

  // //////////////////////////////////////////UPDATING PLANNING SCENE////////////////////////////////////////////////////////
  ros::ServiceClient ps_client=nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");

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

  // //////////////////////////////////////////PATH PLAN & VISUALIZATION////////////////////////////////////////////////////////
  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::ParallelMoveitCollisionChecker>(planning_scene, group_name);

  pathplan::DisplayPtr disp = std::make_shared<pathplan::Display>(planning_scene,group_name,last_link);
  disp->clearMarkers();
  ros::Duration(1).sleep();

  Eigen::VectorXd start_conf(3);
  start_conf << 0.0,0.0,0.0;
  Eigen::VectorXd goal_conf(3);
  goal_conf << 0.8,0.8,0.8;

  int id=100;
  int id_wp = 1000;

  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
  pathplan::BiRRTPtr solver = std::make_shared<pathplan::BiRRT>(metrics,checker,sampler);

  pathplan::PathPtr current_path;
  pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);
  pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);
  solver->computePath(start_node, goal_node, nh, current_path);

  disp->displayPathAndWaypoints(current_path,id,id_wp,"pathplan",{0.5,0.5,0.0,1.0});

  ros::Duration(0.5).sleep();

  Eigen::VectorXd parent = current_path->getConnections().at(1)->getParent()->getConfiguration();
  Eigen::VectorXd child = current_path->getConnections().at(1)->getChild()->getConfiguration();

  Eigen::VectorXd current_configuration1 = parent + (child-parent)*0.5;
  Eigen::VectorXd current_configuration2 = parent + (child-parent)*0.2;
  Eigen::VectorXd current_configuration3 = parent + (child-parent)*0.9;

  parent = current_path->getConnections().at(2)->getParent()->getConfiguration();
  child = current_path->getConnections().at(2)->getChild()->getConfiguration();

  Eigen::VectorXd current_configuration4 = parent + (child-parent)*0.5;
  Eigen::VectorXd current_configuration5 = parent + (child-parent)*0.2;

  // //////////////////////// VISUALIZATION OF CURRENT NODE ////////////////////////

  disp->nextButton();
  pathplan::NodePtr node2add = current_path->addNodeAtCurrentConfig(current_configuration1,true);
  node2add = current_path->addNodeAtCurrentConfig(current_configuration2,true);
  node2add = current_path->addNodeAtCurrentConfig(current_configuration3,true);

  node2add = current_path->addNodeAtCurrentConfig(current_configuration4,true);
  node2add = current_path->addNodeAtCurrentConfig(current_configuration5,true);

  disp->clearMarkers();
  disp->displayPathAndWaypoints(current_path);
  disp->nextButton();

  std::vector<pathplan::NodePtr> white_list;
  white_list.push_back(node2add);
  if(!current_path->removeNodes(white_list))
    ROS_ERROR("NOT REMOVOVED");

  disp->clearMarkers();
  ros::Duration(1).sleep();
  disp->displayPathAndWaypoints(current_path);

  return 0;
}

