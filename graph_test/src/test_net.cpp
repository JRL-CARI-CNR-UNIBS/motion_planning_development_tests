#include <ros/ros.h>
#include <graph_core/metrics.h>
#include <graph_core/graph/node.h>
#include <graph_core/graph/path.h>
#include <graph_core/graph/tree.h>
#include <graph_core/graph/net.h>
#include <graph_core/graph/net_connection.h>
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
  ros::init(argc, argv, "node_net");
  ros::AsyncSpinner spinner(4);
  spinner.start();

  ros::NodeHandle nh;

  double k;
  if(!nh.getParam("k",k))
    k = 0.1;

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
  pathplan::TreePtr tree = current_path->getTree();

  ros::Duration(1.0).sleep();

  disp->displayPathAndWaypoints(current_path,id,id_wp,"pathplan",{0.5,0.5,0.0,1.0});
  disp->displayTree(tree);

  if(goal_node != current_path->getNodes().back())
    assert(0);

  pathplan::NodePtr node = tree->getNodes().at(4);
  pathplan::NetConnectionPtr net_conn = std::make_shared<pathplan::NetConnection>(node,goal_node);
  double cost = metrics->cost(node->getConfiguration(),goal_conf);
  net_conn->setCost(cost);
  net_conn->add();

  node = current_path->getNodes().at(13);
  pathplan::NodePtr goal2 = current_path->getNodes().at(current_path->getNodes().size()-2);
  net_conn = std::make_shared<pathplan::NetConnection>(node,goal2);
  cost = metrics->cost(node->getConfiguration(),goal2->getConfiguration());
  net_conn->setCost(cost);
  net_conn->add();

  pathplan::NodePtr path_node = current_path->getNodes().at(3);

  // Loop 1
  Eigen::VectorXd q1,q2;
  q1 = path_node->getConfiguration();
  q1(0) = q1(0)+k;
  q2 = q1;
  q2(1) = q1(1)+k;

  pathplan::NodePtr node1 = std::make_shared<pathplan::Node>(q1);
  pathplan::NodePtr node2 = std::make_shared<pathplan::Node>(q2);

  net_conn = std::make_shared<pathplan::NetConnection>(node1,path_node);
  cost = metrics->cost(node1->getConfiguration(),path_node->getConfiguration());
  net_conn->setCost(cost);
  net_conn->add();

  disp->displayConnection(net_conn);
  ros::Duration(0.1).sleep();

  pathplan::ConnectionPtr conn = std::make_shared<pathplan::Connection>(node2,node1);
  cost = metrics->cost(node2->getConfiguration(),node1->getConfiguration());
  conn->setCost(cost);
  conn->add();

  disp->displayConnection(conn);
  ros::Duration(0.1).sleep();

  conn = std::make_shared<pathplan::Connection>(path_node,node2);
  cost = metrics->cost(path_node->getConfiguration(),node2->getConfiguration());
  conn->setCost(cost);
  conn->add();

  disp->displayConnection(conn);
  ros::Duration(0.1).sleep();

  // Loop 2
  path_node = current_path->getNodes().at(18);

  q1 = path_node->getConfiguration();
  q1(0) = q1(0)+k;
  q2 = q1;
  q2(1) = q1(1)+k;

  node1 = std::make_shared<pathplan::Node>(q1);
  node2 = std::make_shared<pathplan::Node>(q2);

  net_conn = std::make_shared<pathplan::NetConnection>(node1,path_node);
  cost = metrics->cost(node1->getConfiguration(),path_node->getConfiguration());
  net_conn->setCost(cost);
  net_conn->add();

  disp->displayConnection(net_conn);
  ros::Duration(0.1).sleep();

  conn = std::make_shared<pathplan::Connection>(node2,node1);
  cost = metrics->cost(node2->getConfiguration(),node1->getConfiguration());
  conn->setCost(cost);
  conn->add();

  disp->displayConnection(conn);
  ros::Duration(0.1).sleep();

  path_node = current_path->getNodes().at(20);
  conn = std::make_shared<pathplan::Connection>(path_node,node2);
  cost = metrics->cost(path_node->getConfiguration(),node2->getConfiguration());
  conn->setCost(cost);
  conn->add();

  disp->displayConnection(conn);
  ros::Duration(0.1).sleep();

  pathplan::NetPtr net = std::make_shared<pathplan::Net>(tree);
  std::multimap<double,std::vector<pathplan::ConnectionPtr>> map_of_paths;
  map_of_paths = net->getConnectionToNode(goal_node);

  ROS_WARN("Getting the all possible paths to goal, without considering infinite loops..");

  for(const std::pair<double,std::vector<pathplan::ConnectionPtr>> pair:map_of_paths)
  {
    disp->nextButton();
    disp->clearMarkers();

    ROS_INFO_STREAM("path size:"<<pair.second.size());

    pathplan::PathPtr path = std::make_shared<pathplan::Path>(pair.second,metrics,checker);
    disp->displayPathAndWaypoints(path,"pathplan",{0.0,0.0,1.0,1.0},false);

    for(const Eigen::VectorXd wp:path->getWaypoints())
      ROS_INFO_STREAM("WP: "<<wp.transpose());
  }

  ROS_WARN("Getting the all possible paths to goal, without considering infinite loops, starting from a node different from the tree root");

  node = tree->getNodes().at(2);
  map_of_paths = net->getConnectionBetweenNodes(node,goal_node);

  for(const std::pair<double,std::vector<pathplan::ConnectionPtr>> pair:map_of_paths)
  {
    disp->nextButton();
    disp->clearMarkers();

    pathplan::PathPtr path = std::make_shared<pathplan::Path>(pair.second,metrics,checker);
    disp->displayPathAndWaypoints(path,"pathplan",{0.0,0.0,1.0,1.0},false);

    for(const Eigen::VectorXd wp:path->getWaypoints())
      ROS_INFO_STREAM("WP: "<<wp.transpose());
  }

  ROS_WARN("Getting the net paths between two nodes");

  ROS_WARN("1) A path should exist");
  node = tree->getNodes().at(3);
  map_of_paths = net->getNetConnectionBetweenNodes(node,goal_node);

  for(const std::pair<double,std::vector<pathplan::ConnectionPtr>> pair:map_of_paths)
  {
    disp->nextButton();
    disp->clearMarkers();

    pathplan::PathPtr path = std::make_shared<pathplan::Path>(pair.second,metrics,checker);
    disp->displayPathAndWaypoints(path,"pathplan",{0.0,0.0,1.0,1.0},false);

    for(const Eigen::VectorXd wp:path->getWaypoints())
      ROS_INFO_STREAM("WP: "<<wp.transpose());
  }

  ROS_WARN("2) A path should exist");
  map_of_paths = net->getNetConnectionBetweenNodes(node,goal2);

  for(const std::pair<double,std::vector<pathplan::ConnectionPtr>> pair:map_of_paths)
  {
    disp->nextButton();
    disp->clearMarkers();

    pathplan::PathPtr path = std::make_shared<pathplan::Path>(pair.second,metrics,checker);
    disp->displayPathAndWaypoints(path,"pathplan",{0.0,0.0,1.0,1.0},false);

    for(const Eigen::VectorXd wp:path->getWaypoints())
      ROS_INFO_STREAM("WP: "<<wp.transpose());
  }

  ROS_WARN("3) A path should NOT exist");
  path_node = tree->getNodes().at(10);
  map_of_paths = net->getNetConnectionBetweenNodes(node,path_node);

  disp->clearMarkers();
  ros::Duration(0.1).sleep();
  disp->clearMarkers();

  for(const std::pair<double,std::vector<pathplan::ConnectionPtr>> pair:map_of_paths)
  {
    disp->nextButton();
    disp->clearMarkers();

    pathplan::PathPtr path = std::make_shared<pathplan::Path>(pair.second,metrics,checker);
    disp->displayPathAndWaypoints(path,"pathplan",{0.0,0.0,1.0,1.0},false);

    for(const Eigen::VectorXd wp:path->getWaypoints())
      ROS_INFO_STREAM("WP: "<<wp.transpose());
  }

  return 0;
}

