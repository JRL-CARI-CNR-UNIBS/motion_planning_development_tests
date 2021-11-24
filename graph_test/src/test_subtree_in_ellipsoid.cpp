#include <ros/ros.h>
#include <graph_core/graph/subtree.h>
#include <graph_core/moveit_collision_checker.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <graph_core/graph/graph_display.h>
#include <graph_core/solvers/anytime_rrt.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_AnytimeRRT");
  ros::NodeHandle nh;

  ros::AsyncSpinner aspin(4);

  std::string group_name = "cartesian_arm";
  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

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

  pathplan::Display display(planning_scene,group_name);

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
  checker->setPlanningSceneMsg(ps_srv.response.scene);

  Eigen::Vector3d start_conf;
  Eigen::Vector3d goal_conf;
  start_conf = {0.0,0.0,0.0};
  goal_conf = {0.8,0.8,0.8};

  pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);
  pathplan::NodePtr goal_node  = std::make_shared<pathplan::Node>(goal_conf);

  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf, goal_conf, lb, ub);
  pathplan::RRTPtr solver = std::make_shared<pathplan::RRT>(metrics,checker,sampler);
  solver->setMaxDistance(0.1);

  pathplan::PathPtr path;
  bool success = solver->computePath(start_node, goal_node,nh,path);

  if(success)
  {
    display.clearMarkers();
    ros::Duration(1).sleep();

    display.displayPath(path);

    display.nextButton("Press next to display the tree");
    display.clearMarkers();
    ros::Duration(1).sleep();

    ROS_INFO_STREAM("Tree number of nodes: "<<path->getTree()->getNodes().size());
    display.displayTree(path->getTree());

    display.nextButton("Press next to display the subtree");
    display.clearMarkers();
    ros::Duration(1).sleep();

    pathplan::NodePtr root = path->getNodes().at(3);
    pathplan::SubtreePtr subtree = pathplan::Subtree::createSubtree(path->getTree(),root);

    display.changeNodeSize({0.04,0.04,0.04});
    display.displayNode(root);
    display.defaultNodeSize();

    display.displaySubtree(subtree);

    display.nextButton("Press next to display the subtree inside the ellipsoid");
    display.clearMarkers();
    ros::Duration(1).sleep();

    pathplan::NodePtr goal = path->getNodes().back();
    double cost = 1.2*(goal->getConfiguration()-root->getConfiguration()).norm();
    ROS_INFO_STREAM("COST: "<<cost);
    pathplan::SubtreePtr purged_subtree = pathplan::Subtree::createSubtree(path->getTree(),root,root->getConfiguration(),goal->getConfiguration(),cost);

    display.changeNodeSize({0.04,0.04,0.04});
    display.displayNode(root);
    display.defaultNodeSize();

    ROS_INFO("Subtree in ellipsoid -> nodes:");
    for(const pathplan::NodePtr& n:purged_subtree->getNodes())
    {
      ROS_INFO_STREAM(n->getConfiguration().transpose());
      display.displayNode(n);
    }

    display.displaySubtree(purged_subtree);
    ros::Duration(1).sleep();
  }
  else
  {
    ROS_ERROR("Path not found");
  }

  return 0;
}
