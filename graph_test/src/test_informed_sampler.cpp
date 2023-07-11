#include <ros/ros.h>

#include <graph_core/informed_sampler.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <graph_core/graph/graph_display.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "node");
  ros::NodeHandle nh;

  ros::AsyncSpinner aspin(4);
  aspin.start();

  bool display;
  nh.getParam("display",display);

  std::string group_name;
  nh.getParam("group_name",group_name);

  std::vector<double> scale_vector;
  nh.getParam("scale_vector",scale_vector);

  std::vector<double> cost_factor;
  nh.getParam("cost_factor",cost_factor);

  std::vector<double> start;
  nh.getParam("start",start);

  std::vector<double> goal;
  nh.getParam("goal",goal);

  int n_test_per_cost_factor;
  nh.getParam("n_test_per_cost_factor",n_test_per_cost_factor);

  moveit::planning_interface::MoveGroupInterface move_group(group_name);
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr       kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScenePtr planning_scene = std::make_shared<planning_scene::PlanningScene>(kinematic_model);

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

  pathplan::Display disp(planning_scene,group_name);

  Eigen::VectorXd start_conf = Eigen::Map<Eigen::VectorXd>(start.data(), start.size());
  Eigen::VectorXd goal_conf  = Eigen::Map<Eigen::VectorXd>(goal.data(), goal.size());
  Eigen::VectorXd scale = Eigen::Map<Eigen::VectorXd>(scale_vector.data(), scale_vector.size());

  ros::WallDuration(5.0).sleep();

  Eigen::VectorXd q;
  double start_q, q_goal, start_q2, q_goal2;
  for(const double& cf:cost_factor)
  {
    pathplan::NodePtr n = std::make_shared<pathplan::Node>(start_conf);
    disp.displayNode(n);
    ros::WallDuration(0.5).sleep();
    n = std::make_shared<pathplan::Node>(goal_conf);
    disp.displayNode(n);
    ros::WallDuration(0.5).sleep();

    double cost = (start_conf-goal_conf).cwiseProduct(scale).norm()*cf;
    pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(start_conf,goal_conf,lb,ub,scale,cost);

    ROS_WARN_STREAM("Setting cost to "<<cost);

    for(int i=0;i<n_test_per_cost_factor;i++)
    {
      q = sampler->sample();
      start_q = ((q-start_conf).cwiseProduct(scale)).norm();
      q_goal = ((goal_conf-q).cwiseProduct(scale)).norm();

      start_q2 = (q.cwiseProduct(scale) - start_conf.cwiseProduct(scale)).norm();
      q_goal2 = (q.cwiseProduct(scale)  - goal_conf.cwiseProduct(scale)).norm();

      if(std::abs(start_q-start_q2)>1e-06)
      {
        ROS_INFO_STREAM("start_q "<<start_q<<" start_q2 "<<start_q2);
        throw std::runtime_error("start-q different");
      }
      else
        ROS_INFO_STREAM("check start ok:  "<<start_q<<" = "<<start_q2);

      if(std::abs(q_goal-q_goal2)>1e-06)
      {
        ROS_INFO_STREAM("q_goal "<<q_goal<<" q_goal2 "<<q_goal2);
        throw std::runtime_error("q_goal different");
      }
      else
        ROS_INFO_STREAM("check goal ok:  "<<q_goal<<" = "<<q_goal2);

      if(not sampler->inBounds(q))
        throw std::runtime_error("q not in bounds");
      else
        ROS_INFO("q in bounds");

      if((start_q+q_goal)-cost>1e-06)
      {
        if(display)
        {
          n = std::make_shared<pathplan::Node>(q);
          disp.displayNode(n,"pathpalan",{0.0,1.0,0.0,1.0});
          ROS_INFO_STREAM("dist s-q "<<(q-start_conf).norm()<<" dist q-goal "<<(goal_conf-q).norm());
          ROS_INFO("from start to q: %f, from q to goal: %f, cost %f",start_q,q_goal,cost);
          ROS_INFO_STREAM("diff "<<std::abs(cost-start_q-q_goal));

          throw std::runtime_error("q not inside prolate hyperspheroid");
        }
      }
      else
      {
        if(display)
        {
          n = std::make_shared<pathplan::Node>(q);
          disp.displayNode(n,"pathpalan",{0.0,1.0,0.0,0.3});
        }
      }
    }

    ROS_INFO("Press any button");
    std::cin.get();
    disp.clearMarkers();
    ros::WallDuration(0.5).sleep();
    disp.clearMarkers();
    ros::WallDuration(0.5).sleep();
    disp.clearMarkers();
    ros::WallDuration(0.5).sleep();
  }


  return 0;
}
