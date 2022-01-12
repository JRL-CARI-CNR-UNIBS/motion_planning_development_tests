#include <ros/ros.h>
#include <graph_core/solvers/multigoal.h>
#include <graph_core/narrow_pass_checker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_multigoal");
  ros::NodeHandle nh("~");


  double cylinder_radius=1;
  if (!nh.getParam("cylinder_radius",cylinder_radius))
  {
    ROS_ERROR("cylinder_radius is not set");
    cylinder_radius=1;
  }

  double cylinder_width=1;
  if (!nh.getParam("cylinder_width",cylinder_width))
  {
    ROS_ERROR("cylinder_width is not set");
    cylinder_width=1;
  }

  double measures_ratio=0.5;
  if (!nh.getParam("measures_ratio",measures_ratio))
  {
    ROS_ERROR("measures_ratio is not set");
    measures_ratio=0.5;
  }

  int dof;
  if (!nh.getParam("dof",dof))
  {
    ROS_ERROR("dof is not set");
    dof=2;
  }

  int maximum_iter=1000000;
  if (!nh.getParam("maximum_iter",maximum_iter))
  {
    ROS_ERROR("maximum_iter is not set");
    maximum_iter=1000000;
  }



  Eigen::VectorXd lb(dof);
  Eigen::VectorXd ub(dof);
  lb.setConstant(-5.0);
  ub.setConstant( 5.0);

  double hole_radius=cylinder_radius*std::pow(measures_ratio,1.0/(static_cast<double>(dof)-1.0));
  double b=cylinder_width*0.6;
  double a=(cylinder_radius+3*hole_radius)/4.0;

  pathplan::MetricsPtr metrics = std::make_shared<pathplan::Metrics>();
  pathplan::CollisionCheckerPtr checker = std::make_shared<pathplan::NarrowPassChecker>(hole_radius,cylinder_radius,cylinder_width);
  pathplan::SamplerPtr sampler = std::make_shared<pathplan::InformedSampler>(lb, ub, lb, ub);




  ros::WallTime t0=ros::WallTime::now();

  ros::NodeHandle mg_nh=ros::NodeHandle("~multigoal");

  pathplan::MultigoalSolver mg(metrics,checker,sampler);
  if (not mg.config(mg_nh))
  {
    ROS_ERROR("unable to config multigoal");
    return 0;
  }

  Eigen::VectorXd start_conf(dof);
  start_conf.setConstant(0);
  start_conf(0) = -b;
  start_conf(1) = a;

  Eigen::VectorXd goal_conf(dof);
  goal_conf.setConstant(0);
  goal_conf(0) = b;
  goal_conf(1) =  a;

  pathplan::NodePtr start_node = std::make_shared<pathplan::Node>(start_conf);
  pathplan::NodePtr goal_node = std::make_shared<pathplan::Node>(goal_conf);

  mg.addStart(start_node);
  mg.addGoal(goal_node);

//  for (int idx=0;idx<100;idx++)
//  {
//    goal_conf.setRandom();
//    pathplan::NodePtr goal2 = std::make_shared<pathplan::Node>(10*goal_conf);
//    mg.addGoal(goal2);
//  }
  if (mg.completed())
  {
    ROS_INFO("found direct solution");
    return 0;
  }
  ROS_INFO("Start planning");

  pathplan::PathPtr solution;
  double cost=0;
  bool solved=false;

  printf("iteration, cost, execution time (seconds), nodes\n");

  for (int idx=0; idx<= maximum_iter;idx++)
  {
    mg.update(solution);
    cost=mg.getCost();
    if (mg.solved() && not solved)
    {
      ROS_DEBUG("found a solution. iteration %d cost=%f",idx,cost);
      ROS_DEBUG_STREAM("solution\n"<<*solution);
      solved=true;
    }
    if (idx%100==0)
      printf("%10d, %12f, %10.4f, %15u;\n",idx,cost,(ros::WallTime::now()-t0).toSec(),mg.getStartTree()->getNumberOfNodes());
    if (not ros::ok())
      break;
  }

  ROS_INFO("Executed in %f seconds",(ros::WallTime::now()-t0).toSec());


  ros::spin();
  return 0;
}
