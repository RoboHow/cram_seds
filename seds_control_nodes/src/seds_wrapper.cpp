#include "include/seds_control_nodes/seds_wrapper.hpp"
#include <iostream>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <ros/ros.h>

using namespace std;

void SedsWrapper::parse_urdf(const string &param_name, const string &root_name,
    const string &tip_name) 
{
  KDL::Tree tree;
  KDL::Chain chain;

  if (!kdl_parser::treeFromParam(param_name, tree)) {
    ROS_ERROR("Could not parse the parameter into a kdl_tree.");
    throw;
  }

  if (!tree.getChain(root_name, tip_name, chain)) {
    ROS_ERROR("Could not initialize chain object");
    throw;
  }

  dof_ = chain.getNrOfJoints();

  fk_solver_ = new KDL::ChainFkSolverPos_recursive(chain);
//  q_tmp_.resize(dof_);
}

KDL::Frame SedsWrapper::get_fk(const KDL::JntArray& q)
{
  assert(q.rows() == dof_);

  KDL::Frame pose;
  fk_solver_->JntToCart(q, pose);

  return pose;
}

int main(int argc, char **argv)
{

  std::cout << "lalala\n";

  ros::init(argc, argv, "seds_wrapper");
  //ros::NodeHandle n;

  SedsWrapper sedswrapper;

  //Get better frames into the URDF
  sedswrapper.parse_urdf("robot_description", "base_link", "left_arm_7_link");

  KDL::JntArray jntarray(7);
  for(unsigned int i=0; i < 7; i++)
    jntarray(i) = 0.0;
  KDL::Frame pose = sedswrapper.get_fk(jntarray);
  cout << "Position: " << pose.p(0) << "," << pose.p(1) << "," << pose.p(2) << endl;

  return 0;
};
