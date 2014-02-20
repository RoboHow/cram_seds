#include <ros/ros.h>
#include <seds_control_nodes/cds_wrapper.hpp>

class CdsControlNode
{
  public:
    CdsControlNode(const ros::NodeHandle& nh) : nh_(nh)
    {
      this->init();
    }

    ~CdsControlNode() {}

  private:
    ros::NodeHandle nh_;
    CdsWrapper cds_;

    void init()
    {
      ArmKinematicsParams arm_params = readArmKinematicsParameters();
    }

    ArmKinematicsParams readArmKinematicsParameters() const
    {
      std::string root_link, tip_link;
      nh_.param<std::string>("root_link", root_link, "calib_right_arm_base_link");
      nh_.param<std::string>("tip_link", tip_link, "right_arm_7_link");

      double dt, lambda;
      nh_.param<double>("lambda", lambda, 0.1);
      nh_.param<double>("control_dt", dt, 0.002);
 
      int dof;
      nh_.param<int>("dof", dof, 7);

      Eigen::MatrixXd task_weights = Eigen::MatrixXd::Identity(6,6);
      Eigen::MatrixXd joint_weights = Eigen::MatrixXd::Identity(dof, dof);

      urdf::Model robot_model;
      std::string robot_model_param;
      nh_.param<std::string>("robot_model_param", robot_model_param, "robot_description");
      if(!robot_model.initParam(robot_model_param))
        throw;

      ArmKinematicsParams arm_params;
      arm_params.robot_model_ = robot_model;
      arm_params.root_name_ = root_link;
      arm_params.tip_name_ = tip_link;
      arm_params.lambda_ = lambda;
      arm_params.task_weights_ = task_weights;
      arm_params.joint_weights_ = joint_weights; 
 
      return arm_params;
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cds_control_node");
  ros::NodeHandle nh("~");
  CdsControlNode my_controller(nh);
  while(ros::ok())
    ros::spinOnce();
  return 0;
}
