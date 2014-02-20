#include <ros/ros.h>
#include <seds_control_nodes/cds_wrapper.hpp>
#include <dlr_msgs/rcu2tcu.h>
#include <realtime_bridge_msgs/ImpedanceCommand.h>
#include <seds_control_nodes/msg_conversions.hpp>

class CdsControlNode
{
  public:
    CdsControlNode(const ros::NodeHandle& nh) : nh_(nh), controller_running_(false)
    {
      this->init();
    }

    ~CdsControlNode() {}

  private:
    ros::NodeHandle nh_;
    ros::Subscriber robot_subscriber_;
    ros::Publisher robot_publisher_;
    realtime_bridge_msgs::ImpedanceCommand command_msg_;

    CdsWrapper cds_;
    ArmKinematicsParams arm_params_;
    bool controller_running_;
    double dt_;
    KDL::JntArray q_, qdot_des_;

    void init()
    {
      int dof;
      nh_.param<int>("dof", dof, 7);

      nh_.param<double>("control_dt", dt_, 0.002);

      q_.resize(dof);
      qdot_des_.resize(dof);

      arm_params_ = readArmKinematicsParameters(dof);

      command_msg_ = initCommandMsg(dof);

      robot_subscriber_ = initSubscriber();
      robot_publisher_ = initPublisher();

      stopController();
   }

    ArmKinematicsParams readArmKinematicsParameters(unsigned int dof) const
    {
      std::string root_link, tip_link;
      nh_.param<std::string>("root_link", root_link, "calib_right_arm_base_link");
      nh_.param<std::string>("tip_link", tip_link, "right_arm_7_link");

      double lambda;
      nh_.param<double>("lambda", lambda, 0.1);
 
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

    realtime_bridge_msgs::ImpedanceCommand initCommandMsg(unsigned int dof) const
    {
      realtime_bridge_msgs::ImpedanceCommand msg;

      msg.velocity.resize(dof);
      msg.stiffness.resize(0);
      msg.damping.resize(0);
      msg.add_torque.resize(0);

      for(unsigned int i=0; i<dof; i++)
        msg.velocity[i] = 0.0;

      return msg;
    }

    ros::Publisher initPublisher()
    {
      return nh_.advertise<realtime_bridge_msgs::ImpedanceCommand>("robot_command_topic", 1);
    }
    
    ros::Subscriber initSubscriber()
    {
      return nh_.subscribe("robot_state_topic", 1, 
          &CdsControlNode::robotStateCallback, this);
    }

    void robotStateCallback(const dlr_msgs::rcu2tcu::ConstPtr& msg)
    {
      if(controller_running_)
      {
        fromMsg(*msg, q_);

        qdot_des_ = cds_.update(q_, dt_);

        toMsg(qdot_des_, command_msg_);
        robot_publisher_.publish(command_msg_);
      }
    }

    void startController()
    {
      controller_running_ = true;
    }

    void stopController()
    {
      controller_running_ = false;
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
