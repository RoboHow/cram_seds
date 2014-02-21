#include <ros/ros.h>
#include <seds_control_nodes/cds_wrapper.hpp>
#include <dlr_msgs/rcu2tcu.h>
#include <sensor_msgs/JointState.h>
#include <realtime_bridge_msgs/ImpedanceCommand.h>
#include <seds_control_nodes/msg_conversions.hpp>
#include <tf/transform_broadcaster.h>

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
    ros::Publisher robot_pos_publisher_;
    realtime_bridge_msgs::ImpedanceCommand command_msg_;
    sensor_msgs::JointState position_msg_;
    tf::TransformBroadcaster br;

    CdsWrapper cds_;
    bool controller_running_;
    double dt_;
    KDL::JntArray q_, qdot_des_;

    void broadcastTF(const KDL::Frame& frame)
    {
      tf::Transform transform;
      transform.setOrigin( tf::Vector3(frame.p.x(), frame.p.y(), frame.p.z()));
      double x,y,z,w;
      frame.M.GetQuaternion(x,y,z,w);
      transform.setRotation( tf::Quaternion(x,y,z,w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "base_link", "cds_des_pose"));
    }

    void init()
    {
      int dof;
      nh_.param<int>("dof", dof, 7);

      nh_.param<double>("control_dt", dt_, 0.01);

      using Eigen::operator<<;
      q_.resize(dof);

q_.data << -1.53904891014099, 0.602084994316101, -1.94086837768555,
           -1.68737697601318, 1.23532223701477, 1.22554242610931,
            0.398510962724686;
//      q_.data << -1.33658051490784, 0.406352370977402, -1.83030450344086,
//                 -1.66620182991028, 0.924314498901367, 1.16480243206024,
//                 0.636807262897492;
// q_.data << -1.2786, -0.6182, -2.0733, -1.5392, 1.0747, 0.9713, 0.3275;


      qdot_des_.resize(dof);
      qdot_des_.data << 0, 0, 0, 0, 0, 0, 0;

      CdsWrapperParams cds_params = initCdsWrapperParams(dof, dt_);
      cds_.init(cds_params, q_);

      command_msg_ = initCommandMsg(dof);
      position_msg_ = initPositionMsg();

      robot_subscriber_ = initSubscriber();
      robot_publisher_ = initPublisher();
      robot_pos_publisher_ = initPositionPublisher();

      startController();
    }

    CdsWrapperParams initCdsWrapperParams(unsigned int dof, double dt)
    {
      CdsWrapperParams cds_wrapper_params;
      cds_wrapper_params.cds_params_ = readCDSParameters(dt_);
      cds_wrapper_params.arm_params_ = readArmKinematicsParameters(dof);

      return cds_wrapper_params;
   }

    ArmKinematicsParams readArmKinematicsParameters(unsigned int dof) const
    {
      std::string root_link, tip_link;
      nh_.param<std::string>("root_link", root_link, "base_link");
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

    CDSExecutionParams readCDSParameters(double dt)
    {
      CDSExecutionParams cds_params;
      unsigned int num_states, num_vars;

      GMMStates* master_gmm = readGMMStatesFromFile("/home/georg/ros/hydro/rosbuild_ws/cram_seds/cds_control_nodes/test_data/masterGMM.txt", num_states, num_vars);
      cds_params.master_dyn_ = new GMRDynamics(master_gmm, num_states, num_vars);

      GMMStates* slave_gmm = readGMMStatesFromFile("/home/georg/ros/hydro/rosbuild_ws/cram_seds/cds_control_nodes/test_data/slaveGMM.txt", num_states, num_vars);
      cds_params.slave_dyn_ =  new GMRDynamics(slave_gmm, num_states, num_vars);

      GMMStates* coupling_gmm = readGMMStatesFromFile("/home/georg/ros/hydro/rosbuild_ws/cram_seds/cds_control_nodes/test_data/cplGMM.txt", num_states, num_vars);
      cds_params.coupling_ = new GMR(coupling_gmm, num_states, num_vars);

      cds_params.alpha_ = 10;
      cds_params.beta_ = 1; 
      cds_params.lambda_ = 1;
      cds_params.reachingThreshold_ = 0.001;
      cds_params.dt_ = dt;

      cds_params.object_frame_ = KDL::Frame(
          KDL::Rotation(-0.4481, -0.7341, 0.5103, -0.5061, 0.6788, 0.5321, -0.7370, -0.0198, -0.6757),
          KDL::Vector(0.6755, -0.8704, 0.9681));
      cds_params.attractor_frame_ = KDL::Frame::Identity();

      return cds_params;
    }


    sensor_msgs::JointState initPositionMsg() const
    {
      sensor_msgs::JointState msg;
      msg.position.resize(7);
      msg.name.resize(7);

      msg.name[0] = "right_arm_0_joint";
      msg.name[1] = "right_arm_1_joint";
      msg.name[2] = "right_arm_2_joint";
      msg.name[3] = "right_arm_3_joint";
      msg.name[4] = "right_arm_4_joint";
      msg.name[5] = "right_arm_5_joint";
      msg.name[6] = "right_arm_6_joint";
     
      return msg;
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

    GMMStates* readGMMStatesFromFile(const char* file, unsigned int& num_states, unsigned int& num_vars)
    {
      FILE *fid;
      int res;
      double dtemp;
      fid = fopen(file, "r");
      if(!fid)
      {
              cout<<"Error opening file \""<<file<<"\"\n";
              throw;
      }
      res=fscanf(fid, "%lf\n", &dtemp);
      num_states = (int)dtemp;
      res=fscanf(fid, "%lf\n", &(dtemp));
      num_vars = (int)dtemp;
      GMMStates* GMMState  = (GMMStates  *)malloc(num_states*sizeof(GMMStates ) );
      for( unsigned int s=0; s<num_states; s++ ){
              GMMState[s].Mu       = svector(num_vars);
              GMMState[s].Sigma    = smatrix(num_vars, num_vars );
      }
      // Read priors
      for( unsigned int s=0; s<num_states; s++ )
              res=fscanf(fid, "%lf", &(GMMState[s].Prio ) );

      // Read Mu
      for( unsigned int i=0; i<num_vars; i++ )
              for( unsigned int s=0; s<num_states; s++ )
                      res=fscanf(fid, "%lf", &(GMMState[s].Mu[i]) );
      // Read Sigmas
      for( unsigned int s=0; s<num_states; s++ )
              for( unsigned int i=0; i<num_vars; i++ )
                      for( unsigned int j=0; j<num_vars; j++ )
                              res=fscanf(fid, "%lf", &(GMMState[s].Sigma[i][j]));
      fclose(fid);

      return GMMState;
    }

    ros::Publisher initPublisher()
    {
      return nh_.advertise<realtime_bridge_msgs::ImpedanceCommand>("robot_command_topic", 1);
    }

    ros::Publisher initPositionPublisher()
    {
      return nh_.advertise<sensor_msgs::JointState>("sim_joint_state", 1);
    }
    
    ros::Subscriber initSubscriber()
    {
      return nh_.subscribe("robot_state_topic", 1, 
          &CdsControlNode::robotStateCallback, this);
    }

    void robotStateCallback(const dlr_msgs::rcu2tcu::ConstPtr& msg)
    {
int counter = 0;
      if(controller_running_)
      {
//SIMULATION!!
   //     fromMsg(*msg, q_);
if(counter==0)
{
KDL::Frame des_pose;
        qdot_des_ = cds_.update(q_, dt_, des_pose);

        toMsg(qdot_des_, command_msg_);

        robot_publisher_.publish(command_msg_);

//SIMULATION!!
KDL::Multiply(qdot_des_, dt_*0.01, qdot_des_);
KDL::Add(q_, qdot_des_, q_);

toMsg(q_, position_msg_);

robot_pos_publisher_.publish(position_msg_);
broadcastTF(des_pose);
}
counter++;
if(counter==10)
  counter = 0;
      }
    }

    // TODO(GEORG): add action client to get in SEDS description from high-level!

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
