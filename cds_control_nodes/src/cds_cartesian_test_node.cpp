#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>
#include <seds_control_nodes/cds_cartesian_wrapper.hpp>

class CdsCartesianTestNode
{
  public:
    CdsCartesianTestNode(const ros::NodeHandle& nh) : nh_(nh)
    {
      this->init();
    }

    ~CdsCartesianTestNode() {}

    void run()
    {
      ros::Rate rate(1.0/dt_);

      while(ros::ok())
      {
        simulateOneCycle();
        ros::spinOnce();
        rate.sleep();
      }
    }
     
  private:
    ros::NodeHandle nh_;
    tf::TransformBroadcaster br;
    CdsCartesianWrapper cds_;
    double dt_;
    std::string child_frame_, parent_frame_;
    tf::Transform state_;

    void init()
    {
      nh_.param<double>("dt", dt_, 0.01);

      KDL::Frame start_pose = readStartPose();
      tf::transformKDLToTF(start_pose, state_);

      cds_.init(readCDSParams(), start_pose);
    } 

    void simulateOneCycle()
    {
      KDL::Frame state_copy;
      tf::transformTFToKDL(state_, state_copy);
      tf::transformKDLToTF(cds_.update(state_copy), state_);

      broadcastTF();
    }

    void broadcastTF()
    {
      br.sendTransform(tf::StampedTransform(state_, ros::Time::now(),
          parent_frame_, child_frame_));
    }

    CDSExecutionParams readCDSParams()
    {
      CDSExecutionParams cds_params;
      unsigned int num_states, num_vars;

      std::string filename = "";

      nh_.getParam("master_gmm_file", filename);
      GMMStates* master_gmm = readGMMStatesFromFile(filename.c_str(),
          num_states, num_vars);
      cds_params.master_dyn_ = new GMRDynamics(master_gmm, num_states, num_vars);

      nh_.getParam("slave_gmm_file", filename);
     
      GMMStates* slave_gmm = readGMMStatesFromFile(filename.c_str(),
          num_states, num_vars);
      cds_params.slave_dyn_ =  new GMRDynamics(slave_gmm, num_states, num_vars);

      nh_.getParam("coupling_gmm_file", filename);
      GMMStates* coupling_gmm = readGMMStatesFromFile(filename.c_str(),
          num_states, num_vars);
      cds_params.coupling_ = new GMR(coupling_gmm, num_states, num_vars);

      nh_.param<double>("alpha", cds_params.alpha_, 10);
      nh_.param<double>("beta", cds_params.beta_, 1); 
      nh_.param<double>("lambda", cds_params.lambda_, 1);
      nh_.param<double>("reaching_threshold", cds_params.reachingThreshold_, 0.001);
      nh_.param<double>("dt", cds_params.dt_, 0.01);

      nh_.getParam("parent_frame", parent_frame_);
      nh_.getParam("child_frame", child_frame_);

      cds_params.object_frame_ = readFrameFromParameterServer("object_frame");
      cds_params.attractor_frame_ = readFrameFromParameterServer("attractor_frame");

      return cds_params;
    }

    KDL::Frame readStartPose()
    {
      return readFrameFromParameterServer("start_pose");
    }

    KDL::Frame readFrameFromParameterServer(const std::string& param_name)
    {
      double x, y, z, q_x, q_y, q_z, q_w;

      nh_.param<double>(param_name + "/origin/x", x, 0.0);
      nh_.param<double>(param_name + "/origin/y", y, 0.0);
      nh_.param<double>(param_name + "/origin/z", z, 0.0);
      nh_.param<double>(param_name + "/orientation/x", q_x, 0.0);
      nh_.param<double>(param_name + "/orientation/y", q_y, 0.0);
      nh_.param<double>(param_name + "/orientation/z", q_z, 0.0);
      nh_.param<double>(param_name + "/orientation/w", q_w, 1.0);

      return KDL::Frame(KDL::Rotation::Quaternion(q_x, q_y, q_z, q_w),
          KDL::Vector(x, y, z));
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
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cds_control_node");
  ros::NodeHandle nh("~");
  CdsCartesianTestNode my_test(nh);
  my_test.run();
  return 0;
}
