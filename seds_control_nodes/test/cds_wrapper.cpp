#include <gtest/gtest.h>

#include <seds_control_nodes/cds_wrapper.hpp>

class CdsWrapperTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      initCdsParams();
      q_start.resize(7);
      q_start.data <<  -1.0241795778274536, 0.8732185363769531, 0.2493722140789032, -0.5471154451370239, 0.0947522297501564, 0.7503224015235901, -0.33370718359947205;
      dt = 0.002;
      object_frame = KDL::Frame(KDL::Rotation::Quaternion(-0.413, -0.692, -0.170, 0.567), KDL::Vector(-0.251, 0.428, 0.846));
      attractor_frame = KDL::Frame::Identity();
    }

    virtual void TearDown()
    {
    }

    CdsWrapperParams params;
    KDL::JntArray q_start;
    double dt;
    KDL::Frame object_frame, attractor_frame;

    void initCdsParams()
    {
      unsigned int num_states, num_vars;
      GMMStates* master_gmm = readGMMStatesFromFile("test_data/masterGMM.txt", num_states, num_vars);
      params.cds_params_.master_dyn_ = new GMRDynamics(master_gmm, num_states, num_vars);

      GMMStates* slave_gmm = readGMMStatesFromFile("test_data/slaveGMM.txt", num_states, num_vars);
      params.cds_params_.slave_dyn_ =  new GMRDynamics(slave_gmm, num_states, num_vars);

      GMMStates* coupling_gmm = readGMMStatesFromFile("test_data/cplGMM.txt", num_states, num_vars);
      params.cds_params_.coupling_ = new GMR(coupling_gmm, num_states, num_vars);

      params.cds_params_.alpha_ = 10;
      params.cds_params_.beta_ = 1; 
      params.cds_params_.lambda_ = 1;
      params.cds_params_.reachingThreshold_ = 0.001;
      params.cds_params_.dt_ = dt;

      params.arm_params_.robot_model_.initFile("test_data/boxy_fixed_torso_description.urdf");
      params.arm_params_.root_name_ = "calib_left_arm_base_link";
      params.arm_params_.tip_name_ = "left_arm_7_link";
      params.arm_params_.lambda_ = 0.1;
      params.arm_params_.joint_weights_ = Eigen::MatrixXd::Identity(7,7);
      params.arm_params_.task_weights_ = Eigen::MatrixXd::Identity(6,6);
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

    void printFrame(const KDL::Frame& frame) const
    {
      std::cout << "p: " << frame.p.x() << ", " << frame.p.y() << ", " << frame.p.z() << "\n";
      double x, y, z, w;
      frame.M.GetQuaternion(x, y, z, w);
      std::cout << "M: " << x << ", " << y << ", " << z << ", " << w << "\n";
    }

    void printConfig(const KDL::JntArray& q) const
    {
      std::cout << "q: ";
      for(unsigned int i=0; i<q.rows(); i++)
        std::cout << q(i) << ", ";
      std::cout << "\n";
    }
};

TEST_F(CdsWrapperTest, Init)
{
  CdsWrapper cds_controller(params);
  cds_controller.setGoal(object_frame, attractor_frame);
  KDL::JntArray q = q_start;
  printConfig(q_start);

  for(unsigned int i=0; i<10; i++)
  {
    KDL::JntArray q_dot = cds_controller.update(q, dt);
    KDL::Multiply(q_dot, dt, q_dot);
    KDL::Add(q, q_dot, q);
    std::cout << "GOAL:\n";
    printFrame(object_frame);
    std::cout << "\n"; 
    printConfig(q);
  }
}
