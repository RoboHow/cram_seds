#include <gtest/gtest.h>

#include <seds_control_nodes/cds_wrapper.hpp>

class CdsWrapperTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      initParams();
      q_start.resize(7);
      q_start.data <<  -1.2786, 0.6182, -2.0733, -1.5392, 1.0747, 0.9713, 0.3275;
   }

    virtual void TearDown()
    {
    }

    CdsWrapperParams params;
    KDL::JntArray q_start;
    double dt;

    void initParams()
    {
      dt = 0.01;
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

//TODO(Georg): UNDO THIS!!
      params.cds_params_.object_frame_ = KDL::Frame(
//          KDL::Rotation(-0.6314, 0.3683, 0.0689, 0.1038, 0.0905, 0.0912, 0.9958, 0.1032, 0.9945), 
          KDL::Rotation::Identity(),
          KDL::Vector(0.7, -0.8, 0.8));
      params.cds_params_.attractor_frame_ = KDL::Frame::Identity();
 
      params.arm_params_.robot_model_.initFile("test_data/boxy_fixed_torso_description.urdf");
      params.arm_params_.root_name_ = "base_link";
      params.arm_params_.tip_name_ = "right_arm_7_link";
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
//      double x, y, z, w;
//      frame.M.GetQuaternion(x, y, z, w);
//      std::cout << "M: " << x << ", " << y << ", " << z << ", " << w << "\n";
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
  CdsWrapper cds_controller(params, q_start);
  KDL::JntArray q = q_start;
  std::cout << "GOAL:\n";
  printFrame(params.cds_params_.object_frame_);
  std::cout << "\n"; 

  printFrame(cds_controller.arm_.get_pos_fk(q)); 
  
  unsigned int counter = 0;
  for(unsigned int i=0; i<2000; i++)
  {
    KDL::JntArray q_dot = cds_controller.update(q, dt);
    KDL::Multiply(q_dot, dt, q_dot);
    KDL::Add(q, q_dot, q);

    counter++;
    if(counter == 100)
    {
      printFrame(cds_controller.arm_.get_pos_fk(q)); 
      counter = 0;
    }
  }
}
