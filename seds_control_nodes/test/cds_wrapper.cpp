#include <gtest/gtest.h>

#include <seds_control_nodes/cds_wrapper.hpp>

class CdsWrapperTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      initCdsParams();
      q_start.resize(7);
      q_start.data << 0, DEG2RAD(10), 0, DEG2RAD(-90), 0, DEG2RAD(45), DEG2RAD(45);
      dt = 0.002;
    }

    virtual void TearDown()
    {
    }

    CdsWrapperParams params;
    KDL::JntArray q_start;
    double dt;

    void initCdsParams()
    {
      unsigned int num_states, num_vars;
      GMMStates* master_gmm = readGMMStatesFromFile("test_data/masterGMM.txt", num_states, num_vars);
      params.cds_params_.master_dyn_ = new GMRDynamics(master_gmm, num_states, num_vars);

      GMMStates* slave_gmm = readGMMStatesFromFile("test_data/slaveGMM.txt", num_states, num_vars);
      params.cds_params_.slave_dyn_ =  new GMRDynamics(slave_gmm, num_states, num_vars);

      GMMStates* coupling_gmm = readGMMStatesFromFile("test_data/cplGMM.txt", num_states, num_vars);
      params.cds_params_.coupling_ = new GMR(coupling_gmm, num_states, num_vars);

      params.cds_params_.alpha_ = 1;
      params.cds_params_.beta_ = 1; 
      params.cds_params_.lambda_ = 1;
      params.cds_params_.reachingThreshold_ = 1;
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
};

TEST_F(CdsWrapperTest, Init)
{
}
