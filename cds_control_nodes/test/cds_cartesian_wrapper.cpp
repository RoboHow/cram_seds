#include <gtest/gtest.h>

#include <seds_control_nodes/cds_cartesian_wrapper.hpp>

class CdsCartesianWrapperTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      initParams();
      pose_start = KDL::Frame(
//          KDL::Rotation::Quaternion(-0.394948, 0.779915, 0.163176, 0.457299),
//          KDL::Vector(0.526501, -1.0469, 1.07618));
            KDL::Rotation(-0.1950, -0.4692, 0.8613, -0.4962, 0.8047, 0.3260, -0.8460, -0.3638, -0.3898),
            KDL::Vector(-0.1862, -0.1699, 0.1124));
 
//            KDL::Rotation(-0.6314, 0.3683, 0.0689, 0.1038, 0.0905, 0.0912, 0.9958, 0.1032, 0.9945),
//            KDL::Vector(-0.7004, 0.1520, 0.2));
    }
 
    virtual void TearDown()
    {
    }

    CDSExecutionParams params;
    KDL::Frame pose_start;
    double dt;

    void initParams()
    {
      dt = 0.001;
      unsigned int num_states, num_vars;
      GMMStates* master_gmm = readGMMStatesFromFile("test_data/masterGMM.txt", num_states, num_vars);
      params.master_dyn_ = new GMRDynamics(master_gmm, num_states, num_vars);

      GMMStates* slave_gmm = readGMMStatesFromFile("test_data/slaveGMM.txt", num_states, num_vars);
      params.slave_dyn_ =  new GMRDynamics(slave_gmm, num_states, num_vars);

      GMMStates* coupling_gmm = readGMMStatesFromFile("test_data/cplGMM.txt", num_states, num_vars);
      params.coupling_ = new GMR(coupling_gmm, num_states, num_vars);

      params.alpha_ = 1;
      params.beta_ = 1; 
      params.lambda_ = 1;
      params.reachingThreshold_ = 0.001;
      params.dt_ = dt;

      params.object_frame_ = KDL::Frame(
          KDL::Rotation(-0.6314, 0.3683, 0.0689, 0.1038, 0.0905, 0.0912, 0.9958, 0.1032, 0.9945),
          KDL::Vector(-0.7004, 0.1520, 0.2));
      params.attractor_frame_ = KDL::Frame::Identity();
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
};
 
TEST_F(CdsCartesianWrapperTest, Looping)
{
  CdsCartesianWrapper cds_controller(params, pose_start);
//  std::cout << "GOAL:\n";
//  printFrame(params.object_frame_);
//  std::cout << "\nSTART:\n";
//  printFrame(pose_start);
//  std::cout << "\n\n\n";

  unsigned int counter = 0;
  KDL::Frame simulated_state = pose_start;
  for(unsigned int i=0; i<20; i++)
  {
//    if(counter == 0)
//      printFrame(simulated_state);

    simulated_state = cds_controller.update(simulated_state);

    counter++;
    if(counter == 10)
      counter=0;
  }

  EXPECT_TRUE(KDL::Equal(params.object_frame_.p, simulated_state.p, 0.01));
  EXPECT_TRUE(KDL::Equal(params.object_frame_.M, simulated_state.M, 0.5));
}
