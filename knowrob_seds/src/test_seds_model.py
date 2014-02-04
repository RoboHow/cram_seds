#!/usr/bin/env python

import roslib;
roslib.load_manifest('knowrob_seds')

import rospy
import json_prolog
from std_msgs.msg import String
from robohow_common_msgs.msg import MotionPhase
from robohow_common_msgs.msg import MotionModel
from robohow_common_msgs.msg import GaussianMixtureModel
from robohow_common_msgs.msg import GaussianDistribution


if __name__ == '__main__':
  
    rospy.init_node('gmm_dummy_publisher')
    pub = rospy.Publisher('gmm_msgs', MotionPhase)
    
    prolog = json_prolog.Prolog()

    try:
        
        query = prolog.query("""rdfs_subclass_of(Phase, seds:'SEDSMotion'),
                                phase_properties(Phase, Models),
                                member(Model, Models),
                                motion_properties(Model, Type, GMMs),
                                member(GMM, GMMs),
                                gmm_properties(GMM, GMMType, InputType, InputDim, OutputType, OutputDim, Gaussians),
                                member(Gaussian, Gaussians),
                                gaussian_components(Gaussian, Mean, Cov, Prior),
                                vector_elements(Mean, MeanVec),
                                matrix_elements(Cov, CovMat)""")
        
        for solution in query.solutions():

            gauss = GaussianDistribution(
                solution['Gaussian'],
                solution['Prior'],
                solution['MeanVec'],
                solution['CovMat'])

                #String(solution['GMM'].split('#')[1]),
            #gmm = GaussianMixtureModel(
                #"sdf",
                #solution['GMMType'],
                #solution['InputType'],
                #solution['InputType'],
                #solution['InputDim'],
                #solution['InputDim'],
                #[gauss] )
            gmm = GaussianMixtureModel()
            gmm.id = solution['GMM']
            gmm.gaussian_dist.append(gauss)
            gmm.type = solution['GMMType']
            gmm.input_type  = solution['InputType']
            gmm.output_type = solution['InputType']
            gmm.input_dim   = solution['InputDim']
            gmm.output_dim  = solution['InputDim']

            m = MotionModel()
            m.id = solution['Model']
            m.type = solution['Type']
            m.described_by_GMM.append(gmm)
            
            p = MotionPhase()
            p.id = solution['Phase']
            p.described_by_motion_model.append(m)
            
            pub.publish(p)
            rospy.sleep(1.0)

            # also print some parts to the terminal
            print "\n\n= = = = = = = = = = = = = = = = = = = = = "
            print solution['Phase'].split('#')[1]
            print solution['Model'].split('#')[1]
            print solution['GMM'].split('#')[1]
            print solution['Gaussian'].split('#')[1]
            print 'Mean = %s' % (solution['MeanVec'])
            print 'Cov  = %s' % (solution['CovMat'])
            print 'Prior  = %s' % (solution['Prior'])
        query.finish()
        
    except rospy.ROSInterruptException:
        pass
