#!/usr/bin/env python

import roslib; roslib.load_manifest('json_prolog')

import rospy
import json_prolog

if __name__ == '__main__':
    rospy.init_node('test_json_prolog')
    prolog = json_prolog.Prolog()
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
        print "\n\n= = = = = = = = = = = = = = = = = = = = = "
        print solution['Phase'].split('#')[1]
        print solution['Model'].split('#')[1]
        print solution['GMM'].split('#')[1]
        print solution['Gaussian'].split('#')[1]
        print 'Mean = %s' % (solution['MeanVec'])
        print 'Cov  = %s' % (solution['CovMat'])
        print 'Prior  = %s' % (solution['Prior'])
    query.finish()
