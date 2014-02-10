;;; Copyright (c) 2014, Georg Bartels <georg.bartels@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-seds)

(defgeneric to-msg (data))

(defmethod to-msg ((data seds-phase))
  (roslisp:make-msg
   "robohow_common_msgs/MotionPhase"
   :id (id data)
   :described_by_motion_model (coerce (mapcar #'to-msg (models data)) 'vector)))

(defmethod to-msg ((data seds-motion-model))
  (roslisp:make-msg
   "robohow_common_msgs/MotionModel"
   :id (id data)
   :type (model-type data)
   :described_by_gmm (coerce (mapcar #'to-msg (gmms data)) 'vector)))

(defmethod to-msg ((data seds-gmm))
  (roslisp:make-msg
   "robohow_common_msgs/GaussianMixtureModel"
   :id (id data)
   :type (gmm-type data)
   :input_type (in-type data)
   :output_type (out-type data)
   :input_dim (in-dim data)
   :output_dim (out-dim data)
   :gaussian_dist (coerce (mapcar #'to-msg (gaussians data)) 'vector)))
   
(defmethod to-msg ((data seds-gaussian))
  (roslisp:make-msg 
   "robohow_common_msgs/GaussianDistribution"
   :id (id data)
   :prior (prior data)
   :mean (mean data)
   :cov (covariance data)))