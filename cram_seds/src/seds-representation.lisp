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

(defclass seds-phase ()
  ((id :initarg :id :reader id :type string 
       :documentation "Knowrob identifier of the motion phase.")
   (models :initarg :models :reader models :type list
           :documentation "List of motion models describing the motion phase."))
  (:documentation "Single phase of a SEDS motion description."))

(defclass seds-motion-model () 
  ((id :initarg :id :reader id :type string
       :documentation "Knowrob identifier of the motion model.")
   (model-type :initarg :model-type :reader model-type :type string
               :documentation "Type of the motion model.")
   (gmms :initarg :gmms :reader gmms :type list
         :documentation "List of Gaussian Mixture Models of the motion model."))
  (:documentation "Single motion model used in SEDS motion descriptions."))

(defclass seds-gmm ()
  ((gmm-type :initarg :gmm-type :reader gmm-type :type string)
   (in-type :initarg :in-type :reader in-type :type string)
   (out-type :initarg :out-type :reader out-type :type string)
   (in-dim :initarg :in-dim :reader in-dim :type vector)
   (out-dim :initarg :out-dim :reader out-dim :type vector)
   (gaussians :initarg :gaussians :reader gaussians :type list))
  (:documentation "GMM representation for SEDS motion descriptions."))

(defclass seds-gaussian ()
  ((id :initarg :id :reader id :type string)
   (prior :initarg :prior :reader prior :type number)
   (mean :initarg :mean :reader mean :type vector)
   (covariance :initarg :covariance :reader covariance :type vector))
  (:documentation "Gaussian distribution representation for SEDS motion descriptions."))
       
(defun make-seds-phase (&key id models)
  ;; TODO(Georg): declare type
  (make-instance 'seds-phase :id id :models models))

(defun make-seds-motion-model (&key id type gmms)
  ;; TODO(Georg): declare type
  (make-instance 'seds-motion-model :id id :model-type type :gmms gmms))

(defun make-seds-gmm (&key type in-type out-type in-dim out-dim gaussians)
  ;; TODO(Georg): declare type
  (make-instance 'seds-gmm :gmm-type type :in-type in-type :out-type out-type
                 :in-dim in-dim :out-dim out-dim :gaussians gaussians))
   
(defun make-seds-gaussian (&key id prior mean covariance)
  ;; TODO(Georg): declare type
  (make-instance 'seds-gaussian :id id :prior prior :mean mean :covariance covariance))