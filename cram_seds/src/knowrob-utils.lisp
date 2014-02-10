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

(defun read-all-seds-phases-from-database ()
  (let* ((seds-bindings 
           (cut:force-ll
            (json-prolog:prolog-simple
             (concatenate
              'string
              "owl_direct_subclass_of(Phase,"
              "'http://ias.cs.tum.edu/kb/knowrob-seds.owl#SEDSMotion')"))))
         (seds-phases
           (mapcar
            (lambda (seds-binding)
              (cut:with-vars-bound (|?Phase|) seds-binding
                (query-phase
                 (knowrob-symbol->string |?Phase| nil))))
            seds-bindings)))
    seds-phases))

(defun query-phase (phase-knowrob-name)
  "Reads SEDS phase with knowrob name `phase-knowrob-name' from Knowrob and returns an
 instance of type 'seds-phase' filled with the read content."
  (declare (type string phase-knowrob-name))
  (let* ((model-bindings 
           (cut:force-ll 
            (json-prolog:prolog-simple
             (concatenate 'string
                          "phase_properties("
                          phase-knowrob-name
                          ", Models),"
                          "member(Model, Models)"))))
         (models (mapcar (lambda (model-binding)
                         (cut:with-vars-bound (|?Model|) model-binding
                           (query-motion-model
                            (knowrob-symbol->string |?Model| nil))))
                       model-bindings)))
    (make-seds-phase :id phase-knowrob-name :models models)))

(defun query-motion-model (model-knowrob-name)
  "Reads the content of the motion model with knowrob name `model-knowrob-name' and returns
 an instance of type 'seds-motion-model' filled with data read from knowrob."
  (declare (type string model-knowrob-name))
  (let ((type (query-motion-type model-knowrob-name))
        (gmms (query-motion-gmms model-knowrob-name)))
    (make-seds-motion-model :id model-knowrob-name :type type :gmms gmms)))

(defun query-motion-type (model-knowrob-name)
  "Reads the type of motion model with knowrob name `model-knowrob-name' and returns it as
 a string."
  (declare (type string model-knowrob-name))
  (let* ((type-binding 
           (cut:lazy-car 
            (json-prolog:prolog-simple
             (concatenate 'string
                          "motion_properties("
                          model-knowrob-name
                          ", Type, _)"))))
         (type (cut:with-vars-bound (|?Type|) type-binding
                 (knowrob-symbol->string |?Type| nil))))
    type))

(defun query-motion-gmms (model-knowrob-name)
  "Reads the GMMs of motion model with knowrob name `model-knowrob-name' and returns them
 as a list of instances of type 'seds-gmm'."
  (declare (type string model-knowrob-name))
  (let* ((gmm-bindings (cut:force-ll 
                        (json-prolog:prolog-simple
                         (concatenate 'string
                                      "motion_properties("
                                      model-knowrob-name
                                      ", _, GMMs),"
                                      "member(GMM, GMMs)"))))
         (gmms (mapcar (lambda (gmm-binding)
                         (cut:with-vars-bound (|?GMM|) gmm-binding
                           (query-gmm
                            (knowrob-symbol->string |?GMM| nil))))
                       gmm-bindings)))
    gmms))

(defun query-gmm (gmm-knowrob-name)
  "Reads out the gmm with knowrob name `gmm-knowrob-name' and returns the result as an
 instance of type 'seds-gmm'."
  (declare (type string gmm-knowrob-name))
  (multiple-value-bind (type in-type in-dim out-type out-dim)
      (query-gmm-properties gmm-knowrob-name)
    (let ((gaussians (query-gmm-gaussians gmm-knowrob-name)))
      (make-seds-gmm
       :id gmm-knowrob-name
       :type type
       :in-type in-type
       :out-type out-type
       :in-dim in-dim
       :out-dim out-dim
       :gaussians gaussians))))

(defun query-gmm-properties (gmm-knowrob-name)
  "Reads out the non-gaussian properties of the gmm with knowrob name `gmm-knowrob-name', 
 and returns the results as values."
  (let* ((query
           (concatenate 
            'string 
            "gmm_properties(" 
            gmm-knowrob-name 
            ", Type, InType, InDim, OutType, OutDim, _)"))
         (bindings (cut:lazy-car (json-prolog:prolog-simple query))))
    (cut:with-vars-bound (|?Type| |?InType| |?InDim| |?OutType| |?OutDim|) bindings
      (values (knowrob-symbol->string |?Type| nil)
              (knowrob-symbol->string |?InType|)
              (coerce (mapcar #'knowrob-symbol->string |?InDim|) 'vector)
              (knowrob-symbol->string |?OutType|)
              (coerce (mapcar #'knowrob-symbol->string |?OutDim|) 'vector)))))

(defun query-gmm-gaussians (gmm-knowrob-name)
  "Reads out gaussians of gmm with knowrob name `gmm-knowrob-name', and returns the result
 as a list of instances of type 'seds-gaussian'."
  (declare (type string gmm-knowrob-name))
  (let* ((query
           (concatenate 
            'string 
            "gmm_properties(" 
            gmm-knowrob-name 
            ", _, _, _, _, _, Gaussians),"
            "member(Gaussian, Gaussians)"))
         (bindings (cut:force-ll (json-prolog:prolog-simple query)))
         (gaussians (mapcar (lambda (binding)
                              (cut:with-vars-bound (|?Gaussian|) binding
                                (query-gaussian (knowrob-symbol->string |?Gaussian| nil))))
                            bindings)))
     gaussians))

(defun query-gaussian (gauss-knowrob-name)
  "Reads out the description of the Gaussian with knowrob name `gauss-knowrob-name' and 
 returns an instance of type 'seds-gaussian' filled with content read from Knowrob."
  (declare (type string gauss-knowrob-name))
  (let ((gaussian-query 
          (concatenate 'string "gaussian_components(" gauss-knowrob-name ", Mean, Cov, Prior)")))
    (cut:with-vars-bound (|?Mean| |?Cov| |?Prior|)
        (cut:lazy-car
         (json-prolog:prolog-simple gaussian-query))
      (let ((mean (query-mean (knowrob-symbol->string |?Mean| nil )))
            (covariance (query-covariance (knowrob-symbol->string |?Cov| nil)))
            (prior |?Prior|))
        (make-seds-gaussian
         :id gauss-knowrob-name
         :prior prior
         :mean mean
         :covariance covariance)))))

(defun query-covariance (cov-knowrob-name)
  "Reads out the covariance matrix with knowrob name `mean-knowrob-name' and returns the result
 as a vector of numbers."
  (declare (type string cov-knowrob-name))
  (let ((query (concatenate 'string "matrix_elements(" cov-knowrob-name ", CovMatrix)")))
    (cut:with-vars-bound (|?CovMatrix|)
        (cut:lazy-car (json-prolog:prolog-simple query))
      (when |?CovMatrix|
        (coerce |?CovMatrix| 'vector)))))

(defun query-mean (mean-knowrob-name)
  "Reads out mean vector with knowrob name `mean-knowrob-name' and returns the result as a 
 vector of numbers."
  (declare (type string mean-knowrob-name))
  (let ((query (concatenate 'string "vector_elements(" mean-knowrob-name ", MeanVector)")))
    (cut:with-vars-bound (|?MeanVector|)
        (cut:lazy-car (json-prolog:prolog-simple query))
      (when |?MeanVector|
        (coerce |?MeanVector| 'vector)))))
  
(defun knowrob-symbol->string (knowrob-symbol &optional (remove-quotes t))
  "Takes a 'knowrob-symbol' as typically returned when asking knowrob through
 json-prolog-client and returns the equivalent string. If remove-quotes is not NIL, the
 first and last character of the name of the symbol will be removed."
  (declare (type symbol knowrob-symbol))
  (let ((long-symbol-name (symbol-name knowrob-symbol)))
    (unless (> (length long-symbol-name) 1)
      (error
       'simple-error
       :format-control "Tried removing quotes from string with less than 2 symbols: ~a"
       :format-arguments '(long-symbol-name)))
    (if remove-quotes
        (subseq long-symbol-name 1 (- (length long-symbol-name) 1))
        long-symbol-name)))