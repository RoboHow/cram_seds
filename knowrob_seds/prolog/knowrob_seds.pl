%%
%% Copyright (C) 2013 by Moritz Tenorth
%%
%% This module provides methods for representing and reasoning about
%% SEDS motion specifications in KnowRob.
%%
%% This program is free software; you can redistribute it and/or modify
%% it under the terms of the GNU General Public License as published by
%% the Free Software Foundation; either version 3 of the License, or
%% (at your option) any later version.
%%
%% This program is distributed in the hope that it will be useful,
%% but WITHOUT ANY WARRANTY; without even the implied warranty of
%% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%% GNU General Public License for more details.
%%
%% You should have received a copy of the GNU General Public License
%% along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%


:- module(knowrob_seds,
    [
      phase_properties/2,
      motion_properties/3,
      gmm_properties/6,
      gaussian_components/4,
      vector_elements/2,
      matrix_elements/2
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).

:-  rdf_meta
      phase_properties(r,r),
      motion_properties(r,r,?),
      gmm_properties(r,?,?,?,?,?),
      gaussian_components(r,r,r,r),
      vector_elements(r, ?),
      matrix_elements(r, ?).


:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:- rdf_db:rdf_register_ns(seds, 'http://ias.cs.tum.edu/kb/knowrob-seds.owl#', [keep(true)]).


%% phase_properties(MotionPhase, MotionModels) is nondet.
%
% Read properties of a MotionPhase, in particular the list
% of models describing it
%
% @param MotionPhase   Instance of a seds:'SEDSMotion'
% @param MotionModels  List of instances of seds:'MotionModel'
%
phase_properties(MotionPhase, MotionModels) :-
  findall(MotionModel,
      class_properties(MotionPhase, seds:describedByMotionModel, MotionModel),
      MotionModels).



%% motion_properties(MotionModel, Type, GMMs) is nondet.
%
% Read properties of a MotionModel, in particular its type and the
% GaussianMixtureModels describing it
%
% @param MotionModel  Instance of a motion model
% @param Type         Class of the motion model, subclass of seds:'MotionModel'
% @param GMMs         List of instances of seds:'GaussianMixtureModel'
%
motion_properties(MotionModel, Type, GMMs) :-
  rdf_has(MotionModel, rdf:type, Type),
  owl_subclass_of(Type, seds:'MotionModel'),
  findall(GMM, (rdf_has(MotionModel, seds:describedByGMM, GMM)), GMMs).




%% gmm_properties(GMM, InputType, InputDim, OutputType, OutputDim, Gaussians) is nondet.
%
% Read properties of the GaussianMixtureModel GMM
%
% @param GMM          Instance of a seds:'GaussianMixtureModel'
% @param InputType    String describing the quantity of the input values (e.g. Position, Velocity, Other)
% @param InputDim     List of atoms describing which dimensions are described by the input values (e.g. [x,z] or [*,*])
% @param OutputType   String describing the quantity of the output values (e.g. Position, Velocity, Other)
% @param OutputDim    List of atoms describing which dimensions are described by the output values (e.g. [x,z] or [*,*])
% @param Gaussians    List of instances of seds:'GaussianDistribution'
%
gmm_properties(GMM, InputType, InputDim, OutputType, OutputDim, Gaussians) :-
  rdf_has(GMM, seds:inputType,  literal(type('http://www.w3.org/2001/XMLSchema#string', InputType))),
  rdf_has(GMM, seds:inputDim,   literal(type('http://www.w3.org/2001/XMLSchema#string', InputDimStr))),
  atomic_list_concat(InputDim, '-', InputDimStr),
  rdf_has(GMM, seds:outputType, literal(type('http://www.w3.org/2001/XMLSchema#string', OutputType))),
  rdf_has(GMM, seds:outputDim,  literal(type('http://www.w3.org/2001/XMLSchema#string', OutputDimStr))),
  atomic_list_concat(OutputDim, '-', OutputDimStr),
  findall(G, rdf_has(GMM, seds:gaussianDist, G), Gaussians).




%% gaussian_components(+Gaussian, -Mean, -Cov, -Prior) is nondet.
%
% Read components of a Gaussian distribution
%
% @param Gaussian  Instance of seds:GaussianDistribution
% @param Mean      Mean vector (Instance of knowrob:Vector)
% @param Cov       Covariance matrix (Instance of knowrob:Matrix)
% @param Prior     Prior (double value denoting the prior for this Gaussian distribution in a GMM)
%
gaussian_components(Gaussian, Mean, Cov, Prior) :-
  rdf_has(Gaussian, seds:mean, Mean),
  rdf_has(Gaussian, seds:cov,  Cov),
  rdf_has(Gaussian, seds:prior, literal(type(xsd:double,P))),
  term_to_atom(Prior, P).



%% matrix_elements(+Mat, -ElemList) is nondet.
%
% Return all elements of the matrix Mat in list ElemList (row-based)
%
% @param Mat       Instance of knowrob:Matrix
% @param ElemList  Row-based list of elements of Mat
%
%
matrix_elements(Mat, ElemList) :-

  findall(P-V, (rdfs_subproperty_of(Prop, knowrob:matrixElement),
                \+ rdf_equal(Prop, knowrob:matrixElement),
                rdf_split_url(_, P, Prop),
                rdf_has(Mat, Prop, literal(type(xsd:double,Va))),
                term_to_atom(V, Va)), PVs),
  keysort(PVs, PVsorted),
  pairs_values(PVsorted, ElemList).



%% vector_elements(+Vec, -ElemList) is nondet.
%
% Return all elements of the vector Vec in list ElemList
%
% @param Vec       Instance of knowrob:Vector
% @param ElemList  Row-based list of elements of Vec
%
vector_elements(Vec, ElemList) :-

  findall(P-V, (rdfs_subproperty_of(Prop, knowrob:vectorElement),
                \+ rdf_equal(Prop, knowrob:vectorElement),
                rdf_split_url(_, P, Prop),
                rdf_has(Vec, Prop, literal(type(xsd:double,Va))),
                term_to_atom(V, Va)), PVs),
  keysort(PVs, PVsorted),
  pairs_values(PVsorted, ElemList).


