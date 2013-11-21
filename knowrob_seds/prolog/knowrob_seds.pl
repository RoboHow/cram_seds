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
      vector_elements/2,
      matrix_elements/2,
      motion_gmms/2,
      motion_master_gmm/2,
      motion_slave_gmm(/2,
      motion_force_gmm/2,
      motion_coupling_gmm/2,
      gmm_gaussians/2,
      gaussian_components/4
    ]).

:- use_module(library('semweb/rdfs')).
:- use_module(library('thea/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('semweb/rdfs_computable')).
:- use_module(library('jpl')).

:-  rdf_meta
      vector_elements(r, ?),
      matrix_elements(r, ?),
      motion_gmms(r,r),
      motion_master_gmm(r,r),
      motion_slave_gmm(r,r),
      motion_force_gmm(r,r),
      motion_coupling_gmm(r,r),
      gmm_gaussians(r,r),
      gaussian_components(r,r,r,r).


:- rdf_db:rdf_register_ns(rdf, 'http://www.w3.org/1999/02/22-rdf-syntax-ns#', [keep(true)]).
:- rdf_db:rdf_register_ns(owl, 'http://www.w3.org/2002/07/owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://ias.cs.tum.edu/kb/knowrob.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(xsd, 'http://www.w3.org/2001/XMLSchema#', [keep(true)]).

:- rdf_db:rdf_register_ns(seds, 'http://ias.cs.tum.edu/kb/knowrob-seds.owl#', [keep(true)]).


motion_gmms(Motion, GMM) :-
  class_properties(Motion, P, GMM),
  rdfs_subproperty_of(P, seds:describedByGMM).

motion_master_gmm(Motion, GMM) :-
  class_properties(Motion, P, GMM),
  rdfs_subproperty_of(P, seds:masterGMM).
motion_slave_gmm(Motion, GMM) :-
  class_properties(Motion, P, GMM),
  rdfs_subproperty_of(P, seds:slaveGMM).
motion_force_gmm(Motion, GMM) :-
  class_properties(Motion, P, GMM),
  rdfs_subproperty_of(P, seds:forceGMM).
motion_coupling_gmm(Motion, GMM) :-
  class_properties(Motion, P, GMM),
  rdfs_subproperty_of(P, seds:couplingGMM).

gmm_gaussians(GMM, Gaussian) :-
  rdf_has(GMM, seds:gaussianDist, Gaussian).

gaussian_components(Gaussian, Mean, Cov, Prior) :-
  rdf_has(Gaussian, seds:mean, Mean),
  rdf_has(Gaussian, seds:cov,  Cov),
  Prior = 0. % TODO: represent and query for prior
  %rdf_has(Gaussian, seds:prior, Mean).

matrix_elements(Mat, ElemList) :-

  findall(P-V, (rdfs_subproperty_of(Prop, knowrob:matrixElement),
                \+ rdf_equal(Prop, knowrob:matrixElement),
                rdf_split_url(_, P, Prop),
                rdf_has(Mat, Prop, literal(type(xsd:double,Va))),
                term_to_atom(V, Va)), PVs),
  keysort(PVs, PVsorted),
  pairs_values(PVsorted, ElemList).


vector_elements(Vec, ElemList) :-

  findall(P-V, (rdfs_subproperty_of(Prop, knowrob:vectorElement),
                \+ rdf_equal(Prop, knowrob:vectorElement),
                rdf_split_url(_, P, Prop),
                rdf_has(Vec, Prop, literal(type(xsd:double,Va))),
                term_to_atom(V, Va)), PVs),
  keysort(PVs, PVsorted),
  pairs_values(PVsorted, ElemList).


