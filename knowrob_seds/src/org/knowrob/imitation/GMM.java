package org.knowrob.imitation;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.GMatrix;
import javax.vecmath.GVector;

import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLDataProperty;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import edu.tum.cs.ias.knowrob.owl.OWLThing;

public class GMM {

	protected String name = "";

	protected  ArrayList<Double> priors;
	protected List<GVector> means;
	protected List<GMatrix> covs;
	
	public GMM(String name) {

		this.name = name;
		this.priors = new ArrayList<Double>();
		this.means  = new ArrayList<GVector>();
		this.covs   = new ArrayList<GMatrix>();

	}

	public void readFromFile(String filename) {
		
		BufferedReader br = null;
		try {
			String cur;
			br = new BufferedReader(new FileReader(filename));
 
			// read dimensions
			
			while((cur = br.readLine()).equals("")) {}
			
			int m = (int) Math.round(Double.valueOf(cur.replaceAll("\\s+","")));
			while((cur = br.readLine()).equals("")) {}
			
			int n = (int) Math.round(Double.valueOf(cur.replaceAll("\\s+","")));
			while((cur = br.readLine()).equals("")) {}
			
			// set up lists of vectors and matrices for these dimensions:
			for(int i=0;i<m;i++) {
				means.add(new GVector(new double[n]));
			}
			for(int i=0;i<m;i++) {
				covs.add(new GMatrix(n,n));
			}
			
			
			
			// read 'm' priors
			for(String p : cur.trim().split("\\s+")) {
				priors.add(Double.valueOf(p));
			}
			while((cur = br.readLine()).equals("")) {}
			
			
			// read 'm' mean vectors of dimension 'n' each
			for(int j=0;j<n;j++) {
							
				String[] elems = cur.trim().split("\\s+");
				
				// read 'jth' element of mean vector
				for(int i=0;i<m;i++) {
					means.get(i).setElement(j, Double.valueOf(elems[i]));
				}
				while((cur = br.readLine()).equals("")) {}	
			}
			
			
			// read 'm'covariance matrices of dimension 'nxn' each
			
			for(int k=0;k<m;k++) {
			
				for(int i=0;i<n;i++) {

					String[] row = cur.trim().split("\\s+");

					// read 'jth' element of mean vector
					for(int j=0;j<n;j++) {
						covs.get(k).setElement(i, j, Double.valueOf(row[j]));
					}
					
					cur = br.readLine();
					if(cur == null || cur.equals("")) break;
				}
				
				while((cur = br.readLine()) != null) {
					if(!cur.equals(""))
						break;
				}
			}
			
			
		} catch (IOException e) {
			e.printStackTrace();
			
		} finally {
			try {
				if (br != null)
					br.close();
			} catch (IOException ex) {
				ex.printStackTrace();
			}
		}
	}

	

	public OWLNamedIndividual writeToOWL(OWLOntologyManager manager, OWLDataFactory factory, DefaultPrefixManager pm, OWLOntology ontology) {

		// create GMM class
		OWLClass gmmType = factory.getOWLClass(IRI.create(GMMToOWL.CONSTR + "GaussianMixtureModel"));
		OWLNamedIndividual gmmInd = factory.getOWLNamedIndividual(OWLThing.getUniqueID("knowrob:"+name), pm);
		manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(gmmType, gmmInd));
 
		ArrayList<OWLNamedIndividual> gaussians = new ArrayList<OWLNamedIndividual>();
		OWLClass gauss_class = factory.getOWLClass("knowrob:GaussianDistribution", pm);
		
		// create instances of GaussianDistributions that hold the respective mean/cov/prior values
		for(int i = 0; i < means.size(); i++) {
			OWLNamedIndividual g = factory.getOWLNamedIndividual(OWLThing.getUniqueID("knowrob:GaussianDistribution"), pm);
			manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(gauss_class, g));

			// link to GMM class
			OWLObjectProperty pGauss = factory.getOWLObjectProperty(IRI.create(GMMToOWL.CONSTR + "gaussianDist"));
			manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(pGauss,  gmmInd, g));
			
			gaussians.add(g);
		}
		
		// create instances for mean vectors
		for(int g = 0; g < means.size();g++) {
			
			GVector vec = means.get(g);

			// create vector instance
			OWLClass vec_class = factory.getOWLClass("knowrob:Vector", pm);
			OWLNamedIndividual vec_inst = factory.getOWLNamedIndividual(OWLThing.getUniqueID("knowrob:Vector"), pm);
			manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(vec_class, vec_inst));

			// set vector elements
			for(int i=0;i<vec.getSize();i++) {
				OWLDataProperty prop = factory.getOWLDataProperty("knowrob:vec"+i, pm);
				manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(prop,  vec_inst, vec.getElement(i)));
			}
			
			// link to Gaussian instance
			OWLObjectProperty pMean = factory.getOWLObjectProperty(IRI.create(GMMToOWL.CONSTR + "mean"));
			manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(pMean, gaussians.get(g), vec_inst));
		}
		
		// create instances for covariance matrices
		for(int g = 0; g < covs.size();g++) {
			
			GMatrix mat=covs.get(g);
		
			// create matrix instance
			OWLClass mat_class = factory.getOWLClass("knowrob:Matrix", pm);
			OWLNamedIndividual mat_inst = factory.getOWLNamedIndividual(OWLThing.getUniqueID("knowrob:Matrix"), pm);
			manager.addAxiom(ontology, factory.getOWLClassAssertionAxiom(mat_class, mat_inst));

			// set pose properties
			for(int i=0;i<mat.getNumRow();i++) {
				for(int j=0;j<mat.getNumCol();j++) {

					OWLDataProperty prop = factory.getOWLDataProperty("knowrob:m"+i+j, pm);
					manager.addAxiom(ontology, factory.getOWLDataPropertyAssertionAxiom(prop,  mat_inst, mat.getElement(i, j)));
				}
			}

			// link to Gaussian instance
			OWLObjectProperty pMean = factory.getOWLObjectProperty(IRI.create(GMMToOWL.CONSTR + "cov"));
			manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(pMean, gaussians.get(g), mat_inst));
		}
		
		return gmmInd;
	}

}
