package org.knowrob.imitation;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.GMatrix;
import javax.vecmath.GVector;

import org.semanticweb.owlapi.model.AddAxiom;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import edu.tum.cs.ias.knowrob.owl.OWLThing;

public class MotionPhase {

	protected String name = "";

	protected ArrayList<Double> priors;
	protected List<GVector> means;
	protected List<GMatrix> covs;

	GMM master;
	GMM slave;
	GMM coupling;
	GMM force;
	
	public MotionPhase(String name) {

		this.name = name;
		master    = new GMM("MasterDynGMM");
		slave     = new GMM("SlaveDynGMM");
		coupling  = new GMM("CouplingDynGMM");
		force     = new GMM("ForceDynGMM");
	}

	
	public void readFromFile(String gmmFolder) {
		
		File dir = new File(gmmFolder);
		for(String f : dir.list()) {
			if(f.equals("masterDyn.gmm")) {
				master.readFromFile(gmmFolder + "/" + f);
			} else if(f.equals("slaveDyn.gmm")) {
				slave.readFromFile(gmmFolder + "/" + f);
			} else if(f.equals("couplingDyn.gmm")) {
				coupling.readFromFile(gmmFolder + "/" + f);	
			} else if(f.equals("forceDyn.gmm")) {
//				force.readFromFile(gmmFolder + "/" + f);	
			} 
		}
	}
	

	public OWLClass writeToOWL(OWLOntologyManager manager, OWLDataFactory factory, DefaultPrefixManager pm, OWLOntology ontology) {

		// create class for phase
		OWLClass phaseCls = factory.getOWLClass(IRI.create(OWLThing.getUniqueID(GMMToOWL.KNOWROB + name)));
		
		
		OWLClass phaseType = factory.getOWLClass(IRI.create(GMMToOWL.SEDS + "SEDSMotion"));
		manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(phaseCls, phaseType)));	


		OWLObjectProperty masterGMM = factory.getOWLObjectProperty(IRI.create(GMMToOWL.SEDS + "masterGMM"));
		OWLNamedIndividual masterInst = master.writeToOWL(manager, factory, pm, ontology);
		manager.applyChange(new AddAxiom(ontology, 
				factory.getOWLSubClassOfAxiom(phaseCls, 
				factory.getOWLObjectHasValue(masterGMM, masterInst)))); 
		
		OWLObjectProperty slaveGMM = factory.getOWLObjectProperty(IRI.create(GMMToOWL.SEDS + "slaveGMM"));
		OWLNamedIndividual slaveInst = slave.writeToOWL(manager, factory, pm, ontology);
		manager.applyChange(new AddAxiom(ontology, 
				factory.getOWLSubClassOfAxiom(phaseCls, 
				factory.getOWLObjectHasValue(slaveGMM, slaveInst)))); 
		
		OWLObjectProperty couplingGMM = factory.getOWLObjectProperty(IRI.create(GMMToOWL.SEDS + "couplingGMM"));
		OWLNamedIndividual couplingInst = coupling.writeToOWL(manager, factory, pm, ontology);
		manager.applyChange(new AddAxiom(ontology, 
				factory.getOWLSubClassOfAxiom(phaseCls, 
				factory.getOWLObjectHasValue(couplingGMM, couplingInst)))); 
		
		OWLObjectProperty forceGMM = factory.getOWLObjectProperty(IRI.create(GMMToOWL.SEDS + "forceGMM"));
		OWLNamedIndividual forceInst = master.writeToOWL(manager, factory, pm, ontology);
		manager.applyChange(new AddAxiom(ontology, 
				factory.getOWLSubClassOfAxiom(phaseCls, 
				factory.getOWLObjectHasValue(forceGMM, forceInst)))); 

		return phaseCls;
	}
}
