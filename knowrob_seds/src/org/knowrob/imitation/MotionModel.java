package org.knowrob.imitation;

import java.io.File;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

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

public class MotionModel {

	String name;
	
	private enum ModelType {CDS, GMRDynamics, GMR};
	ModelType type;

	Set<GMM> models;
	
	
	
	/**
	 * Constructor
	 * 
	 * @param name Identifier of this motion model
	 */
	public MotionModel(String type) {
		
		// set model type
		if(type.equals("CDS"))
			this.type = ModelType.CDS;
		else if(type.equals("GMRDynamics"))
			this.type = ModelType.GMRDynamics;
		else if(type.equals("GMR"))
			this.type = ModelType.GMR;
		
		models = new HashSet<GMM>();
		
	}
	

	/**
	 * Load motion model definition from files.
	 * 
	 * @param gmmFolder
	 */
	@SuppressWarnings({ "rawtypes" })
	public void readFromFile(String gmmFolder, List<Map> modelfiles) {
		
		for(Map mf : modelfiles) {
			
			String file   = (String) mf.get("name");
			
			GMM mod = new GMM(file.split("\\.")[0]);
			mod.readFromFile(gmmFolder + File.separator + file);
			
			Map input = (Map) mf.get("input");
			mod.setInputType((String) input.get("type"));
			mod.setInputDim((String) input.get("dim"));
			
			Map output = (Map) mf.get("output");
			mod.setOutputType((String) output.get("type"));
			mod.setOutputDim((String) output.get("dim"));
			
			models.add(mod);
		}
	}

	
	/**
	 * @param manager
	 * @param factory
	 * @param pm
	 * @param ontology
	 * @return
	 */
	public OWLNamedIndividual writeToOWL(OWLOntologyManager manager, OWLDataFactory factory, DefaultPrefixManager pm, OWLOntology ontology) {

		// create class for phase
		OWLClass modelType = null;
		
		switch(type) {
			case CDS:
				modelType = factory.getOWLClass(IRI.create(GMMToOWL.SEDS + "CDSModel"));
				break;
			case GMRDynamics:
				modelType = factory.getOWLClass(IRI.create(GMMToOWL.SEDS + "GMRDynamicsModel"));
				break;
			case GMR:
				modelType = factory.getOWLClass(IRI.create(GMMToOWL.SEDS + "GMRModel"));
				break;
		}
		
		OWLNamedIndividual modelInst = factory.getOWLNamedIndividual(IRI.create(OWLThing.getUniqueID(modelType.getIRI().toString())));
		manager.applyChange(new AddAxiom(ontology, factory.getOWLClassAssertionAxiom(modelType, modelInst)));	

		
		for(GMM m : models) {
			OWLObjectProperty describedByGMM = factory.getOWLObjectProperty(IRI.create(GMMToOWL.SEDS + "describedByGMM"));
			OWLNamedIndividual masterInst = m.writeToOWL(manager, factory, pm, ontology);
			manager.addAxiom(ontology, factory.getOWLObjectPropertyAssertionAxiom(describedByGMM, modelInst,  masterInst));
		}
		
		return modelInst;
	}
}
