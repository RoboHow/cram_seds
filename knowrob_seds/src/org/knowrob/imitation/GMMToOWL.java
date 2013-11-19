package org.knowrob.imitation;

import java.io.File;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import edu.tum.cs.ias.knowrob.owl.utils.OWLFileUtils;

import ros.*;


public class GMMToOWL {


	////////////////////////////////////////////////////////////////////////////////
	// Set IRIs for the ontologies used here
	//

	// Base IRI for KnowRob ontology
	public final static String KNOWROB = "http://ias.cs.tum.edu/kb/knowrob.owl#";

	// Base IRI for OWL ontology
	public final static String OWL = "http://www.w3.org/2002/07/owl#";

	// Base IRI for RDFS
	public final static String RDFS = "http://www.w3.org/2000/01/rdf-schema#";

	// Base IRI for motion constraints ontology	
	public final static String CONSTR = "http://ias.cs.tum.edu/kb/motion-constraints.owl#";

	// Base IRI for new ontology
	public final static String MOTION = "http://ias.cs.tum.edu/kb/motion-def.owl#";


	// Prefix manager
	public final static DefaultPrefixManager PREFIX_MANAGER = new DefaultPrefixManager(CONSTR);
	static {
		PREFIX_MANAGER.setPrefix("knowrob:", KNOWROB);
		PREFIX_MANAGER.setPrefix("constr:", CONSTR);
		PREFIX_MANAGER.setPrefix("owl:", OWL);
		PREFIX_MANAGER.setPrefix("rdfs:", RDFS);
		PREFIX_MANAGER.setPrefix("motion:", MOTION);
	}


	

	/**
	 * Constructor. Advertises the needed ROS services.
	 */
	public GMMToOWL() {


	}

	public String gmmToOWL(String gmmFolder) {
			
			// Create ontology manager and data factory
			OWLOntologyManager manager = OWLManager.createOWLOntologyManager();
			OWLDataFactory factory = manager.getOWLDataFactory();
			DefaultPrefixManager pm = PREFIX_MANAGER;

			OWLOntology ontology = null;
			
			// Create empty OWL ontology
			try {

				ontology = manager.createOntology(IRI.create(MOTION));
				manager.setOntologyFormat(ontology, new RDFXMLOntologyFormat());

			} catch (OWLOntologyCreationException e) {
				e.printStackTrace();
			}

			// Import motion constraints ontology
			OWLImportsDeclaration oid = factory.getOWLImportsDeclaration(IRI.create(CONSTR));
			AddImport addImp = new AddImport(ontology,oid);
			manager.applyChange(addImp);


			// import GMMs and write to OWL
			File dir = new File(gmmFolder);
			
			GMM master   = new GMM("master");
			GMM slave    = new GMM("slave");
			GMM coupling = new GMM("coupling");
			GMM force    = new GMM("forceDyn");
			
			for(String f : dir.list()) {
				
				if(f.equals("masterDyn.gmm")) {
					master.readFromFile(gmmFolder + "/" + f);
					master.writeToOWL(manager, factory, pm, ontology);
				} else if(f.equals("slaveDyn.gmm")) {
					slave.readFromFile(gmmFolder + "/" + f);
					slave.writeToOWL(manager, factory, pm, ontology);
				} else if(f.equals("couplingDyn.gmm")) {
					coupling.readFromFile(gmmFolder + "/" + f);	
					coupling.writeToOWL(manager, factory, pm, ontology);
				} else if(f.equals("forceDyn.gmm")) {
//					force.readFromFile(gmmFolder + "/" + f);	
//					force.writeToOWL(manager, factory, pm, ontology);
				} 
			}
			
			
			
			return OWLFileUtils.saveOntologytoString(ontology, manager.getOntologyFormat(ontology));
			

	}


	public static void main(String[] args) {
		System.err.println(new GMMToOWL().gmmToOWL("/home/tenorth/work/ros/groovy/rosbuild_ws/sandbox/cram_seds/knowrob_seds/data"));
	}
}







