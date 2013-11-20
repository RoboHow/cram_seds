package org.knowrob.imitation;

import java.io.File;
import java.util.ArrayList;

import org.semanticweb.owlapi.apibinding.OWLManager;
import org.semanticweb.owlapi.io.RDFXMLOntologyFormat;
import org.semanticweb.owlapi.model.*;
import org.semanticweb.owlapi.util.DefaultPrefixManager;

import edu.tum.cs.ias.knowrob.owl.utils.OWLFileUtils;


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
	public final static String CONSTR = "http://ias.cs.tum.edu/kb/knowrob_seds.owl#";

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

	ArrayList<MotionPhase> phases;

	/**
	 * Constructor. Advertises the needed ROS services.
	 */
	public GMMToOWL() {
		phases = new ArrayList<MotionPhase>();
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


			// import GMMs
			File dir = new File(gmmFolder);
			for(File subdir : dir.listFiles()) {
				if(subdir.isDirectory()) {
					MotionPhase p = new MotionPhase(subdir.getName());
					p.readFromFile(subdir.getAbsolutePath());
					phases.add(p);
				}
			}

			// export to OWL:
			for(MotionPhase p : phases) {
				p.writeToOWL(manager, factory, pm, ontology);
			}
			
			// generate OWL file string
			String owl_data = OWLFileUtils.saveOntologytoString(ontology, manager.getOntologyFormat(ontology));
			return beautifyOWL(owl_data);
			
	}



	
	private String beautifyOWL(String owl_data) {
		
		String header = "\n\n" +
		"<!DOCTYPE rdf:RDF [\n" +
		"    <!ENTITY owl \"http://www.w3.org/2002/07/owl#\" >\n" +
		"    <!ENTITY xsd \"http://www.w3.org/2001/XMLSchema#\" >\n" +
		"    <!ENTITY knowrob \"http://ias.cs.tum.edu/kb/knowrob.owl#\" >\n" +
		"    <!ENTITY seds \"http://ias.cs.tum.edu/kb/knowrob_seds.owl#\" >\n" +
		"    <!ENTITY rdfs \"http://www.w3.org/2000/01/rdf-schema#\" >\n" +
		"    <!ENTITY rdf \"http://www.w3.org/1999/02/22-rdf-syntax-ns#\" >\n" +
		"]>\n\n<rdf:RDF";
		
		owl_data = owl_data.replace("rdf:resource=\"http://ias.cs.tum.edu/kb/knowrob.owl#", 
				"rdf:resource=\"&knowrob;");
		owl_data = owl_data.replace("rdf:about=\"http://ias.cs.tum.edu/kb/knowrob.owl#", 
				"rdf:resource=\"&knowrob;");

		owl_data = owl_data.replace("rdf:resource=\"http://ias.cs.tum.edu/kb/knowrob_seds.owl#", 
				"rdf:resource=\"&seds;");
		
		owl_data = owl_data.replace("rdf:datatype=\"http://www.w3.org/2001/XMLSchema#double\"", 
									"rdf:datatype=\"&xsd;double\"");
		
		owl_data = owl_data.replace("<rdf:RDF", header);
		return owl_data;
	}


	public static void main(String[] args) {
		System.err.println(new GMMToOWL().gmmToOWL("/home/tenorth/work/ros/groovy/rosbuild_ws/sandbox/cram_seds/knowrob_seds/data"));
	}
}







