package org.knowrob.imitation;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import javax.vecmath.GMatrix;
import javax.vecmath.GVector;
import javax.vecmath.Matrix4d;

import org.semanticweb.owlapi.model.AddAxiom;
import org.semanticweb.owlapi.model.IRI;
import org.semanticweb.owlapi.model.OWLClass;
import org.semanticweb.owlapi.model.OWLDataFactory;
import org.semanticweb.owlapi.model.OWLNamedIndividual;
import org.semanticweb.owlapi.model.OWLObjectProperty;
import org.semanticweb.owlapi.model.OWLOntology;
import org.semanticweb.owlapi.model.OWLOntologyManager;
import org.semanticweb.owlapi.util.DefaultPrefixManager;
import org.yaml.snakeyaml.Yaml;

import edu.tum.cs.ias.knowrob.owl.OWLIndividual;
import edu.tum.cs.ias.knowrob.owl.OWLThing;

public class MotionPhase {

	protected String name = "";

	protected ArrayList<Double> priors;
	protected List<GVector> means;
	protected List<GMatrix> covs;

	double id = -1;
	double threshold = -1;
	String obj = "";
	
	Matrix4d attractor = null;
	
	List<MotionModel> models;

	
	/**
	 * Constructor
	 * 
	 * @param name Identifier of this motion phase
	 */
	public MotionPhase(String name) {

		this.name = name;
		this.models = new ArrayList<MotionModel>();
	}

	
	/**
	 * Load motion phase definition from files.
	 * 
	 * @param gmmFolder
	 */
	@SuppressWarnings({ "rawtypes", "unchecked" })
	public void readFromFile(String gmmFolder) {
		
		File dir = new File(gmmFolder);
		for(String f : dir.list()) {
			
			if(f.equalsIgnoreCase("constraints.yaml")) {

				try {
					
					InputStream in = new FileInputStream(new File(gmmFolder + File.separator + f));

					Yaml yaml = new Yaml();
					Map constr = (Map) yaml.load(in);

					// read simple properties
					if(constr.get("id") instanceof Double)
						id = (Double) constr.get("id"); 

					if(constr.get("threshold") instanceof Double)
						threshold = (Double) constr.get("threshold");
					
					obj = (String) constr.get("object");
					
					// read pose matrix
					List<List<Double>> attr = (List<List<Double>>) constr.get("attractor");

					assert(attr.size() == 4);
					assert(attr.get(0).size() == 4);
					
					double[] p = new double[16];
					for(int i=0;i<attr.size();i++) {
						for(int j=0;j<attr.get(i).size();j++) {
							p[i*4+j] = attr.get(i).get(j);
						}
					}
					this.attractor = new Matrix4d(p);
				
					
					// read motion models:
					for(Map m : (List<Map>) constr.get("model")) {
						
						MotionModel mod = new MotionModel((String) m.get("modeltype"));
						models.add(mod);
						
						mod.readFromFile(gmmFolder, (List<Map>) m.get("modelfile"));
					}
					
				} catch (FileNotFoundException e) {
					e.printStackTrace();
				}
			}
		}
	}
	

	/**
	 * @param manager
	 * @param factory
	 * @param pm
	 * @param ontology
	 * @return
	 */
	public OWLClass writeToOWL(OWLOntologyManager manager, OWLDataFactory factory, DefaultPrefixManager pm, OWLOntology ontology) {

        // create class for phase
        OWLClass phaseCls = factory.getOWLClass(IRI.create(OWLThing.getUniqueID(GMMToOWL.SEDS + name)));
        
        OWLClass phaseType = factory.getOWLClass(IRI.create(GMMToOWL.SEDS + "SEDSMotion"));
        manager.applyChange(new AddAxiom(ontology, factory.getOWLSubClassOfAxiom(phaseCls, phaseType)));        

        // write models to ontology:
        
        OWLObjectProperty describedByMotionModel = factory.getOWLObjectProperty(IRI.create(GMMToOWL.SEDS + "describedByMotionModel"));
        
        for(MotionModel model : models) {
        	
        	OWLNamedIndividual modelInst = model.writeToOWL(manager, factory, pm, ontology);
        	manager.applyChange(new AddAxiom(ontology,
        						factory.getOWLSubClassOfAxiom(phaseCls,
        						factory.getOWLObjectHasValue(describedByMotionModel, modelInst))));
        }

		
		return phaseCls;
	}


}
