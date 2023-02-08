package us.ihmc.simplePendulum;

import us.ihmc.scs2.SimulationConstructionSet2;

public class SimplePendulumSimulation {

//   // simulation step duration
//   public static final double DT = 0.001;
//
//   private SimulationConstructionSet sim;

	public SimplePendulumSimulation() {

		SimplePendulumDefinition pendulum = new SimplePendulumDefinition();
		// robot.setController(new SimplePendulumController(robot));

//      /* Creates simulation parameters */
//      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();

		// Instantiate a SCS object
		SimulationConstructionSet2 scs = new SimulationConstructionSet2();

		// Add the pendulum robot to the simulation
		scs.addRobot(pendulum);

		// terrain
//		RigidBodyTransform terrainPose = new RigidBodyTransform();
//		terrainPose.getTranslation().subZ(0.05);
//		GeometryDefinition terrainGeometry = new Box3DDefinition(1000, 1000, 0.1);
//		TerrainObjectDefinition terrain = new TerrainObjectDefinition(
//				new VisualDefinition(terrainPose, terrainGeometry, new MaterialDefinition(ColorDefinitions.DarkGrey())),
//				new CollisionShapeDefinition(terrainPose, terrainGeometry));
//		scs.addTerrainObject(terrain);

//      // Sets data buffer to allow for this number of values for each variable to be saved.
		scs.setBufferRecordTickPeriod(32000);
		
		

		// Launch the simulator
		scs.start(false, false, false);

//      // Creates a new simulation
//      sim = new SimulationConstructionSet(robot, parameters);
//
//      /*
//       * Sets the simulation to collect data every 20 simulation steps This is used to prune data so a
//       * smaller buffer is sufficient.
//       */
//      sim.setDT(DT, 20);
//
//      // Sets the ground to be visible in the 3D view
//      sim.setGroundVisible(true);

		// Sets location and orientation of the camera
		scs.setCameraPosition(0, -9.0, 0.6);
		scs.setCameraFocusPosition(0.0, 0.0, 0.70);
//
//      /*
//       * Specifies that the simulation will only run for a duration of 60 seconds. For this tutorial, this
//       * allows the simulation to run to a point where it doesn't overflow the data buffer
//       */
//      sim.setSimulateDuration(60.0);
//
//      // Launch the simulator.
//      sim.startOnAThread();
	}

	public static void main(String[] args) {
		new SimplePendulumSimulation();
	}
}
