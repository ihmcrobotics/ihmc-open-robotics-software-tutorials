package us.ihmc.simplePendulum;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class SimplePendulumSimulation {

	// simulation step duration
	public static final double DT = 0.001;

	private SimulationConstructionSet sim;

	public SimplePendulumSimulation() {

		SimplePendulumRobot robot = new SimplePendulumRobot();
		robot.setController(new SimplePendulumController(robot));

		/* Creates simulation parameters */
		SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
		/*
		 * Sets data buffer to allow for this number of values for each variable to be
		 * saved.
		 */
		parameters.setDataBufferSize(32000);

		// Creates a new simulation
		sim = new SimulationConstructionSet(robot, parameters);

		/*
		 * Sets the simulation to collect data every 20 simulation steps This is used to
		 * prune data so a smaller buffer is sufficient.
		 */
		sim.setDT(DT, 20);

		// Sets the ground to be visbile in the 3D view
		sim.setGroundVisible(true);

		// Sets location and orientation of the camera
		sim.setCameraPosition(0, -9.0, 0.6);
		sim.setCameraFix(0.0, 0.0, 0.70);

		/*
		 * Specifies that the simulation will only run for a duration of 60 seconds. For
		 * this tutorial, this allows the simulation to run to a point where it doesn't
		 * overflow the data buffer
		 */
		sim.setSimulateDuration(60.0);

		// Launch the simulator.
		sim.startOnAThread();
	}

	public static void main(String[] args) {
		new SimplePendulumSimulation();
	}
}
