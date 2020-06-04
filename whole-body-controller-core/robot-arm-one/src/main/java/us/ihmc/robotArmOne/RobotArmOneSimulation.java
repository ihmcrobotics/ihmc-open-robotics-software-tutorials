package us.ihmc.robotArmOne;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 * In this example simulation, we will create a simple 7-DoF robot arm
 * {@code RobotArmOne} which is controlled with simple PD controllers on each
 * joint to track sine-wave trajectories, see {@code RobotArmOneController}.
 * <p>
 * This is the main class that sets up and starts the simulation environment.
 * </p>
 */
public class RobotArmOneSimulation {
	public RobotArmOneSimulation() {
		// Create an instance of the robot arm.
		RobotArmOne robotArm = new RobotArmOne();
		// Create an instance of the controller.
		RobotArmOneController robotArmController = new RobotArmOneController(robotArm);
		// Make sure to initialize the controller.
		robotArmController.initialize();
		// Attach the controller to the robot.
		robotArm.setController(robotArmController);

		// Creating the simulation.
		SimulationConstructionSet scs = new SimulationConstructionSet(robotArm);
		// As this example simulation is rather simple, let's prevent SCS from
		// simulating faster than real-time.
		scs.setSimulateNoFasterThanRealTime(true);
		// Defining the simulation DT and the frequency at which data is logged.
		scs.setDT(1.0e-4, 10);
		// Defining the buffer size to ensure a minimum simulation duration before
		// filling the graphs in the simulator.
		scs.setMaxBufferSize(65536);
		// Launch the simulator.
		scs.startOnAThread();
	}

	public static void main(String[] args) {
		new RobotArmOneSimulation();
	}
}
