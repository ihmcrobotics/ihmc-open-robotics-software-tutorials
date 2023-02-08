package us.ihmc.mobile;

import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.simulation.SimulationSession;

/**
 * A simulation of a child's mobile toy that uses a tree structure of 21 gimbal
 * joints (63 degrees of
 * 
 * 
 * freedom total).
 */
public class MobileSimulation {

	public MobileSimulation() {
		// Create an instance of the Mobile-Robot
		MobileDefinition mobile = new MobileDefinition();

		// Instantiate a SCS object and add the MobileRobot
		SimulationSession scs = new SimulationSession();

		scs.addRobot(mobile);

		// Launch the simulator
		SessionVisualizer.startSessionVisualizer(scs);
	}

	public static void main(String[] args) {
		new MobileSimulation();
	}

}
