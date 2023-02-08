package us.ihmc.mobile;

import us.ihmc.scs2.SimulationConstructionSet2;

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
		SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.impulseBasedPhysicsEngineFactory());

		scs.addRobot(mobile);
		
		scs.setDT(0.001);
		scs.setBufferRecordTickPeriod(1);
		// Launch the simulator
		scs.start(true, false, false);
	}

	public static void main(String[] args) {
		new MobileSimulation();
	}

}
