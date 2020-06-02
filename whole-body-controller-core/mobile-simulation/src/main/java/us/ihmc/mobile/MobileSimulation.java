package us.ihmc.mobile;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

/**
 * A simulation of a child's mobile toy that uses a tree structure of 21 gimbal joints (63 degrees
 * of freedom total).
 */
public class MobileSimulation
{
   private SimulationConstructionSet simulationConstructionSet;

   public MobileSimulation()
   {
      // Create an instance of MobileRobot
      MobileRobot mobile = new MobileRobot();

      // Instantiate a SCS object using the MobileRobot object reference
      simulationConstructionSet = new SimulationConstructionSet(mobile);
      // By default a ground plane is added in SCS, we have no need for it in this example.
      simulationConstructionSet.setGroundVisible(false);

      simulationConstructionSet.setCameraTracking(false, false, false, false);
      simulationConstructionSet.setCameraDolly(false, false, false, false);

      // set camera to a convenient viewing angle
      simulationConstructionSet.setCameraPosition(3.0, 2.0, 1.5);
      simulationConstructionSet.setCameraFix(0.0, 0.0, 0.8);

      simulationConstructionSet.setCameraTrackingVars("ef_track00_x", "ef_track00_y", "ef_track00_z");

      // As this example simulation is rather simple, let's prevent SCS from simulating faster than real-time.
      simulationConstructionSet.setSimulateNoFasterThanRealTime(true);
      // Defining the simulation tick duration and the rate at which the buffer should record data.
      simulationConstructionSet.setDT(0.02, 1);
      // Setting a default simulation duration after what SCS stops simulating. Simulation can be still be resumed in the GUI.
      simulationConstructionSet.setSimulateDuration(30.0);
      // Setting the buffer such that when SCS reaches the simulation duration, the buffer is entirely filled and the graphs are
      simulationConstructionSet.setMaxBufferSize(1501);
      // Launch the simulator.
      simulationConstructionSet.startOnAThread();
   }

   public static void main(String[] args)
   {
      new MobileSimulation();
   }
}
