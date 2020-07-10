package us.ihmc.fallingBrick;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class FallingBrickSimulation
{
   SimulationConstructionSet sim;

   public FallingBrickSimulation()
   {
      FallingBrickRobot FallingBrick = new FallingBrickRobot();
      /* Creates simulation parameters */
      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      // Sets data buffer to allow for this number of values for each variable to be saved.
      parameters.setDataBufferSize(16342);
      // Creates a new simulation
      sim = new SimulationConstructionSet(FallingBrick, parameters);

      /*
       * Sets the simulation to collect data every 20 simulation steps This is used to prune data so a
       * smaller buffer is sufficient.
       */
      sim.setDT(0.001, 20);

      // Sets location and orientation of the camera
      sim.setCameraPosition(-0.5, 8.25, 3.5);
      sim.setCameraFix(0.0, 0.0, 0.4);

      /*
       * Modifies the camera tracking state for the selected viewport. A camera set to track will not
       * move. Instead, it will rotate to keep the target in view.
       */
      sim.setCameraTracking(false, true, true, false);
      /*
       * Modifies the camera dolly state for the selected viewport. A camera with dolly enabled will move
       * to keep its target in view from the same orientation.
       */
      sim.setCameraDolly(false, true, true, false);

      // Set up a graph of the Z position.
      sim.setupGraph("q_z");

      // Adds an entry box for the specified variable.  
      sim.setupEntryBox("qd_x");
      sim.setupEntryBox("qd_y");
      sim.setupEntryBox("qd_z");

      sim.setupEntryBox("qd_wx");
      sim.setupEntryBox("qd_wy");
      sim.setupEntryBox("qd_wz");

      // Simulating in real-time
      sim.setSimulateNoFasterThanRealTime(true);

      // Launch the simulator.
      sim.startOnAThread();
   }

   public static void main(String[] args)
   {
      new FallingBrickSimulation();
   }
}