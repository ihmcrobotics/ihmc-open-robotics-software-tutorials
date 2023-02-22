package us.ihmc.simplePendulum;

import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.robot.Robot;

public class SimplePendulumSimulation
{
   public static final double DT = 0.001;

   public SimplePendulumSimulation()
   {
      // Setup the definition of our pendulum
      SimplePendulumDefinition pendulumDef = new SimplePendulumDefinition();

      // Instantiate a SCS object
      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.contactPointBasedPhysicsEngineFactory());

      // Generate a pendulum robot object according to our definition
      Robot pendulumRobot = new Robot(pendulumDef, scs.getInertialFrame());

      // Add the pendulum robot to the simulation
      pendulumRobot = scs.addRobot(pendulumDef);

      // Add a controller to the pendulum robot
      SimplePendulumController penulumController = new SimplePendulumController(pendulumRobot.getControllerInput(), pendulumRobot.getControllerOutput());
      // pendulumRobot.addController(penulumController);

      // The simulation time step.
      scs.setDT(DT);

      // Set the frequency at which data is logged.
      scs.setBufferRecordTickPeriod(20);

      // Sets location and orientation of the camera
      scs.setCameraPosition(0, -9.0, 0.6);
      scs.setCameraFocusPosition(0.0, 0.0, 0.70);

      // Sets data buffer to allow for this number of values for each variable to be saved.
      scs.changeBufferSize(32000);

      // Sets if the simulation should not run faster than real-time
      scs.setRealTimeRateSimulation(true);

      // Launch the simulator
      scs.start(true, false, false);

   }

   public static void main(String[] args)
   {
      new SimplePendulumSimulation();
   }
}
