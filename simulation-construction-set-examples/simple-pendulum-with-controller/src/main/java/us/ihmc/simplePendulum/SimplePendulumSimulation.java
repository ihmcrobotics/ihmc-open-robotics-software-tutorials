package us.ihmc.simplePendulum;

import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.controller.implementations.ControllerCollectionDefinition;
import us.ihmc.scs2.simulation.robot.Robot;

public class SimplePendulumSimulation
{
   public SimplePendulumSimulation()
   {
      // Setup the definition of our pendulum
      SimplePendulumDefinition pendulumDef = new SimplePendulumDefinition();

      // Instantiate a SCS object
      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.impulseBasedPhysicsEngineFactory());

      // Generate a pendulum robot object according to our definition
      Robot pendulumRobot = new Robot(pendulumDef, scs.getInertialFrame());

      // Add the pendulum robot to the simulation
      pendulumRobot = scs.addRobot(pendulumDef);

      // Add a controller to the pendulum robot
      SimplePendulumController penController = new SimplePendulumController(pendulumRobot.getControllerInput(), pendulumRobot.getControllerOutput());
      pendulumRobot.addController(penController);
      
      // Sets data buffer to allow for this number of values for each variable to be saved.
      scs.setBufferRecordTickPeriod(32000);

      scs.setDT(0.001);
      scs.setBufferRecordTickPeriod(1);
      // Launch the simulator
      scs.start(false, false, false);

      // Sets location and orientation of the camera
      scs.setCameraPosition(0, -9.0, 0.6);
      scs.setCameraFocusPosition(0.0, 0.0, 0.70);
   }

   public static void main(String[] args)
   {
      new SimplePendulumSimulation();
   }
}
