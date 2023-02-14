package us.ihmc.robotArmOne;

import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * In this example simulation, we will create a simple 7-DoF robot arm {@code RobotArmOne} which is
 * controlled with simple PD controllers on each joint to track sine-wave trajectories, see
 * {@code RobotArmOneController}.
 * <p>
 * This is the main class that sets up and starts the simulation environment.
 * </p>
 */
public class RobotArmOneSimulation
{
   public RobotArmOneSimulation()
   {
      // Create an instance of the robot arm
      RobotArmOneDefinition robotArmDef = new RobotArmOneDefinition();

      // Instantiate a SCS object
      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.impulseBasedPhysicsEngineFactory());

      // Generate a robot object according to our definition
      Robot robotArm = new Robot(robotArmDef, scs.getInertialFrame());

      // Add the robot to the simulation
      robotArm = scs.addRobot(robotArmDef);

      // Create an instance of the controller
      RobotArmOneController armController = new RobotArmOneController(robotArm.getControllerInput(), robotArm.getControllerOutput());

      // Attach the controller to the robot
      robotArm.addController(armController);

      // Add a terrain
      scs.addTerrainObject(new FlatGroundDefinition());

      // As this example simulation is rather simple, let's prevent SCS from simulating faster than real-time
      scs.setRealTimeRateSimulation(false);

      // Defining the simulation DT 
      scs.setDT(1.0e-4);

      // Defining the buffer size to ensure a minimum simulation duration before
      scs.changeBufferSize(100000);

      // Launch the simulator.
      scs.start(false, false, false);
   }

   public static void main(String[] args)
   {
      new RobotArmOneSimulation();
   }
}
