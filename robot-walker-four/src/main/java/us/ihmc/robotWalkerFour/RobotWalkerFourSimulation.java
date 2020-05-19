package us.ihmc.robotWalkerFour;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

/**
 * In this simulation example, we will see how the IHMC whole-body controller core can be used to
 * implement a controller for achieving quasi-static walking with the good old M2 robot model.
 */
public class RobotWalkerFourSimulation
{
   public RobotWalkerFourSimulation()
   {
      // The gravity has to be explicitly defined for the controller core (maybe a robot on the Moon someday...?)
      double gravityMagnitude = 9.81;
      // This time, we will make the controller run at a slower frequency than the simulation.
      double controllerDT = 1.0e-2;
      // The simulation DT.
      double simulateDT = 4.0e-4;
      // This is an additional registry that allows to display 3D graphics in the simulation. This feature is not demonstrated in this example.
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      // Create an instance of the robot walker.
      RobotWalkerFour robotWalker = new RobotWalkerFour();
      // Use the simulated definition of the robot to define the simulation environment.
      Robot simulatedRobot = robotWalker.getSimulatedRobot();

      /*
       * This time, we need to setup a ground contact model that will make the ground interact with
       * the ground contact points of the robot. Without it, the robot would simply fall through the
       * ground.
       */
      double groundKxy = 40000.0;
      double groundBxy = 100.0;
      double groundKz = 500.0;
      double groundBz = 250.0;

      GroundContactModel linearGroundModel = new LinearGroundContactModel(simulatedRobot, groundKxy, groundBxy, groundKz, groundBz,
                                                                          simulatedRobot.getRobotsYoVariableRegistry());
      simulatedRobot.setGroundContactModel(linearGroundModel);

      // Make sure the simulation and the controller are using the same value for the gravity.
      simulatedRobot.setGravity(-gravityMagnitude);
      // Create an instance of the controller.
      RobotWalkerFourController robotArmController = new RobotWalkerFourController(robotWalker, controllerDT, gravityMagnitude, yoGraphicsListRegistry);
      // Make sure to initialize the controller.
      robotArmController.initialize();
      // Attach the controller to the robot.
      simulatedRobot.setController(robotArmController, (int) (simulateDT / controllerDT));

      // Creating the simulation.
      SimulationConstructionSet scs = new SimulationConstructionSet(simulatedRobot);
      // Defining the simulation DT and the frequency at which data is logged.
      scs.setDT(simulateDT, 10);
      // Defining the buffer size to ensure a minimum simulation duration before filling the graphs in the simulator.
      scs.setMaxBufferSize(65536);
      // Launch the simulator.
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new RobotWalkerFourSimulation();
   }
}
