package us.ihmc.robotArmThree;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.robotArmOne.FlatGroundDefinition;
import us.ihmc.robotArmOne.RobotArmOneDefinition;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * As in the {@code RobotArmOneSimulation} and {@code RobotArmTwoSimulation} examples, we will use
 * the same 7-DoF robot arm which is controlled using the IHMC whole-body controller core. This
 * time, the end-effector is controlled in taskspace to follow a 3D position and orientation
 * trajectory, see {@link RobotArmThreeController}.
 * <p>
 * This example highlights the protocol for setting the controller core and the possibility to run
 * it with 3 different modes: inverse dynamics, inverse kinematics, and virtual model control.
 * </p>
 * <p>
 * controlling a much more complicated robot system remains extremely similar.
 * </p>
 * <p>
 * This is the main class that sets up and starts the simulation environment.
 * </p>
 */
public class RobotArmThreeSimulation
{
   /**
    * Three modes are available for the controller core:
    * <ul>
    * <li>Inverse Dynamics: Given desired accelerations and contact states, the controller core
    * computes desired joint torques.
    * <li>Virtual Model Control: It is a generalization of the "Jacobian transpose" method to a
    * whole-body framework, the output is desired joint torques.
    * <li>Inverse Kinematics: Given desired velocities, the controller core can integrate the these
    * velocities to output both desired joint velocities and positions.
    * </ul>
    */

   private final WholeBodyControllerCoreMode controlMode = WholeBodyControllerCoreMode.INVERSE_DYNAMICS;
   //   private final WholeBodyControllerCoreMode controlMode = WholeBodyControllerCoreMode.INVERSE_KINEMATICS;
   //   private final WholeBodyControllerCoreMode controlMode = WholeBodyControllerCoreMode.VIRTUAL_MODEL;

   public RobotArmThreeSimulation()
   {
      // Create an instance of the robot arm
      RobotArmOneDefinition robotArmDef = new RobotArmOneDefinition();

      // Instantiate a SCS object - create the simulation
      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.contactPointBasedPhysicsEngineFactory());

      // Ignore joints in controller for inverse kinematics
      if (controlMode == WholeBodyControllerCoreMode.INVERSE_KINEMATICS)
      {
         robotArmDef.ignoreAllJoints();
      }

      // Generate a robot for the simulation
      Robot robotArm = scs.addRobot(robotArmDef);

      // The control frequency, which is equal to simulation frequency in this example, has to be provided to the controller core.
      double simulateDT = 1.0e-4;
      // Make sure the simulation uses the same DT
      scs.setDT(simulateDT);

      // The gravity has to be explicitly defined for the controller core (maybe a robot on the Moon someday...?)
      double gravityMagnitude = 9.81;
      // Make sure the simulation and the controller are using the same value for the gravity.
      scs.getGravity().set(0.0, 0.0, -gravityMagnitude);

      // Create an instance of the controller.
      RobotArmThreeController robotArmController = new RobotArmThreeController(robotArm.getControllerInput(),
                                                                               robotArm.getControllerOutput(),
                                                                               simulateDT,
                                                                               gravityMagnitude,
                                                                               controlMode);
      // Add the YoGraphics to the simulation
      scs.addYoGraphic(robotArmController.getYoGraphicDefinition());

      // Camera settings
      scs.setCameraFocusPosition(0.0, 0.0, 1.0);
      scs.setCameraPosition(0.0, 5.0, 2.0);

      // Make sure to initialize the controller.
      robotArmController.initialize();

      // Attach the controller to the robot
      robotArm.addController(robotArmController);

      // Add a terrain
      scs.addTerrainObject(new FlatGroundDefinition());

      // As this example simulation is rather simple, let's prevent SCS from simulating faster than real-time.
      scs.setRealTimeRateSimulation(false);

      // Defining the buffer size to ensure a minimum simulation duration before filling the graphs in the simulator.
      scs.changeBufferSize(65536);

      // Launch the simulator.
      scs.start(false, false, false);
   }

   public static void main(String[] args)
   {
      new RobotArmThreeSimulation();
   }
}
