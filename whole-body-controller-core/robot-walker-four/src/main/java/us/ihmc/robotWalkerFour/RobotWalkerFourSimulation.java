package us.ihmc.robotWalkerFour;

import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * In this simulation example, we will see how the IHMC whole-body controller core can be used to
 * implement a controller for achieving quasi-static walking with the good old M2 robot model.
 */

public class RobotWalkerFourSimulation
{
   public RobotWalkerFourSimulation()
   {

      // Create an instance of the robot arm
      M2RobotDefinition walkerDef = new M2RobotDefinition();

      // Instantiate a SCS object
      ContactPointBasedContactParameters contact = ContactPointBasedContactParameters.defaultParameters();
      contact.setKxy(40000.0);
      contact.setBxy(100.0);
      contact.setKz(500.0);
      contact.setBz(250.0);
      
      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.contactPointBasedPhysicsEngineFactory(contact));

      // The gravity has to be explicitly defined for the controller core (maybe a robot on the Moon someday...?)
      double gravityMagnitude = 9.81;
      // Make sure the simulation and the controller are using the same value for the gravity.
      scs.getGravity().set(0.0, 0.0, -gravityMagnitude);

      // This time, we will make the controller run at a slower frequency than the
      // simulation.
      double controllerDT = 1.0e-2;

      // The simulation DT.
      // TODO frequency at which data is logged.
      double simulateDT = 4.0e-4;
      scs.setDT(simulateDT);

      // Generate a robot for the simulation
      Robot walker = scs.addRobot(walkerDef);

      
      // Add a terrain
      scs.addTerrainObject(new FlatGroundDefinition());

      // Create an instance of the controller.
      RobotWalkerFourController walkerController = new RobotWalkerFourController(walker.getControllerInput(),
                                                                                 walker.getControllerOutput(),
                                                                                 controllerDT,
                                                                                 gravityMagnitude,
                                                                                 walkerDef);
      // Make sure to initialize the controller.
      walkerController.initialize();
      // Attach the controller to the robot.
      walker.addController(walkerController);

      // Defining the buffer size to ensure a minimum simulation duration before filling the graphs in the simulator.
      scs.changeBufferSize(65536);

      // Launch the simulator.
      scs.start(false, false, false);
   }

   public static void main(String[] args)
   {
      new RobotWalkerFourSimulation();
   }
}
