package us.ihmc.robotWalkerFive;

import us.ihmc.graphicsDescription.conversion.YoGraphicConversionTools;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.robot.Robot;

/**
 * In this simulation example, we will see how the IHMC whole-body controller core can be used to
 * implement a controller for achieving quasi-static walking with the good old M2 robot model.
 */

public class RobotWalkerFiveSimulation
{
   public RobotWalkerFiveSimulation()
   {
      // Create an instance of the robot arm
      M2RobotDefinition walkerDef = new M2RobotDefinition();

      // Define ground contact parameters
      ContactPointBasedContactParameters contact = ContactPointBasedContactParameters.defaultParameters();
      contact.setKxy(40000.0);
      contact.setBxy(100.0);
      contact.setKz(500.0);
      contact.setBz(250.0);

      // Instantiate a SCS object
      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.contactPointBasedPhysicsEngineFactory(contact));
      double controllerDT = 1.0e-4;
      double simulateDT = 4.0e-4;

      // Define ground contact parameters
      //      ContactParameters contact = new ContactParameters();
      //      contact.setMinimumPenetration(5.0e-5);
      //      contact.setCoefficientOfFriction(0.8);
      //      contact.setCoefficientOfRestitution(0.0);
      //      contact.setRestitutionThreshold(0.0);
      //      contact.setErrorReductionParameter(0.0);
      //      contact.setComputeFrictionMoment(true);
      //      contact.setCoulombMomentFrictionRatio(1.2);
      //
      //      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.impulseBasedPhysicsEngineFactory(contact));
      //      double controllerDT = 1e-3;
      //      double simulateDT = 1.0e-4;

      // The gravity has to be explicitly defined for the controller core (maybe a robot on the Moon someday...?)
      double gravityMagnitude = 9.81;

      // Make sure the simulation and the controller are using the same value for the gravity.
      scs.getGravity().set(0.0, 0.0, -gravityMagnitude);

      scs.setDT(simulateDT);
      // Set the frequency at which data is logged.
      scs.setBufferRecordTickPeriod(10);

      // Generate a robot for the simulation
      Robot walker = scs.addRobot(walkerDef);

      // Add a terrain
      scs.addTerrainObject(new FlatGroundDefinition());

      // Create an instance of the controller.
      RobotWalkerFiveController walkerController = new RobotWalkerFiveController(walker.getControllerInput(),
                                                                                 walker.getControllerOutput(),
                                                                                 controllerDT,
                                                                                 gravityMagnitude,
                                                                                 walkerDef);

      scs.addYoGraphics(YoGraphicConversionTools.toYoGraphicDefinitions(walkerController.getYoGraphicsListRegistry()));
      //      scs.addYoGraphics(walkerController.getYoGraphicsListRegistry().getYoGraphicsLists());

      // Make sure to initialize the controller.
      walkerController.initialize();

      // Add some variables - add it once, save scs2 simulation when closing it, and then comment out
//      scs.addYoEntry("walk");
//      scs.addYoEntry("useCapturePoint");
//      scs.addYoEntry("rotationPerStep");
//      scs.addYoEntry("transferDuration");
//      scs.addYoEntry("swingDuration");
//      scs.addYoEntry("stepLength");
//      scs.addYoEntry("sideWayStepLength");
//      scs.addYoEntry("stepWidth");
//      scs.addYoEntry("addTakeOffVelocity");
//      scs.addYoEntry("addTouchDownVelocity");
//      scs.addYoEntry("walkerWillFreakOut");
//      scs.addYoEntry("desiredCapturePointX");
//      scs.addYoEntry("desiredCapturePointY");
//      scs.addYoEntry("desiredCapturePointZ");

      // Add the YoGraphics to the simulation
      //      scs.addYoGraphic(new YoGraphicGroupDefinition("mygroup"));

      scs.addYoGraphic(walkerController.getYoGraphicDefinition());

      // Attach the controller to the robot.
      walker.addController(walkerController);

      // Defining the buffer size to ensure a minimum simulation duration before filling the graphs in the simulator.
      scs.changeBufferSize(65536);

      // Camera settings
      scs.setCameraFocusPosition(0.0, 0.0, 0.8);
      scs.setCameraPosition(0.0, -5.0, 2.0);

      // Track the robot with the camera
      scs.requestCameraRigidBodyTracking(scs.getRobots().get(0).getName(), scs.getRobots().get(0).getAllJoints().get(0).getSuccessor().getName());
      scs.requestPlotter2DCoordinateTracking("measuredCapturePointX", "measuredCapturePointY", "worldFrame");
      scs.showOverheadPlotter2D(true);

      // Launch the simulator.
      scs.start(false, false, false);
   }

   public static void main(String[] args)
   {
      new RobotWalkerFiveSimulation();
   }
}
