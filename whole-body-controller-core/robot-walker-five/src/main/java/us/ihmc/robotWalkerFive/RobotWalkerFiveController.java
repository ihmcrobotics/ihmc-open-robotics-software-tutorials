package us.ihmc.robotWalkerFive;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsOrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * The objective here is to implement small controller that uses the IHMC whole-body controller to
 * make the M2 robot achieve quasi-static walking.
 * <p>
 * In this controller, we use a set of additional features with respect to the previous examples:
 * <ul>
 * <li>Requesting the controller core to use a rigid-body for supporting the body weight using the
 * new command: {@code PlaneContactStateCommand}.
 * <li>controlling the robot center of mass position with the command:
 * {@code CenterOfMassFeedbackControlCommand}.
 * <li>The direct control of a rigid-body acceleration using: {@code SpatialAccelerationCommand}.
 * </ul>
 * Finally, this controller also introduces the state machine framework: {@code StateMachine}.
 * </p>
 */
public class RobotWalkerFiveController implements Controller
{
   private static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   private static final double CENTER_OF_MASS_HEIGHT = 0.75;
   /**
    * We use this registry to keep track of the controller variables which can then be viewed in
    * Simulation Construction Set.
    */

   private WholeBodyControlCoreToolbox toolbox;
   private final YoRegistry registry = new YoRegistry("Controller");

   private final YoDouble errorCapturePoint = new YoDouble("errorCapturePoint", registry);

   private final YoBoolean walkerIsFalling = new YoBoolean("walkerIsFalling", registry);
   private final YoBoolean walkerWillFreakOut = new YoBoolean("walkerWillFreakOut", registry);
   private final YoBoolean useCapturePoint = new YoBoolean("useCapturePoint", registry);
   private final YoBoolean useCapturePointTrajectory = new YoBoolean("useCapturePointTrajectory", registry);

   private final YoBoolean addTakeOffVelocity = new YoBoolean("addTakeOffVelocity", registry);
   private final YoBoolean addTouchDownVelocity = new YoBoolean("addTouchDownVelocity", registry);
   private final YoFramePoint3D feedForwardLinearVelocity = new YoFramePoint3D("feedForwardLinearVelocity", WORLD_FRAME, registry);
   private final YoFramePoint3D currentMiddlePosition = new YoFramePoint3D("currentMiddlePosition", WORLD_FRAME, registry);
   private FramePoint3D measuredCPPosition = new FramePoint3D(WORLD_FRAME);
   private final FrameQuaternion desiredPelvisOrientation = new FrameQuaternion();

   private FramePoint3D measuredCoMPosition;
   private FrameVector3D measuredCoMVelocity;
   private FramePoint3D desCentroidalMomentPivot;

   private final YoGraphicDefinition graphicsGroup;
   private final ArrayList<YoFramePoint3D> yoGraphicPositions = new ArrayList<>();
   private final YoInteger numberOfControlTicksPerVizUpdate = new YoInteger("numberOfControlTicksPerVizUpdate", registry);
   private int index;
   private int tickCount = 0;

   
   private final YoFramePoint3D desiredCurrentFootPosition = new YoFramePoint3D("desiredCurrentFootPosition", WORLD_FRAME, registry);
   private final YoFrameVector3D headingDirection = new YoFrameVector3D("headingDirection", WORLD_FRAME, registry);
   private final YoFramePoint3D measuredCenterOfMass = new YoFramePoint3D("measuredCenterOfMass", WORLD_FRAME, registry);
   private final YoFramePoint3D measuredCapturePointPosition = new YoFramePoint3D("measuredCapturePoint", WORLD_FRAME, registry);

   private final YoFramePose3D desiredNextPoseOnPath = new YoFramePose3D("desiredNextPoseOnPath", WORLD_FRAME, registry);
   private final YoFramePose3D currentPoseOnPath = new YoFramePose3D("currentPoseOnPath", WORLD_FRAME, registry);

   private final YoFramePoint3D desiredCMP = new YoFramePoint3D("desiredCentroidalMomentPivotPosition", WORLD_FRAME, registry);
   private final YoFramePoint3D correctedCMP = new YoFramePoint3D("correctedCentroidalMomentPivotPosition", WORLD_FRAME, registry);
   private final YoFramePoint3D originalCMP = new YoFramePoint3D("originalCentroidalMomentPivotPosition", WORLD_FRAME, registry);

   private final YoDouble rotationPerStep = new YoDouble("rotationPerStep", registry);

   private final YoBoolean isFirstStep = new YoBoolean("isFirstStep", registry);
   private NFootstepListVisualizer visualizerPlannedFootSteps;

   private final YoFramePoint3D desiredCapturePoint = new YoFramePoint3D("desiredCapturePoint", WORLD_FRAME, registry);

   private double omega0 = Math.sqrt(9.81 / CENTER_OF_MASS_HEIGHT);
   int numberOfStepsToPlan = 3;

   /**
    * This is the robot the controller uses.
    */
   private final MultiBodySystemBasics controllerRobot;

   /**
    * This variable triggers the controller to initiate walking.
    */
   private final YoBoolean walk = new YoBoolean("walk", registry);
   /**
    * The duration for the double support phase.
    */
   private final YoDouble transferDuration = new YoDouble("transferDuration", registry);
   /**
    * The duration for the single support phase.
    */
   private final YoDouble swingDuration = new YoDouble("swingDuration", registry);
   /**
    * The desired step length with respect to the support foot.
    */
   private final YoDouble stepLength = new YoDouble("stepLength", registry);
   /**
    * The desired step length with respect to the support foot.
    */
   private final YoDouble sideWayStepLength = new YoDouble("sideWayStepLength", registry);

   /**
    * The desired distance between the two feet.
    */
   private final YoDouble stepWidth = new YoDouble("stepWidth", registry);

   /**
    * This is the IHMC whole-body controller core which is used at the core of the walking controller
    * on Atlas and Valkyrie.
    */
   private final WholeBodyControllerCore wholeBodyControllerCore;

   /**
    * This is the controller input. This is needed to setup a local version of the robot for the
    * controller.
    */
   private final ControllerInput controllerInput;

   /**
    * We define as an enum the list of possible states that the state machine will be allowed to go
    * through.
    */
   private enum WalkingStateEnum
   {
      /**
       * The simulation starts with the robot standing. It will switch to the initial transfer only when
       * the variable {@code walk} is set to {@code true}.
       */
      STANDING,
      /**
       * This is the double support phase that moves the center of mass from the right foot to the left
       * foot.
       */
      TRANSFER_TO_LEFT,
      /**
       * This is the double support phase that moves the center of mass from the left foot to the right
       * foot.
       */
      TRANSFER_TO_RIGHT,
      /**
       * This is the single support phase during which the center of mass stays still while the right is
       * swinging to the desired footstep location.
       */
      LEFT_SUPPORT,
      /**
       * This is the single support phase during which the center of mass stays still while the left is
       * swinging to the desired footstep location.
       */
      RIGHT_SUPPORT
   };

   /**
    * The finite state machine to which we register a set of specialized controllers and a set of
    * transitions to go from one controller to another.
    */
   private final StateMachine<WalkingStateEnum, State> stateMachine;
   private final RobotWalkerFive robotWalkerFive;

   /**
    * We will use a single instance of the controller core command for convenience. Note that in this
    * example, only the Inverse Dynamics control mode will be demonstrated.
    */
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

   /**
    * We will define only one set of gains for controlling the whole robot. This is not ideal but is
    * enough for this example.
    */
   private final DefaultYoPIDSE3Gains gainsCoMandPelvis = new DefaultYoPIDSE3Gains("gainsCoMControl", GainCoupling.XYZ, false, registry);
   private final DefaultYoPIDSE3Gains gainsSwingFoot = new DefaultYoPIDSE3Gains("gainsSwingFoot", GainCoupling.XYZ, false, registry);
   YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   BipedSupportPolygons bipedSupportPolygons;
   ReferenceFrame midFeetFrame;
   SideDependentList<ContactablePlaneBody> contactableFeet;
   SideDependentList<ReferenceFrame> soleFrames, soleZUpFrames;
   SideDependentList<PlaneContactStateCommand> planeContactStateCommands;
   SideDependentList<RigidBodyBasics> feet;

   public RobotWalkerFiveController(ControllerInput controllerInput,
                                    ControllerOutput controllerOutput,
                                    double controlDT,
                                    double gravityZ,
                                    M2RobotDefinition robotDefinition)
   {
      this.controllerInput = controllerInput;
      this.robotWalkerFive = new RobotWalkerFive(controllerInput, controllerOutput, robotDefinition);

      wholeBodyControllerCore = createControllerCore(controlDT, gravityZ, yoGraphicsListRegistry);
      feet = new SideDependentList<>(side -> robotWalkerFive.getFoot(side));
      soleFrames = new SideDependentList<>(side -> robotWalkerFive.getFootContactableBody(side).getSoleFrame());
      soleZUpFrames = new SideDependentList<>(side -> new ZUpFrame(soleFrames.get(side), soleFrames.get(side).getName() + "ZUp"));
      midFeetFrame = new MidFrameZUpFrame("midFeetZUpFrame", WORLD_FRAME, soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));

      contactableFeet = new SideDependentList<>(side -> robotWalkerFive.getFootContactableBody(side));

      bipedSupportPolygons = new BipedSupportPolygons(midFeetFrame, soleZUpFrames, soleFrames, registry, yoGraphicsListRegistry);
      stateMachine = createStateMachine();

      useCapturePoint.set(true);
      stepWidth.set(0.2);
      isFirstStep.set(true);
      
      if (useCapturePoint.getBooleanValue())
      {
         transferDuration.set(1.2);
         swingDuration.set(0.5);
         stepLength.set(0.15);
         sideWayStepLength.set(0.0);
         // more challenging settings:
         //         transferDuration.set(0.85);
         //         swingDuration.set(0.4);
         //         stepLength.set(0.15);
      }
      else
      {
         transferDuration.set(1.2);
         swingDuration.set(0.9);
         stepLength.set(0.15);
         sideWayStepLength.set(0.0);
      }

      controllerRobot = MultiBodySystemBasics.toMultiBodySystemBasics(MultiBodySystemFactories.cloneMultiBodySystem(controllerInput.getInput().getRootBody(),
                                                                                                                    ReferenceFrame.getWorldFrame(),
                                                                                                                    ""));
      visualizerPlannedFootSteps = new NFootstepListVisualizer(contactableFeet, yoGraphicsListRegistry, registry);
      //  create our graphics here
      index = 0;
      numberOfControlTicksPerVizUpdate.set(100);
      this.graphicsGroup = createVisualization();
   }

   public YoGraphicDefinition createVisualization()
   {
      // define a group of YoGraphic definitions
      YoGraphicGroupDefinition graphicsGroup = new YoGraphicGroupDefinition("Controller");
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("desiredCapturePoint", desiredCapturePoint, 0.02, ColorDefinitions.Red()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("measuredCapturePoint",
                                                                            measuredCapturePointPosition,
                                                                            0.02,
                                                                            ColorDefinitions.Blue()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("measuredCenterOfMass", measuredCenterOfMass, 0.02, ColorDefinitions.Black()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("desiredCentroidalMomentPivotPoint", desiredCMP, 0.02, ColorDefinitions.Green()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("desiredCurrentFootPosition",
                                                                            desiredCurrentFootPosition,
                                                                            0.03,
                                                                            ColorDefinitions.Blue()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("currentMiddlePosition", currentMiddlePosition, 0.02, ColorDefinitions.Black()));

      
      // this defines how many balls we can display at once
      String name = "cpTrajectory";
      YoRegistry ballregistry = new YoRegistry("CPBalls");

      int numberOfBalls = 300;
      // this defines the radius for each ball
      double ballSize = 0.004;
      for (int i = 0; i <= numberOfBalls; i++)
      {
         YoFramePoint3D yoFramePoint = new YoFramePoint3D(name + i, "", WORLD_FRAME, ballregistry);
         yoFramePoint.setToNaN();
         yoGraphicPositions.add(yoFramePoint);

         graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D(name + i, yoFramePoint, ballSize, ColorDefinitions.Yellow()));
      }
      registry.addChild(ballregistry);


      
      return graphicsGroup;
   }

   private WholeBodyControllerCore createControllerCore(double controlDT, double gravityZ, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      // This time the robot has a floating joint.
      FloatingJointBasics rootJoint = robotWalkerFive.getRootJoint();
      RigidBodyBasics elevator = robotWalkerFive.getElevator();

      // These are all the joints of the robot arm.
      JointBasics[] jointsArray = MultiBodySystemTools.collectSubtreeJoints(elevator);
      OneDoFJoint[] controlledJoints = MultiBodySystemTools.filterJoints(jointsArray, OneDoFJoint.class);

      // This class contains basic optimization settings required for QP formulation.
      ControllerCoreOptimizationSettings controllerCoreOptimizationSettings = new RobotWalkerFiveOptimizationSettings();
      // This is the toolbox for the controller core with everything it needs to run properly.
      toolbox = new WholeBodyControlCoreToolbox(controlDT,
                                                gravityZ,
                                                rootJoint,
                                                controlledJoints,
                                                robotWalkerFive.getCenterOfMassFrame(),
                                                controllerCoreOptimizationSettings,
                                                yoGraphicsListRegistry,
                                                registry);

      // The controller core needs all the possibly contacting bodies of the robot to create all the modules needed for later.
      toolbox.setupForInverseDynamicsSolver(Arrays.asList(robotWalkerFive.getFootContactableBody(RobotSide.LEFT),
                                                          robotWalkerFive.getFootContactableBody(RobotSide.RIGHT)));

      /*
       * We register all the commands that we will use in this controller, i.e. commands for the feet that
       * we'll for the swing, an orientation command for the pelvis to keep it level to the ground, and
       * the command for controlling the center of mass.
       */
      FeedbackControllerTemplate template = new FeedbackControllerTemplate();

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics foot = robotWalkerFive.getFoot(robotSide);
         template.enableSpatialFeedbackController(foot);
      }

      template.enableOrientationFeedbackController(robotWalkerFive.getPelvis());
      template.enableCenterOfMassFeedbackController();

      // Finally we can create the controller core.
      return new WholeBodyControllerCore(toolbox, template, registry);
   }

   private StateMachine<WalkingStateEnum, State> createStateMachine()
   {
      // The creation of a state machine is done using a factory:
      StateMachineFactory<WalkingStateEnum, State> factory = new StateMachineFactory<>(WalkingStateEnum.class);
      // The namePrefix will be used to create internally some YoVariables.
      factory.setNamePrefix("stateMachine");
      // The registry to which the YoVariables will be registered with.
      factory.setRegistry(registry);
      // We will need a clock to have access to the time spent in each state for computing trajectories.
      factory.buildYoClock(controllerInput::getTime);

      /*
       * Then we get to the point where actually create the state and the transitions. In this example, we
       * will only use what is called here "done transitions". These transitions get triggered as a state
       * reports that it is done, when triggered the state machine goes to the next state.
       */
      // Here we setup the STANDING state. When done, the state machine will transition to the TRANSFER_TO_LEFT state.
      factory.addStateAndDoneTransition(WalkingStateEnum.STANDING, new StandingState(), WalkingStateEnum.TRANSFER_TO_LEFT);

      // Here using the transitions, we create a cycle that will result in having the robot walking indefinitely.
      factory.addStateAndDoneTransition(WalkingStateEnum.TRANSFER_TO_LEFT, new TransferState(RobotSide.LEFT), WalkingStateEnum.LEFT_SUPPORT);
      factory.addStateAndDoneTransition(WalkingStateEnum.LEFT_SUPPORT, new SingleSupportState(RobotSide.LEFT), WalkingStateEnum.TRANSFER_TO_RIGHT);
      factory.addStateAndDoneTransition(WalkingStateEnum.TRANSFER_TO_RIGHT, new TransferState(RobotSide.RIGHT), WalkingStateEnum.RIGHT_SUPPORT);
      factory.addStateAndDoneTransition(WalkingStateEnum.RIGHT_SUPPORT, new SingleSupportState(RobotSide.RIGHT), WalkingStateEnum.TRANSFER_TO_LEFT);

      // Finally we can build the state machine which will start with the STANDING state.
      return factory.build(WalkingStateEnum.STANDING);
   }

   @Override
   public void initialize()
   {
      // We initialize the gains. As in the previous examples, the values here are rather arbitrary.      
      double p_gains = 50.0;
      double d_gains = GainCalculator.computeDerivativeGain(p_gains, 1.0);
      gainsCoMandPelvis.setPositionProportionalGains(p_gains);
      gainsCoMandPelvis.setPositionDerivativeGains(d_gains);
      gainsCoMandPelvis.setOrientationProportionalGains(p_gains);
      gainsCoMandPelvis.setOrientationDerivativeGains(d_gains);

      walkerWillFreakOut.set(false);
      desiredPelvisOrientation.setToZero();

   }


   /**
    * This time, the {@code doControl} method is rather empty as most of the controller is actually
    * implemented in the different states declared further down.
    */
   @Override
   public void doControl()
   {
      // We update the configuration state of our inverse dynamics robot model from the latest state of the simulated robot.
      robotWalkerFive.updateInverseDynamicsRobotState();
      soleZUpFrames.forEach(frame -> frame.update());
      midFeetFrame.update();
      walkerWillFreakOut.set(false);

      if (!bipedSupportPolygons.getSupportPolygonInWorld().isPointInside(new FramePoint2D(WORLD_FRAME, desiredCMP.getX(), desiredCMP.getY())))
      {
         // CMP should not be outside support polygon
         walkerWillFreakOut.set(true);
      }
      /*
       * Here we request the state machine to call the doAction() method of the active state and to check
       * if the transition to the next state should be engaged. See further below for the implementation
       * of the doAction() method for each state.
       */
      stateMachine.doActionAndTransition();

      
   
      /*
       * As the pelvis is to kept level to the ground independently to the active state, we can simply
       * setup the command here.
       */
      OrientationFeedbackControlCommand pelvisOrientationCommand = new OrientationFeedbackControlCommand();
      // sets control mode to inverse dynamics
      pelvisOrientationCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);
      pelvisOrientationCommand.set(robotWalkerFive.getElevator(), robotWalkerFive.getPelvis());
      pelvisOrientationCommand.setInverseDynamics(desiredPelvisOrientation,
                                                  new FrameVector3D(WORLD_FRAME, 0.0, 0.0, 0.0),
                                                  new FrameVector3D(WORLD_FRAME, 0.0, 0.0, 0.0));
      pelvisOrientationCommand.setGains(gainsCoMandPelvis.getOrientationGains());
      pelvisOrientationCommand.setWeightForSolver(1.0);
      controllerCoreCommand.addFeedbackControlCommand(pelvisOrientationCommand);

      // Submit all the objectives to be achieved to the controller core.
      // Magic happens here.
      wholeBodyControllerCore.compute(controllerCoreCommand);

      // And collect the output to update the simulated robot.
      JointDesiredOutputListReadOnly outputForLowLevelController = wholeBodyControllerCore.getOutputForLowLevelController();

      for (int i = 0; i < outputForLowLevelController.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJointReadOnly oneDoFJoint = outputForLowLevelController.getOneDoFJoint(i);
         JointDesiredOutputReadOnly jointDesiredOutput = outputForLowLevelController.getJointDesiredOutput(i);
         robotWalkerFive.setDesiredEffort(oneDoFJoint.getName(), jointDesiredOutput.getDesiredTorque());
      }

      // check if current capture point is inside the support polygon
      FramePoint2D capturePoint2D = new FramePoint2D();
      capturePoint2D.setX(measuredCapturePointPosition.getX());
      capturePoint2D.setY(measuredCapturePointPosition.getY());

      boolean isFalling = false;
      if (!bipedSupportPolygons.getSupportPolygonInWorld().isPointInside(capturePoint2D))
      {
         isFalling = true;
         walkerIsFalling.set(isFalling);
         // cannot use LIP model anymore because the robot height will no longer be constant...
      }
   }

   /**
    * A {@code calculateDesiredCoMAcceleration}
    * 
    * @param measuredCoMPosition      refers to the current center of motion
    * @param desCentroidalMomentPivot refers to the desired CMP
    */
   public FrameVector3D calculateDesiredCoMAcceleration(FramePoint3D measuredCoMPosition, FramePoint3D desCentroidalMomentPivot)
   {
      FrameVector3D desiredCoMAcceleration = new FrameVector3D(WORLD_FRAME);
      desiredCoMAcceleration.sub(measuredCoMPosition, desCentroidalMomentPivot);
      desiredCoMAcceleration.scale(omega0 * omega0);

      return desiredCoMAcceleration;
   }

   /**
    * The {@code calculateDesiredCentroidalMomentPivot} calculated the deisred CMP based on a desired
    * capture point poisition and the current capture point position
    * 
    * @param desiredCapturePointPosition  refers to the desired capture point position
    * @param desiredCapturePointVelocity  refers to the desired capture point velocity
    * @param measuredCapturePointPosition refers to the current capture point position
    */
   public FramePoint3D calculateMeasuredCapturePointPosition(FrameVector3D measuredCoMVelocity, FramePoint3D measuredCoMPosition)
   {
      FramePoint3D measuredCPPosition = new FramePoint3D(WORLD_FRAME);
      measuredCPPosition.scaleAdd(1.0 / omega0, measuredCoMVelocity, measuredCoMPosition);

      return measuredCPPosition;
   }

   /**
    * The {@code calculateDesiredCentroidalMomentPivot} calculated the deisred CMP based on a desired
    * capture point poisition and the current capture point position
    * 
    * @param desiredCapturePointPosition  refers to the desired capture point position
    * @param desiredCapturePointVelocity  refers to the desired capture point velocity
    * @param measuredCapturePointPosition refers to the current capture point position
    */
   public FramePoint3D calculateDesiredCentroidalMomentPivot(FramePoint3D desiredCapturePointPosition,
                                                             FrameVector3D desiredCapturePointVelocity,
                                                             FramePoint3D measuredCapturePointPosition)
   {
      double gain_p = 2.0;
      FramePoint3D errorCapturePointPosition = new FramePoint3D(WORLD_FRAME);
      errorCapturePointPosition.sub(measuredCapturePointPosition, desiredCapturePointPosition);
      errorCapturePoint.set(Math.sqrt(errorCapturePointPosition.getX() * errorCapturePointPosition.getX()
            + errorCapturePointPosition.getY() * errorCapturePointPosition.getY() + errorCapturePointPosition.getZ() * errorCapturePointPosition.getZ()));

      FramePoint3D desCentroidalMomentPivot = new FramePoint3D(WORLD_FRAME);
      desCentroidalMomentPivot.scaleAdd(gain_p, errorCapturePointPosition, measuredCapturePointPosition);
      desCentroidalMomentPivot.scaleAdd(-1.0 / omega0, desiredCapturePointVelocity, desCentroidalMomentPivot);

      return desCentroidalMomentPivot;
   }

   /**
    * A {@code sendCenterOfMassCommand} is created using the new calculated
    * {@code centerOfMassPosition} and adds the command to the controller core.
    * 
    * @param centerOfMassPosition refers to newly calculated position of the center of mass of the
    *                             robot.
    */
   public void sendCenterOfMassCommand(FramePoint3DReadOnly centerOfMassPosition, FrameVector3D feedForwardLinearVelocity)
   {
      CenterOfMassFeedbackControlCommand centerOfMassCommand = new CenterOfMassFeedbackControlCommand();
      centerOfMassCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS); // sets control mode to inverse dynamics
      FrameVector3D feedForwardLinearAcceleration = new FrameVector3D(WORLD_FRAME, 0.0, 0.0, 0.0);
      centerOfMassCommand.setInverseDynamics(centerOfMassPosition, feedForwardLinearVelocity, feedForwardLinearAcceleration);
      centerOfMassCommand.setGains(gainsCoMandPelvis.getPositionGains());
      centerOfMassCommand.setWeightForSolver(1.0);
      controllerCoreCommand.addFeedbackControlCommand(centerOfMassCommand);

      measuredCenterOfMass.set(centerOfMassPosition);
   }

   /**
    * A {@code sendCapturePointCommand} is created {@code desiredCapturePointPosition} and adds the
    * command to the controller core.
    * 
    * @param desiredCapturePointPosition refers to the desired capture poitn position
    */
   public void sendCapturePointCommand(FramePoint3D desiredCapturePointPosition, FrameVector3D desiredCapturePointVelocity)
   {

      measuredCoMPosition = new FramePoint3D(WORLD_FRAME, toolbox.getCenterOfMassFrame().getTransformToWorldFrame().getTranslation());
      measuredCoMVelocity = new FrameVector3D(WORLD_FRAME, toolbox.getCentroidalMomentumRateCalculator().getCenterOfMassVelocity());
      measuredCPPosition = calculateMeasuredCapturePointPosition(measuredCoMVelocity, measuredCoMPosition);
      desCentroidalMomentPivot = calculateDesiredCentroidalMomentPivot(desiredCapturePointPosition, desiredCapturePointVelocity, measuredCPPosition);

      FrameVector3D desiredCoMAcceleration = calculateDesiredCoMAcceleration(measuredCoMPosition, desCentroidalMomentPivot);
      double gain_p = 10.0;
      double gain_d = GainCalculator.computeDerivativeGain(gain_p, 1.0);
      desiredCoMAcceleration.setZ(gain_p * (CENTER_OF_MASS_HEIGHT - measuredCoMPosition.getZ()) - gain_d * measuredCoMVelocity.getZ());

      // send the desired linear momentum rate change as a command to the controller
      FrameVector3D desiredLinMomentumRate = new FrameVector3D(WORLD_FRAME, desiredCoMAcceleration);
      desiredLinMomentumRate.scale(toolbox.getTotalRobotMass());
      MomentumRateCommand momentumRateCommand = new MomentumRateCommand();
      momentumRateCommand.setWeight(1.0);
      momentumRateCommand.setLinearMomentumRate(desiredLinMomentumRate);
      momentumRateCommand.setSelectionMatrixForLinearControl();

      // update visualization variables
      desiredCapturePoint.set(desiredCapturePointPosition);
      desiredCMP.set(desCentroidalMomentPivot);
      originalCMP.set(desCentroidalMomentPivot);

      correctedCMP.set(new Point2D(0.0, 0.0));
      
      // TODO fix CMP correction
      // correct desired centroidal moment pivot to stay within support polygon
      if (!bipedSupportPolygons.getSupportPolygonInWorld()
                               .isPointInside(new FramePoint2D(WORLD_FRAME, desCentroidalMomentPivot.getX(), desCentroidalMomentPivot.getY()))
            && !desCentroidalMomentPivot.equals(measuredCPPosition))
      {
         // CMP should not be outside support polygon
         FramePoint2D corrCMP2D = new FramePoint2D(WORLD_FRAME);
         FramePoint2D firstPointOnLine = new FramePoint2D(WORLD_FRAME);
         FramePoint2D secondPointOnLine = new FramePoint2D(WORLD_FRAME);

         firstPointOnLine.set(desCentroidalMomentPivot.getX(), desCentroidalMomentPivot.getY());
         secondPointOnLine.set(measuredCPPosition.getX(), measuredCPPosition.getY());
         FrameLine2DReadOnly ray = new FrameLine2D(WORLD_FRAME, firstPointOnLine, secondPointOnLine);
         bipedSupportPolygons.getSupportPolygonInWorld().getClosestPointWithRay(ray, corrCMP2D);

         correctedCMP.set(corrCMP2D);
         desCentroidalMomentPivot.setX(corrCMP2D.getX());
         desCentroidalMomentPivot.setY(corrCMP2D.getY());
      }

      desiredCMP.set(desCentroidalMomentPivot);

      measuredCenterOfMass.set(measuredCoMPosition);
      FramePoint3D measuredCPPosition = new FramePoint3D(WORLD_FRAME);
      measuredCPPosition = calculateMeasuredCapturePointPosition(measuredCoMVelocity, measuredCoMPosition);
      measuredCapturePointPosition.set(measuredCPPosition);

      // send command to controller
      controllerCoreCommand.addInverseDynamicsCommand(momentumRateCommand);

   }

   /**
    * During the standing state, the center of mass is kept at a constant position right between the
    * feet and the feet are both used for support.
    */
   private class StandingState implements State
   {
      @Override
      public void onEntry()
      {
      }

      @Override
      public void doAction(double timeInState)
      {
         if (useCapturePoint.getBooleanValue())
         {
            // desired capture point from the gui
            FramePoint3D capturePointPosition = new FramePoint3D(WORLD_FRAME);
            capturePointPosition.set(desiredCapturePoint);
            capturePointPosition.setZ(CENTER_OF_MASS_HEIGHT);

            // And now we pack the command for the controller core.        
            sendCapturePointCommand(capturePointPosition,
                                    new FrameVector3D(WORLD_FRAME,
                                                      feedForwardLinearVelocity.getX(),
                                                      feedForwardLinearVelocity.getY(),
                                                      feedForwardLinearVelocity.getZ()));
         }
         else
         {
            // Here we get the position of both feet to compute the middle.
            FramePoint3D leftSolePosition = new FramePoint3D(robotWalkerFive.getSoleFrame(RobotSide.LEFT));
            leftSolePosition.changeFrame(WORLD_FRAME);
            FramePoint3D rightSolePosition = new FramePoint3D(robotWalkerFive.getSoleFrame(RobotSide.RIGHT));
            rightSolePosition.changeFrame(WORLD_FRAME);
            FramePoint3D centerOfMassPosition = new FramePoint3D(WORLD_FRAME);
            // The desired center of mass position is set to be right in between the feet.
            centerOfMassPosition.interpolate(leftSolePosition, rightSolePosition, 0.5);
            // We set the desired height.
            centerOfMassPosition.setZ(CENTER_OF_MASS_HEIGHT);

            // So now, we just have pack the command for the controller core.
            sendCenterOfMassCommand(centerOfMassPosition,
                                    new FrameVector3D(WORLD_FRAME,
                                                      feedForwardLinearVelocity.getX(),
                                                      feedForwardLinearVelocity.getY(),
                                                      feedForwardLinearVelocity.getZ()));
         }

         // As for the standing state, we request both feet to be in support.
         planeContactStateCommands = new SideDependentList<>(side -> createPlaneContactStateCommand(side, true));

         // Now it is the turn of the feet.
         for (RobotSide robotSide : RobotSide.values)
         {
            // As their are in support, they should not be accelerating. So we make a zero-acceleration command.
            SpatialAccelerationCommand footZeroAcceleration = createFootZeroAccelerationCommand(robotSide);
            controllerCoreCommand.addInverseDynamicsCommand(footZeroAcceleration);
            controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommands.get(robotSide));
         }
         // update the support polygons
         bipedSupportPolygons.updateUsingContactStateCommand(planeContactStateCommands);
      }

      @Override
      public void onExit(double timeInState)
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         /*
          * As soon as the walk variable is set to true via the simulation GUI, the state machine will leave
          * this state causing the robot to start walking.
          */
         return walk.getValue();
      }
   }

   /**
    * During the transfer state, the robot is still in double support but the center of mass is now
    * moving to be above the leading foot.
    */
   private class TransferState implements State
   {
      private final RobotSide transferToSide;
      private final FramePoint3D initialPosition = new FramePoint3D();
      private final FramePoint3D finalPosition = new FramePoint3D();
      /**
       * We use a 5th order polynomial to smooth out the velocity of the center of mass at the start and
       * end of this state. This allows the robot to walk slightly faster without falling.
       */
      private final YoPolynomial desiredTrajectory;

      public TransferState(RobotSide transferToSide)
      {
         this.transferToSide = transferToSide;
         desiredTrajectory = new YoPolynomial(transferToSide.getCamelCaseName() + "desiredTrajectory", 6, registry);
      }

      @Override
      public void onEntry()
      {
         /*
          * This method is called once when this state becomes active. this is the perfect time to initialize
          * the endpoints for the center of mass. It should start from wherever it is at the start of this
          * state and end above the leading foot.
          */
         initialPosition.setToZero(robotWalkerFive.getCenterOfMassFrame());
         initialPosition.changeFrame(WORLD_FRAME);
         finalPosition.setToZero(robotWalkerFive.getSoleFrame(transferToSide));
         finalPosition.changeFrame(WORLD_FRAME);
         finalPosition.setZ(CENTER_OF_MASS_HEIGHT);
         // The trajectory is setup such that it will always from 0.0 to 1.0 within the given transferDuration.
         desiredTrajectory.setQuintic(0, transferDuration.getValue(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

      }

      @Override
      public void doAction(double timeInState)
      {
         // The trajectory generator is updated to get the current progression percentage alpha which is in [0, 1].
         desiredTrajectory.compute(MathTools.clamp(timeInState, 0.0, transferDuration.getValue()));
         double alpha = desiredTrajectory.getValue();
         double alphaDot = desiredTrajectory.getVelocity();
         FrameVector3D velocity = new FrameVector3D();
         velocity.sub(finalPosition, initialPosition);
         velocity.scale(alphaDot);

         if (useCapturePoint.getBooleanValue())
         {
            // The alpha parameter is now used to interpolate between the initial and final center of mass positions.
            FramePoint3D capturePointPosition = new FramePoint3D(WORLD_FRAME);
            capturePointPosition.interpolate(initialPosition, finalPosition, alpha);
            capturePointPosition.setZ(CENTER_OF_MASS_HEIGHT);

            // And now we pack the command for the controller core.
            sendCapturePointCommand(capturePointPosition, velocity);
         }
         else
         {
            // The alpha parameter is now used to interpolate between the initial and final center of mass positions.
            FramePoint3D centerOfMassPosition = new FramePoint3D(WORLD_FRAME);
            centerOfMassPosition.interpolate(initialPosition, finalPosition, alpha);
            centerOfMassPosition.setZ(CENTER_OF_MASS_HEIGHT);

            // And now we pack the command for the controller core.
            sendCenterOfMassCommand(centerOfMassPosition, new FrameVector3D(WORLD_FRAME, 0.0, 0.0, 0.0));
         }

         // As for the standing state, we request both feet to be in support.
         planeContactStateCommands = new SideDependentList<>(side -> createPlaneContactStateCommand(side, true));

         for (RobotSide robotSide : RobotSide.values)
         {
            SpatialAccelerationCommand footZeroAcceleration = createFootZeroAccelerationCommand(robotSide);
            controllerCoreCommand.addInverseDynamicsCommand(footZeroAcceleration);
            controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommands.get(robotSide));
         }
         bipedSupportPolygons.updateUsingContactStateCommand(planeContactStateCommands);
      }

      @Override
      public void onExit(double timeInState)
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         // As soon as the time reaches the given transferDuration, the state machine will enter the next single support state.
         return timeInState >= transferDuration.getValue();
      }
   }

   /**
    * During the single support state, only one foot is in support while the other is swinging to the
    * next footstep. The center of mass stays still.
    */
   private class SingleSupportState implements State
   {
      private final RobotSide supportSide;
      private final RobotSide swingSide;
      /** We will use a trajectory generator for the swing. */
      private final TwoWaypointSwingGenerator swingPositionTrajectory;
      private final MultipleWaypointsOrientationTrajectoryGenerator swingOrientationTrajectory;
      private final CapturePointTrajectory capturePointTrajectory;
      /**
       * Generate some YoVariables that we can monitor during the simulation.
       */
      private final YoFramePoint3D nextFootstepPosition;
      //      private final YoFramePoint2D next2DFootstepPosition;
      private final YoFramePoint3D initialFootPosition;
      private final YoFrameVector3D touchdownVelocity;
      private final FramePose3D swingControlFramePose = new FramePose3D();
      private final FrameQuaternion finalFootOrienation = new FrameQuaternion();
      private RobotWalkerFootStepPlanner footStepPlanner;

      public SingleSupportState(RobotSide supportSide)
      {
         this.supportSide = supportSide;
         swingSide = supportSide.getOppositeSide();
         initialFootPosition = new YoFramePoint3D(swingSide.getCamelCaseName() + "SwingStart", WORLD_FRAME, registry);
         nextFootstepPosition = new YoFramePoint3D(swingSide.getCamelCaseName() + "NextFootstep", WORLD_FRAME, registry);
         touchdownVelocity = new YoFrameVector3D(swingSide.getCamelCaseName() + "TouchdownVelocity", WORLD_FRAME, registry);
         touchdownVelocity.set(0.0, 0.0, -0.1);
         double groundClearance = 0.0;
         // The trajectory generator such that the swing starts from the current foot position and ends at the given nextFootstepPosition.
         swingPositionTrajectory = new TwoWaypointSwingGenerator(swingSide.getCamelCaseName() + "Swing", 0.05, 0.15, groundClearance, 0.0, registry, null);
         swingOrientationTrajectory = new MultipleWaypointsOrientationTrajectoryGenerator(swingSide.getCamelCaseName() + "SwingOrientation",
                                                                                          WORLD_FRAME,
                                                                                          registry);

         // Here we define the control frame pose that is needed to specify the point on the foot we want to control to the controller core.
         swingControlFramePose.setToZero(robotWalkerFive.getSoleFrame(swingSide));
         swingControlFramePose.changeFrame(robotWalkerFive.getFoot(swingSide).getBodyFixedFrame());

         // Setup capture point trajectory planner
         footStepPlanner = new RobotWalkerFootStepPlanner(feet, soleFrames, numberOfStepsToPlan);
         capturePointTrajectory = new CapturePointTrajectory(null, swingDuration.getDoubleValue());
      }

      @Override
      public void onEntry()
      {
         // Get current state of swing-foot
         FramePoint3D swingFootPosition = new FramePoint3D(robotWalkerFive.getSoleFrame(swingSide));
         swingFootPosition.changeFrame(WORLD_FRAME);
         FrameQuaternion swingFootOrientation = new FrameQuaternion(robotWalkerFive.getSoleFrame(swingSide));
         swingFootOrientation.changeFrame(WORLD_FRAME);
         initialFootPosition.set(swingFootPosition);

         // Generate a set of planned footsteps and select the next one as desired footstep
         // init planner
         footStepPlanner.initialize(swingSide,
                                    stepLength.getDoubleValue(),
                                    stepWidth.getDoubleValue(),
                                    rotationPerStep.getValue(),
                                    sideWayStepLength.getValue());
         footStepPlanner.updateCurrentPelvisPose(robotWalkerFive.getPelvis());
         
         ArrayList<Footstep> currentPlannedFootStepList = new ArrayList<Footstep>();
         currentPlannedFootStepList = footStepPlanner.generateDesiredFootstepList();
         footStepPlanner.generateFootsteps(currentPlannedFootStepList);

         // update visuals
         desiredNextPoseOnPath.set(footStepPlanner.getDesiredNextPoseOnPath());
         currentPoseOnPath.set(footStepPlanner.getCurrentPoseOnPath());
         visualizerPlannedFootSteps.update(currentPlannedFootStepList);

         // Grab the first planned foot step as next desired footstep
         Footstep nextFootStep = currentPlannedFootStepList.get(0);
         nextFootStep.getOrientation(finalFootOrienation);
         FramePoint3D nextFootstepPosition3D = new FramePoint3D();
         nextFootStep.getPosition(nextFootstepPosition3D);
         nextFootstepPosition.set(nextFootstepPosition3D);

         // Setup trajectory for capture point
//         if(isFirstStep.getBooleanValue())
//         {
         Footstep currentFootStep = new Footstep(supportSide);
         FramePose3D footPose = new FramePose3D(soleFrames.get(supportSide));
         footPose.changeFrame(WORLD_FRAME);
         currentFootStep.setPose(footPose);
         currentPlannedFootStepList.add(0, currentFootStep); 
//         isFirstStep.set(false);
//         }
         capturePointTrajectory.initialize(currentPlannedFootStepList,swingDuration.getDoubleValue());
         
         // Setup swing foot orientation trajectory
         swingOrientationTrajectory.clear();
         swingOrientationTrajectory.appendWaypoint(swingDuration.getValue(), finalFootOrienation, new Vector3D(0.0, 0.0, 0.0));
         swingOrientationTrajectory.initialize();

         // Setup swing foot position trajectory
         swingPositionTrajectory.setInitialConditions(initialFootPosition, new FrameVector3D());
         swingPositionTrajectory.setFinalConditions(nextFootstepPosition, touchdownVelocity);
         swingPositionTrajectory.setTrajectoryType(TrajectoryType.DEFAULT);
         swingPositionTrajectory.setStepTime(swingDuration.getValue());
         swingPositionTrajectory.initialize();
         while (swingPositionTrajectory.doOptimizationUpdate())
            ;
      }

      @Override
      public void doAction(double timeInState)
      {
         // During this state, the center of mass is kept right above the support foot.
         if(useCapturePointTrajectory.getBooleanValue())
         {
            FramePoint3D capturePointPosition = new FramePoint3D();
            FrameVector3D capturePointVelocity = new FrameVector3D();
            capturePointTrajectory.compute(timeInState,capturePointPosition,capturePointVelocity);

            // And now we pack the command for the controller core.
            sendCapturePointCommand(capturePointPosition, new FrameVector3D(WORLD_FRAME, capturePointVelocity));
            
            tickCount++;

            if (tickCount >= numberOfControlTicksPerVizUpdate.getValue())
            {
               tickCount = 0;
               if (index >= yoGraphicPositions.size())
               {
                  index = 0;
               }
               yoGraphicPositions.get(index).set(capturePointPosition);
               index++;
            }
            
            
         }
         //TODO fix selection of mode
//         else if(useCapturePoint.getBooleanValue())
//         {
//            FramePoint3D capturePointPosition = new FramePoint3D(robotWalkerFive.getSoleFrame(supportSide));
//            capturePointPosition.changeFrame(WORLD_FRAME);
//            capturePointPosition.setZ(CENTER_OF_MASS_HEIGHT);
//
//            // And now we pack the command for the controller core.
//            sendCapturePointCommand(capturePointPosition, new FrameVector3D(WORLD_FRAME, 0.0, 0.0, 0.0));
//         }
         else
         {
            FramePoint3D centerOfMassPosition = new FramePoint3D(robotWalkerFive.getSoleFrame(supportSide));
            centerOfMassPosition.changeFrame(WORLD_FRAME);
            centerOfMassPosition.setZ(CENTER_OF_MASS_HEIGHT);

            // We pack the center of mass command for the controller core.
            sendCenterOfMassCommand(centerOfMassPosition, new FrameVector3D(WORLD_FRAME, 0.0, 0.0, 0.0));
         }

         // As in the standing state, the support is specified with the contact state
         // command and zero acceleration command.
         SpatialAccelerationCommand footZeroAcceleration = createFootZeroAccelerationCommand(supportSide);
         controllerCoreCommand.addInverseDynamicsCommand(footZeroAcceleration);

         // We need to specify the contact states for swing and support foot
         planeContactStateCommands = new SideDependentList<>();
         planeContactStateCommands.put(supportSide, createPlaneContactStateCommand(supportSide, true));
         planeContactStateCommands.put(swingSide, createPlaneContactStateCommand(swingSide, false));

         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommands.get(swingSide));
         controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommands.get(supportSide));

         /*
          * Using the swing trajectory generator, we can compute the current desired position, velocity and
          * acceleration for the swing foot.
          */
         FramePoint3D position = new FramePoint3D();
         FrameVector3D velocity = new FrameVector3D();
         FrameVector3D acceleration = new FrameVector3D();

         FrameQuaternion orientation = new FrameQuaternion();
         FrameVector3D angularVelocity = new FrameVector3D();
         FrameVector3D angularAcceleration = new FrameVector3D();

         swingPositionTrajectory.compute(timeInState);
         swingPositionTrajectory.getLinearData(position, velocity, acceleration);

         swingOrientationTrajectory.compute(timeInState);
         swingOrientationTrajectory.getAngularData(orientation, angularVelocity, angularAcceleration);

         // Change the desired pelvis orientation accordingly (control sent in DoControl())          
         desiredPelvisOrientation.set(orientation);

         // Update graph
         desiredCurrentFootPosition.set(position);

         if (addTakeOffVelocity.getBooleanValue())
         {
            FrameVector3D takeOffVelocity = new FrameVector3D(WORLD_FRAME, 0.0, 0.0, 1.0);
            if (timeInState < swingDuration.getValue() * 0.1)
            {
               velocity.add(takeOffVelocity);
            }
         }

         if (addTouchDownVelocity.getBooleanValue())
         {
            FrameVector3D additionalTouchDownVelocity = new FrameVector3D(WORLD_FRAME, 0.0, 0.0, -1.0);
            if (timeInState > swingDuration.getValue() * 0.9)
            {
               velocity.add(additionalTouchDownVelocity);
            }
         }

         // Finally, we pack the swing foot command for the controller core.
         SpatialFeedbackControlCommand swingFootCommand = new SpatialFeedbackControlCommand();
         swingFootCommand.setControlMode(WholeBodyControllerCoreMode.INVERSE_DYNAMICS); // sets control mode to
         // inverse dynamics
         swingFootCommand.set(robotWalkerFive.getElevator(), robotWalkerFive.getFoot(swingSide));
         swingFootCommand.setInverseDynamics(position, velocity, acceleration);
         swingFootCommand.setControlFrameFixedInEndEffector(swingControlFramePose);
         swingFootCommand.setInverseDynamics(orientation, angularVelocity, angularAcceleration);

         gainsSwingFoot.setPositionProportionalGains(300.0);
         gainsSwingFoot.setPositionDerivativeGains(GainCalculator.computeDerivativeGain(gainsSwingFoot.getPositionGains().getProportionalGains()[0], 1.0));
         gainsSwingFoot.setOrientationProportionalGains(300.0);
         gainsSwingFoot.setOrientationDerivativeGains(GainCalculator.computeDerivativeGain(gainsSwingFoot.getOrientationGains().getProportionalGains()[0],
                                                                                           1.0));
         swingFootCommand.setGains(gainsSwingFoot);
         swingFootCommand.setWeightForSolver(1.0);

         controllerCoreCommand.addFeedbackControlCommand(swingFootCommand);
         bipedSupportPolygons.updateUsingContactStateCommand(planeContactStateCommands);
      }

      @Override
      public void onExit(double timeInState)
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         // As soon as the time reaches the given swingDuration, the state machine will
         // enter the next transfer state.
         return timeInState >= swingDuration.getValue();
      }
   }
   
   /**
    * Creates a spatial acceleration command to request a zero acceleration of the right or left foot.
    * 
    * @param robotSide refers to which foot the command is to be created for.
    * @return the zero acceleration command.
    */
   public SpatialAccelerationCommand createFootZeroAccelerationCommand(RobotSide robotSide)
   {
      SpatialAccelerationCommand footZeroAcceleration = new SpatialAccelerationCommand();
      footZeroAcceleration.set(robotWalkerFive.getElevator(), robotWalkerFive.getFoot(robotSide));
      footZeroAcceleration.setWeight(1.0);
      return footZeroAcceleration;
   }

   /**
    * Creates the command for the controller core to indicate whether a foot is in support or not.
    * 
    * @param robotSide refers to which foot the command is to be created for.
    * @param inSupport whether the foot should be used to support the robot weight or not.
    * @return the command for the controller core.
    */
   public PlaneContactStateCommand createPlaneContactStateCommand(RobotSide robotSide, boolean inSupport)
   {
      ContactablePlaneBody footContactableBody = robotWalkerFive.getFootContactableBody(robotSide);

      PlaneContactStateCommand planeContactStateCommand = new PlaneContactStateCommand();
      planeContactStateCommand.setContactingRigidBody(footContactableBody.getRigidBody());

      if (inSupport)
      {
         List<FramePoint2D> contactPoints2d = footContactableBody.getContactPoints2d();
         for (int i = 0; i < contactPoints2d.size(); i++)
         {
            FramePoint2D contactPoint = contactPoints2d.get(i);
            planeContactStateCommand.addPointInContact(contactPoint);
         }
      }
      planeContactStateCommand.setCoefficientOfFriction(0.8);
      planeContactStateCommand.setContactNormal(new FrameVector3D(WORLD_FRAME, 0.0, 0.0, 1.0));
      return planeContactStateCommand;
   }

   @Override
   public String getName()
   {
      return "RobotWalkerFiveController";
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public YoGraphicDefinition getYoGraphicDefinition()
   {
      return graphicsGroup;
   }

}
