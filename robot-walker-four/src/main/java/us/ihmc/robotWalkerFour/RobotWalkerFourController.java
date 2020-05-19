package us.ihmc.robotWalkerFour;

import java.util.Arrays;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.robotics.math.trajectories.ParabolicPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.QuinticPolynomialTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

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
public class RobotWalkerFourController implements RobotController
{
   private static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   private static final double CENTER_OF_MASS_HEIGHT = 0.75;
   /**
    * We use this registry to keep track of the controller variables which can then be viewed in
    * Simulation Construction Set.
    */
   private final YoVariableRegistry registry = new YoVariableRegistry("Controller");
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
    * This is the IHMC whole-body controller core which is used at the core of the walking
    * controller on Atlas and Valkyrie.
    */
   private final WholeBodyControllerCore wholeBodyControllerCore;

   /**
    * We define as an enum the list of possible states that the state machine will be allowed to go
    * through.
    */
   private enum WalkingStateEnum
   {
      /**
       * The simulation starts with the robot standing. It will switch to the initial transfer only
       * when the variable {@code walk} is set to {@code true}.
       */
      STANDING,
      /**
       * This is the double support phase that moves the center of mass from the right foot to the
       * left foot.
       */
      TRANSFER_TO_LEFT,
      /**
       * This is the double support phase that moves the center of mass from the left foot to the
       * right foot.
       */
      TRANSFER_TO_RIGHT,
      /**
       * This is the single support phase during which the center of mass stays still while the
       * right is swinging to the desired footstep location.
       */
      LEFT_SUPPORT,
      /**
       * This is the single support phase during which the center of mass stays still while the left
       * is swinging to the desired footstep location.
       */
      RIGHT_SUPPORT
   };

   /**
    * The finite state machine to which we register a set of specialized controllers and a set of
    * transitions to go from one controller to another.
    */
   private final StateMachine<WalkingStateEnum, State> stateMachine;
   /**
    * This variable stores the current simulation time and is updated by the simulation.
    */
   private final YoDouble time;
   private final RobotWalkerFour robotWalkerFour;

   /**
    * The elevator is a massless, sizeless rigid-body fixed in world to which the first joint of the
    * robot is attached. The name comes from the use of this rigid-body to add the gravity effect to
    * the robot by making it accelerate like an elevator when it starts moving. However, this
    * elevator is always fixed in world with no velocity.
    */
   private final RigidBody elevator;
   /**
    * We will use a single instance of the controller core command for convenience. Note that in
    * this example, only the Inverse Dynamics control mode will be demonstrated.
    */
   private final ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(WholeBodyControllerCoreMode.INVERSE_DYNAMICS);

   /**
    * We will define only one set of gains for controlling the whole robot. This is not ideal but is
    * enough for this example.
    */
   private final DefaultYoPIDSE3Gains gains = new DefaultYoPIDSE3Gains("gains", GainCoupling.XYZ, false, registry);

   public RobotWalkerFourController(RobotWalkerFour robotWalkerFour, double controlDT, double gravityZ, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robotWalkerFour = robotWalkerFour;
      time = robotWalkerFour.getYoTime();
      elevator = robotWalkerFour.getElevator();
      wholeBodyControllerCore = createControllerCore(controlDT, gravityZ, yoGraphicsListRegistry);
      stateMachine = createStateMachine();

      transferDuration.set(1.3);
      swingDuration.set(1.0);
      stepLength.set(0.15);
   }

   private WholeBodyControllerCore createControllerCore(double controlDT, double gravityZ, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      // This time the robot has a floating joint.
      FloatingInverseDynamicsJoint rootJoint = robotWalkerFour.getRootJoint();
      // These are all the joints of the robot arm.
      InverseDynamicsJoint[] inverseDynamicsJoints = ScrewTools.computeSubtreeJoints(elevator);
      // This class contains basic optimization settings required for QP formulation.
      ControllerCoreOptimizationSettings controllerCoreOptimizationSettings = new RobotWalkerFourOptimizationSettings();
      // This is the toolbox for the controller core with everything it needs to run properly.
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT, gravityZ, rootJoint, inverseDynamicsJoints,
                                                                            robotWalkerFour.getCenterOfMassFrame(), controllerCoreOptimizationSettings,
                                                                            yoGraphicsListRegistry, registry);
      // The controller core needs all the possibly contacting bodies of the robot to create all the modules needed for later.
      toolbox.setupForInverseDynamicsSolver(Arrays.asList(robotWalkerFour.getFootContactableBody(RobotSide.LEFT),
                                                          robotWalkerFour.getFootContactableBody(RobotSide.RIGHT)));

      /*
       * Finally, we register all the commands that we will use in this controller, i.e. commands
       * for the feet that we'll for the swing, an orientation command for the pelvis to keep it
       * level to the ground, and the command for controlling the center of mass.
       */
      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();
      for (RobotSide robotSide : RobotSide.values)
      {
         SpatialFeedbackControlCommand footCommand = new SpatialFeedbackControlCommand();
         RigidBody foot = robotWalkerFour.getFoot(robotSide);
         footCommand.set(elevator, foot);
         allPossibleCommands.addCommand(footCommand);
      }

      OrientationFeedbackControlCommand pelvisOrientationCommand = new OrientationFeedbackControlCommand();
      pelvisOrientationCommand.set(elevator, robotWalkerFour.getPelvis());
      allPossibleCommands.addCommand(pelvisOrientationCommand);

      CenterOfMassFeedbackControlCommand centerOfMassCommand = new CenterOfMassFeedbackControlCommand();
      allPossibleCommands.addCommand(centerOfMassCommand);

      // Finally we can create the controller core.
      return new WholeBodyControllerCore(toolbox, allPossibleCommands, registry);
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
      factory.buildYoClock(time);

      /*
       * Then we get to the point where actually create the state and the transitions. In this
       * example, we will only use what is called here "done transitions". These transitions get
       * triggered as a state reports that it is done, when triggered the state machine goes to the
       * next state.
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
      gains.setPositionProportionalGains(100.0);
      gains.setPositionDerivativeGains(10.0);
      gains.setOrientationProportionalGains(100.0);
      gains.setOrientationDerivativeGains(10.0);
   }

   /**
    * This time, the {@code doControl} method is rather empty as most of the controller is actually
    * implemented in the different states declared further down.
    */
   @Override
   public void doControl()
   {
      // We update the configuration state of our inverse dynamics robot model from the latest state of the simulated robot.
      robotWalkerFour.updateInverseDynamicsRobotState();

      /*
       * Here we request the state machine to call the doAction() method of the active state and to
       * check if the transition to the next state should be engaged. See further below for the
       * implementation of the doAction() method for each state.
       */
      stateMachine.doActionAndTransition();

      /*
       * As the pelvis is to kept level to the ground independently to the active state, we can
       * simply setup the command here.
       */
      OrientationFeedbackControlCommand pelvisOrientationCommand = new OrientationFeedbackControlCommand();
      pelvisOrientationCommand.set(elevator, robotWalkerFour.getPelvis());
      pelvisOrientationCommand.setGains(gains.getOrientationGains());
      pelvisOrientationCommand.setWeightForSolver(1.0);
      controllerCoreCommand.addFeedbackControlCommand(pelvisOrientationCommand);

      // As before, we submit the commands to the controller core.
      wholeBodyControllerCore.submitControllerCoreCommand(controllerCoreCommand);
      // Let the magic happen
      wholeBodyControllerCore.compute();
      // And collect the output to update the simulated robot.
      JointDesiredOutputListReadOnly outputForLowLevelController = wholeBodyControllerCore.getOutputForLowLevelController();

      for (int i = 0; i < outputForLowLevelController.getNumberOfJointsWithDesiredOutput(); i++)
      {
         OneDoFJoint oneDoFJoint = outputForLowLevelController.getOneDoFJoint(i);
         JointDesiredOutputReadOnly jointDesiredOutput = outputForLowLevelController.getJointDesiredOutput(i);
         robotWalkerFour.setDesiredEffort(oneDoFJoint, jointDesiredOutput.getDesiredTorque());
      }
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
         // Here we get the position of both feet to compute the middle.
         FramePoint3D leftSolePosition = new FramePoint3D(robotWalkerFour.getSoleFrame(RobotSide.LEFT));
         leftSolePosition.changeFrame(WORLD_FRAME);
         FramePoint3D rightSolePosition = new FramePoint3D(robotWalkerFour.getSoleFrame(RobotSide.RIGHT));
         rightSolePosition.changeFrame(WORLD_FRAME);
         FramePoint3D centerOfMassPosition = new FramePoint3D(WORLD_FRAME);
         // The desired center of mass position is set to be right in between the feet.
         centerOfMassPosition.interpolate(leftSolePosition, rightSolePosition, 0.5);
         // We set the desired height.
         centerOfMassPosition.setZ(CENTER_OF_MASS_HEIGHT);

         // So now, we just have pack the command for the controller core.
         CenterOfMassFeedbackControlCommand centerOfMassCommand = new CenterOfMassFeedbackControlCommand();
         centerOfMassCommand.set(centerOfMassPosition);
         centerOfMassCommand.setGains(gains.getPositionGains());
         centerOfMassCommand.setWeightForSolver(1.0);
         controllerCoreCommand.addFeedbackControlCommand(centerOfMassCommand);

         // Now it is the turn of the feet.
         for (RobotSide robotSide : RobotSide.values)
         {
            // As their are in support, they should not be accelerating. So we make a zero-acceleration command.
            SpatialAccelerationCommand footZeroAcceleration = createFootZeroAccelerationCommand(robotSide);
            controllerCoreCommand.addInverseDynamicsCommand(footZeroAcceleration);
            // This is the command that we can use to request a contactable body to be used for support or not.
            PlaneContactStateCommand planeContactStateCommand = createPlaneContactStateCommand(robotSide, true);
            controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);
         }
      }

      @Override
      public void onExit()
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         /*
          * As soon as the walk variable is set to true via the simulation GUI, the state machine
          * will leave this state causing the robot to start walking.
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
      private final FramePoint3D initialCenterOfMassPosition = new FramePoint3D();
      private final FramePoint3D finalCenterOfMassPosition = new FramePoint3D();
      /**
       * We use a 5th order polynomial to smooth out the velocity of the center of mass at the start
       * and end of this state. This allows the robot to walk slightly faster without falling.
       */
      private final QuinticPolynomialTrajectoryGenerator centerOfMassTrajectory;

      public TransferState(RobotSide transferToSide)
      {
         this.transferToSide = transferToSide;
         // The trajectory is setup such that it will always from 0.0 to 1.0 within the given transferDuration.
         DoubleProvider initialPositionProvider = () -> 0.0;
         DoubleProvider initialVelocityProvider = () -> 0.0;
         DoubleProvider finalPositionProvider = () -> 1.0;
         DoubleProvider finalVelocityProvider = () -> 0.0;
         centerOfMassTrajectory = new QuinticPolynomialTrajectoryGenerator(transferToSide.getCamelCaseName() + "CenterOfMassTrajectory",
                                                                           initialPositionProvider, initialVelocityProvider, finalPositionProvider,
                                                                           finalVelocityProvider, transferDuration, registry);
      }

      @Override
      public void onEntry()
      {
         /*
          * This method is called once when this state becomes active. this is the perfect time to
          * initialize the endpoints for the center of mass. It should start from wherever it is at
          * the start of this state and end above the leading foot.
          */
         initialCenterOfMassPosition.setToZero(robotWalkerFour.getCenterOfMassFrame());
         initialCenterOfMassPosition.changeFrame(WORLD_FRAME);
         finalCenterOfMassPosition.setToZero(robotWalkerFour.getSoleFrame(transferToSide));
         finalCenterOfMassPosition.changeFrame(WORLD_FRAME);
         centerOfMassTrajectory.initialize();
      }

      @Override
      public void doAction(double timeInState)
      {
         // The trajectory generator is updated to get the current progression percentage alpha which is in [0, 1].
         centerOfMassTrajectory.compute(timeInState);
         double alpha = centerOfMassTrajectory.getValue();

         // The alpha parameter is now used to interpolate between the initial and final center of mass positions.
         FramePoint3D centerOfMassPosition = new FramePoint3D(WORLD_FRAME);
         centerOfMassPosition.interpolate(initialCenterOfMassPosition, finalCenterOfMassPosition, alpha);
         centerOfMassPosition.setZ(CENTER_OF_MASS_HEIGHT);

         // And now we pack the command for the controller core.
         CenterOfMassFeedbackControlCommand centerOfMassCommand = new CenterOfMassFeedbackControlCommand();
         centerOfMassCommand.set(centerOfMassPosition);
         centerOfMassCommand.setGains(gains.getPositionGains());
         centerOfMassCommand.setWeightForSolver(1.0);
         controllerCoreCommand.addFeedbackControlCommand(centerOfMassCommand);

         // as for the standing state, we request both feet to be in support.
         for (RobotSide robotSide : RobotSide.values)
         {
            SpatialAccelerationCommand footZeroAcceleration = createFootZeroAccelerationCommand(robotSide);
            controllerCoreCommand.addInverseDynamicsCommand(footZeroAcceleration);

            PlaneContactStateCommand planeContactStateCommand = createPlaneContactStateCommand(robotSide, true);
            controllerCoreCommand.addInverseDynamicsCommand(planeContactStateCommand);
         }
      }

      @Override
      public void onExit()
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
    * During the single support state, only one foot is in support while the other is swinging to
    * the next footstep. The center of mass stays still.
    */
   private class SingleSupportState implements State
   {
      private final RobotSide supportSide;
      private final RobotSide swingSide;
      /** We will use a trajectory generator for the swing. */
      private final ParabolicPositionTrajectoryGenerator swingPositionTrajectory;
      /** YoVariable for the next footstep position so we can monitor it during the simulation. */
      private final YoFramePoint3D footstepPosition;
      private final FramePose3D swingControlFramePose = new FramePose3D();

      public SingleSupportState(RobotSide supportSide)
      {
         this.supportSide = supportSide;
         swingSide = supportSide.getOppositeSide();
         footstepPosition = new YoFramePoint3D(swingSide.getCamelCaseName() + "Footstep", WORLD_FRAME, registry);

         double groundClearance = 0.10;
         // The trajectory generator such that the swing starts from the current foot position and ends at the given footstepPosition.
         PositionProvider initialPositionProvider = initialPosition -> initialPosition.setToZero(robotWalkerFour.getSoleFrame(swingSide));
         PositionProvider finalPositionProvider = finalPosition -> finalPosition.setIncludingFrame(footstepPosition);
         swingPositionTrajectory = new ParabolicPositionTrajectoryGenerator(swingSide + "Swing", WORLD_FRAME, swingDuration, initialPositionProvider,
                                                                            finalPositionProvider, groundClearance, registry);
         // Here we define the control frame pose that is needed to specify the point on the foot we want to control to the controller core.
         swingControlFramePose.setToZero(robotWalkerFour.getSoleFrame(swingSide));
         swingControlFramePose.changeFrame(robotWalkerFour.getFoot(swingSide).getBodyFixedFrame());
      }

      @Override
      public void onEntry()
      {
         // Here we compute the position for the next footstep.
         FramePoint3D supportFootPosition = new FramePoint3D(robotWalkerFour.getSoleFrame(supportSide));
         supportFootPosition.changeFrame(WORLD_FRAME);
         footstepPosition.setFromReferenceFrame(robotWalkerFour.getSoleFrame(swingSide));
         footstepPosition.setX(supportFootPosition.getX() + stepLength.getValue());
         swingPositionTrajectory.initialize();
      }

      @Override
      public void doAction(double timeInState)
      {
         // During this state, the center of mass is kept right above the support foot.
         FramePoint3D centerOfMassPosition = new FramePoint3D(robotWalkerFour.getSoleFrame(supportSide));
         centerOfMassPosition.changeFrame(WORLD_FRAME);
         centerOfMassPosition.setZ(CENTER_OF_MASS_HEIGHT);

         // We pack the center of mass command for the controller core.
         CenterOfMassFeedbackControlCommand centerOfMassCommand = new CenterOfMassFeedbackControlCommand();
         centerOfMassCommand.set(centerOfMassPosition);
         centerOfMassCommand.setGains(gains.getPositionGains());
         centerOfMassCommand.setWeightForSolver(1.0);
         controllerCoreCommand.addFeedbackControlCommand(centerOfMassCommand);

         // As in the standing state, the support is specified with the contact state command and zero acceleration command.
         SpatialAccelerationCommand footZeroAcceleration = createFootZeroAccelerationCommand(supportSide);
         controllerCoreCommand.addInverseDynamicsCommand(footZeroAcceleration);
         controllerCoreCommand.addInverseDynamicsCommand(createPlaneContactStateCommand(supportSide, true));

         // We need to specify that the swing foot is not in contact anymore.
         controllerCoreCommand.addInverseDynamicsCommand(createPlaneContactStateCommand(swingSide, false));

         /*
          * Using the swing trajectory generator, we can compute the current desired position,
          * velocity and acceleration for the swing foot.
          */
         FramePoint3D position = new FramePoint3D();
         FrameVector3D velocity = new FrameVector3D();
         FrameVector3D acceleration = new FrameVector3D();
         swingPositionTrajectory.compute(timeInState);
         swingPositionTrajectory.getLinearData(position, velocity, acceleration);

         // Finally, we pack the swing foot command for the controller core.
         SpatialFeedbackControlCommand swingFootCommand = new SpatialFeedbackControlCommand();
         swingFootCommand.set(elevator, robotWalkerFour.getFoot(swingSide));
         swingFootCommand.set(position, velocity);
         swingFootCommand.setControlFrameFixedInEndEffector(swingControlFramePose);
         swingFootCommand.setFeedForwardLinearAction(acceleration);
         swingFootCommand.setGains(gains);
         swingFootCommand.setWeightForSolver(1.0);
         controllerCoreCommand.addFeedbackControlCommand(swingFootCommand);
      }

      @Override
      public void onExit()
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         // As soon as the time reaches the given swingDuration, the state machine will enter the next transfer state.
         return timeInState >= swingDuration.getValue();
      }
   }

   /**
    * Creates a spatial acceleration command to request a zero acceleration of the right or left
    * foot.
    * 
    * @param robotSide refers to which foot the command is to be created for.
    * @return the zero acceleration command.
    */
   public SpatialAccelerationCommand createFootZeroAccelerationCommand(RobotSide robotSide)
   {
      SpatialAccelerationCommand footZeroAcceleration = new SpatialAccelerationCommand();
      footZeroAcceleration.set(elevator, robotWalkerFour.getFoot(robotSide));
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
      ContactablePlaneBody footContactableBody = robotWalkerFour.getFootContactableBody(robotSide);

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
      return "RobotWalkerFourController";
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getDescription()
   {
      return "Controller demonstrating a quasi-static walking controller using the IHMC whole-body controller core.";
   }
}
