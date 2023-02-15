package us.ihmc.robotArmTwo;

import java.util.Collections;
import java.util.EnumMap;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.commonWalkingControlModules.controllerCore.FeedbackControllerTemplate;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotArmOne.SevenDoFArmParameters.SevenDoFArmJointEnum;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RobotArmTwoController implements Controller
{

   private static final double TWO_PI = 2.0 * Math.PI;
   /**
    * We use this registry to keep track of the controller variables which can then be viewed in
    * Simulation Construction Set.
    */
   private final YoRegistry registry = new YoRegistry("Controller");
   /**
    * Desired position for each joint. {@code YoDouble}s are used instead of simple {@code double} so
    * they can be monitored via the Simulation Construction Set.
    */
   private final EnumMap<SevenDoFArmJointEnum, YoDouble> desiredPositions = new EnumMap<>(SevenDoFArmJointEnum.class);
   /**
    * Desired velocity for each joint. {@code YoDouble}s are used instead of simple {@code double} so
    * they can be monitored via the Simulation Construction Set.
    */
   private final EnumMap<SevenDoFArmJointEnum, YoDouble> desiredVelocities = new EnumMap<>(SevenDoFArmJointEnum.class);

   /**
    * One set of gains that will be used for every joint.
    */
   private final YoPDGains gains = new YoPDGains("jointsGains", registry);

   /**
    * This is the IHMC whole-body controller core which is used at the core of the walking controller
    * on Atlas and Valkyrie.
    */
   private final WholeBodyControllerCore wholeBodyControllerCore;
   /**
    * The controller core can run 3 different frame work:
    * <ul>
    * <li>Inverse Dynamics: Given desired accelerations and contact states, the controller core
    * computes desired joint torques.
    * <li>Virtual Model Control: It is a generalization of the "Jacobian transpose" method to a
    * whole-body framework, the output is desired joint torques.
    * <li>Inverse Kinematics: Given desired velocities, the controller core can integrate the these
    * velocities to output both desired joint velocities and positions.
    * </p>
    */
   private final WholeBodyControllerCoreMode controllerCoreMode;
   private final ControllerInput controllerInput;

   private final MultiBodySystemBasics controllerRobot;
   private final CenterOfMassReferenceFrame centerOfMassFrame;
   private final EnumMap<SevenDoFArmJointEnum, OneDoFJointBasics> controllerJoints = new EnumMap<>(SevenDoFArmJointEnum.class);
   private final EnumMap<SevenDoFArmJointEnum, OneDoFJointStateBasics> controllerJointOutputs = new EnumMap<>(SevenDoFArmJointEnum.class);

   /**
    * @param robotArm               this is our simulated robot.
    * @param controlDT              the duration of a controller tick. In this example, it should be
    *                               equal to the simulation DT.
    * @param gravityZ               the magnitude of the gravitational acceleration. It should be the
    *                               same as the one used for the simulation.
    * @param controllerCoreMode     indicates the mode for this simulation.
    * @param yoGraphicsListRegistry in this example, we do not have use for this. In general, this
    *                               registry allows to create dynamic 3D graphics in the simulation.
    */
   public RobotArmTwoController(ControllerInput controllerInput,
                                ControllerOutput controllerOutput,
                                double controlDT,
                                double gravityZ,
                                WholeBodyControllerCoreMode controllerCoreMode,
                                YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.controllerInput = controllerInput;
      this.controllerCoreMode = controllerCoreMode;

      controllerRobot = MultiBodySystemBasics.toMultiBodySystemBasics(MultiBodySystemFactories.cloneMultiBodySystem(controllerInput.getInput().getRootBody(),
                                                                                                                    ReferenceFrame.getWorldFrame(),
                                                                                                                    ""));
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame(), controllerRobot.getRootBody());
      wholeBodyControllerCore = createControllerCore(controlDT, gravityZ, yoGraphicsListRegistry);

      /*
       * YoDoubles need to be created first with a given name that is to represent the variable in the
       * Simulation Construction Set, and the registry so the simulation can find them.
       */
      for (SevenDoFArmJointEnum jointEnum : SevenDoFArmJointEnum.values())
      {
         String jointNameCapitalized = StringUtils.capitalize(jointEnum.getJointName());
         desiredPositions.put(jointEnum, new YoDouble("desiredPosition" + jointNameCapitalized, registry));
         desiredVelocities.put(jointEnum, new YoDouble("desiredVelocity" + jointNameCapitalized, registry));

         controllerJoints.put(jointEnum, (OneDoFJointBasics) controllerRobot.findJoint(jointEnum.getJointName()));
         controllerJointOutputs.put(jointEnum, controllerOutput.getOneDoFJointOutput(jointEnum.getJointName()));
      }
   }

   private WholeBodyControllerCore createControllerCore(double controlDT, double gravityZ, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      // As our robot arm only has a fixed-base, we just set the floating joint to null indicating that there is none.
      FloatingJointBasics rootJoint = null;

      /*
       * The elevator is a massless, sizeless rigid-body fixed in world to which the first joint of the
       * robot is attached. The name comes from the use of this rigid-body to add the gravity effect to
       * the robot by making it accelerate like an elevator when it starts moving. However, this elevator
       * is always fixed in world with no velocity.
       */
      RigidBodyBasics elevator = controllerRobot.getRootBody();

      // These are all the joints of the robot arm.
      JointBasics[] jointsArray = MultiBodySystemTools.collectSubtreeJoints(elevator);

      // The same joint but casted as we know they are all one degree-of-freedom joints.
      OneDoFJointBasics[] controlledJoints = MultiBodySystemTools.filterJoints(jointsArray, OneDoFJointBasics.class);

      // This class contains basic optimization settings required for QP formulation.
      ControllerCoreOptimizationSettings controllerCoreOptimizationSettings = new RobotArmTwoOptimizationSettings();

      // This is the toolbox for the controller core with everything it needs to run properly.
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT,
                                                                            gravityZ,
                                                                            rootJoint,
                                                                            controlledJoints,
                                                                            centerOfMassFrame,
                                                                            controllerCoreOptimizationSettings,
                                                                            yoGraphicsListRegistry,
                                                                            registry);
      /*
       * We indicate in the following that we want to be able to run the 3 different control modes. As our
       * robot has only a fixed-base and supporting rigid-body, we just provide and empty list for the
       * contacting bodies.
       */
      toolbox.setupForInverseKinematicsSolver();
      toolbox.setupForInverseDynamicsSolver(Collections.emptyList());
      toolbox.setupForVirtualModelControlSolver(elevator, Collections.emptyList());

      /*
       * Upon instantiation, the controller core attempts to create all the YoVariables needed for debug
       * purpose and monitoring. For this purpose, a template of all the feedback commands that'll be
       * needed during this control session should be provided. In this example, we only are interested in
       * controlling the robot in jointspace, so we make a template that is about controlling every joint
       * individually.
       */
      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();
      JointspaceFeedbackControlCommand jointCommands = new JointspaceFeedbackControlCommand();

      for (OneDoFJointBasics joint : controlledJoints)
      {
         // No need to put actual data, only the joint being controlled is used at this stage.
         jointCommands.addJointCommand(joint);

      }
      allPossibleCommands.addCommand(jointCommands);

      // Finally we can create the controller core.
      return new WholeBodyControllerCore(toolbox, new FeedbackControllerTemplate(allPossibleCommands), registry);
   }

   @Override
   public void initialize()
   {
      /*
       * Initialize the gains. The first term is the proportional gain while the second is the damping
       * ratio. A value of 1.0 for the damping ratio will compute a derivative gain to achieve critical
       * damping.
       */
      gains.setPDGains(50.0, 1.0);
      // This is to indicate that we want to use damping ratio of directly setting the
      // derivative gain.
      gains.createDerivativeGainUpdater(true);
   }

   /**
    * This method is called by the simulation every simulation tick. This is where the control part is
    * to be implemented.
    * <p>
    * In this example, we will make the robot follow simple sine wave trajectories on the shoulder yaw
    * and elbow pitch joints.
    * </p>
    */
   @Override
   public void doControl()
   {
      // We update the configuration state of our inverse dynamics robot model from the latest state of the simulated robot.
      MultiBodySystemTools.copyJointsState(controllerInput.getInput().getAllJoints(), controllerRobot.getAllJoints(), JointStateType.CONFIGURATION);
      MultiBodySystemTools.copyJointsState(controllerInput.getInput().getAllJoints(), controllerRobot.getAllJoints(), JointStateType.VELOCITY);
      // Need to update frames
      controllerRobot.getRootBody().updateFramesRecursively();
      centerOfMassFrame.update();

      // Update the joint desired positions and velocities.
      updateDesireds();

      // We store the objective for each joint in this command that will be processed by the controller core.
      JointspaceFeedbackControlCommand jointCommands = new JointspaceFeedbackControlCommand();

      for (SevenDoFArmJointEnum jointEnum : SevenDoFArmJointEnum.values())
      {
         OneDoFJointBasics joint = controllerJoints.get(jointEnum);

         double desiredPosition = desiredPositions.get(jointEnum).getValue();
         double desiredVelocity = desiredVelocities.get(jointEnum).getValue();
         OneDoFJointFeedbackControlCommand jointCommand = jointCommands.addJointCommand(joint);

         // Configure the feedback controller according to the desired mode.
         switch (controllerCoreMode)
         {
            case INVERSE_DYNAMICS:
               /*
                * Note that for high trajectory tracking quality, you would want to define the desired
                * acceleration, which would be used as a feed-forward term in the controller.
                */
               jointCommand.setInverseDynamics(desiredPosition, desiredVelocity, 0.0);
               break;
            case INVERSE_KINEMATICS:
               jointCommand.setInverseKinematics(desiredPosition, desiredVelocity);
               break;
            case VIRTUAL_MODEL:
               /*
                * Note that for high trajectory tracking quality, you would want to define the desired
                * torque/force, which would be used as a feed-forward term in the controller.
                */
               jointCommand.setVirtualModelControl(desiredPosition, desiredVelocity, 0.0);
               break;
            default:
               throw new IllegalStateException("Unexpected mode: " + controllerCoreMode);
         }
      }

      jointCommands.setGains(gains);
      /*
       * This weight is used to prioritize this command in the optimization problem. As this command is
       * the only one in this example, the weight value is not very important.
       */
      jointCommands.setWeightForSolver(1.0);

      ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(controllerCoreMode);
      controllerCoreCommand.addFeedbackControlCommand(jointCommands);

      if (controllerCoreMode == WholeBodyControllerCoreMode.INVERSE_KINEMATICS)
      {
         InverseKinematicsOptimizationSettingsCommand invKinOptimizationSettingsCmd = new InverseKinematicsOptimizationSettingsCommand();
         invKinOptimizationSettingsCmd.setJointAccelerationWeight(0.0);
         controllerCoreCommand.addInverseKinematicsCommand(invKinOptimizationSettingsCmd);
      }

      // Submit all the objectives to be achieved to the controller core.
      // Magic happens here.
      wholeBodyControllerCore.compute(controllerCoreCommand);
      // Get the result for this control tick.
      JointDesiredOutputListReadOnly outputForLowLevelController = wholeBodyControllerCore.getOutputForLowLevelController();

      // Write the controller output to the simulated joints.
      for (SevenDoFArmJointEnum jointEnum : SevenDoFArmJointEnum.values())
      {
         OneDoFJointBasics joint = controllerJoints.get(jointEnum);
         JointDesiredOutputReadOnly jointDesiredOutput = outputForLowLevelController.getJointDesiredOutput(joint);

         if (controllerCoreMode != WholeBodyControllerCoreMode.INVERSE_KINEMATICS)
         { // The control mode is either Inverse Dynamics or Virtual Model Control, so the joint output is desired torque.
            controllerJointOutputs.get(jointEnum).setEffort(jointDesiredOutput.getDesiredTorque());
         }
         else
         { // The control mode is Inverse Kinematics, so the joint output is desired position and velocity.
            double desiredPosition = jointDesiredOutput.getDesiredPosition();
            double desiredVelocity = jointDesiredOutput.getDesiredVelocity();
            controllerJointOutputs.get(jointEnum).setConfiguration(desiredPosition);
            controllerJointOutputs.get(jointEnum).setVelocity(desiredVelocity);
            controllerJointOutputs.get(jointEnum).setAcceleration(0);
         }
      }
   }

   /**
    * The desired joint positions and velocities are computed here.
    */
   public void updateDesireds()
   {
      { // Making the shoulder yaw joint follow a sine wave trajectory:
         double frequency = 0.2;
         double phase = Math.PI;
         double amplitude = 0.5;
         double omega = TWO_PI * frequency;
         double q = amplitude * Math.sin(omega * controllerInput.getTime() + phase);
         double qDot = omega * amplitude * Math.cos(omega * controllerInput.getTime() + phase);

         desiredPositions.get(SevenDoFArmJointEnum.shoulderYaw).set(q);
         desiredVelocities.get(SevenDoFArmJointEnum.shoulderYaw).set(qDot);
      }

      { // Making the elbow pitch joint follow a sine wave trajectory:
         double offset = 0.5 * Math.PI;
         double frequency = 0.2;
         double phase = -0.5 * Math.PI;
         double amplitude = 0.5;
         double omega = TWO_PI * frequency;
         double q = offset + amplitude * Math.sin(omega * controllerInput.getTime() + phase);
         double qDot = omega * amplitude * Math.cos(omega * controllerInput.getTime() + phase);

         desiredPositions.get(SevenDoFArmJointEnum.elbowPitch).set(q);
         desiredVelocities.get(SevenDoFArmJointEnum.elbowPitch).set(qDot);
      }
   }

   @Override
   public String getName()
   {
      return "RobotArmTwoController";
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
