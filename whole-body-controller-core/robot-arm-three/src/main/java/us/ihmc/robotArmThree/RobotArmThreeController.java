package us.ihmc.robotArmThree;

import java.util.Collections;

import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotArmOne.SevenDoFArmParameters.SevenDoFArmJointEnum;
import us.ihmc.robotArmTwo.RobotArmTwo;
import us.ihmc.robotArmTwo.RobotArmTwoOptimizationSettings;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFramePoseUsingYawPitchRoll;
import us.ihmc.yoVariables.variable.YoFrameQuaternion;
import us.ihmc.yoVariables.variable.YoFrameVector3D;
import us.ihmc.yoVariables.variable.YoInteger;

public class RobotArmThreeController implements RobotController
{
   private static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   private static final double TWO_PI = 2.0 * Math.PI;
   /**
    * We use this registry to keep track of the controller variables which can then be viewed in
    * Simulation Construction Set.
    */
   private final YoVariableRegistry registry = new YoVariableRegistry("Controller");
   /**
    * This variable stores the current simulation time and is updated by the simulation.
    */
   private final YoDouble time;

   /**
    * The desired position for the end-effector that can be monitored in the Simulation Construction
    * Set.
    */
   private final YoFramePoint3D desiredEndEffectorPosition = new YoFramePoint3D("desiredEndEffector", WORLD_FRAME, registry);
   /**
    * The desired orientation for the end-effector that can be monitored in the Simulation Construction
    * Set.
    */
   private final YoFrameQuaternion desiredEndEffectorOrientation = new YoFrameQuaternion("desiredEndEffector", WORLD_FRAME, registry);
   /**
    * The desired linear velocity for the end-effector that can be monitored in the Simulation
    * Construction Set.
    */
   private final YoFrameVector3D desiredEndEffectorLinearVelocity = new YoFrameVector3D("desiredEndEffectorLinearVelocity", WORLD_FRAME, registry);
   /**
    * The desired angular velocity for the end-effector that can be monitored in the Simulation
    * Construction Set.
    */
   private final YoFrameVector3D desiredEndEffectorAngularVelocity = new YoFrameVector3D("desiredEndEffectorAngularVelocity", WORLD_FRAME, registry);

   //inserted
   private final YoFrameVector3D feedForwardLinearAcceleration = new YoFrameVector3D("feedForwardLinearAcceleration", WORLD_FRAME, registry);

   /**
    * This time we define "SE3" which allows us to specify gains for each degree of freedom (DoF).
    * <p>
    * In this example, we use {@code GainCoupling.XYZ} which specify that we are going to use the same
    * gains for the 3 position DoFs, and the 3 orientation DoFs.
    * </p>
    */
   private final DefaultYoPIDSE3Gains gains = new DefaultYoPIDSE3Gains("jointsGains", new PIDSE3Configuration(GainCoupling.XYZ, false), registry);

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
   private final RobotArmTwo robotArm;
   /**
    * In this controller, we add some visualization to see the end-effector desired trajectory in the
    * Simulation Construction Set. In addition to have an interesting name, a {@code BagOfBalls} allows
    * to easily dynamically display 3D spheres given a position.
    */
   private final BagOfBalls trajectoryPositionVisualization;
   /**
    * We use this field to control the rate at which a new sphere is to be displayed.
    */
   private final YoInteger numberOfControlTicksPerVizUpdate = new YoInteger("numberOfControlTicksPerVizUpdate", registry);
   private final YoFramePoseUsingYawPitchRoll controlFramePoseForVisualization = new YoFramePoseUsingYawPitchRoll("controlFrame", WORLD_FRAME, registry);

   /**
    * @param robotArm               this is our simulated robot.
    * @param controlDT              the duration of a controller tick. In this example, it should be
    *                               equal to the simulation DT.
    * @param gravityZ               the magnitude of the gravitational acceleration. It should be the
    *                               same as the one used for the simulation.
    * @param controllerCoreMode     indicates the mode for this simulation.
    * @param yoGraphicsListRegistry in this example, we use this registry to enable the
    *                               {@link #trajectoryPositionVisualization}.
    */
   public RobotArmThreeController(RobotArmTwo robotArm, double controlDT, double gravityZ, WholeBodyControllerCoreMode controllerCoreMode,
                                  YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.robotArm = robotArm;
      time = robotArm.getYoTime();
      this.controllerCoreMode = controllerCoreMode;

      wholeBodyControllerCore = createControllerCore(controlDT, gravityZ, yoGraphicsListRegistry);

      // This defines how many balls we can display at once.
      int numberOfBalls = 300;
      // This defines the radius for each ball.
      double ballSize = 0.005;
      trajectoryPositionVisualization = new BagOfBalls(numberOfBalls, ballSize, YoAppearance.Yellow(), registry, yoGraphicsListRegistry);
      numberOfControlTicksPerVizUpdate.set(300);

      /*
       * We create here a graphical coordinate system so we can visualize where the control frame is in
       * the simulation.
       */
      YoGraphicCoordinateSystem controlFrameGraphic = new YoGraphicCoordinateSystem("controlFrameFrame",
                                                                                    controlFramePoseForVisualization,
                                                                                    0.2,
                                                                                    YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("control", controlFrameGraphic);
   }

   private WholeBodyControllerCore createControllerCore(double controlDT, double gravityZ, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      // The following steps are the same as in RobotArmTwoController. 
      FloatingJointBasics rootJoint = null;
      RigidBodyBasics elevator = robotArm.getElevator();
      //InverseDynamicsJoint[] inverseDynamicsJoints = ScrewTools.computeSubtreeJoints(elevator);
      JointBasics[] jointsArray = MultiBodySystemTools.collectSubtreeJoints(elevator);
      OneDoFJoint[] controlledJoints = MultiBodySystemTools.filterJoints(jointsArray, OneDoFJoint.class);

      ControllerCoreOptimizationSettings controllerCoreOptimizationSettings = new RobotArmTwoOptimizationSettings();

      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controlDT,
                                                                            gravityZ,
                                                                            rootJoint,
                                                                            controlledJoints,
                                                                            robotArm.getCenterOfMassFrame(),
                                                                            controllerCoreOptimizationSettings,
                                                                            yoGraphicsListRegistry,
                                                                            registry);

      toolbox.setupForInverseKinematicsSolver();
      toolbox.setupForInverseDynamicsSolver(Collections.emptyList());
      toolbox.setupForVirtualModelControlSolver(elevator, Collections.emptyList());

      /*
       * This time, we will control the robot in taskspace by directly commanding the end-effector pose in
       * space. We will be using a SpatialFeedbackControlCommand for this purpose.
       */
      RigidBodyBasics endEffector = robotArm.getEndEffector();
      FeedbackControlCommandList allPossibleCommands = new FeedbackControlCommandList();
      SpatialFeedbackControlCommand command = new SpatialFeedbackControlCommand();
      command.set(elevator, endEffector); // No need to add any additional information for now.
      allPossibleCommands.addCommand(command);

      /*
       * In addition to switching to taskspace control, we will also use a feature of the controller core
       * called the privileged configuration. When enabled, the controller core will attempt to keep the
       * robot as close as possible to preferred configuration without disturbing the other objectives.
       * This is useful for instance to use the robot redundancy to keep its joints in the middle of their
       * range of motion.
       */
      toolbox.setJointPrivilegedConfigurationParameters(new JointPrivilegedConfigurationParameters());

      // Finally we can create the controller core.
      return new WholeBodyControllerCore(toolbox, allPossibleCommands, registry);
   }

   @Override
   public void initialize()
   {
      /*
       * Initialize the gains. These are rather arbitrary values giving a decent tracking.
       */
      if (controllerCoreMode != WholeBodyControllerCoreMode.VIRTUAL_MODEL)
      {
         gains.setPositionProportionalGains(50.0);
         gains.setPositionDerivativeGains(10.0);
      }
      else
      { // In Virtual Control Mode, there is no gravity compensation, so the gains have to be much higher.
         gains.setPositionProportionalGains(1000.0);
         gains.setPositionDerivativeGains(100.0);
      }
      gains.setOrientationProportionalGains(50.0);
      gains.setOrientationDerivativeGains(10.0);
   }

   /**
    * This method is called by the simulation every simulation tick. This is where the control part is
    * to be implemented.
    * <p>
    * In this example, we will make the robot's end-effector follow a 3D trajectory composed of sine
    * wave trajectories, see {@link #updateDesireds()}.
    * </p>
    */
   @Override
   public void doControl()
   {
      // We update the configuration state of our inverse dynamics robot model from the latest state of the simulated robot.
      robotArm.updateInverseDynamicsRobotState();
      // Update the joint desired positions and velocities.
      updateDesireds();

      // We store the objective for each joint in this command that will be processed by the controller core.
      SpatialFeedbackControlCommand command = new SpatialFeedbackControlCommand();
      /*
       * We indicate that to achieve this command the controller core is to use the set of joints located
       * between the elevator and the end-effector.
       */
      RigidBodyBasics elevator = robotArm.getElevator();
      RigidBodyBasics endEffector = robotArm.getEndEffector();
      command.set(elevator, endEffector);
      // Then we provide the current desireds for the end-effector.
      command.setInverseDynamics(desiredEndEffectorPosition, desiredEndEffectorLinearVelocity, feedForwardLinearAcceleration);
      command.setInverseDynamics(desiredEndEffectorOrientation, desiredEndEffectorAngularVelocity, feedForwardLinearAcceleration);
      // Update the gains
      command.setGains(gains);
      /*
       * This weight is used to prioritize this command in the optimization problem. As this command is
       * the only one in this example, the weight value is not very important.
       */
      command.setWeightForSolver(1.0);
      /*
       * This time, we also have the option indicate which degrees of freedom are to be controlled. In
       * this example, we control all the degrees of freedom.
       */
      SelectionMatrix6D selectionMatrix6D = new SelectionMatrix6D();
      selectionMatrix6D.setLinearAxisSelection(true, true, true);
      selectionMatrix6D.setAngularAxisSelection(true, true, true);
      command.setSelectionMatrix(selectionMatrix6D);

      /*
       * Here we define exactly what part of the end-effector is meant to track our trajectory. this is
       * done by defining the pose of what is called: the control frame. It is defined as an offset from
       * the end-effector's body-fixed frame located at its center of mass.
       */
      FramePose3D controlFramePose = new FramePose3D(endEffector.getBodyFixedFrame());
      controlFramePose.setZ(0.1); // Let's offset the control frame to be located at the tip of the end-effector.
      command.setControlFrameFixedInEndEffector(controlFramePose);
      // Let's update the following variable which in turn will cause the graphical coordinate system to be updated in the simulation.
      controlFramePoseForVisualization.setMatchingFrame(controlFramePose);

      ControllerCoreCommand controllerCoreCommand = new ControllerCoreCommand(controllerCoreMode);
      controllerCoreCommand.addFeedbackControlCommand(command);

      /*
       * We create a PrivilegedConfigurationCommand to indicate that this feature is desired and that we
       * want to use it to bring the joints towards the middle of their range of motion using the robot
       * redundancy.
       */
      PrivilegedConfigurationCommand privilegedCommand = new PrivilegedConfigurationCommand();
      privilegedCommand.setPrivilegedConfigurationOption(PrivilegedConfigurationOption.AT_MID_RANGE);
      controllerCoreCommand.addInverseDynamicsCommand(privilegedCommand);

      // Submit all the objectives to be achieved to the controller core.
      wholeBodyControllerCore.submitControllerCoreCommand(controllerCoreCommand);
      // Magic happens here.
      wholeBodyControllerCore.compute();
      // Get the result for this control tick.
      JointDesiredOutputListReadOnly outputForLowLevelController = wholeBodyControllerCore.getOutputForLowLevelController();

      // Write the controller output to the simulated joints.
      for (SevenDoFArmJointEnum jointEnum : SevenDoFArmJointEnum.values())
      {
         OneDoFJointBasics joint = robotArm.getJoint(jointEnum);
         JointDesiredOutputReadOnly jointDesiredOutput = outputForLowLevelController.getJointDesiredOutput(joint);

         if (controllerCoreMode != WholeBodyControllerCoreMode.INVERSE_KINEMATICS)
         { // The control mode is either Inverse Dynamics or Virtual Model Control, so the joint output is desired torque.
            robotArm.setDesiredEffort(jointEnum, jointDesiredOutput.getDesiredTorque());
         }
         else
         { // The control mode is Inverse Kinematics, so the joint output is desired position and velocity.
            double desiredPosition = jointDesiredOutput.getDesiredPosition();
            double desiredVelocity = jointDesiredOutput.getDesiredVelocity();
            robotArm.setSimulatedJointConfiguration(jointEnum, desiredPosition, desiredVelocity);
         }
      }
   }

   private int tickCount = 0;

   /**
    * The desired end-effector pose and velocity are computed here.
    */
   public void updateDesireds()
   {
      { // Let's make a 3D position trajectory using a composition of sine-waves:
         Vector3D frequencies = new Vector3D(0.1, 0.1, 0.2);
         Vector3D amplitudes = new Vector3D(0.3, 0.1, 0.075);
         Vector3D offsets = new Vector3D(0.0, 0.0, 0.55);
         Vector3D phases = new Vector3D(0.5 * Math.PI, 0.0, -0.5 * Math.PI);

         for (int axisIndex = 0; axisIndex < 3; axisIndex++)
         {
            double amplitude = amplitudes.getElement(axisIndex);
            double omega = TWO_PI * frequencies.getElement(axisIndex);
            double phase = phases.getElement(axisIndex);
            double offset = offsets.getElement(axisIndex);

            double x = offset + amplitude * Math.sin(omega * time.getValue() + phase);
            double xDot = omega * amplitude * Math.cos(omega * time.getValue() + phase);

            desiredEndEffectorPosition.setElement(axisIndex, x);
            desiredEndEffectorLinearVelocity.setElement(axisIndex, xDot);
            feedForwardLinearAcceleration.setElement(axisIndex, 0.0); //set acceleration to 0 in every axis

         }
      }

      { // Let's make some 3D orientation trajectory
        // We use arrays of 3 elements here to represent in order the rotations: yaw (z-axis), pitch (y-axis), and roll (x-axis).
         double[] frequencies = {0.0, 0.1, 0.0};
         double[] amplitudes = {0.0, Math.toRadians(45.0), 0.0};
         double[] offsets = {0.0, 0.0, 0.0};
         double[] phases = {0.0, -0.5 * Math.PI, 0.0};

         double[] yawPitchRoll = new double[3];
         double[] yawPitchRollRates = new double[3];

         for (int rotationIndex = 0; rotationIndex < 3; rotationIndex++)
         {
            double amplitude = amplitudes[rotationIndex];
            double omega = TWO_PI * frequencies[rotationIndex];
            double phase = phases[rotationIndex];
            double offset = offsets[rotationIndex];

            yawPitchRoll[rotationIndex] = offset + amplitude * Math.sin(omega * time.getValue() + phase);
            yawPitchRollRates[rotationIndex] = omega * amplitude * Math.cos(omega * time.getValue() + phase);
         }

         //desiredEndEffectorOrientation.setYawPitchRoll(yawPitchRoll);
         desiredEndEffectorOrientation.setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
         // We use the following tool to compute the 3D angular velocity vector corresponding to the rates of the yaw, pitch, and roll angles.
         RotationTools.computeAngularVelocityInWorldFrameFromYawPitchRollAnglesRate(yawPitchRoll, yawPitchRollRates, desiredEndEffectorAngularVelocity);
      }

      tickCount++;

      if (tickCount >= numberOfControlTicksPerVizUpdate.getValue())
      {
         tickCount = 0;
         trajectoryPositionVisualization.setBallLoop(desiredEndEffectorPosition, YoAppearance.Yellow());
      }
   }

   @Override
   public String getName()
   {
      return "RobotArmTwoController";
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getDescription()
   {
      return "Controller demonstrating a taskspace control using the IHMC whole-body controller core.";
   }
}
