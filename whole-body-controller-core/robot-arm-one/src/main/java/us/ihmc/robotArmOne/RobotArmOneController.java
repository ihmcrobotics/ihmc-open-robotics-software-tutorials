package us.ihmc.robotArmOne;

import java.util.EnumMap;

import org.apache.commons.lang3.StringUtils;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.robotArmOne.SevenDoFArmParameters.SevenDoFArmJointEnum;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RobotArmOneController implements Controller
{
   // A name for this controller
   private final String name = "robotArmController";

   private final EnumMap<SevenDoFArmJointEnum, OneDoFJointReadOnly> robotJoints = new EnumMap<>(SevenDoFArmJointEnum.class);
//   private OneDoFJointReadOnly shoulderYawJoint;
//   private OneDoFJointReadOnly shoulderRollJoint;
//   private OneDoFJointReadOnly shoulderPitchJoint;
//   private OneDoFJointReadOnly elbowPitchJoint;
//   private OneDoFJointReadOnly wristPitchJoint;
//   private OneDoFJointReadOnly wristRollJoint;
//   private OneDoFJointReadOnly wristYawJoint;

   private ControllerInput controllerInput;

   private ControllerOutput controllerOutput;

   private static final double TWO_PI = 2.0 * Math.PI;

//   /**
//    * Current position for each joint. {@code YoDouble}s are used instead of simple {@code double} so
//    * they can be monitored via the Simulation Construction Set.
//    */
//   private final EnumMap<SevenDoFArmJointEnum, YoDouble> currentPositions = new EnumMap<>(SevenDoFArmJointEnum.class);
//
//   /**
//    * Current velocities for each joint. {@code YoDouble}s are used instead of simple {@code double} so
//    * they can be monitored via the Simulation Construction Set.
//    */
//   private final EnumMap<SevenDoFArmJointEnum, YoDouble> currentVelocities = new EnumMap<>(SevenDoFArmJointEnum.class);

   /**
    * We use this registry to keep track of the controller variables which can then be viewed in
    * Simulation Construction Set.
    */
   private final YoRegistry registry = new YoRegistry("Controller");
   /**
    * Proportional gain for each joint. {@code YoDouble}s are used instead of simple {@code double} so
    * they can be viewed and modified via the Simulation Construction Set.
    */
   private final EnumMap<SevenDoFArmJointEnum, YoDouble> kps = new EnumMap<>(SevenDoFArmJointEnum.class);
   /**
    * Derivative gain for each joint. {@code YoDouble}s are used instead of simple {@code double} so
    * they can be viewed and modified via the Simulation Construction Set.
    */
   private final EnumMap<SevenDoFArmJointEnum, YoDouble> kds = new EnumMap<>(SevenDoFArmJointEnum.class);
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
    * The error in position for the tracking of each joint. {@code YoDouble}s are used instead of
    * simple {@code double} so they can be monitored via the Simulation Construction Set.
    */
   private final EnumMap<SevenDoFArmJointEnum, YoDouble> positionErrors = new EnumMap<>(SevenDoFArmJointEnum.class);
   /**
    * The error in velocity for the tracking of each joint. {@code YoDouble}s are used instead of
    * simple {@code double} so they can be monitored via the Simulation Construction Set.
    */
   private final EnumMap<SevenDoFArmJointEnum, YoDouble> velocityErrors = new EnumMap<>(SevenDoFArmJointEnum.class);

   public RobotArmOneController(ControllerInput controllerInput, ControllerOutput controllerOutput)
   {

      this.controllerInput = controllerInput;
      this.controllerOutput = controllerOutput;

      // Get the objects of the fulcrum joint to read input and write output in the control loop
//      this.shoulderYawJoint = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(SevenDoFArmJointEnum.shoulderYaw.getJointName());
//      this.shoulderRollJoint = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(SevenDoFArmJointEnum.shoulderRoll.getJointName());
//      this.shoulderPitchJoint = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(SevenDoFArmJointEnum.shoulderPitch.getJointName());
//      this.elbowPitchJoint = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(SevenDoFArmJointEnum.elbowPitch.getJointName());
//      this.wristYawJoint = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(SevenDoFArmJointEnum.wristYaw.getJointName());
//      this.wristRollJoint = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(SevenDoFArmJointEnum.wristRoll.getJointName());
//      this.wristPitchJoint = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(SevenDoFArmJointEnum.wristPitch.getJointName());

      /*
       * YoDoubles need to be created first with a given name that is to represent the variable in the
       * Simulation Construction Set, and the registry so the simulation can find them.
       */
      for (SevenDoFArmJointEnum jointEnum : SevenDoFArmJointEnum.values())
      {
         String jointName = StringUtils.capitalize(jointEnum.getJointName());
         kps.put(jointEnum, new YoDouble("kp" + jointName, registry));
         kds.put(jointEnum, new YoDouble("kd" + jointName, registry));
         desiredPositions.put(jointEnum, new YoDouble("desiredPosition" + jointName, registry));
         desiredVelocities.put(jointEnum, new YoDouble("desiredVelocity" + jointName, registry));

         positionErrors.put(jointEnum, new YoDouble("positionError" + jointName, registry));
         velocityErrors.put(jointEnum, new YoDouble("velocityError" + jointName, registry));
         
         robotJoints.put(jointEnum, (OneDoFJointReadOnly) controllerInput.getInput().findJoint(SevenDoFArmJointEnum.wristPitch.getJointName()));
//         currentPositions.put(jointEnum, new YoDouble("currentPosition" + jointName, registry));
//         currentVelocities.put(jointEnum, new YoDouble("currentVelocities" + jointName, registry));
      }
   }

   /**
    * This method allows to manage the initialization part needed to get the controller ready to go. It
    * is NOT called by default by the simulation, it has to be called at the desired location in the
    * configuration of the robot and control environment.
    */
   @Override
   public void initialize()
   {
      // Initializing the gain values.
      kps.get(SevenDoFArmJointEnum.shoulderYaw).set(50.0);
      kps.get(SevenDoFArmJointEnum.shoulderRoll).set(50.0);
      kps.get(SevenDoFArmJointEnum.shoulderPitch).set(50.0);
      kps.get(SevenDoFArmJointEnum.elbowPitch).set(50.0);
      kps.get(SevenDoFArmJointEnum.wristPitch).set(50.0);
      kps.get(SevenDoFArmJointEnum.wristRoll).set(20.0);
      kps.get(SevenDoFArmJointEnum.wristYaw).set(20.0);

      kds.get(SevenDoFArmJointEnum.shoulderYaw).set(5.0);
      kds.get(SevenDoFArmJointEnum.shoulderRoll).set(5.0);
      kds.get(SevenDoFArmJointEnum.shoulderPitch).set(5.0);
      kds.get(SevenDoFArmJointEnum.elbowPitch).set(5.0);
      kds.get(SevenDoFArmJointEnum.wristPitch).set(5.0);
      kds.get(SevenDoFArmJointEnum.wristRoll).set(2.0);
      kds.get(SevenDoFArmJointEnum.wristYaw).set(2.0);
   }

   /**
    * This method is called by the simulation every simulation tick. This is where the control part is
    * to be implemented.
    */
   @Override
   public void doControl()
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

//      {// Update current joint position and velocity for control 
//         currentPositions.get(SevenDoFArmJointEnum.shoulderYaw).set(shoulderYawJoint.getQ());
//         currentPositions.get(SevenDoFArmJointEnum.shoulderRoll).set(shoulderRollJoint.getQ());
//         currentPositions.get(SevenDoFArmJointEnum.shoulderPitch).set(shoulderPitchJoint.getQ());
//         currentPositions.get(SevenDoFArmJointEnum.elbowPitch).set(elbowPitchJoint.getQ());
//         currentPositions.get(SevenDoFArmJointEnum.wristYaw).set(wristYawJoint.getQ());
//         currentPositions.get(SevenDoFArmJointEnum.wristRoll).set(wristRollJoint.getQ());
//         currentPositions.get(SevenDoFArmJointEnum.wristPitch).set(wristPitchJoint.getQ());
//
//         currentVelocities.get(SevenDoFArmJointEnum.shoulderYaw).set(shoulderYawJoint.getQd());
//         currentVelocities.get(SevenDoFArmJointEnum.shoulderRoll).set(shoulderRollJoint.getQd());
//         currentVelocities.get(SevenDoFArmJointEnum.shoulderPitch).set(shoulderPitchJoint.getQd());
//         currentVelocities.get(SevenDoFArmJointEnum.elbowPitch).set(elbowPitchJoint.getQd());
//         currentVelocities.get(SevenDoFArmJointEnum.wristYaw).set(wristYawJoint.getQd());
//         currentVelocities.get(SevenDoFArmJointEnum.wristRoll).set(wristRollJoint.getQd());
//         currentVelocities.get(SevenDoFArmJointEnum.wristPitch).set(wristPitchJoint.getQd());
//      }

      // In the following, we perform the feedback control using simple PD
      // controllers.
      for (SevenDoFArmJointEnum jointEnum : SevenDoFArmJointEnum.values())
      {
         OneDoFJointReadOnly joint = robotJoints.get(jointEnum);
         // calculate desired effort based on current joint state
         double qError = desiredPositions.get(jointEnum).getValue() - joint.getQ();
         double qErrorDot = desiredVelocities.get(jointEnum).getValue() - joint.getQd();
         double desiredEffort = kps.get(jointEnum).getValue() * qError + kds.get(jointEnum).getValue() * qErrorDot;

         positionErrors.get(jointEnum).set(qError);
         velocityErrors.get(jointEnum).set(qErrorDot);

         // Write desired effort as controller output
         this.controllerOutput.getOneDoFJointOutput(jointEnum.getJointName()).setEffort(desiredEffort);
      }

   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

}
