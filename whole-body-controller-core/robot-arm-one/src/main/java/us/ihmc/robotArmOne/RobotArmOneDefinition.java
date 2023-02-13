package us.ihmc.robotArmOne;

import java.util.EnumMap;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotArmOne.SevenDoFArmParameters.SevenDoFArmJointEnum;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.OneDoFJointState;

/**
 * This the robot-arm we will use for the next few examples.
 * <p>
 * It is a simple robot arm with a fixed-base and 7 degrees of freedom.
 * </p>
 */
public class RobotArmOneDefinition extends RobotDefinition
{

   private static final String ROBOTARM = "robotArm";

   private final RevoluteJointDefinition shoulderYawJoint;
   private final RevoluteJointDefinition shoulderRollJoint;
   private final RevoluteJointDefinition shoulderPitchJoint;
   private final RevoluteJointDefinition elbowPitchJoint;
   private final RevoluteJointDefinition wristPitchJoint;
   private final RevoluteJointDefinition wristRollJoint;
   private final RevoluteJointDefinition wristYawJoint;

   private final EnumMap<SevenDoFArmJointEnum, RevoluteJointDefinition> jointMap = new EnumMap<>(SevenDoFArmJointEnum.class);

   public RobotArmOneDefinition()
   {
      super(ROBOTARM);

      // Create the top (fixed) link that serves as the base of the pendulum
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      setRootBodyDefinition(elevator);

      // Lets create all the joints and attach them in series
      shoulderYawJoint = createArmJoint(SevenDoFArmJointEnum.shoulderYaw, elevator);
      shoulderRollJoint = createArmJoint(SevenDoFArmJointEnum.shoulderRoll, SevenDoFArmJointEnum.shoulderYaw.getChildRigidBody());
      shoulderPitchJoint = createArmJoint(SevenDoFArmJointEnum.shoulderPitch, SevenDoFArmJointEnum.shoulderRoll.getChildRigidBody());
      elbowPitchJoint = createArmJoint(SevenDoFArmJointEnum.elbowPitch, SevenDoFArmJointEnum.shoulderPitch.getChildRigidBody());
      wristPitchJoint = createArmJoint(SevenDoFArmJointEnum.wristPitch, SevenDoFArmJointEnum.elbowPitch.getChildRigidBody());
      wristRollJoint = createArmJoint(SevenDoFArmJointEnum.wristRoll, SevenDoFArmJointEnum.wristPitch.getChildRigidBody());
      wristYawJoint = createArmJoint(SevenDoFArmJointEnum.wristYaw, SevenDoFArmJointEnum.wristRoll.getChildRigidBody());

      // Add joints to joint map
      jointMap.put(SevenDoFArmJointEnum.shoulderYaw, shoulderYawJoint);
      jointMap.put(SevenDoFArmJointEnum.shoulderRoll, shoulderRollJoint);
      jointMap.put(SevenDoFArmJointEnum.shoulderPitch, shoulderPitchJoint);
      jointMap.put(SevenDoFArmJointEnum.elbowPitch, elbowPitchJoint);
      jointMap.put(SevenDoFArmJointEnum.wristPitch, wristPitchJoint);
      jointMap.put(SevenDoFArmJointEnum.wristRoll, wristRollJoint);
      jointMap.put(SevenDoFArmJointEnum.wristYaw, wristYawJoint);
   }

   private RevoluteJointDefinition createArmJoint(SevenDoFArmJointEnum jointEnum, RigidBodyDefinition predecessor)
   {
      String jointName = jointEnum.getJointName();
      Vector3D jointOffset = jointEnum.getJointOffset();
      Vector3DReadOnly jointAxis = jointEnum.getJointAxis();

      RevoluteJointDefinition pinJoint = new RevoluteJointDefinition(jointName, jointOffset, jointAxis);
      pinJoint.setPositionLimits(jointEnum.getJointLowerLimit(), jointEnum.getJointUpperLimit());
      pinJoint.setDamping(10.0);
      pinJoint.setKpSoftLimitStop(50.0); 
      pinJoint.setKdSoftLimitStop(10.0);
      RigidBodyDefinition linkBody = jointEnum.getChildRigidBody();

      linkBody.setMass(jointEnum.getChildLinkMass());
      linkBody.setCenterOfMassOffset(jointEnum.getChildLinkCoM());
      linkBody.setMomentOfInertia(jointEnum.getChildLinkInertia());

      predecessor.getChildrenJoints().add(pinJoint);
      pinJoint.setSuccessor(linkBody);
      
      // Define the initial joint state
      pinJoint.setInitialJointState(new OneDoFJointState(0.0));

      return pinJoint;
   }

   public RevoluteJointDefinition getJoint(SevenDoFArmJointEnum jointEnum)
   {
      return jointMap.get(jointEnum);
   }
}
