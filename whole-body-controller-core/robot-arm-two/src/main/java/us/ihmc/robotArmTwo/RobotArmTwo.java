package us.ihmc.robotArmTwo;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotArmOne.RobotArmOne;
import us.ihmc.robotArmOne.SevenDoFArmParameters.SevenDoFArmJointEnum;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.sensorProcessing.simulatedSensors.InverseDynamicsJointsFromSCSRobotGenerator;
import us.ihmc.sensorProcessing.simulatedSensors.SCSToInverseDynamicsJointMap;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This is physically the same robot as {@link RobotArmOne}, it includes an additional
 * representation: "inverse dynamics robot model" that is used by the IHMC whole-body controller
 * core.
 */
public class RobotArmTwo
{
   private static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   /**
    * We will simulate the same robot arm as in the first example.
    */
   private final RobotArmOne simulatedRobotArm;

   // The following is needed to create the IHMC whole-body controller core.

   /**
    * A generator tool that can convert a simulated robot into what we often call an "inverse dynamics
    * robot". This inverse dynamics represents the same robot but offers a different set of features.
    * This type of robot is used by the controller core.
    */
   private final InverseDynamicsJointsFromSCSRobotGenerator inverseDynamicsRobot;
   /**
    * A mapping to easily retrieve joints in the inverse dynamics robot given a joint from the
    * simulated robot.
    */
   private final SCSToInverseDynamicsJointMap jointMap;
   /**
    * A reference frame which origin is at the center of mass of the robot. It is not used besides for
    * creating the controller core in this example.
    */
   private final ReferenceFrame centerOfMassFrame;
   /**
    * The array containing in order from the base to the end-effector the joints our robot.
    */
   private final OneDoFJoint[] controlledJoints;

   public RobotArmTwo()
   {
      simulatedRobotArm = new RobotArmOne();

      inverseDynamicsRobot = new InverseDynamicsJointsFromSCSRobotGenerator(simulatedRobotArm); // elevator is rigid
      // body
      jointMap = inverseDynamicsRobot.getSCSToInverseDynamicsJointMap();
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", WORLD_FRAME, inverseDynamicsRobot.getElevator());

      // These are all the joints of the robot arm.
      JointBasics[] jointsArray = MultiBodySystemTools.collectSubtreeJoints(getElevator());

      // The same joint but casted as we know they are all one degree-of-freedom
      // joints.
      controlledJoints = MultiBodySystemTools.filterJoints(jointsArray, OneDoFJoint.class);
   }

   /**
    * Updates the state of the inverse dynamics robot model based on the state of the simulated robot.
    */
   public void updateInverseDynamicsRobotState()
   {
      inverseDynamicsRobot.updateInverseDynamicsRobotModelFromRobot(true);
      centerOfMassFrame.update();
   }

   /**
    * Gets the variable holding the current simulation time.
    * 
    * @return the yo-time.
    */
   public YoDouble getYoTime()
   {
      return simulatedRobotArm.getYoTime();
   }

   /**
    * Gets the robot that will be used for the simulation.
    * 
    * @return the simulated robot.
    */
   public Robot getSimulatedRobot()
   {
      return simulatedRobotArm;
   }

   /**
    * Gets the root body of this robot arm.
    * <p>
    * The elevator is a massless, sizeless rigid-body fixed in world to which the first joint of the
    * robot is attached. The name comes from the use of this rigid-body to add the gravity effect to
    * the robot by making it accelerate like an elevator when it starts moving. However, this elevator
    * is always fixed in world with no velocity.
    * </p>
    * 
    * @return the elevator.
    */

   public RigidBodyBasics getElevator()
   {
      return inverseDynamicsRobot.getElevator();
   }

   /**
    * Retrieves the last rigid-body of the robot arm.
    * 
    * @return the end-effector.
    */
   public RigidBodyBasics getEndEffector()
   {
      return controlledJoints[controlledJoints.length - 1].getSuccessor();
   }

   /**
    * The joints to use for control.
    * 
    * @return the controlled joints.
    */
   public OneDoFJointBasics[] getControlledJoints() // also changed here
   {
      return controlledJoints;
   }

   /**
    * Gets the reference frame which is origin is located at the robot's center of mass.
    * <p>
    * Its axes are aligned with the world frame.
    * </p>
    * 
    * @return the center of mass reference frame.
    */
   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   /**
    * Retrieves a joint given its corresponding enum.
    * <p>
    * The joint returned here is an "inverse dynamics joint" which is used by the controller core.
    * </p>
    * 
    * @param jointEnum the corresponding joint enum.
    * @return the inverse dynamics joint.
    */
   public OneDoFJointBasics getJoint(SevenDoFArmJointEnum jointEnum)
   {
      return jointMap.getInverseDynamicsOneDoFJoint(simulatedRobotArm.getJoint(jointEnum));
   }

   /**
    * Sets the desired effort for a joint given its corresponding enum.
    * <p>
    * The joints are assumed to be perfectly actuated, i.e. the desired effort is exactly applied on
    * the joint.
    * </p>
    * 
    * @param jointEnum     the enum of the joint we want to apply the desired effort on.
    * @param desiredEffort the desired effort value.
    */
   public void setDesiredEffort(SevenDoFArmJointEnum jointEnum, double desiredEffort)
   {
      simulatedRobotArm.setDesiredJointEffort(jointEnum, desiredEffort);
   }

   /**
    * Sets the configuration of a joint of the simulated robot.
    * <p>
    * This is useful when visualizing the result of an inverse kinematics controller which output
    * desired joint configurations
    * </p>
    * 
    * @param jointEnum the enum of the joint we want to set the configuration of.
    * @param position  the joint new position.
    * @param velocity  the joint new velocity.
    */
   public void setSimulatedJointConfiguration(SevenDoFArmJointEnum jointEnum, double position, double velocity)
   {
      PinJoint joint = simulatedRobotArm.getJoint(jointEnum);
      joint.setQ(position);
      joint.setQd(velocity);
   }
}
