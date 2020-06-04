package us.ihmc.robotArmOne;

import java.util.EnumMap;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotArmOne.SevenDoFArmParameters.SevenDoFArmJointEnum;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

/**
 * This the robot-arm we will use for the next few examples.
 * <p>
 * It is a simple robot arm with a fixed-base and 7 degrees of freedom.
 * </p>
 */
public class RobotArmOne extends Robot {
	private final PinJoint shoulderYawJoint;
	private final PinJoint shoulderRollJoint;
	private final PinJoint shoulderPitchJoint;
	private final PinJoint elbowPitchJoint;
	private final PinJoint wristPitchJoint;
	private final PinJoint wristRollJoint;
	private final PinJoint wristYawJoint;

	private final EnumMap<SevenDoFArmJointEnum, PinJoint> jointMap = new EnumMap<>(SevenDoFArmJointEnum.class);

	public RobotArmOne() {
		super("RobotArm");

		/*
		 * Let's first create all the joints, they are all PinJoints which is used to
		 * define a 1-DoF revolute joint.
		 */
		shoulderYawJoint = createArmJoint(SevenDoFArmJointEnum.shoulderYaw);
		shoulderRollJoint = createArmJoint(SevenDoFArmJointEnum.shoulderRoll);
		shoulderPitchJoint = createArmJoint(SevenDoFArmJointEnum.shoulderPitch);
		elbowPitchJoint = createArmJoint(SevenDoFArmJointEnum.elbowPitch);
		wristPitchJoint = createArmJoint(SevenDoFArmJointEnum.wristPitch);
		wristRollJoint = createArmJoint(SevenDoFArmJointEnum.wristRoll);
		wristYawJoint = createArmJoint(SevenDoFArmJointEnum.wristYaw);

		// We create and attach the links here.
		setupLinks();

		addRootJoint(shoulderYawJoint);
		shoulderYawJoint.addJoint(shoulderRollJoint);
		shoulderRollJoint.addJoint(shoulderPitchJoint);
		shoulderPitchJoint.addJoint(elbowPitchJoint);
		elbowPitchJoint.addJoint(wristPitchJoint);
		wristPitchJoint.addJoint(wristRollJoint);
		wristRollJoint.addJoint(wristYawJoint);

		jointMap.put(SevenDoFArmJointEnum.shoulderYaw, shoulderYawJoint);
		jointMap.put(SevenDoFArmJointEnum.shoulderRoll, shoulderRollJoint);
		jointMap.put(SevenDoFArmJointEnum.shoulderPitch, shoulderPitchJoint);
		jointMap.put(SevenDoFArmJointEnum.elbowPitch, elbowPitchJoint);
		jointMap.put(SevenDoFArmJointEnum.wristPitch, wristPitchJoint);
		jointMap.put(SevenDoFArmJointEnum.wristRoll, wristRollJoint);
		jointMap.put(SevenDoFArmJointEnum.wristYaw, wristYawJoint);
	}

	private PinJoint createArmJoint(SevenDoFArmJointEnum jointEnum) {
		String jointName = jointEnum.getJointName();
		Vector3D jointOffset = jointEnum.getJointOffset();
		Vector3DReadOnly jointAxis = jointEnum.getJointAxis();
		PinJoint pinJoint = new PinJoint(jointName, jointOffset, this, jointAxis);
		double lowerLimit = jointEnum.getJointLowerLimit();
		double upperLimit = jointEnum.getJointUpperLimit();
		pinJoint.setLimitStops(lowerLimit, upperLimit, 50.0, 10.0);
		return pinJoint;
	}

	private void setupLinks() {
		// Let's first create all the links.
		Link shoulderYawChildLink = createChildLink(SevenDoFArmJointEnum.shoulderYaw);
		Link shoulderRollChildLink = createChildLink(SevenDoFArmJointEnum.shoulderRoll);
		Link shoulderPitchChildLink = createChildLink(SevenDoFArmJointEnum.shoulderPitch);
		Link elbowPitchChildLink = createChildLink(SevenDoFArmJointEnum.elbowPitch);
		Link wristPitchChildLink = createChildLink(SevenDoFArmJointEnum.wristPitch);
		Link wristRollChildLink = createChildLink(SevenDoFArmJointEnum.wristRoll);
		Link wristYawChildLink = createChildLink(SevenDoFArmJointEnum.wristYaw);

		// Now we can attach the links to their respective parent joint.
		shoulderYawJoint.setLink(shoulderYawChildLink);
		shoulderRollJoint.setLink(shoulderRollChildLink);
		shoulderPitchJoint.setLink(shoulderPitchChildLink);
		elbowPitchJoint.setLink(elbowPitchChildLink);
		wristPitchJoint.setLink(wristPitchChildLink);
		wristRollJoint.setLink(wristRollChildLink);
		wristYawJoint.setLink(wristYawChildLink);
	}

	private Link createChildLink(SevenDoFArmJointEnum jointEnum) {
		Link childLink = new Link(jointEnum.getChildLinkName());
		childLink.setMass(jointEnum.getChildLinkMass());
		childLink.setComOffset(jointEnum.getChildLinkCoM());
		childLink.setMomentOfInertia(jointEnum.getChildLinkInertia());
		childLink.setLinkGraphics(jointEnum.getChildLinkGraphics());
		return childLink;
	}

	public PinJoint getJoint(SevenDoFArmJointEnum jointEnum) {
		return jointMap.get(jointEnum);
	}

	/**
	 * Retrieves the current joint position given its joint enum.
	 * 
	 * @param jointEnum the enum of the joint we need the position of.
	 * @return the joint current position.
	 */
	public double getCurrentJointPosition(SevenDoFArmJointEnum jointEnum) {
		return getJoint(jointEnum).getQ();
	}

	/**
	 * Retrieves the current joint velocity given its joint enum.
	 * 
	 * @param jointEnum the enum of the joint we need the velocity of.
	 * @return the joint current velocity.
	 */
	public double getCurrentJointVelocity(SevenDoFArmJointEnum jointEnum) {
		return getJoint(jointEnum).getQD();
	}

	/**
	 * Sets the desired effort for a joint given its corresponding enum.
	 * <p>
	 * The joints are assumed to be perfectly actuated, i.e. the desired effort is
	 * exactly applied on the joint.
	 * </p>
	 * 
	 * @param jointEnum     the enum of the joint we want to apply the desired
	 *                      effort on.
	 * @param desiredEffort the desired effort value.
	 */
	public void setDesiredJointEffort(SevenDoFArmJointEnum jointEnum, double desiredEffort) {
		getJoint(jointEnum).setTau(desiredEffort);
	}
}
