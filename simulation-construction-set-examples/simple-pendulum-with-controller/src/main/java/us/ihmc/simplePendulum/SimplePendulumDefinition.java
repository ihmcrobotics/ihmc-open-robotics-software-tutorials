package us.ihmc.simplePendulum;

import java.util.stream.Stream;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.controller.implementations.ControllerCollectionDefinition;
import us.ihmc.scs2.definition.controller.implementations.OneDoFJointDampingControllerDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.JointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimplePendulumDefinition extends RobotDefinition {
	private static final String PENDULUM = "pendulum";

	// Define the parameters of the robot
	public static final double ROD_LENGTH = 1.0;
	public static final double ROD_RADIUS = 0.01;
	public static final double ROD_MASS = 0.00;

	public static final double TIP_RADIUS = 0.02;

	public static final double FULCRUM_RADIUS = 0.02;

	public static final double BALL_RADIUS = 0.05;
	public static final double BALL_MASS = 1.0;

	// I = mr^2 pendulum's resistance changes to its rotation in kg.m^2
	public static final double FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y = ROD_LENGTH * ROD_LENGTH * BALL_MASS;

	// Initial state of the pendulum
	private double fulcrumInitialPositionDegrees = 45.0;
	private double fulcrumInitialPositionRadians = fulcrumInitialPositionDegrees * Math.PI / 180.0;
	private double fulcrumInitialVelocity = 0.0;

	private static final double DAMP1 = 0.0;

	private final OneDoFJointDampingControllerDefinition jointLvl1DampingControllerDefinition = new OneDoFJointDampingControllerDefinition();

	/*
	 * private final OneDoFJointDampingControllerDefinition
	 * jointLvl1DampingControllerDefinition = new
	 * OneDoFJointDampingControllerDefinition();
	 * 
	 * Some joint state variables. Allows SimplePendulumRobot to have access to and
	 * set fulcrum joint properties.
	 */
	private YoDouble tau_fulcrum, q_fulcrum, qd_fulcrum; // Respectively Torque, Position, Velocity

	// Define its constructor
	public SimplePendulumDefinition() {
		// Call parent class "RobotDefinition" constructor. The string PENDULUM will be
		// the name
		// of the robot.
		super(PENDULUM); // creates an instance of the class RobotDefinition named "pendulum" in the SCS2
							// system.

		// implement controllers with 3 different damping levels
		jointLvl1DampingControllerDefinition.setControllerName("DampLevel1").createDampingVariable("damp1", DAMP1);
		
	
		
		// Creates and adds a joint to the robot

		// Create the top (fixed) link that serves as the base of the pendulum
		RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
		setRootBodyDefinition(elevator);

		/*
		 * The first parameter "FulcrumPin" is the name of the joint and will be used in
		 * all the variables associated with the joint. The second parameter
		 * "new Vector3d(0.0, 0.0, 1.5)" defines the offset of this joint from the
		 * previous joint. Since we want to position the fulcrum of the pendulum at a
		 * height of 1.5 meters above the ground, the default vector (0.0, 0.0, 1.5)
		 * will be used. The parameter "Axis3D.Y" defines the axis of rotation for this
		 * pin joint.
		 */
		OneDoFJointDefinition fulcrumPinJoint = new RevoluteJointDefinition("FulcrumPin", new Vector3D(0.0, 0.0, 1.5),
				Axis3D.Y);
//		fulcrumPinJoint.setPredecessor(elevator);
		elevator.getChildrenJoints().add(fulcrumPinJoint);

		/*
		 * Sets initial state of the pin joint. The pendulum will start is course from a
		 * horizontal position with no initial speed.
		 */
		fulcrumPinJoint.setInitialJointState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);

		jointLvl1DampingControllerDefinition
		.addJointsToControl(Stream.of(fulcrumPinJoint).map(JointDefinition::getName).toArray(String[]::new));

		
		createPendulumLink("pendulumLink", fulcrumPinJoint, BALL_MASS, ROD_LENGTH, ROD_RADIUS, 0.0,
				FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y, 0.0);

		// Adds some damping to force the pendulum ball to converge faster to equilibrium position.
		addControllerDefinition(new ControllerCollectionDefinition().setControllerName("pendulumController")
				.addControllerOutputReset().addControllerDefinition(jointLvl1DampingControllerDefinition));
	}

	/**
	 * Retrieves and returns the fulcrum's angular position {@code YoDouble} in
	 * radians
	 *
	 * @return the angular position {@code YoDouble} in radians
	 */
	public double getFulcrumAngularPosition() {
		return q_fulcrum.getDoubleValue();
	}

	/**
	 * Retrieves and returns the fulcrum's angular velocity {@code YoDouble} in
	 * radians per seconds
	 *
	 * @return the angular velocity {@code YoDouble} in radians per seconds
	 */
	public double getFulcrumAngularVelocity() {
		return qd_fulcrum.getDoubleValue();
	}

	/**
	 * Retrieves and returns the fulcrum's torque {@code YoDouble} in Newton meter
	 *
	 * @return the torque {@code YoDouble} in Newton meter
	 */
	public double getFulcrumTorque() {
		return tau_fulcrum.getDoubleValue();
	}

	/**
	 * Sets and returns the fulcrum's torque {@code YoDouble} in Newton meter
	 *
	 * @return the torque {@code YoDouble} in Newton meter
	 */
	public void setFulcrumTorque(double tau) {
		this.tau_fulcrum.set(tau);
	}

	/**
	 * Create the link for the pendulum robot.
	 */
	private RigidBodyDefinition createPendulumLink(String name, JointDefinition parentJoint, double mass, double length,
			double radius, double Ixx, double Iyy, double Izz) {

		// Generate pendulum Link object
		RigidBodyDefinition pendulumLink = new RigidBodyDefinition(name);

		// Sets link's physical properties
		/*
		 * Sets the moment of inertia about the X,Y,Z axis. Note that the moment of
		 * inertia is defined about the center of mass, so if set to zero, the link will
		 * be a point mass.
		 */
		pendulumLink.getMomentOfInertia().setToDiagonal(Ixx, Iyy, Izz);
		// sets the mass of the link
		pendulumLink.setMass(mass);
		// Sets center of mass offset to be located at tip of rod
		pendulumLink.setCenterOfMassOffset(0.0, 0.0, -length);
		parentJoint.setSuccessor(pendulumLink);

		// Set up the material properties and 3D shapes
		MaterialDefinition redMaterial = new MaterialDefinition(ColorDefinitions.Red());
		MaterialDefinition blackMaterial = new MaterialDefinition(ColorDefinitions.Black());
		MaterialDefinition greenMaterial = new MaterialDefinition(ColorDefinitions.Chartreuse());
		GeometryDefinition sphere1 = new Sphere3DDefinition(TIP_RADIUS);
		GeometryDefinition sphere2 = new Sphere3DDefinition(BALL_RADIUS);
		GeometryDefinition cylinder = new Cylinder3DDefinition(length, radius);

		// Set up the position offsets for the different elements
		RigidBodyTransform linkPose = new RigidBodyTransform(new AxisAngle(), new Vector3D(0.0, 0.0, -0.5 * length));
		RigidBodyTransform linkTip = new RigidBodyTransform(new AxisAngle(), new Vector3D(0.0, 0.0, -length));

		// create the different visual elements
		pendulumLink.addVisualDefinition(new VisualDefinition(sphere1, redMaterial));
		pendulumLink.addVisualDefinition(new VisualDefinition(linkPose, cylinder, blackMaterial));
		pendulumLink.addVisualDefinition(new VisualDefinition(linkTip, sphere2, greenMaterial));


		return pendulumLink;
	}

}
