package us.ihmc.simplePendulum;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

/**
 *
 * In this tutorial, lengths are expressed in meters (m), masses in kilograms
 * (kg)
 *
 */
// By extending Robot, SimplePendulumRobot is inheriting some properties and methods from the Robot class
public class SimplePendulumRobot extends Robot {
	/*
	 * Define the parameters of the robot
	 */
	public static final double ROD_LENGTH = 1.0;
	public static final double ROD_RADIUS = 0.01;
	public static final double ROD_MASS = 0.00;

	public static final double FULCRUM_RADIUS = 0.02;

	public static final double BALL_RADIUS = 0.05;
	public static final double BALL_MASS = 1.0;

	// I = mr^2 pendulum's resistance changes to its rotation in kg.m^2
	public static final double FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y = ROD_LENGTH * ROD_LENGTH * BALL_MASS;

	/*
	 * Initial state of the pendulum
	 */
	private double fulcrumInitialPositionDegrees = 90.0;
	private double fulcrumInitialPositionRadians = fulcrumInitialPositionDegrees * Math.PI / 180.0;
	private double fulcrumInitialVelocity = 0.0;

	/*
	 * Some joint state variables. Allows SimplePendulumRobot to have access to and
	 * set fulcrum joint properties
	 */
	private YoDouble tau_fulcrum, q_fulcrum, qd_fulcrum; // Respectively Torque, Position, Velocity

	/*
	 * Define its constructor
	 */
	public SimplePendulumRobot() {
		/*
		 * Call parent class "Robot" constructor. The string "pendulum" will be the name
		 * of the robot.
		 */
		super("pendulum"); // creates an instance of the class Robot named "pendulum" in the SCS system.

		/* Creates and adds a joint to the robot */
		/*
		 * The first parameter "FulcrumPin" is the name of the joint and will be used in
		 * all the variables associated with the joint. The second parameter "new
		 * Vector3d(0.0, 0.0, 1.5)" defines the offset of this joint from the previous
		 * joint. Since we want to position the fulcrum of the pendulum at a height of
		 * 1.5 meters above the ground, the default vector (0.0, 0.0, 1.5) will be used.
		 * The third parameter "this" refers to the robot itself. The final parameter
		 * "Axis3D.Y" defines the axis of rotation for this pin joint.
		 */
		PinJoint fulcrumPinJoint = new PinJoint("FulcrumPin", new Vector3D(0.0, 0.0, 1.5), this, Axis3D.Y);

		/* Set some properties for this joint and attaches a Link to this joint */
		/*
		 * Sets initial state of the pin joint. The pendulum will start is course from a
		 * horizontal position with no initial speed.
		 */
		fulcrumPinJoint.setInitialState(fulcrumInitialPositionRadians, fulcrumInitialVelocity);
		/*
		 * Sets the link created previously as the link for this joint (pendulumLink()
		 * method defined next).
		 */
		fulcrumPinJoint.setLink(pendulumLink());
		/*
		 * Adds some damping to force the pendulum ball to converge faster to
		 * equilibrium position.
		 */
		fulcrumPinJoint.setDamping(0.3);

		// Gets position, velocity, and torque of the fulcrum joint
		q_fulcrum = fulcrumPinJoint.getQYoVariable();
		qd_fulcrum = fulcrumPinJoint.getQDYoVariable();
		tau_fulcrum = fulcrumPinJoint.getTauYoVariable();

		/* Add a RootJoint to the robot */
		/*
		 * This line adds fulcrumPinJoint as a rootJoint of the robot. In order to be
		 * part of a robot, a joint must either be added as a root joint, or be attached
		 * to a parent joint. This ensures the tree structure (forest structure if there
		 * are multiple root joints) of the robot.
		 */
		this.addRootJoint(fulcrumPinJoint);
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
	 * Create the first link for the DoublePendulumRobot.
	 * 
	 * @return the pendulum link {@code Link} created.
	 */
	private Link pendulumLink() {
		// creates a new link named "PendulumLink"
		Link pendulumLink = new Link("PendulumLink");

		/* Sets link's physical properties */
		/*
		 * Sets the moment of inertia about the Y axis. Note that the moment of inertia
		 * is defined about the center of mass, so if set to zero, the link will be a
		 * point mass.
		 */
		pendulumLink.setMomentOfInertia(0.0, FULCRUM_MOMENT_OF_INERTIA_ABOUT_Y, 0.0);
		// sets the mass of the link to 1 kg
		pendulumLink.setMass(BALL_MASS);
		/*
		 * Sets center of mass offset to be located at tip of rod, which is 1.0m
		 * opposite the pivot joint.
		 */
		pendulumLink.setComOffset(0.0, 0.0, -ROD_LENGTH);

		/* Use to visually represent links in the SCS 3D view */
		Graphics3DObject pendulumGraphics = new Graphics3DObject();

		/* Add the pivot */
		/*
		 * Adds a purple sphere 3D object to pendulumGraphics. Since no transformation
		 * has been applied to this graphic component, the sphere's position is (0.0,
		 * 0.0, 0.0). This is used to represent the pivot/fulcrum of the pendulum.
		 */
		pendulumGraphics.addSphere(FULCRUM_RADIUS, YoAppearance.BlueViolet());

		/* Add the rod */
		/*
		 * Translates from the current position by the specified distances. Graphic
		 * components added after translation will appear in the new location. The
		 * coordinate system for these translations is based on those that preceded it.
		 */
		pendulumGraphics.translate(0.0, 0.0, -ROD_LENGTH);
		// Adds 1m long black cylinder representing the rod.
		pendulumGraphics.addCylinder(ROD_LENGTH, ROD_RADIUS, YoAppearance.Black());

		/* Add the ball */
		// Adds yellow sphere representing the ball at the end of the rod.
		pendulumGraphics.addSphere(BALL_RADIUS, YoAppearance.Chartreuse());

		/* Attach the Graphics3DObject to its link */
		/*
		 * Associates our pendulumGraphics object with the pendulumLink object and in
		 * doing so, translates and rotates the graphic components to be in the same
		 * frame of reference as the pendulumLink.
		 */
		pendulumLink.setLinkGraphics(pendulumGraphics);

		return pendulumLink;
	}

}
