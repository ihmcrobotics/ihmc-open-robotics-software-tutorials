package us.ihmc.fallingBrick;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.simulation.parameters.ContactParameters;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;
import us.ihmc.scs2.simulation.physicsEngine.PhysicsEngineFactory;

public class FallingBrickSimulation
{
   /**
    * <ul>
    * <li>When {@code true}, the simulation is created using the impulse base physics which handles
    * shape-to-shape collisions. Main inconvenient of physics engine is that it is less stable. Main
    * advantage is that it resolves contact as hard constraints without requiring much parameter
    * tuning.
    * <li>When {@code false}, the simulation is created using the contact point based physics which
    * only handles point-to-shape contact. Main inconvenients of this physics engine are the required
    * tuning step of the contact gains and the softness of the contacts. Main advantage is its
    * stability.
    * </ul>
    */
   private final static boolean USE_IMPULSE_PHYSICS = false;

   public FallingBrickSimulation()
   {
      // Create an instance of the falling brick
      FallingBrickDefinition fallingBrick = new FallingBrickDefinition();

      // Set the initial positions, velocities, and accelerations of the brick
      SixDoFJointState initialJointState = new SixDoFJointState();
      initialJointState.setConfiguration(new Pose3D(0.0, 0.0, 2.0, 0.0, 0.0, 0.0));
      initialJointState.setAngularVelocity(new Vector3D(-0.1, -1.0, 10.0));
      initialJointState.setLinearVelocity(new Vector3D(0.0, -0.1, 0.5));
      fallingBrick.getFloatingRootJointDefinition().setInitialJointState(initialJointState);

      // Initialize the physics engine factory which allows to specify which one to use as well as to specify its parameters
      PhysicsEngineFactory physicsEngineFactory;

      if (USE_IMPULSE_PHYSICS)
      {
         // Setting up the impulse based physics engine
         // It relies on forward dynamics to compute the robot joint accelerations and includes robot <-> environment
         // interactions that uses the robot physical properties to compute contact impulses such that the contacts
         // behave as hard constraints.
         // Note that this physics engine only uses the collision shapes attached to the robot together with ones defined for the environment.
         ContactParameters contactParameters = new ContactParameters();
         contactParameters.setMinimumPenetration(5.0e-5);
         contactParameters.setCoefficientOfFriction(0.7);
         contactParameters.setCoefficientOfRestitution(0.0);
         contactParameters.setRestitutionThreshold(0.0);
         contactParameters.setErrorReductionParameter(0.0);
         contactParameters.setComputeFrictionMoment(true);
         contactParameters.setCoulombMomentFrictionRatio(0.3);
         physicsEngineFactory = SimulationConstructionSet2.impulseBasedPhysicsEngineFactory(contactParameters);
      }
      else
      {
         // Setting up the contact-point based physics engine.
         // It relies on forward dynamics to compute robot joint acceleration and includes robot <-> environment
         // interactions using a non-linear spring damping contact force calculator. As such, the contacts
         // behave as soft constraints, the maximum penetration will be function of the stiffness set below.
         // Note that the stiffness and damping are dependent to the robot mass and inertia properties.
         // Note that this physics engine only uses the ground contact points attached to the robot together with
         // the collision shapes defined for the environment.
         // Define ground contact parameters stiffness (k) and damping (b). These parameters are used for the spring-damper model of all surfaces in the simulation
         ContactPointBasedContactParameters contact = ContactPointBasedContactParameters.defaultParameters();
         contact.setKxy(40000.0); // Stiffness for computing the tangential force.
         contact.setBxy(100.0); // Damping for computing the tangential force.
         contact.setKz(500.0); // Stiffness for computing the normal force.
         contact.setBz(250.0); // Damping for computing the normal force.
         physicsEngineFactory = SimulationConstructionSet2.contactPointBasedPhysicsEngineFactory(contact);
      }

      // Create the simulation
      SimulationConstructionSet2 scs = new SimulationConstructionSet2(physicsEngineFactory);

      // Add the brick robot to the simulation
      scs.addRobot(fallingBrick);

      // Set the location and orientation for the camera
      scs.setCameraPosition(0.0, 5.0, 3.5);
      scs.setCameraFocusPosition(0.0, 0.0, 0.8);

      // Track the brick with the camera
      scs.requestCameraRigidBodyTracking(fallingBrick.getName(), FallingBrickDefinition.BRICK_BODY);

      // Add an entry box for these existing YoVariables.
      scs.addYoEntry("time[sec]");
      scs.addYoEntry("qd_rootJoint_world_x");
      scs.addYoEntry("qd_rootJoint_world_z");

      // Add a terrain
      scs.addTerrainObject(new ClutteredGroundDefinition());

      // Simulate no faster than real-time
      scs.setRealTimeRateSimulation(true);

      // Launch the simulator
      scs.start(true, false, false);
   }

   public static void main(String[] args)
   {
      new FallingBrickSimulation();
   }
}