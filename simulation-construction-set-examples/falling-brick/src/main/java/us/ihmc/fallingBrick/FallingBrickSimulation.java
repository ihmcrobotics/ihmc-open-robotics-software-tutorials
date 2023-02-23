package us.ihmc.fallingBrick;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.simulation.parameters.ContactPointBasedContactParameters;

public class FallingBrickSimulation
{
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

      // Instantiate a SCS2 object and specify the physics engine to use
      /*
       * Creating the simulation environment using the contact-point based physics engine. This physics
       * engine relies on forward dynamics to simulate robot joint acceleration and includes robot <->
       * environment interactions using a non-linear spring damping contact force calculator. Note that
       * this physics engine only uses the ground contact points attached to the robot (defined as
       * GroundContactPointDefinition in the RobotDefinition) and TerrainObjectDefinition for the
       * environment.
       */
      // Define ground contact parameters stiffness (k) and damping (b). These parameters are used for the spring-damper model of all surfaces in the simulation
      ContactPointBasedContactParameters contact = ContactPointBasedContactParameters.defaultParameters();
      contact.setKxy(40000.0);
      contact.setBxy(100.0);
      contact.setKz(500.0);
      contact.setBz(250.0);

      // Setup the simulation
      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.contactPointBasedPhysicsEngineFactory(contact));

      // Add the brick robot to the simulation
      scs.addRobot(fallingBrick);

      // Set the location and orientation for the camera
      scs.setCameraPosition(0.0, 5.0, 3.5);
      scs.setCameraFocusPosition(0.0, 0.0, 0.8);

      // Track the brick with the camera
      scs.requestCameraRigidBodyTracking(scs.getRobots().get(0).getName(), scs.getRobots().get(0).getAllJoints().get(0).getSuccessor().getName());

      // Add an entry box for these existing YoVariables
      scs.addYoEntry("time[sec]");
      scs.addYoEntry("gravityZ");
      scs.addYoEntry("q_rootJoint_z");

      // Add a terrain
      scs.addTerrainObject(new ClutteredGroundDefinition());

      // Simulating in real-time
      scs.setRealTimeRateSimulation(true);

      // Launch the simulator
      scs.start(true, false, false);
   }

   public static void main(String[] args)
   {
      new FallingBrickSimulation();
   }
}