package us.ihmc.fallingBrick;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.state.SixDoFJointState;

public class FallingBrickSimulation
{
   public FallingBrickSimulation()
   {
      // Create an instance of the falling brick
      FallingBrickDefinition fallingBrick = new FallingBrickDefinition();

      // Set the initial positions, velocities, and accelerations of the brick
      SixDoFJointState initialJointState = new SixDoFJointState();
      initialJointState.setConfiguration(new Pose3D(-0.5, 0.0, 2.0, 0.0, 0.0, 0.0));
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
      SimulationConstructionSet2 scs = new SimulationConstructionSet2(SimulationConstructionSet2.contactPointBasedPhysicsEngineFactory());

      // Add the brick robot to the simulation
      scs.addRobot(fallingBrick);

      // Sets location and orientation of the camera
      scs.setCameraPosition(-0.4, 6.0, 4.0);
      scs.setCameraFocusPosition(0.0, 0.0, 0.3);

      // Add a terrain
      scs.addTerrainObject(new ClutteredGroundDefinition());

      // Launch the simulator
      scs.start(true, false, false);

      // Simulating in real-time
      scs.setRealTimeRateSimulation(true);
   }

   public static void main(String[] args)
   {
      new FallingBrickSimulation();
   }
}