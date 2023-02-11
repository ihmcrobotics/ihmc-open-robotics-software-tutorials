package us.ihmc.fallingBrick;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.SimulationConstructionSet2;
import us.ihmc.scs2.definition.state.SixDoFJointState;

public class FallingBrickSimulation{
   public FallingBrickSimulation()
   {
      // Create an instance of the falling brick
      FallingBrickDefinition fallingBrick = new FallingBrickDefinition();

      // Set the initial positions, velocities, and accelerations of the brick
      SixDoFJointState initialJointState = new SixDoFJointState();
      initialJointState.setConfiguration(new Pose3D(-0.5, 0.0, 2.0, 0.0, 0.0, 0.0));
      initialJointState.setAngularVelocity(new Vector3D(-0.1, -1.0, 10.0));
      initialJointState.setLinearVelocity(new Vector3D(0.0, -0.1, 0.5));
      fallingBrick.getRootJointDefinitions().get(0).setInitialJointState(initialJointState);
      
      // Instantiate a SCS object and ground contact model
      /*
       * This model is like a controller that is called every simulation tick and whose job is to detect
       * contact point colliding with the ground. When a contact point collides with the ground, the model
       * then computes the force to be applied on the robot at the contact point to resolve collision. The
       * LinearGroundContactModel uses a spring-damper based strategy to compute the force to be applied,
       * with the stiffness and damping parameters being axis dependent. One set of stiffness and damping
       * values is used to compute the force along the contact normal while the other set of stiffness and
       * damping values are used to compute the force tangent to the contact, or the friction force. It is
       * also worth nothing that internally, the ground contact model uses a default coefficient of
       * friction that is used to ensure that the ground reaction force remains within a friction cone.
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