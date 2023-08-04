package us.ihmc.fallingBrick;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.GroundContactPointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

/**
 * The falling brick definition is used to define the various properties of the robot. It is
 * considered a robot as it is a dynamic object that can interact with the environment. The
 * properties defined here are:
 * <ul>
 * <li>the visual properties: how the robot will appear in the 3D view,
 * <li>the kinematics properties: the type of joints and how they are connected,
 * <li>the mass properties: the mass and inertia for each rigid body,
 * <li>the collision properties: the shapes to consider when the robot interacts with the
 * environment.
 * </ul>
 */
public class FallingBrickDefinition extends RobotDefinition
{
   public static final String FALLING_BRICK = "fallingBrick";
   public static final String BRICK_BODY = "Brick";

   // Define the parameters of the brick
   private static final double BASE_H = 0.1, BASE_W = 0.2, BASE_L = 0.3;
   private static final double M1 = 0.5;
   private static final double Ixx1 = (1.0 / 12.0) * M1 * (BASE_W * BASE_W + BASE_H * BASE_H);
   private static final double Iyy1 = (1.0 / 12.0) * M1 * (BASE_L * BASE_L + BASE_H * BASE_H);
   private static final double Izz1 = (1.0 / 12.0) * M1 * (BASE_L * BASE_L + BASE_W * BASE_W);

   public FallingBrickDefinition()
   {
      // Call parent class "Robot" constructor. The string "FallingBrick" will be the name of the robot.
      super(FALLING_BRICK); // creates an instance of the class Robot named "FallingBrick" in the SCS system.

      // Create the top (fixed) link that serves as the base of the brick
      // Define this link as the root body of our robot. It is fixed in the world.
      RigidBodyDefinition rootBody = new RigidBodyDefinition("rootBody");
      setRootBodyDefinition(rootBody);
      /*
       * This method returns a brick as a rigid object.
       */
      // Define rigid body 
      // Create the brick body
      RigidBodyDefinition brick = new RigidBodyDefinition(BRICK_BODY);
      // Define brick geometry
      GeometryDefinition geometryDefinition = new Box3DDefinition(BASE_L, BASE_W, BASE_H);
      // Define material and visuals
      brick.addVisualDefinition(new VisualDefinition(geometryDefinition, new MaterialDefinition(ColorDefinitions.Purple())));
      // Setup collision properties based on defined geometry. We re-use the same geometry as for the visual such that the collisions will be consistent with what we see.
      // This is only needed in case we use impulse based physics engine.
      brick.addCollisionShapeDefinition(new CollisionShapeDefinition(geometryDefinition));
      // Finally setting the mass properties of the brick.
      brick.setMass(M1);
      brick.getMomentOfInertia().setToDiagonal(Ixx1, Iyy1, Izz1);
      brick.setCenterOfMassOffset(0.0, 0.0, 0.0);

      // Define and add a floating joint to the brick. This is how the brick will move freely in the world.
      SixDoFJointDefinition floatingJoint = new SixDoFJointDefinition(getRootJointName());
      // Connect this joint to the robot base
      rootBody.addChildJoint(floatingJoint);
      floatingJoint.setSuccessor(brick);

      // Add ground contact points to the brick
      /*
       * Each GroundContactPointDefinition will be used to create a GroundContactPoint attached to the
       * robot. The ground contact points are used by the contact-point based physics engine to prevent
       * penetration into the environment. The first parameter is the name of the ground contact point.
       * The second parameter is the offset vector with respect to the joint the contact point will be
       * attached to.
       */
      // Corner points
      GroundContactPointDefinition gc1 = new GroundContactPointDefinition("gc1", new Vector3D(0.5 * BASE_L, 0.5 * BASE_W, 0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc1);
      GroundContactPointDefinition gc2 = new GroundContactPointDefinition("gc2", new Vector3D(0.5 * BASE_L, -0.5 * BASE_W, 0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc2);
      GroundContactPointDefinition gc3 = new GroundContactPointDefinition("gc3", new Vector3D(0.5 * BASE_L, 0.5 * BASE_W, -0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc3);
      GroundContactPointDefinition gc4 = new GroundContactPointDefinition("gc4", new Vector3D(0.5 * BASE_L, -0.5 * BASE_W, -0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc4);
      GroundContactPointDefinition gc5 = new GroundContactPointDefinition("gc5", new Vector3D(-0.5 * BASE_L, 0.5 * BASE_W, 0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc5);
      GroundContactPointDefinition gc6 = new GroundContactPointDefinition("gc6", new Vector3D(-0.5 * BASE_L, -0.5 * BASE_W, 0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc6);
      GroundContactPointDefinition gc7 = new GroundContactPointDefinition("gc7", new Vector3D(-0.5 * BASE_L, 0.5 * BASE_W, -0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc7);
      GroundContactPointDefinition gc8 = new GroundContactPointDefinition("gc8", new Vector3D(-0.5 * BASE_L, -0.5 * BASE_W, -0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc8);
      GroundContactPointDefinition gc9 = new GroundContactPointDefinition("gc9", new Vector3D(0.0, 0.0, 0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc9);
      GroundContactPointDefinition gc10 = new GroundContactPointDefinition("gc10", new Vector3D(0.0, 0.0, -0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc10);

      // Surface points
      GroundContactPointDefinition gc11 = new GroundContactPointDefinition("gc11", new Vector3D(0.0, -0.5 * BASE_W, -0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc11);
      GroundContactPointDefinition gc12 = new GroundContactPointDefinition("gc12", new Vector3D(0.0, -0.5 * BASE_W, 0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc12);
      GroundContactPointDefinition gc13 = new GroundContactPointDefinition("gc13", new Vector3D(0.0, 0.5 * BASE_W, -0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc13);
      GroundContactPointDefinition gc14 = new GroundContactPointDefinition("gc14", new Vector3D(0.0, 0.5 * BASE_W, 0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc14);

      // Edge points
      GroundContactPointDefinition gc15 = new GroundContactPointDefinition("gc15", new Vector3D(0.5 * BASE_L, 0.0, -0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc15);
      GroundContactPointDefinition gc16 = new GroundContactPointDefinition("gc16", new Vector3D(0.5 * BASE_L, 0.0, 0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc16);
      GroundContactPointDefinition gc17 = new GroundContactPointDefinition("gc17", new Vector3D(-0.5 * BASE_L, 0.0, -0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc17);
      GroundContactPointDefinition gc18 = new GroundContactPointDefinition("gc18", new Vector3D(-0.5 * BASE_L, 0.0, 0.5 * BASE_H));
      floatingJoint.addGroundContactPointDefinition(gc18);

      // Visualization of the ground contact points on the brick
      for (GroundContactPointDefinition contactPoint : floatingJoint.getGroundContactPointDefinitions())
      {
         brick.addVisualDefinition(new VisualDefinition(contactPoint.getTransformToParent().getTranslation(),
                                                        new Sphere3DDefinition(0.005),
                                                        new MaterialDefinition(ColorDefinitions.Aqua())));
      }
   }

   public String getRootJointName()
   {
      return "rootJoint";
   }
}
