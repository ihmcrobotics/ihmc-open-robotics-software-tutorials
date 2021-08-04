package us.ihmc.fallingBrick;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class FallingBrickRobot extends Robot
{
   private static final double BASE_H = 0.1, BASE_W = 0.2, BASE_L = 0.3;
   private static final double B1 = BASE_H / 2.0;
   private static final double M1 = 1.7;
   private static final double Ixx1 = 0.1, Iyy1 = 0.5, Izz1 = 0.9;
   private static final double G = 9.81;

   /*
    * The first parameter "base" is the name of the joint and will be used in all the variables
    * associated with the joint. The second parameter "new Vector3d(0.0, 0.0, 0.0)" defines the offset
    * of this joint from with respect to world's origin. The third parameter "this" refers to the robot
    * itself.
    */
   FloatingJoint floatingJoint = new FloatingJoint("base", new Vector3D(0.0, 0.0, 0.0), this);;

   public FallingBrickRobot()
   {
      // Call parent class "Robot" constructor. The string "FallingBrick" will be the name of the robot.
      super("FallingBrick"); // creates an instance of the class Robot named "FallingBrick" in the SCS system.

      this.setGravity(0.0, 0.0, -G);

      // create the brick as a floating joint and adds it to the robot
      Link link1 = base("base", YoAppearance.Purple());
      // Sets the link created previously as the link for this joint
      floatingJoint.setLink(link1);

      // Add a RootJoint to the robot
      /*
       * This line adds floatingJoint as a rootJoint of the robot. In order to be part of a robot, a joint
       * must either be added as a root joint, or be attached to a parent joint. This ensures the tree
       * structure (forest structure if there are multiple root joints) of the robot.
       */
      addRootJoint(floatingJoint);

      // add ground contact points to the brick
      /*
       * The first parameter is the name of the ground contact point. The second parameter is the offset
       * vector with respect to the joint the contact point will be attached to. The third parameter
       * refers to the robot.
       */
      GroundContactPoint gc1 = new GroundContactPoint("gc1", new Vector3D(BASE_L / 2.0, BASE_W / 2.0, BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc1);
      GroundContactPoint gc2 = new GroundContactPoint("gc2", new Vector3D(BASE_L / 2.0, -BASE_W / 2.0, BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc2);
      GroundContactPoint gc3 = new GroundContactPoint("gc3", new Vector3D(-BASE_L / 2.0, BASE_W / 2.0, BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc3);
      GroundContactPoint gc4 = new GroundContactPoint("gc4", new Vector3D(-BASE_L / 2.0, -BASE_W / 2.0, BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc4);

      GroundContactPoint gc5 = new GroundContactPoint("gc5", new Vector3D(BASE_L / 2.0, BASE_W / 2.0, -BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc5);
      GroundContactPoint gc6 = new GroundContactPoint("gc6", new Vector3D(BASE_L / 2.0, -BASE_W / 2.0, -BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc6);
      GroundContactPoint gc7 = new GroundContactPoint("gc7", new Vector3D(-BASE_L / 2.0, BASE_W / 2.0, -BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc7);
      GroundContactPoint gc8 = new GroundContactPoint("gc8", new Vector3D(-BASE_L / 2.0, -BASE_W / 2.0, -BASE_H / 2.0), this);
      floatingJoint.addGroundContactPoint(gc8);

      GroundContactPoint gc9 = new GroundContactPoint("gc9", new Vector3D(0.0, 0.0, BASE_H / 2.0 + BASE_H), this);
      floatingJoint.addGroundContactPoint(gc9);
      GroundContactPoint gc10 = new GroundContactPoint("gc10", new Vector3D(0.0, 0.0, -BASE_H / 2.0 - BASE_H), this);
      floatingJoint.addGroundContactPoint(gc10);

      // instantiate ground contact model
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
      GroundContactModel groundModel = new LinearGroundContactModel(this, 1422, 150.6, 50.0, 1000.0, getRobotsYoRegistry());
      // A GroundProfile3D is a height map that is used to define the ground
      GroundProfile3D profile = new WavyGroundProfile();
      groundModel.setGroundProfile3D(profile);
      setGroundContactModel(groundModel);

      initializeRobot();
   }

   /**
    * This method returns a brick link instance.
    */
   private Link base(String name, AppearanceDefinition appearance)
   {
      // creates a new link with the name specified in the parameter
      Link ret = new Link(name);
      // sets the mass of the link
      ret.setMass(M1);
      // sets the moment of inertia
      ret.setMomentOfInertia(Ixx1, Iyy1, Izz1);
      // sets the center of mass offset with respect to the parent joint
      ret.setComOffset(0.0, 0.0, 0.0);
      // Use to visually represent links in the SCS 3D view
      Graphics3DObject linkGraphics = new Graphics3DObject();
      /*
       * Translates from the current position by the specified distances. Graphic components added after
       * translation will appear in the new location. The coordinate system for these translations is
       * based on those that preceded it.
       */
      linkGraphics.translate(0.0, 0.0, -B1);

      // add the pyramid cube 
      linkGraphics.addPyramidCube(BASE_L, BASE_W, BASE_H, BASE_H, appearance);

      // Attach the Graphics3DObject to its link
      /*
       * Associates our linkGraphics object with the ret object and in doing so, translates and rotates
       * the graphic components to be in the same frame of reference as the ret.
       */
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   /**
    * This method sets the initial positions, velocities, and accelerations of the brick
    */
   public void initializeRobot()
   {
      // sets the initial position of the falling brick
      floatingJoint.setPosition(0.0, 0.0, 0.6);

      // sets quaternion 
      floatingJoint.q_qs.set(0.707);
      floatingJoint.q_qx.set(0.3);
      floatingJoint.q_qy.set(0.4);
      floatingJoint.q_qz.set(0.5);

      // sets the angular velocity of the falling back
      floatingJoint.qd_wx.set(0.0001);
      floatingJoint.qd_wy.set(1.0);
      floatingJoint.qd_wz.set(0.5001);

      // sets the initial positions, velocities, and accelerations of the falling brick

   }
}
