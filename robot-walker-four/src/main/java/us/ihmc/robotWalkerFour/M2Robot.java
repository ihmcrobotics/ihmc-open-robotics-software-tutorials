package us.ihmc.robotWalkerFour;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.UniversalJoint;

public class M2Robot extends Robot
{
   // This scale is used to make the feet wider so it is easier to keep the robot balance in single support.
   private static final double FOOT_WIDTH_SCALE_FACTOR = 1.5;

   public static final double HIP_OFFSET_Y = 0.184 / 2.0;
   public static final double HIP_JOINT_OFF = 0.0254;
   public static final double HIP_TO_THIGH_OFF = 0.0064;
   public static final double THIGH_LENGTH = 0.432;
   public static final double SHIN_LENGTH = 0.432;
   public static final double ANKLE_JOINT_OFF = 0.0254;
   public static final double FOOT_FORWARD = 0.1524;
   public static final double FOOT_BACK = 0.051;
   public static final double FOOT_LENGTH = FOOT_BACK + FOOT_FORWARD;
   public static final double FOOT_WIDTH = FOOT_WIDTH_SCALE_FACTOR * 0.0889;
   public static final double FOOT_HEIGHT = 0.051;

   public static final double BODY_CYLINDER_HEIGHT = 0.0381;
   public static final double BODY_R = 0.203;
   public static final double BODY_ELLIPSE_HEIGHT = 0.381;

   public static final double THIGH_R = 0.051;
   public static final double SHIN_R = 0.051;

   public static final double BODY_MASS = 12.1011;
   public static final Vector3DReadOnly BODY_COM = new Vector3D(0.0, 0.0, 0.159854);
   public static final Vector3DReadOnly BODY_I = new Vector3D(0.019 * BODY_MASS, 0.019 * BODY_MASS, 0.017 * BODY_MASS);

   public static final double WAIST_MASS = 0.325575;
   public static final Vector3DReadOnly WAIST_COM = new Vector3D(0.0, 0.0, 0.0);
   public static final Vector3DReadOnly WAIST_I = new Vector3D(0.000529262, 0.000529262, 0.000529262);

   public static final double THIGH_MASS = 2.735745;
   public static final Vector3DReadOnly L_THIGH_COM = new Vector3D(0.0, 0.006400, -0.21600);
   public static final Vector3DReadOnly R_THIGH_COM = new Vector3D(0.0, -0.006400, -0.21600);
   public static final Vector3DReadOnly THIGH_I = new Vector3D(0.0443252, 0.0443252, 0.00355784);

   public static final double SHIN_MASS = 2.69484;
   public static final Vector3DReadOnly SHIN_COM = new Vector3D(0.0, 0.0, -0.181082);
   public static final Vector3DReadOnly SHIN_I = new Vector3D(0.054163, 0.054163, 0.003457);

   public static final double RETINACULUM_MASS = 0.250041;
   public static final Vector3DReadOnly RETINACULUM_COM = new Vector3D(0.0, 0.0, 0.0);
   public static final Vector3DReadOnly RETINACULUM_I = new Vector3D(0.000260143, 0.000260143, 0.000260143);

   public static final double FOOT_MASS = 0.414988;
   public static final Vector3DReadOnly FOOT_COM = new Vector3D(0.050700, 0.0, -0.025500);
   public static final Vector3DReadOnly FOOT_I = new Vector3D(0.00036326, 0.00152067, 0.00170404);

   private FloatingJoint bodyJoint;
   private SideDependentList<Joint> footParentJoints = new SideDependentList<>();

   public M2Robot()
   {
      super("M2");

      bodyJoint = new FloatingJoint("body", new Vector3D(), this);
      bodyJoint.setPosition(0.0, 0.0, 0.97);
      Link bodyLink = body();
      bodyJoint.setLink(bodyLink);
      this.addRootJoint(bodyJoint);

      // RIGHT LEG.

      Joint rightHipUni = new UniversalJoint("right_hip_yaw", "right_hip_roll", new Vector3D(0.0, -HIP_OFFSET_Y, 0.0), this, Axis.Z, Axis.X);
      Link rightWaistLink = waist();
      rightHipUni.setLink(rightWaistLink);
      bodyJoint.addJoint(rightHipUni);

      Joint rightHipPitch = new PinJoint("right_hip_pitch", new Vector3D(0.0, 0.0, -HIP_JOINT_OFF), this, Axis.Y);
      Link rightThighLink = rightThigh();
      rightHipPitch.setLink(rightThighLink);
      rightHipUni.addJoint(rightHipPitch);

      PinJoint rightKnee = new PinJoint("right_knee", new Vector3D(0.0, -HIP_TO_THIGH_OFF, -THIGH_LENGTH), this, Axis.Y);
      Link rightShinLink = shin();
      rightKnee.setLink(rightShinLink);
      rightKnee.setLimitStops(0.0, Math.PI, 5000.0, 400.0);
      rightHipPitch.addJoint(rightKnee);

      Joint rightAnkleRoll = new PinJoint("right_ankle_roll", new Vector3D(0.0, 0.0, -SHIN_LENGTH), this, Axis.X);
      Link rightRetinaculumLink = retinaculum();
      rightAnkleRoll.setLink(rightRetinaculumLink);
      rightKnee.addJoint(rightAnkleRoll);

      Joint rightAnklePitch = new PinJoint("right_ankle_pitch", new Vector3D(0.0, 0.0, -ANKLE_JOINT_OFF), this, Axis.Y);
      Link rightFootLink = foot(RobotSide.RIGHT);
      rightAnklePitch.setLink(rightFootLink);
      rightAnkleRoll.addJoint(rightAnklePitch);
      footParentJoints.put(RobotSide.RIGHT, rightAnklePitch);

      /*
       * Here are defined the contact points that the simulation can use to make the ground interact
       * with the robot. Without them the robot would fall through the ground.
       */
      GroundContactPoint right_toe_in = new GroundContactPoint("gc_right_toe_in", new Vector3D(FOOT_FORWARD, FOOT_WIDTH / 2.0, -FOOT_HEIGHT), this);
      GroundContactPoint right_toe_out = new GroundContactPoint("gc_right_toe_out", new Vector3D(FOOT_FORWARD, -FOOT_WIDTH / 2.0, -FOOT_HEIGHT), this);
      GroundContactPoint right_heel_in = new GroundContactPoint("gc_right_heel_in", new Vector3D(-FOOT_BACK, FOOT_WIDTH / 2.0, -FOOT_HEIGHT), this);
      GroundContactPoint right_heel_out = new GroundContactPoint("gc_right_heel_out", new Vector3D(-FOOT_BACK, -FOOT_WIDTH / 2.0, -FOOT_HEIGHT), this);

      rightAnklePitch.addGroundContactPoint(right_toe_in);
      rightAnklePitch.addGroundContactPoint(right_toe_out);
      rightAnklePitch.addGroundContactPoint(right_heel_in);
      rightAnklePitch.addGroundContactPoint(right_heel_out);

      // LEFT LEG.

      Joint leftHipUni = new UniversalJoint("left_hip_yaw", "left_hip_roll", new Vector3D(0.0, HIP_OFFSET_Y, 0.0), this, Axis.Z, Axis.X);
      Link leftWaistLink = waist();
      leftHipUni.setLink(leftWaistLink);
      bodyJoint.addJoint(leftHipUni);

      Joint leftHipPitch = new PinJoint("left_hip_pitch", new Vector3D(0.0, 0.0, -HIP_JOINT_OFF), this, Axis.Y);
      Link leftThighLink = leftThigh();
      leftHipPitch.setLink(leftThighLink);
      leftHipUni.addJoint(leftHipPitch);

      PinJoint leftKnee = new PinJoint("left_knee", new Vector3D(0.0, HIP_TO_THIGH_OFF, -THIGH_LENGTH), this, Axis.Y);
      Link leftShinLink = shin();
      leftKnee.setLink(leftShinLink);
      leftKnee.setLimitStops(0.0, Math.PI, 5000.0, 400.0);
      leftHipPitch.addJoint(leftKnee);

      Joint leftAnkleRoll = new PinJoint("left_ankle_roll", new Vector3D(0.0, 0.0, -SHIN_LENGTH), this, Axis.X);
      Link leftRetinaculumLink = retinaculum();
      leftAnkleRoll.setLink(leftRetinaculumLink);
      leftKnee.addJoint(leftAnkleRoll);

      Joint leftAnklePitch = new PinJoint("left_ankle_pitch", new Vector3D(0.0, 0.0, -ANKLE_JOINT_OFF), this, Axis.Y);
      Link leftFootLink = foot(RobotSide.LEFT);
      leftAnklePitch.setLink(leftFootLink);
      leftAnkleRoll.addJoint(leftAnklePitch);
      footParentJoints.put(RobotSide.LEFT, leftAnklePitch);

      /*
       * Here are defined the contact points that the simulation can use to make the ground interact
       * with the robot. Without them the robot would fall through the ground.
       */
      GroundContactPoint left_toe_in = new GroundContactPoint("gc_left_toe_in", new Vector3D(FOOT_FORWARD, -FOOT_WIDTH / 2.0, -FOOT_HEIGHT), this);
      GroundContactPoint left_toe_out = new GroundContactPoint("gc_left_toe_out", new Vector3D(FOOT_FORWARD, FOOT_WIDTH / 2.0, -FOOT_HEIGHT), this);
      GroundContactPoint left_heel_in = new GroundContactPoint("gc_left_heel_in", new Vector3D(-FOOT_BACK, -FOOT_WIDTH / 2.0, -FOOT_HEIGHT), this);
      GroundContactPoint left_heel_out = new GroundContactPoint("gc_left_heel_out", new Vector3D(-FOOT_BACK, FOOT_WIDTH / 2.0, -FOOT_HEIGHT), this);

      leftAnklePitch.addGroundContactPoint(left_toe_in);
      leftAnklePitch.addGroundContactPoint(left_toe_out);
      leftAnklePitch.addGroundContactPoint(left_heel_in);
      leftAnklePitch.addGroundContactPoint(left_heel_out);

   }

   public FloatingJoint getFloatingJoint()
   {
      return bodyJoint;
   }

   public Joint getFootParentJoint(RobotSide robotSide)
   {
      return footParentJoints.get(robotSide);
   }

   private Link body()
   {
      Link ret = new Link("body");
      ret.setMass(BODY_MASS);
      ret.setComOffset(new Vector3D(BODY_COM));
      ret.setMomentOfInertia(BODY_I.getX(), BODY_I.getY(), BODY_I.getZ());

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, BODY_CYLINDER_HEIGHT);

      linkGraphics.addHemiEllipsoid(BODY_R, BODY_R, BODY_ELLIPSE_HEIGHT, YoAppearance.Red());
      linkGraphics.translate(0.0, 0.0, -BODY_CYLINDER_HEIGHT);
      linkGraphics.addCylinder(BODY_CYLINDER_HEIGHT, BODY_R);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link waist()
   {
      Link ret = new Link("waist");
      ret.setMass(WAIST_MASS);
      ret.setComOffset(new Vector3D(WAIST_COM));
      ret.setMomentOfInertia(WAIST_I.getX(), WAIST_I.getY(), WAIST_I.getZ());

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(1.25f * THIGH_R, YoAppearance.White());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link leftThigh()
   {
      Link ret = new Link("left_thigh");
      ret.setMass(THIGH_MASS);
      ret.setComOffset(new Vector3D(L_THIGH_COM));
      ret.setMomentOfInertia(THIGH_I.getX(), THIGH_I.getY(), THIGH_I.getZ());

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, HIP_TO_THIGH_OFF, -THIGH_LENGTH);
      linkGraphics.addCylinder(THIGH_LENGTH, THIGH_R);
      linkGraphics.translate(0.0, -HIP_TO_THIGH_OFF, THIGH_LENGTH);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link rightThigh()
   {
      Link ret = new Link("right_thigh");
      ret.setMass(THIGH_MASS);
      ret.setComOffset(new Vector3D(R_THIGH_COM));
      ret.setMomentOfInertia(THIGH_I.getX(), THIGH_I.getY(), THIGH_I.getZ());

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, -HIP_TO_THIGH_OFF, -THIGH_LENGTH);
      linkGraphics.addCylinder(THIGH_LENGTH, THIGH_R);
      linkGraphics.translate(0.0, HIP_TO_THIGH_OFF, THIGH_LENGTH);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link shin()
   {
      Link ret = new Link("shin");
      ret.setMass(SHIN_MASS);
      ret.setComOffset(new Vector3D(SHIN_COM));
      ret.setMomentOfInertia(SHIN_I.getX(), SHIN_I.getY(), SHIN_I.getZ());

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(1.07f * SHIN_R, YoAppearance.White());
      linkGraphics.translate(0.0, 0.0, -SHIN_LENGTH);
      linkGraphics.addCylinder(SHIN_LENGTH, SHIN_R);
      linkGraphics.translate(0.0, 0.0, SHIN_LENGTH);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link retinaculum()
   {
      Link ret = new Link("retinaculum");
      ret.setMass(RETINACULUM_MASS);
      ret.setComOffset(new Vector3D(RETINACULUM_COM));
      ret.setMomentOfInertia(RETINACULUM_I.getX(), RETINACULUM_I.getY(), RETINACULUM_I.getZ());

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addSphere(SHIN_R, YoAppearance.White());
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link foot(RobotSide robotSide)
   {
      Link ret = new Link(robotSide.getCamelCaseName() + "foot");
      ret.setMass(FOOT_MASS);
      ret.setComOffset(new Vector3D(FOOT_COM));
      ret.setMomentOfInertia(FOOT_I.getX(), FOOT_I.getY(), FOOT_I.getZ());

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(FOOT_FORWARD - FOOT_LENGTH / 2.0, 0.0, -FOOT_HEIGHT);
      linkGraphics.addCube(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT, YoAppearance.Red());
      linkGraphics.translate(-FOOT_FORWARD + FOOT_LENGTH / 2.0, 0.0, FOOT_HEIGHT);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }
}
