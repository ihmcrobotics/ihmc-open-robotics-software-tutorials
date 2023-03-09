package us.ihmc.robotWalkerFive;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearanceTransparent;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.HemiEllipsoid3DDefinition;
import us.ihmc.scs2.definition.geometry.STPBox3DDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.robot.GroundContactPointDefinition;
import us.ihmc.scs2.definition.robot.OneDoFJointDefinition;
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class M2RobotDefinition extends RobotDefinition
{
   private static final String M2ROBOT = "M2";

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

   private SixDoFJointDefinition bodyJoint;
   private SideDependentList<RevoluteJointDefinition> footParentJoints = new SideDependentList<>();

   public M2RobotDefinition()
   {
      super(M2ROBOT);

      // Create the top (fixed) link that serves as the base of the robot
      /**
       * The elevator is a mass-less, size-less rigid-body fixed in world to which the first joint of the
       * robot is attached. The name comes from the use of this rigid-body to add the gravity effect to
       * the robot by making it accelerate like an elevator when it starts moving. However, this elevator
       * is always fixed in world with no velocity.
       */
      RigidBodyDefinition elevator = new RigidBodyDefinition("elevator");
      setRootBodyDefinition(elevator);

      // Define and add a floating joint to the robot base   
      SixDoFJointDefinition floatingJoint = new SixDoFJointDefinition(getRootJointName());

      //The robot needs to start standing on the ground because the controller will start in the "standing"-state and will expect ground contact
      floatingJoint.setInitialJointState(new SixDoFJointState(null, new Vector3D(0.0, 0.0, 0.965)));
      elevator.addChildJoint(floatingJoint);

      // Define main body and attach it to the floating joint
      RigidBodyDefinition mainBody = createBody();
      floatingJoint.setSuccessor(mainBody);

      // RIGHT LEG
      // right hip joint
      OneDoFJointDefinition[] rightHipUni = createUniversalJoint("right_hip_yaw",
                                                                 "right_hip_roll",
                                                                 new Vector3D(0.0, -HIP_OFFSET_Y, 0.0),
                                                                 Axis3D.Z,
                                                                 Axis3D.X,
                                                                 mainBody);

      // right hip body
      RigidBodyDefinition rightWaistBody = waist("right");
      rightHipUni[1].setSuccessor(rightWaistBody);

      RevoluteJointDefinition rightHipPitch = new RevoluteJointDefinition("right_hip_pitch", new Vector3D(0.0, 0.0, -HIP_JOINT_OFF), Axis3D.Y);
      RigidBodyDefinition rightThighBody = rightThigh();
      rightWaistBody.addChildJoint(rightHipPitch);
      rightHipPitch.setSuccessor(rightThighBody);

      RevoluteJointDefinition rightKnee = new RevoluteJointDefinition("right_knee", new Vector3D(0.0, -HIP_TO_THIGH_OFF, -THIGH_LENGTH), Axis3D.Y);
      RigidBodyDefinition rightShinBody = shin("right");
      rightThighBody.addChildJoint(rightKnee);
      rightKnee.setSuccessor(rightShinBody);
      rightKnee.setPositionLimits(0.0, Math.PI);
      rightKnee.setKpSoftLimitStop(5000.0);
      rightKnee.setKdSoftLimitStop(400.0);

      RevoluteJointDefinition rightAnkleRoll = new RevoluteJointDefinition("right_ankle_roll", new Vector3D(0.0, 0.0, -SHIN_LENGTH), Axis3D.X);
      RigidBodyDefinition rightRetinaculumBody = retinaculum("right");
      rightShinBody.addChildJoint(rightAnkleRoll);
      rightAnkleRoll.setSuccessor(rightRetinaculumBody);

      RevoluteJointDefinition rightAnklePitch = new RevoluteJointDefinition("right_ankle_pitch", new Vector3D(0.0, 0.0, -ANKLE_JOINT_OFF), Axis3D.Y);
      RigidBodyDefinition rightFootBody = foot(RobotSide.RIGHT);
      rightRetinaculumBody.addChildJoint(rightAnklePitch);
      rightAnklePitch.setSuccessor(rightFootBody);

      footParentJoints.put(RobotSide.RIGHT, rightAnklePitch);

      /*
       * Here are defined the contact points that the simulation can use to make the ground interact with
       * the robot. Without them the robot would fall through the ground.
       */
      GroundContactPointDefinition right_toe_in = new GroundContactPointDefinition("gc_right_toe_in",
                                                                                   new Vector3D(FOOT_FORWARD, FOOT_WIDTH / 2.0, -FOOT_HEIGHT));
      GroundContactPointDefinition right_toe_out = new GroundContactPointDefinition("gc_right_toe_out",
                                                                                    new Vector3D(FOOT_FORWARD, -FOOT_WIDTH / 2.0, -FOOT_HEIGHT));
      GroundContactPointDefinition right_heel_in = new GroundContactPointDefinition("gc_right_heel_in",
                                                                                    new Vector3D(-FOOT_BACK, FOOT_WIDTH / 2.0, -FOOT_HEIGHT));
      GroundContactPointDefinition right_heel_out = new GroundContactPointDefinition("gc_right_heel_out",
                                                                                     new Vector3D(-FOOT_BACK, -FOOT_WIDTH / 2.0, -FOOT_HEIGHT));

      rightAnklePitch.addGroundContactPointDefinition(right_toe_in);
      rightAnklePitch.addGroundContactPointDefinition(right_toe_out);
      rightAnklePitch.addGroundContactPointDefinition(right_heel_in);
      rightAnklePitch.addGroundContactPointDefinition(right_heel_out);

      // LEFT LEG
      OneDoFJointDefinition[] leftHipUni = createUniversalJoint("left_hip_yaw",
                                                                "left_hip_roll",
                                                                new Vector3D(0.0, HIP_OFFSET_Y, 0.0),
                                                                Axis3D.Z,
                                                                Axis3D.X,
                                                                mainBody);
      RigidBodyDefinition leftWaistBody = waist("left");
      leftHipUni[1].setSuccessor(leftWaistBody);

      RevoluteJointDefinition leftHipPitch = new RevoluteJointDefinition("left_hip_pitch", new Vector3D(0.0, 0.0, -HIP_JOINT_OFF), Axis3D.Y);
      RigidBodyDefinition leftThighBody = leftThigh();
      leftWaistBody.addChildJoint(leftHipPitch);
      leftHipPitch.setSuccessor(leftThighBody);

      RevoluteJointDefinition leftKnee = new RevoluteJointDefinition("left_knee", new Vector3D(0.0, HIP_TO_THIGH_OFF, -THIGH_LENGTH), Axis3D.Y);
      RigidBodyDefinition leftShinBody = shin("left");
      leftThighBody.addChildJoint(leftKnee);
      leftKnee.setSuccessor(leftShinBody);
      leftKnee.setPositionLimits(0.0, Math.PI);
      leftKnee.setKpSoftLimitStop(5000.0);
      leftKnee.setKdSoftLimitStop(400.0);

      RevoluteJointDefinition leftAnkleRoll = new RevoluteJointDefinition("left_ankle_roll", new Vector3D(0.0, 0.0, -SHIN_LENGTH), Axis3D.X);
      RigidBodyDefinition leftRetinaculumBody = retinaculum("left");
      leftShinBody.addChildJoint(leftAnkleRoll);
      leftAnkleRoll.setSuccessor(leftRetinaculumBody);

      RevoluteJointDefinition leftAnklePitch = new RevoluteJointDefinition("left_ankle_pitch", new Vector3D(0.0, 0.0, -ANKLE_JOINT_OFF), Axis3D.Y);
      RigidBodyDefinition leftFootBody = foot(RobotSide.LEFT);
      leftRetinaculumBody.addChildJoint(leftAnklePitch);
      leftAnklePitch.setSuccessor(leftFootBody);

      footParentJoints.put(RobotSide.LEFT, leftAnklePitch);

      /*
       * Here are defined the contact points that the simulation can use to make the ground interact with
       * the robot. Without them the robot would fall through the ground.
       */
      GroundContactPointDefinition left_toe_in = new GroundContactPointDefinition("gc_left_toe_in",
                                                                                  new Vector3D(FOOT_FORWARD, -FOOT_WIDTH / 2.0, -FOOT_HEIGHT));
      GroundContactPointDefinition left_toe_out = new GroundContactPointDefinition("gc_left_toe_out",
                                                                                   new Vector3D(FOOT_FORWARD, FOOT_WIDTH / 2.0, -FOOT_HEIGHT));
      GroundContactPointDefinition left_heel_in = new GroundContactPointDefinition("gc_left_heel_in",
                                                                                   new Vector3D(-FOOT_BACK, -FOOT_WIDTH / 2.0, -FOOT_HEIGHT));
      GroundContactPointDefinition left_heel_out = new GroundContactPointDefinition("gc_left_heel_out",
                                                                                    new Vector3D(-FOOT_BACK, FOOT_WIDTH / 2.0, -FOOT_HEIGHT));

      leftAnklePitch.addGroundContactPointDefinition(left_toe_in);
      leftAnklePitch.addGroundContactPointDefinition(left_toe_out);
      leftAnklePitch.addGroundContactPointDefinition(left_heel_in);
      leftAnklePitch.addGroundContactPointDefinition(left_heel_out);

      // Set damping to 0.0 (due to bug in OneDoFJointDefinition which initializes the damping to -1)
      forEachOneDoFJointDefinition(j -> j.setDamping(0.0));
   }

   public SixDoFJointDefinition getFloatingJoint()
   {
      return bodyJoint;
   }

   public OneDoFJointDefinition[] createUniversalJoint(String jname1,
                                                       String jname2,
                                                       Tuple3DReadOnly offset,
                                                       Vector3DReadOnly firstAxis,
                                                       Vector3DReadOnly secondAxis,
                                                       RigidBodyDefinition predecessor)
   {
      RevoluteJointDefinition joint1 = new RevoluteJointDefinition(jname1);
      RevoluteJointDefinition joint2 = new RevoluteJointDefinition(jname2);

      RigidBodyDefinition joint1Body = createNullBody(jname1 + "Body");
      RigidBodyDefinition joint2Body = createNullBody(jname2 + "Body");

      joint1.getTransformToParent().getTranslation().set(offset);
      joint1.getAxis().set(firstAxis);
      joint2.getAxis().set(secondAxis);

      predecessor.getChildrenJoints().add(joint1);
      joint1.setSuccessor(joint1Body);
      joint1Body.getChildrenJoints().add(joint2);
      joint2.setSuccessor(joint2Body);

      return new OneDoFJointDefinition[] {joint1, joint2};
   }

   private RigidBodyDefinition createNullBody(String name)
   {
      RigidBodyDefinition nullBody = new RigidBodyDefinition(name);
      nullBody.setMass(1.0e-12);
      nullBody.getMomentOfInertia().setToDiagonal(1.0e-12, 1.0e-12, 1.0e-12);
      return nullBody;
   }

   public RevoluteJointDefinition getFootParentJoint(RobotSide robotSide)
   {
      return footParentJoints.get(robotSide);
   }

   private RigidBodyDefinition createBody()
   {
      // define body
      RigidBodyDefinition ret = new RigidBodyDefinition("body");
      ret.setMass(BODY_MASS);
      ret.setCenterOfMassOffset(new Vector3D(BODY_COM));
      ret.getMomentOfInertia().setToDiagonal(BODY_I.getX(), BODY_I.getY(), BODY_I.getZ());

      // add graphics definition
      GeometryDefinition hemiEllipsoid = new HemiEllipsoid3DDefinition(BODY_R, BODY_R, BODY_ELLIPSE_HEIGHT);
      RigidBodyTransform hemiEllipsoidPose = new RigidBodyTransform();
      hemiEllipsoidPose.appendTranslation(0.0, 0.0, 0.5 * BODY_CYLINDER_HEIGHT);

      GeometryDefinition cylinder = new Cylinder3DDefinition(BODY_CYLINDER_HEIGHT, BODY_R);
      RigidBodyTransform cylinderPose = new RigidBodyTransform();

      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.DarkCyan());
      MaterialDefinition materialDefinitionCylinder = new MaterialDefinition(ColorDefinitions.Black());

      ret.addVisualDefinition(new VisualDefinition(hemiEllipsoidPose, hemiEllipsoid, materialDefinition));
      ret.addVisualDefinition(new VisualDefinition(cylinderPose, cylinder, materialDefinitionCylinder));

      return ret;
   }

   private RigidBodyDefinition waist(String side)
   {
      RigidBodyDefinition ret = new RigidBodyDefinition(side + "_waist");
      ret.setMass(WAIST_MASS);
      ret.setCenterOfMassOffset(new Vector3D(WAIST_COM));
      ret.getMomentOfInertia().setToDiagonal(BODY_ELLIPSE_HEIGHT, BODY_CYLINDER_HEIGHT, ANKLE_JOINT_OFF);

      GeometryDefinition geometryDefintion = new Sphere3DDefinition(1.25f * THIGH_R);
      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.White());

      ret.addVisualDefinition(new VisualDefinition(geometryDefintion, materialDefinition));

      return ret;
   }

   private RigidBodyDefinition leftThigh()
   {
      RigidBodyDefinition ret = new RigidBodyDefinition("left_thigh");
      ret.setMass(THIGH_MASS);
      ret.setCenterOfMassOffset(new Vector3D(L_THIGH_COM));
      ret.getMomentOfInertia().setToDiagonal(THIGH_I.getX(), THIGH_I.getY(), THIGH_I.getZ());

      GeometryDefinition cylinder = new Cylinder3DDefinition(THIGH_LENGTH, THIGH_R);
      RigidBodyTransform cylinderPose = new RigidBodyTransform();
      cylinderPose.appendTranslation(0.0, HIP_TO_THIGH_OFF, -0.5 * THIGH_LENGTH);

      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.Black());

      ret.addVisualDefinition(new VisualDefinition(cylinderPose, cylinder, materialDefinition));

      return ret;
   }

   private RigidBodyDefinition rightThigh()
   {
      RigidBodyDefinition ret = new RigidBodyDefinition("right_thigh");
      ret.setMass(THIGH_MASS);
      ret.setCenterOfMassOffset(new Vector3D(R_THIGH_COM));
      ret.getMomentOfInertia().setToDiagonal(THIGH_I.getX(), THIGH_I.getY(), THIGH_I.getZ());

      GeometryDefinition cylinder = new Cylinder3DDefinition(THIGH_LENGTH, THIGH_R);
      RigidBodyTransform cylinderPose = new RigidBodyTransform();
      cylinderPose.appendTranslation(0.0, -HIP_TO_THIGH_OFF, -0.5 * THIGH_LENGTH);

      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.Black());

      ret.addVisualDefinition(new VisualDefinition(cylinderPose, cylinder, materialDefinition));

      return ret;
   }

   private RigidBodyDefinition shin(String side)
   {
      RigidBodyDefinition ret = new RigidBodyDefinition(side + "_shin");
      ret.setMass(SHIN_MASS);
      ret.setCenterOfMassOffset(new Vector3D(SHIN_COM));
      ret.getMomentOfInertia().setToDiagonal(SHIN_I.getX(), SHIN_I.getY(), SHIN_I.getZ());

      GeometryDefinition sphere = new Sphere3DDefinition(1.07f * SHIN_R);
      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.White());
      ret.addVisualDefinition(new VisualDefinition(sphere, materialDefinition));
      ret.addCollisionShapeDefinition(new CollisionShapeDefinition(sphere));

      GeometryDefinition cylinder = new Cylinder3DDefinition(SHIN_LENGTH, SHIN_R);
      RigidBodyTransform cylinderPose = new RigidBodyTransform();
      cylinderPose.appendTranslation(0.0, 0.0, -0.5 * SHIN_LENGTH);
      MaterialDefinition materialDefinitionCylinder = new MaterialDefinition(ColorDefinitions.Black());

      ret.addVisualDefinition(new VisualDefinition(cylinderPose, cylinder, materialDefinitionCylinder));

      return ret;
   }

   private RigidBodyDefinition retinaculum(String side)
   {
      RigidBodyDefinition ret = new RigidBodyDefinition(side + "_retinaculum");
      ret.setMass(RETINACULUM_MASS);
      ret.setCenterOfMassOffset(new Vector3D(RETINACULUM_COM));
      ret.getMomentOfInertia().setToDiagonal(RETINACULUM_I.getX(), RETINACULUM_I.getY(), RETINACULUM_I.getZ());

      GeometryDefinition geometryDefintion = new Sphere3DDefinition(SHIN_R);
      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.White());

      ret.addVisualDefinition(new VisualDefinition(geometryDefintion, materialDefinition));

      return ret;
   }

   private RigidBodyDefinition foot(RobotSide robotSide)
   {
      RigidBodyDefinition ret = new RigidBodyDefinition(robotSide.getCamelCaseName() + "foot");
      ret.setMass(FOOT_MASS);
      ret.setCenterOfMassOffset(new Vector3D(FOOT_COM));
      ret.getMomentOfInertia().setToDiagonal(FOOT_I.getX(), FOOT_I.getY(), FOOT_I.getZ());

      GeometryDefinition geometryDefinition = new Box3DDefinition(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT);
      RigidBodyTransform geometryPose = new RigidBodyTransform();
      geometryPose.appendTranslation(FOOT_FORWARD - 0.5 * FOOT_LENGTH, 0.0, -0.5 * FOOT_HEIGHT);

      MaterialDefinition materialDefinition = new MaterialDefinition(ColorDefinitions.DarkCyan());

      ret.addVisualDefinition(new VisualDefinition(geometryPose, geometryDefinition, materialDefinition));
      ret.addCollisionShapeDefinition(new CollisionShapeDefinition(geometryPose, geometryDefinition));

      STPBox3DDefinition geometryDefinition2 = new STPBox3DDefinition(FOOT_LENGTH, FOOT_WIDTH, FOOT_HEIGHT);
      geometryDefinition2.setMaximumMargin(0.0005);
      geometryDefinition2.setMinimumMargin(0.00025);

      ret.addCollisionShapeDefinition(new CollisionShapeDefinition(geometryPose, geometryDefinition2));
      // cannot be visualized yet - visualize using graphic properties -> collisions
      //   MaterialDefinition materialDefinition2 = new MaterialDefinition(ColorDefinitions.Orange());
      //      ret.addVisualDefinition(new VisualDefinition(geometryPose, geometryDefinition2, materialDefinition2));

      return ret;
   }

   public String getRootJointName()
   {
      return "rootJoint";
   }
}
