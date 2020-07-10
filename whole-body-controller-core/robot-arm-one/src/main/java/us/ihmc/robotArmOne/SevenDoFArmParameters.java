package us.ihmc.robotArmOne;

import java.util.EnumMap;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;

/**
 * Use this class to obtain default parameters for building a 7-DoF robot arm.
 * 
 * @author Sylvain Bertrand
 */
public class SevenDoFArmParameters
{
   /**
    * Enum declaring the 7-DoFs of a robot arm in order from the base to the end-effector.
    */
   public enum SevenDoFArmJointEnum
   {
      shoulderYaw, shoulderRoll, shoulderPitch, elbowPitch, wristPitch, wristRoll, wristYaw;

      /**
       * @return human-readable name to use when creating the joint corresponding to the current enum
       *         value.
       */
      public String getJointName()
      {
         return name();
      }

      /**
       * @return offset of the joint with respect to its parent joint.
       */
      public Vector3D getJointOffset()
      {
         if (!jointOffsets.containsKey(this))
            throw new RuntimeException("No offest has been registered for the joint: " + getJointName());
         return jointOffsets.get(this);
      }

      /**
       * @return the unit vector representing this joint axis.
       */
      public Vector3DReadOnly getJointAxis()
      {
         if (!jointAxes.containsKey(this))
            throw new RuntimeException("No axis has been registered for the joint: " + getJointName());
         return jointAxes.get(this);
      }

      /**
       * @return the lower limit of this joint range of motion.
       */
      public double getJointLowerLimit()
      {
         if (!jointLowerLimits.containsKey(this))
            throw new RuntimeException("No lower limit has been registered for the joint: " + getJointName());
         return jointLowerLimits.get(this);
      }

      /**
       * @return the upper limit of this joint range of motion.
       */
      public double getJointUpperLimit()
      {
         if (!jointUpperLimits.containsKey(this))
            throw new RuntimeException("No upper limit has been registered for the joint: " + getJointName());
         return jointUpperLimits.get(this);
      }

      /**
       * @return the name of the link that is attached to this joint.
       */
      public String getChildLinkName()
      {
         if (!jointChildLinkNames.containsKey(this))
            throw new RuntimeException("No name has been registered for the child link of the joint: " + getJointName());
         return jointChildLinkNames.get(this);
      }

      /**
       * @return the mass of the link that is attached to this joint.
       */
      public double getChildLinkMass()
      {
         if (!jointChildLinkMasses.containsKey(this))
            throw new RuntimeException("No mass has been registered for the child link of the joint: " + getJointName());
         return jointChildLinkMasses.get(this);
      }

      /**
       * @return the center of mass of the link that is attached to this joint. It is expressed with
       *         respect to the parent joint.
       */
      public Vector3D getChildLinkCoM()
      {
         if (!jointChildLinkCoMs.containsKey(this))
            throw new RuntimeException("No CoM has been registered for the child link of the joint: " + getJointName());
         return jointChildLinkCoMs.get(this);
      }

      /**
       * @return the inertia matrix of the link that is attached to this joint.
       */
      public Matrix3D getChildLinkInertia()
      {
         if (!jointChildLinkInertias.containsKey(this))
            throw new RuntimeException("No inertia has been registered for the child link of the joint: " + getJointName());
         return jointChildLinkInertias.get(this);
      }

      /**
       * @return graphics to use for representing the child link.
       */
      public Graphics3DObject getChildLinkGraphics()
      {
         if (!jointChildLinkGraphics.containsKey(this))
            throw new RuntimeException("No graphics has been registered for the child link of the joint: " + getJointName());
         return jointChildLinkGraphics.get(this);
      }
   };

   public static final double armLinkLength = 0.35;
   public static final double armLinkMass = 2.2;
   public static final double armLinkRadius = 0.025;
   public static final Vector3D armLinkCoM = new Vector3D(0.0, 0.0, armLinkLength / 2.0);
   public static final Matrix3D armLinkInertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(armLinkMass,
                                                                                                                       armLinkRadius,
                                                                                                                       armLinkLength,
                                                                                                                       Axis3D.Z);
   public static final Matrix3D smallInertia = diagional(1.0e-4, 1.0e-4, 1.0e-4);

   public static final double handMass = 1.2;
   public static final Vector3D handCoM = new Vector3D(0.0, 0.0, 0.05);
   public static final Matrix3D handInertia = RotationalInertiaCalculator.getRotationalInertiaFromRadiiOfGyration(handMass, 0.08, 0.08, 0.08);

   public static final EnumMap<SevenDoFArmJointEnum, Vector3DReadOnly> jointAxes = new EnumMap<>(SevenDoFArmJointEnum.class);
   public static final EnumMap<SevenDoFArmJointEnum, Vector3D> jointOffsets = new EnumMap<>(SevenDoFArmJointEnum.class);
   public static final EnumMap<SevenDoFArmJointEnum, Double> jointLowerLimits = new EnumMap<>(SevenDoFArmJointEnum.class);
   public static final EnumMap<SevenDoFArmJointEnum, Double> jointUpperLimits = new EnumMap<>(SevenDoFArmJointEnum.class);
   public static final EnumMap<SevenDoFArmJointEnum, String> jointChildLinkNames = new EnumMap<>(SevenDoFArmJointEnum.class);
   public static final EnumMap<SevenDoFArmJointEnum, Double> jointChildLinkMasses = new EnumMap<>(SevenDoFArmJointEnum.class);
   public static final EnumMap<SevenDoFArmJointEnum, Vector3D> jointChildLinkCoMs = new EnumMap<>(SevenDoFArmJointEnum.class);
   public static final EnumMap<SevenDoFArmJointEnum, Matrix3D> jointChildLinkInertias = new EnumMap<>(SevenDoFArmJointEnum.class);
   public static final EnumMap<SevenDoFArmJointEnum, Graphics3DObject> jointChildLinkGraphics = new EnumMap<>(SevenDoFArmJointEnum.class);

   static
   {
      jointAxes.put(SevenDoFArmJointEnum.shoulderYaw, Axis3D.Z);
      jointAxes.put(SevenDoFArmJointEnum.shoulderRoll, Axis3D.X);
      jointAxes.put(SevenDoFArmJointEnum.shoulderPitch, Axis3D.Y);
      jointAxes.put(SevenDoFArmJointEnum.elbowPitch, Axis3D.Y);
      jointAxes.put(SevenDoFArmJointEnum.wristPitch, Axis3D.Y);
      jointAxes.put(SevenDoFArmJointEnum.wristRoll, Axis3D.X);
      jointAxes.put(SevenDoFArmJointEnum.wristYaw, Axis3D.Z);

      jointOffsets.put(SevenDoFArmJointEnum.shoulderYaw, new Vector3D(0.0, 0.0, 0.0));
      jointOffsets.put(SevenDoFArmJointEnum.shoulderRoll, new Vector3D(0.0, 0.0, 0.0));
      jointOffsets.put(SevenDoFArmJointEnum.shoulderPitch, new Vector3D(0.0, 0.0, 0.0));
      jointOffsets.put(SevenDoFArmJointEnum.elbowPitch, new Vector3D(0.0, 0.0, armLinkLength));
      jointOffsets.put(SevenDoFArmJointEnum.wristPitch, new Vector3D(0.0, 0.0, armLinkLength));
      jointOffsets.put(SevenDoFArmJointEnum.wristRoll, new Vector3D(0.0, 0.0, 0.0));
      jointOffsets.put(SevenDoFArmJointEnum.wristYaw, new Vector3D(0.0, 0.0, 0.0));

      jointLowerLimits.put(SevenDoFArmJointEnum.shoulderYaw, -Math.PI);
      jointLowerLimits.put(SevenDoFArmJointEnum.shoulderRoll, -Math.PI / 2.0);
      jointLowerLimits.put(SevenDoFArmJointEnum.shoulderPitch, -Math.PI / 2.0);
      jointLowerLimits.put(SevenDoFArmJointEnum.elbowPitch, 0.0);
      jointLowerLimits.put(SevenDoFArmJointEnum.wristPitch, -Math.PI);
      jointLowerLimits.put(SevenDoFArmJointEnum.wristRoll, -Math.PI);
      jointLowerLimits.put(SevenDoFArmJointEnum.wristYaw, -Math.PI);

      jointUpperLimits.put(SevenDoFArmJointEnum.shoulderYaw, Math.PI);
      jointUpperLimits.put(SevenDoFArmJointEnum.shoulderRoll, Math.PI / 2.0);
      jointUpperLimits.put(SevenDoFArmJointEnum.shoulderPitch, Math.PI / 2.0);
      jointUpperLimits.put(SevenDoFArmJointEnum.elbowPitch, Math.PI);
      jointUpperLimits.put(SevenDoFArmJointEnum.wristPitch, Math.PI);
      jointUpperLimits.put(SevenDoFArmJointEnum.wristRoll, Math.PI);
      jointUpperLimits.put(SevenDoFArmJointEnum.wristYaw, Math.PI);

      jointChildLinkNames.put(SevenDoFArmJointEnum.shoulderYaw, "shoulderYawLink");
      jointChildLinkNames.put(SevenDoFArmJointEnum.shoulderRoll, "shoulderRollLink");
      jointChildLinkNames.put(SevenDoFArmJointEnum.shoulderPitch, "upperArm");
      jointChildLinkNames.put(SevenDoFArmJointEnum.elbowPitch, "lowerArm");
      jointChildLinkNames.put(SevenDoFArmJointEnum.wristPitch, "wristPitchLink");
      jointChildLinkNames.put(SevenDoFArmJointEnum.wristRoll, "wristRollLink");
      jointChildLinkNames.put(SevenDoFArmJointEnum.wristYaw, "hand");

      jointChildLinkMasses.put(SevenDoFArmJointEnum.shoulderYaw, 1.0e-4);
      jointChildLinkMasses.put(SevenDoFArmJointEnum.shoulderRoll, 1.0e-4);
      jointChildLinkMasses.put(SevenDoFArmJointEnum.shoulderPitch, armLinkMass);
      jointChildLinkMasses.put(SevenDoFArmJointEnum.elbowPitch, armLinkMass);
      jointChildLinkMasses.put(SevenDoFArmJointEnum.wristPitch, 1.0e-4);
      jointChildLinkMasses.put(SevenDoFArmJointEnum.wristRoll, 1.0e-4);
      jointChildLinkMasses.put(SevenDoFArmJointEnum.wristYaw, handMass);

      jointChildLinkCoMs.put(SevenDoFArmJointEnum.shoulderYaw, new Vector3D(0.0, 0.0, 0.0));
      jointChildLinkCoMs.put(SevenDoFArmJointEnum.shoulderRoll, new Vector3D(0.0, 0.0, 0.0));
      jointChildLinkCoMs.put(SevenDoFArmJointEnum.shoulderPitch, armLinkCoM);
      jointChildLinkCoMs.put(SevenDoFArmJointEnum.elbowPitch, armLinkCoM);
      jointChildLinkCoMs.put(SevenDoFArmJointEnum.wristPitch, new Vector3D(0.0, 0.0, 0.0));
      jointChildLinkCoMs.put(SevenDoFArmJointEnum.wristRoll, new Vector3D(0.0, 0.0, 0.0));
      jointChildLinkCoMs.put(SevenDoFArmJointEnum.wristYaw, handCoM);

      jointChildLinkInertias.put(SevenDoFArmJointEnum.shoulderYaw, smallInertia);
      jointChildLinkInertias.put(SevenDoFArmJointEnum.shoulderRoll, smallInertia);
      jointChildLinkInertias.put(SevenDoFArmJointEnum.shoulderPitch, armLinkInertia);
      jointChildLinkInertias.put(SevenDoFArmJointEnum.elbowPitch, armLinkInertia);
      jointChildLinkInertias.put(SevenDoFArmJointEnum.wristPitch, smallInertia);
      jointChildLinkInertias.put(SevenDoFArmJointEnum.wristRoll, smallInertia);
      jointChildLinkInertias.put(SevenDoFArmJointEnum.wristYaw, handInertia);

      jointChildLinkGraphics.put(SevenDoFArmJointEnum.shoulderYaw, emptyGraphics());
      jointChildLinkGraphics.put(SevenDoFArmJointEnum.shoulderRoll, emptyGraphics());
      jointChildLinkGraphics.put(SevenDoFArmJointEnum.shoulderPitch, createArmLinkGraphics(armLinkLength, armLinkRadius, YoAppearance.CadetBlue()));
      jointChildLinkGraphics.put(SevenDoFArmJointEnum.elbowPitch, createArmLinkGraphics(armLinkLength, armLinkRadius, YoAppearance.DarkRed()));
      jointChildLinkGraphics.put(SevenDoFArmJointEnum.wristPitch, emptyGraphics());
      jointChildLinkGraphics.put(SevenDoFArmJointEnum.wristRoll, emptyGraphics());
      jointChildLinkGraphics.put(SevenDoFArmJointEnum.wristYaw, createHandGraphics(YoAppearance.Thistle()));
   }

   private static Matrix3D diagional(double ixx, double iyy, double izz)
   {
      Matrix3D matrix = new Matrix3D();
      matrix.setM00(ixx);
      matrix.setM11(iyy);
      matrix.setM22(izz);
      return matrix;
   }

   public static Graphics3DObject emptyGraphics()
   {
      return new Graphics3DObject();
   }

   public static Graphics3DObject createArmLinkGraphics(double length, double radius, AppearanceDefinition appearance)
   {
      Graphics3DObject armLinkGraphics = new Graphics3DObject();
      armLinkGraphics.addSphere(radius, YoAppearance.Grey());
      armLinkGraphics.addCylinder(length, radius, appearance);
      return armLinkGraphics;
   }

   public static Graphics3DObject createHandGraphics(AppearanceDefinition appearance)
   {
      Graphics3DObject handGraphics = new Graphics3DObject();
      handGraphics.addSphere(0.025, appearance);
      handGraphics.translate(handCoM);
      handGraphics.addEllipsoid(0.04, 0.01, 0.1);
      return handGraphics;
   }
}
