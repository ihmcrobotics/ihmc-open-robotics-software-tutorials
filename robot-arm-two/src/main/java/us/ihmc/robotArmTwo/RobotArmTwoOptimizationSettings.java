package us.ihmc.robotArmTwo;

import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;
import us.ihmc.euclid.tuple2D.Vector2D;

/**
 * This class defines only required parameters for the optimization problem on the robot arm.
 */
public class RobotArmTwoOptimizationSettings implements ControllerCoreOptimizationSettings
{
   /** {@inheritDoc} */
   @Override
   public double getJointAccelerationWeight()
   {
      return 0.005;
   }

   /** {@inheritDoc} */
   @Override
   public double getJointJerkWeight()
   {
      return 0.0;
   }

   @Override
   public double getRhoWeight()
   { // This parameter is unused for this example.
      return 0;
   }

   @Override
   public double getRhoMin()
   { // This parameter is unused for this example.
      return 0;
   }

   @Override
   public double getRhoRateDefaultWeight()
   { // This parameter is unused for this example.
      return 0;
   }

   @Override
   public Vector2D getCoPWeight()
   { // This parameter is unused for this example.
      return new Vector2D();
   }

   @Override
   public Vector2D getCoPRateDefaultWeight()
   { // This parameter is unused for this example.
      return new Vector2D();
   }

   @Override
   public int getNumberOfBasisVectorsPerContactPoint()
   { // This parameter is unused for this example.
      return 0;
   }

   @Override
   public int getNumberOfContactPointsPerContactableBody()
   { // This parameter is unused for this example.
      return 0;
   }

   @Override
   public int getNumberOfContactableBodies()
   { // This parameter is unused for this example.
      return 0;
   }
}
