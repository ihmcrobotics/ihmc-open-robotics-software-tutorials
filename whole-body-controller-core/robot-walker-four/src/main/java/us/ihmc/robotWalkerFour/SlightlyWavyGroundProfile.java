package us.ihmc.robotWalkerFour;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.jMonkeyEngineToolkit.HeightMapWithNormals;

public class SlightlyWavyGroundProfile implements GroundProfile3D, HeightMapWithNormals
{
   private double xMin = -10.0, xMax = 10.0, yMin = -10.0, yMax = 10.0, zMin = -10.0, zMax = 10.0;

   private BoundingBox3D boundingBox = new BoundingBox3D(new Point3D(xMin, yMin, zMin), new Point3D(xMax, yMax, zMax));

   public SlightlyWavyGroundProfile()
   {
   }

   @Override
   public double heightAndNormalAt(double x, double y, double z, Vector3DBasics normalToPack)
   {
      double heightAt = heightAt(x, y, z);
      surfaceNormalAt(x, y, heightAt, normalToPack);
      return heightAt;
   }

   @Override
   public double heightAt(double x, double y, double z)
   {
      // if in bounds, this is the point where the height of the ground is defined
      // One can change the profile of the terrain by changing this function
      if ((x > xMin) && (x < xMax) && (y > yMin) && (y < yMax))
         return -0.05 * Math.sin(x*5)+0.02;
      else
         return 0.0;
   }

   public void surfaceNormalAt(double x, double y, double z, Vector3DBasics normal)
   {
      // This should normally be the gradient function defined in heightAt(). By not doing so, the ground contact model will not 
      // apply the right set of stiffness and damping along the right axis, but overall the behavior would not change that much 
      // if properly expressed. In this example, it doesn't matter too much so we have set it as such.
      normal.setX(0.0);
      normal.setY(0.0);
      normal.setZ(1.0);
   }

   public void closestIntersectionTo(double x, double y, double z, Point3DBasics point)
   {
      point.setX(x);
      point.setY(y);
      point.setZ(heightAt(x, y, z));
   }

   public void closestIntersectionAndNormalAt(double x, double y, double z, Point3DBasics point, Vector3DBasics normal)
   {
      closestIntersectionTo(x, y, z, point);
      surfaceNormalAt(x, y, z, normal);
   }

   @Override
   public boolean checkIfInside(double x, double y, double z, Point3DBasics intersectionToPack, Vector3DBasics normalToPack)
   {
      closestIntersectionTo(x, y, z, intersectionToPack);
      surfaceNormalAt(x, y, z, normalToPack);

      return (z < intersectionToPack.getZ());
   }

   @Override
   public boolean isClose(double x, double y, double z)
   {
      return boundingBox.isInsideInclusive(x, y, z);
   }

   @Override
   public BoundingBox3D getBoundingBox()
   {
      return boundingBox;
   }

   @Override
   public HeightMapWithNormals getHeightMapIfAvailable()
   {
      return this;
   }
}
