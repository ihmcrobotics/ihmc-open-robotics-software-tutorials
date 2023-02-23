package us.ihmc.fallingBrick;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.Cylinder3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.geometry.Sphere3DDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class ClutteredGroundDefinition extends TerrainObjectDefinition
{
   // Defining the terrain
   /*
    * The terrain object definition is used to visualize a static environment and to define its
    * collision model. The collision model is used to resolve contact between the robots and the
    * environment.
    */
   //
   public ClutteredGroundDefinition()
   {
      super();

      // flat ground
      GeometryDefinition groundGeometryDefinition = new Box3DDefinition(10.0, 10.0, 0.1);
      addVisualDefinition(new VisualDefinition(new RigidBodyTransform(), groundGeometryDefinition, new MaterialDefinition(ColorDefinitions.DarkGray())));
      addCollisionShapeDefinition(new CollisionShapeDefinition(new RigidBodyTransform(), groundGeometryDefinition));

      // add some objects to clutter the ground 
      // flat box
      RigidBodyTransform flatBoxPose = new RigidBodyTransform();
      flatBoxPose.appendTranslation(-0.3, -0.3, 0.1);
      GeometryDefinition boxOnGround = new Box3DDefinition(0.6, 0.6, 0.2);
      addVisualDefinition(new VisualDefinition(flatBoxPose, boxOnGround, new MaterialDefinition(ColorDefinitions.DarkGoldenrod())));
      addCollisionShapeDefinition(new CollisionShapeDefinition(flatBoxPose, boxOnGround));

      // 1st tilted box 
      RigidBodyTransform boxPose = new RigidBodyTransform();
      boxPose.getRotation().setToPitchOrientation(Math.toRadians(45));
      boxPose.getTranslation().set(new Vector3D(0.1, 0.2, 0.8));
      GeometryDefinition tiltedBoxOnGround = new Box3DDefinition(0.2, 0.5, 0.80);
      addVisualDefinition(new VisualDefinition(boxPose, tiltedBoxOnGround, new MaterialDefinition(ColorDefinitions.DarkBlue())));
      addCollisionShapeDefinition(new CollisionShapeDefinition(boxPose, tiltedBoxOnGround));

      // 2nd tilted box
      RigidBodyTransform boxPose2 = new RigidBodyTransform();
      boxPose2.getRotation().setToPitchOrientation(Math.toRadians(-45));
      boxPose2.appendYawRotation(Math.toRadians(50));
      boxPose2.getTranslation().set(new Vector3D(-0.3, -0.2, 1.6));
      GeometryDefinition tiltedBoxOnGround2 = new Box3DDefinition(0.2, 0.5, 0.8);
      addVisualDefinition(new VisualDefinition(boxPose2, tiltedBoxOnGround2, new MaterialDefinition(ColorDefinitions.DarkSeaGreen())));
      addCollisionShapeDefinition(new CollisionShapeDefinition(boxPose2, tiltedBoxOnGround2));

      // sphere
      RigidBodyTransform spherePose = new RigidBodyTransform();
      spherePose.appendTranslation(-0.4, 0.4, 0.2);
      GeometryDefinition sphere = new Sphere3DDefinition(0.3);
      addVisualDefinition(new VisualDefinition(spherePose, sphere, new MaterialDefinition(ColorDefinitions.CadetBlue())));
      addCollisionShapeDefinition(new CollisionShapeDefinition(spherePose, sphere));

      // 3rd tilted box
      RigidBodyTransform boxPose3 = new RigidBodyTransform();
      boxPose3.appendTranslation(-1.0, 0.0, 0.3);
      boxPose3.getRotation().setToPitchOrientation(Math.toRadians(-50));
      GeometryDefinition tiltedBox2OnGround = new Box3DDefinition(0.2, 1.0, 0.4);
      addVisualDefinition(new VisualDefinition(boxPose3, tiltedBox2OnGround, new MaterialDefinition(ColorDefinitions.LightCoral())));
      addCollisionShapeDefinition(new CollisionShapeDefinition(boxPose3, tiltedBox2OnGround));

      // cylinder
      RigidBodyTransform cylinderPose = new RigidBodyTransform();
      cylinderPose.appendTranslation(-1.0, 0.0, 0.3);
      cylinderPose.getRotation().setYawPitchRoll(10, 20, -10);
      GeometryDefinition cylinder = new Cylinder3DDefinition(0.5, 0.5);
      addVisualDefinition(new VisualDefinition(cylinderPose, cylinder, new MaterialDefinition(ColorDefinitions.PapayaWhip())));
      addCollisionShapeDefinition(new CollisionShapeDefinition(cylinderPose, cylinder));
   }
}
