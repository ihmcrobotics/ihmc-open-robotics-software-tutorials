package us.ihmc.robotWalkerFive;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.scs2.definition.collision.CollisionShapeDefinition;
import us.ihmc.scs2.definition.geometry.Box3DDefinition;
import us.ihmc.scs2.definition.geometry.GeometryDefinition;
import us.ihmc.scs2.definition.terrain.TerrainObjectDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinition;

public class FlatGroundDefinition extends TerrainObjectDefinition
{
   public FlatGroundDefinition()
   {
      super();
      // flat ground
      RigidBodyTransform groundPose = new RigidBodyTransform();
      groundPose.getRotation();
      groundPose.appendTranslation(0.0, 0.0, -0.05);

      GeometryDefinition groundGeometryDefinition = new Box3DDefinition(100.0, 100.0, 0.1);
      addVisualDefinition(new VisualDefinition(groundPose, groundGeometryDefinition, new MaterialDefinition(ColorDefinitions.DarkGray())));
      addCollisionShapeDefinition(new CollisionShapeDefinition(groundPose, groundGeometryDefinition));
   }
}
