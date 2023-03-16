package us.ihmc.robotWalkerFive;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RobotWalkerFootStepPlanner
{
   protected static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   protected SideDependentList<RigidBodyBasics> feet;
   protected SideDependentList<ReferenceFrame> soleFrames;

   protected SideDependentList<FramePose3D> currentSolePose;
   final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private double walkingStepWidth;
   private double walkingStepLength;

   private FramePose3D currentPoseOnTrajectory = new FramePose3D(WORLD_FRAME);
   private FramePose3D desiredNextPoseOnTrajectory = new FramePose3D(WORLD_FRAME);
   private RigidBodyTransform tranformPerStep = new RigidBodyTransform();
   private double yawPerStep;
   private double footDistanceX;
   private double footDistanceY;
   private RobotSide currentFootstepSide;
   private int nStepsToPlan;

   public RobotWalkerFootStepPlanner(SideDependentList<RigidBodyBasics> feet, SideDependentList<ReferenceFrame> soleFrames, int nSteps)
   {
      this.feet = feet;
      this.soleFrames = soleFrames;
      this.nStepsToPlan = nSteps;
      this.currentSolePose = new SideDependentList<>(side -> new FramePose3D(soleFrames.get(side)));
   }

   protected void initialize(RobotSide swingFootSide, double walkingStepLength, double walkingStepWidth, double rotationPerStep)
   {
      this.currentFootstepSide = swingFootSide;
      this.walkingStepWidth = walkingStepWidth;
      this.walkingStepLength = walkingStepLength;
      this.yawPerStep = rotationPerStep;
   }

   protected void generateFootsteps(ArrayList<Footstep> footStepList)
   {

      // get current feet position
      initializeFeetPose();

      for (int i = 0; i <= nStepsToPlan; i++)
      {
         updateCurrentPoseOnTrajectory();
         // calculate next middle foot point on trajectory
         updateDesiredNextPoseOnTrajectory();

         // add footsetp for position on the trajectory
         addFootStep(footStepList);

      }
   }

   protected void updatePerStepTransform()
   {
      // TODO: correct foot for distance between feet
      double stepLength = walkingStepLength; // + 0.5 * Math.abs(footDistanceX);
      double stepWidth = 0.0;
      FrameVector3D desiredWalkingPositionOffset = new FrameVector3D(WORLD_FRAME, stepLength, stepWidth, 0.0);

      RigidBodyTransform walkingDirectionTransform = new RigidBodyTransform();
      walkingDirectionTransform.appendTranslation(desiredWalkingPositionOffset);
      walkingDirectionTransform.appendYawRotation(yawPerStep);

      this.tranformPerStep = walkingDirectionTransform;
   }

   protected void updateDesiredNextPoseOnTrajectory()
   {
      updatePerStepTransform();
      FramePose3D desiredNextWalkingPose = new FramePose3D(WORLD_FRAME);
      desiredNextWalkingPose.set(currentPoseOnTrajectory);
      desiredNextWalkingPose.applyTransform(this.tranformPerStep);
      this.desiredNextPoseOnTrajectory.set(desiredNextWalkingPose);
   }

   protected void initializeFeetPose()
   {
      // Here we get the position of both feet to compute the middle.
      FramePose3D leftSolePose = new FramePose3D(soleFrames.get(RobotSide.LEFT));
      leftSolePose.changeFrame(WORLD_FRAME);
      FramePose3D rightSolePose = new FramePose3D(soleFrames.get(RobotSide.RIGHT));
      rightSolePose.changeFrame(WORLD_FRAME);

      this.currentSolePose.set(RobotSide.LEFT, leftSolePose);
      this.currentSolePose.set(RobotSide.RIGHT, rightSolePose);

   }

   protected void updateCurrentPoseOnTrajectory()
   {
      // Here we get the position of both feet to compute the middle.
      FramePose3D currentTrajectoryPose = new FramePose3D();
      currentTrajectoryPose.interpolate(currentSolePose.get(currentFootstepSide), currentSolePose.get(currentFootstepSide.getOppositeSide()), 0.5);
      this.currentPoseOnTrajectory = currentTrajectoryPose;

      updateFeetDistance();
   }

   protected void updateFeetDistance()
   {
      FramePose3D rightSolePose = new FramePose3D(currentSolePose.get(RobotSide.RIGHT));

      // calculate distance between feet
      rightSolePose.changeFrame(currentSolePose.get(RobotSide.LEFT).getReferenceFrame());
      footDistanceX = rightSolePose.getX();
      footDistanceY = rightSolePose.getY();
   }

   // TODO check if this is ok
   public FramePose3D getCurrentPoseOnPath()
   {
      return this.currentPoseOnTrajectory;
   }

   public FramePose3D getDesiredNextPoseOnPath()
   {
      return this.desiredNextPoseOnTrajectory;
   }

   public void updateRotationPerStep(double yaw)
   {
      this.yawPerStep = yaw;
   }

   public void updateStepWidth(double stepWidth)
   {
      this.walkingStepWidth = stepWidth;
   }

   public void updateStepLength(double stepLength)
   {
      this.walkingStepLength = stepLength;
   }

   public ArrayList<Footstep> generateDesiredFootstepList()
   {
      ArrayList<Footstep> footStepList = new ArrayList<Footstep>();
      generateFootsteps(footStepList);

      return footStepList;
   }

   protected void setStartFootSide(RobotSide startFootSide)
   {
      currentFootstepSide = startFootSide;
   }

   private void addFootStep(ArrayList<Footstep> footStepList)
   {
      FrameVector3D offset = getFootOffsetFromPath(currentFootstepSide);
      FramePose3D nextFootPose = new FramePose3D(WORLD_FRAME, this.desiredNextPoseOnTrajectory);
      nextFootPose.appendTranslation(offset);

      Footstep footstep = new Footstep(currentFootstepSide, nextFootPose);
      footStepList.add(footstep);

      currentSolePose.set(currentFootstepSide, nextFootPose);
      currentFootstepSide = currentFootstepSide.getOppositeSide();

   }

   protected FrameVector3D getFootOffsetFromPath(RobotSide currentStepSide)
   {
      FrameVector3D offset = new FrameVector3D(WORLD_FRAME, 0.0, 0.0, 0.0);

      if (currentStepSide.equals(RobotSide.RIGHT))
      {
         offset.setY(-0.5 * walkingStepWidth);
      }
      else
      {
         offset.setY(0.5 * walkingStepWidth);
      }

      RigidBodyTransform footOffsetTransform = new RigidBodyTransform();
      footOffsetTransform.appendOrientation(tranformPerStep.getRotation());
      offset.applyTransform(footOffsetTransform);
      return offset;
   }

}
