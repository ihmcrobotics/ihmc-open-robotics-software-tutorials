package us.ihmc.robotWalkerFive;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class RobotWalkerFootStepPlanner
{
   protected static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();

   protected SideDependentList<RigidBodyBasics> feet;
   protected SideDependentList<ReferenceFrame> soleFrames;
   protected SideDependentList<FramePose3D> currentSolePose;
   private double walkingStepWidth;
   private double walkingStrideLength;
   private double sideWayStepLength;
   private double yawPerStep;
   private RobotSide currentFootstepSide;
   private int nStepsToPlan;

   private FramePose3D currentPelvisPose = new FramePose3D(WORLD_FRAME);
   private FramePose3D currentPoseOnTrajectory = new FramePose3D(WORLD_FRAME);
   private FramePose3D desiredNextPoseOnTrajectory = new FramePose3D(WORLD_FRAME);
   private FrameVector3D desiredWalkingPositionOffset = new FrameVector3D(WORLD_FRAME);

   public RobotWalkerFootStepPlanner(SideDependentList<RigidBodyBasics> feet, SideDependentList<ReferenceFrame> soleFrames, int nSteps)
   {
      this.feet = feet;
      this.soleFrames = soleFrames;
      this.nStepsToPlan = nSteps;
      this.currentSolePose = new SideDependentList<>(side -> new FramePose3D(soleFrames.get(side)));
   }

   protected void initialize(RobotSide swingFootSide, double walkingStrideLength, double walkingStepWidth, double rotationPerStep, double sideWayStepLength)
   {
      this.currentFootstepSide = swingFootSide;
      this.walkingStepWidth = walkingStepWidth;
      this.walkingStrideLength = walkingStrideLength;
      this.sideWayStepLength = sideWayStepLength;
      this.yawPerStep = rotationPerStep;
   }

   protected void generateFootsteps(ArrayList<Footstep> footStepList)
   {
      initializeFeetPose();
      updateStepTranslation();

      for (int i = 0; i <= nStepsToPlan; i++)
      {
         updateCurrentPoseOnTrajectory();
         updateDesiredNextPoseOnTrajectory();
         
         addFootStepToList(desiredNextPoseOnTrajectory, footStepList);

         // pretend to take this step and plan the next step
         takeAStep(footStepList.get(i).getFootstepPose());
      }
   }

   protected void updatecurrentPoseUsingPelvis()
   {
      this.currentPoseOnTrajectory.set(currentPelvisPose);
   }

   protected void updateCurrentPelvisPose(RigidBodyBasics pelvis)
   {
      FramePose3D pelvisPose = new FramePose3D(pelvis.getBodyFixedFrame());
      pelvisPose.changeFrame(WORLD_FRAME);

      this.currentPelvisPose = pelvisPose;
   }

   protected void updateStepTranslation()
   {
      this.desiredWalkingPositionOffset = new FrameVector3D(WORLD_FRAME, walkingStrideLength, sideWayStepLength, 0.0);
   }

   protected void updateDesiredNextPoseOnTrajectory()
   {
      FramePose3D desiredNextWalkingPose = new FramePose3D(WORLD_FRAME);
      desiredNextWalkingPose.set(currentPoseOnTrajectory);
      desiredNextWalkingPose.appendYawRotation(yawPerStep);
      desiredNextWalkingPose.appendTranslation(desiredWalkingPositionOffset);
      
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
   }

   public FramePose3D getCurrentPoseOnPath()
   {
      return this.currentPoseOnTrajectory;
   }

   public FramePose3D getDesiredNextPoseOnPath()
   {
      return this.desiredNextPoseOnTrajectory;
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

   private void addFootStepToList(FramePose3D desiredNextPoseOnTrajectory, ArrayList<Footstep> footStepList )
   {
      FramePose3D nextFootPose = new FramePose3D(WORLD_FRAME, desiredNextPoseOnTrajectory);
      FrameVector3D offset = getFootOffsetFromPath(currentFootstepSide);
      nextFootPose.appendTranslation(offset);

      Footstep footstep = new Footstep(currentFootstepSide, nextFootPose);
      footStepList.add(footstep);
   }

   private void takeAStep(FramePose3D nextFootPose)
   {
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
      return offset;
   }
}
