package us.ihmc.robotWalkerFive;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
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

   /**
    * The current robot pose on the path
    */
   private FramePose3D currentPoseOnPath = new FramePose3D(WORLD_FRAME);

   /**
    * The desired next robot pose on the path
    */
   private FramePose3D desiredNextPoseOnPath = new FramePose3D(WORLD_FRAME);

   /**
    * The desired position offset between the current and the next position on the path
    */
   private FrameVector3D desiredWalkingPositionOffset = new FrameVector3D(WORLD_FRAME);

   public RobotWalkerFootStepPlanner(SideDependentList<RigidBodyBasics> feet, SideDependentList<ReferenceFrame> soleFrames, int nSteps)
   {
      this.feet = feet;
      this.soleFrames = soleFrames;
      this.nStepsToPlan = nSteps;
      this.currentSolePose = new SideDependentList<>(side -> new FramePose3D(soleFrames.get(side)));
   }

   /**
    * The {@code initialize} initializes the footstep planner
    * 
    * @param swingFootSide       swingfootside for the first footstep
    * @param walkingStepWidth    the width of the footsteps
    * @param walkingStrideLength the stride length for walking
    * @param sideWayStepLength   the distance that the robot should move sideways with every footstep
    * @param yawPerStep          the yaw rotation appended to every footstep with respect to the
    *                            previous step
    */
   protected void initialize(RobotSide swingFootSide, double walkingStrideLength, double walkingStepWidth, double rotationPerStep, double sideWayStepLength)
   {
      this.currentFootstepSide = swingFootSide;
      this.walkingStepWidth = walkingStepWidth;
      this.walkingStrideLength = walkingStrideLength;
      this.sideWayStepLength = sideWayStepLength;
      this.yawPerStep = rotationPerStep;
   }

   /**
    * The {@code generateFootsteps} generates a list of future footsteps
    * 
    * @param footStepList list of planned footsteps (modified)
    */

   protected void generateFootsteps(ArrayList<Footstep> footStepListToPack)
   {
      initializeFeetPose();
      updateStepTranslation(walkingStrideLength, sideWayStepLength);

      for (int i = 0; i < nStepsToPlan; i++)
      {
         updateCurrentPoseOnPath(currentPoseOnPath);
         updateDesiredNextPoseOnPath(yawPerStep, currentPoseOnPath, desiredWalkingPositionOffset, desiredNextPoseOnPath);
         addFootStepToList(desiredNextPoseOnPath, footStepListToPack);

         // pretend to take this step and plan the next step
         takeAStep(footStepListToPack.get(i).getFootstepPose());
      }
      footStepListToPack.trimToSize();
   }

   /**
    * The {@code updateStepTranslation} updates the desired translation between each footstep
    */
   protected void updateStepTranslation(double walkingStrideLength, double sideWayStepLength)
   {
      this.desiredWalkingPositionOffset = new FrameVector3D(WORLD_FRAME, walkingStrideLength, sideWayStepLength, 0.0);
   }

   /**
    * The {@code updateDesiredNextPoseOnPath} calculates the next desired pose on the path
    * 
    * @param yawPerStep                   the desired yaw rotation per step along the path
    * @param currentPoseOnPath            hold the current pose on the path
    * @param desiredWalkingPositionOffset is the desired offset between the current pose on the path
    *                                     and the next desired pose on the path
    * @param desiredNextWalkingPoseToPack holds the desired next walking pose on the path
    */
   protected void updateDesiredNextPoseOnPath(double yawPerStep,
                                              FramePose3DReadOnly currentPoseOnPath,
                                              FrameVector3DReadOnly desiredWalkingPositionOffset,
                                              FramePose3DBasics desiredNextWalkingPoseToPack)
   {
      desiredNextWalkingPoseToPack.set(currentPoseOnPath);
      desiredNextWalkingPoseToPack.appendYawRotation(yawPerStep);
      desiredNextWalkingPoseToPack.appendTranslation(desiredWalkingPositionOffset);
   }

   /**
    * The {@code initializeFeetPose} updates the current pose of the robot feet
    */
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

   /**
    * The {@code updateCurrentPoseOnPath} updates the current pose of the robot on the path based on
    * the middle between both feet
    * 
    * @param currentPathPoseToPack will hold the current pose on the path
    */
   protected void updateCurrentPoseOnPath(FramePose3DBasics currentPathPoseToPack)
   {
      currentPathPoseToPack.interpolate(currentSolePose.get(currentFootstepSide), currentSolePose.get(currentFootstepSide.getOppositeSide()), 0.5);
   }

   /**
    * The {@code getCurrentPoseOnPath} returns the current pose on the path
    */
   public FramePose3D getCurrentPoseOnPath()
   {
      return this.currentPoseOnPath;
   }

   /**
    * The {@code getDesiredNextPoseOnPath} returns the desired next pose on the path
    */
   public FramePose3D getDesiredNextPoseOnPath()
   {
      return this.desiredNextPoseOnPath;
   }

   /**
    * The {@code generateDesiredFootstepList} generates and returns an empty footstep list
    */
   public ArrayList<Footstep> generateDesiredFootstepList()
   {
      ArrayList<Footstep> footStepList = new ArrayList<Footstep>();
      generateFootsteps(footStepList);

      return footStepList;
   }

   /**
    * The {@code addFootStepToList} generates and adds a footstep to the footstep list based on the
    * desired next robot pose on the path
    * 
    * @param desiredNextPoseOnPath the desired next robot pose on the path
    * @param footStepList          list of planned footsteps (modified)
    */
   private void addFootStepToList(FramePose3D desiredNextPoseOnPath, ArrayList<Footstep> footStepListToPack)
   {
      FramePose3D nextFootPose = new FramePose3D(WORLD_FRAME, desiredNextPoseOnPath);
      FrameVector3D offset = getFootOffsetFromPath(currentFootstepSide);
      nextFootPose.appendTranslation(offset);

      footStepListToPack.add(new Footstep(currentFootstepSide, nextFootPose));
   }

   /**
    * The {@code takeAStep} virtually takes a step by updating the current sole pose for foot that is
    * stepping and changing the current footstep side to the other side.
    * 
    * @param nextFootPose the desired next pose for the foot
    */
   private void takeAStep(FramePose3D nextFootPose)
   {
      currentSolePose.set(currentFootstepSide, nextFootPose);
      currentFootstepSide = currentFootstepSide.getOppositeSide();
   }

   /**
    * The {@code getFootOffsetFromPath} calculates the offset from the path for the footstep position
    * based on the robot side.
    * 
    * @param currentStepSide the side of the robot that will take the step
    */
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
