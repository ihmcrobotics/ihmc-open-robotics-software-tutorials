package us.ihmc.robotWalkerFive;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;

public class CapturePointTrajectory
{

   private ArrayList<Footstep> plannedFootSteps;
   private final int nStepsToPlan;
   private double omega0 = Math.sqrt(GRAVITY / CENTER_OF_MASS_HEIGHT);
   private static final double CENTER_OF_MASS_HEIGHT = 0.75;
   private static final double GRAVITY = 9.81;
   private double swingDuration;
   private ArrayList<Footstep> virtualRepellentPoints;

   public CapturePointTrajectory(ArrayList<Footstep> plannedFootSteps, double swingDuration)
   {
      this.plannedFootSteps = plannedFootSteps;
      this.swingDuration = swingDuration;
      this.nStepsToPlan = 3;

   }

   void initialize(ArrayList<Footstep> plannedFootSteps, double swingDuration)
   {
      this.plannedFootSteps = plannedFootSteps;
      this.swingDuration = swingDuration;
      this.virtualRepellentPoints = plannedFootSteps; //.forEach(t -> t.addOffset(new FrameVector3D(0.0,0.0,CENTER_OF_MASS_HEIGHT)));
   }

   void compute(double time, FramePoint3D desiredCapturePointPositionToPack, FrameVector3D desiredCaptureVelocityToPack)
   {

      // recursively calculated desired capture point position
      calculateDesiredCapturePointPosition(time, 0, desiredCapturePointPositionToPack, desiredCaptureVelocityToPack);
   }

   void calculateDesiredCapturePointPosition(double time, int stepNumber, FramePoint3D desiredCPPositiontoPack, FrameVector3D desiredCPVelocitytoPack)
   {
      //      FrameVector3D desiredCPVelocity = new FrameVector3D();
      //      FramePoint3D currentDesiredCPPosition = new FramePoint3D();
      
      FramePoint3D endOfStepDesiredCPPosition = new FramePoint3D();

      FramePoint3D virtualRepellentPointPosition = new FramePoint3D();
      virtualRepellentPoints.get(stepNumber).getPosition(virtualRepellentPointPosition);

      if (stepNumber == nStepsToPlan)
      {
         endOfStepDesiredCPPosition.set(virtualRepellentPointPosition);
      }
      else
      {
         // recursion
         FrameVector3D endOfStepCPVelocity = new FrameVector3D();
         calculateDesiredCapturePointPosition(0.0, stepNumber + 1, endOfStepDesiredCPPosition, endOfStepCPVelocity);
      }

      FrameVector3D offset = new FrameVector3D();
      double deltaT = time - swingDuration;
      offset.sub(endOfStepDesiredCPPosition, virtualRepellentPointPosition);
      desiredCPPositiontoPack.scaleAdd(Math.exp(deltaT * omega0), offset, virtualRepellentPointPosition);

      desiredCPVelocitytoPack.set(offset);
      desiredCPVelocitytoPack.scale(omega0 * Math.exp(deltaT * omega0));

   }


   
}
