package us.ihmc.robotWalkerFive;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CapturePointTrajectory
{

   private int nStepsToPlan;
   private double omega0 = Math.sqrt(GRAVITY / CENTER_OF_MASS_HEIGHT);
   private static final double CENTER_OF_MASS_HEIGHT = 0.75;
   private static final double GRAVITY = 9.81;
   private double swingDuration;
   private double doubleSupportDuration;
   private ArrayList<Footstep> virtualRepellentPoints;
   FramePoint3D capturePointIniDS = new FramePoint3D();
   FramePoint3D capturePointEoDST = new FramePoint3D();
   FrameVector3D capturePointIniDSVelocity = new FrameVector3D();
   FrameVector3D capturePointEoDSVelocity = new FrameVector3D();
   FramePoint3D startOfStepDesiredCPPosition = new FramePoint3D();
   FramePoint3D previousVRPPosition = new FramePoint3D();
   FramePoint3D currentVRPPosition = new FramePoint3D();
   double alphaDSini = 0.5;

   private final YoRegistry registry = new YoRegistry("Controller");

   YoPolynomial3D cpTrajctoryDS = new YoPolynomial3D("dsTrajectory", 4, registry);

   boolean isfirststep = true;

   public CapturePointTrajectory(ArrayList<Footstep> plannedFootSteps,double swingDuration)
   {
      this.swingDuration = swingDuration;
      this.nStepsToPlan = 0;
    }

   
   void initialize(ArrayList<Footstep> plannedFootSteps, double swingDuration, double transferDuration)
   {
      this.swingDuration = swingDuration;
      this.doubleSupportDuration = transferDuration;
      this.virtualRepellentPoints = plannedFootSteps;
      this.nStepsToPlan = plannedFootSteps.size()-1;
   }

   void computeSingleSupport(double time, FramePoint3D desiredCapturePointPositionToPack, FrameVector3D desiredCaptureVelocityToPack)
   {
      // we are in single leg swing
      FramePoint3D capturePointIniDS = new FramePoint3D();
      FramePoint3D capturePointEoDSTprevious = new FramePoint3D();

      // end of DS  this step - start
      calculateEndOfStepCPDoubleSupport(0, capturePointEoDSTprevious);
      // start of DS for next step - end
      calculateInitialCPDoubleSupport(1, capturePointIniDS);

      // calculate time dependent position and velocity of CP
      calculateDesiredCPPositionSS(time, capturePointEoDSTprevious, capturePointIniDS, desiredCapturePointPositionToPack);
      calculateDesiredCPVelocitySS(time, capturePointEoDSTprevious, capturePointIniDS, desiredCaptureVelocityToPack);

      this.capturePointEoDST.set(capturePointEoDSTprevious);
      this.capturePointIniDS.set(capturePointIniDS);
   }

   void computeDoubleSupport(int stepNumber, double timeIn, FramePoint3D desiredCPPositionTopack, FrameVector3D desiredCPVelocityToPack)
   {
      FrameVector3D a0 = new FrameVector3D();
      FrameVector3D a1 = new FrameVector3D();
      FrameVector3D a2 = new FrameVector3D();
      FrameVector3D a3 = new FrameVector3D();

      getPolynomParameters(stepNumber, a0, a1, a2, a3);

      FramePoint3D desiredCPPosition = new FramePoint3D();
      desiredCPPosition.scaleAdd(timeIn * timeIn * timeIn, a0, desiredCPPosition);
      desiredCPPosition.scaleAdd(timeIn * timeIn, a1, desiredCPPosition);
      desiredCPPosition.scaleAdd(timeIn, a2, desiredCPPosition);
      desiredCPPosition.add(a3);

      FrameVector3D desiredCPVelocity = new FrameVector3D();
      desiredCPVelocity.scaleAdd(3.0 * timeIn * timeIn, a0, desiredCPVelocity);
      desiredCPVelocity.scaleAdd(2.0 * timeIn, a1, desiredCPVelocity);
      desiredCPVelocity.add(a2);

      // send out
      desiredCPPositionTopack.set(desiredCPPosition);
      desiredCPVelocityToPack.set(desiredCPVelocity);
   }

   void calculateDesiredCPPositionSS(double time, FramePoint3D capturePointStart, FramePoint3D capturePointEnd, FramePoint3D desiredCPPositiontoPack)
   {

      FrameVector3D offset = new FrameVector3D();
      double deltaT = time - swingDuration;
      offset.sub(capturePointEnd, capturePointStart);
      desiredCPPositiontoPack.scaleAdd(Math.exp(deltaT * omega0), offset, capturePointStart);

   }

   void calculateDesiredCPVelocitySS(double time, FramePoint3D capturePointStart, FramePoint3D capturePointEnd, FrameVector3D desiredCPVelocitytoPack)
   {
      FrameVector3D offset = new FrameVector3D();
      double deltaT = time - swingDuration;
      offset.sub(capturePointEnd, capturePointStart);

      desiredCPVelocitytoPack.set(offset);
      desiredCPVelocitytoPack.scale(omega0 * Math.exp(deltaT * omega0));

   }

   void calculateCornerPoint(int stepNumber, FramePoint3D desiredCPPositiontoPack, FrameVector3D desiredCPVelocitytoPack)
   {
      double time = 0.0;

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
         calculateCornerPoint(stepNumber + 1, endOfStepDesiredCPPosition, endOfStepCPVelocity);
      }

      FrameVector3D offset = new FrameVector3D();
      double deltaT = time - swingDuration;
      offset.sub(endOfStepDesiredCPPosition, virtualRepellentPointPosition);
      desiredCPPositiontoPack.scaleAdd(Math.exp(deltaT * omega0), offset, virtualRepellentPointPosition);

      desiredCPVelocitytoPack.set(offset);
      desiredCPVelocitytoPack.scale(omega0 * Math.exp(deltaT * omega0));
   }

   void getPolynomParameters(int stepNumber, FrameVector3D a0, FrameVector3D a1, FrameVector3D a2, FrameVector3D a3)
   {

      double invertedTs = 1.0 / doubleSupportDuration;

      double p11 = (2.0 * invertedTs * invertedTs * invertedTs);
      double p12 = (invertedTs * invertedTs);
      double p13 = (-2.0 * invertedTs * invertedTs * invertedTs);
      double p14 = (invertedTs * invertedTs);

      double p21 = (-3.0 * invertedTs * invertedTs);
      double p22 = (-2.0 * invertedTs);
      double p23 = (3.0 * invertedTs * invertedTs);
      double p24 = (-1.0 * invertedTs);

      FramePoint3D capturePointIniDS = new FramePoint3D();
      FramePoint3D capturePointEoSDS = new FramePoint3D();
      FrameVector3D capturePointIniDSVelocity = new FrameVector3D();
      FrameVector3D capturePointEoDSVelocity = new FrameVector3D();

      calculateInitialCPDoubleSupport(stepNumber, capturePointIniDS);
      calculateEndOfStepCPDoubleSupport(stepNumber, capturePointEoSDS);

      calculateInitialCPDoubleSupportVelocity(stepNumber, capturePointIniDSVelocity);
      calculateEndOfStepCPDoubleSupportVelocity(stepNumber, capturePointEoDSVelocity);

      this.capturePointIniDS.set(capturePointIniDS);
      this.capturePointEoDST.set(capturePointEoSDS);

      a0.set(capturePointIniDS);
      a0.scale(p11);
      a0.scaleAdd(p12, capturePointIniDSVelocity, a0);
      a0.scaleAdd(p13, capturePointEoSDS, a0);
      a0.scaleAdd(p14, capturePointEoDSVelocity, a0);

      a1.set(capturePointIniDS);
      a1.scale(p21);
      a1.scaleAdd(p22, capturePointIniDSVelocity, a1);
      a1.scaleAdd(p23, capturePointEoSDS, a1);
      a1.scaleAdd(p24, capturePointEoDSVelocity, a1);

      a2.set(capturePointIniDSVelocity);

      a3.set(capturePointIniDS);

   }

   void calculateEndOfStepCPDoubleSupport(int stepNumber, FramePoint3D capturePointEoSDSToPack)
   {
      FramePoint3D currentVRPPosition = new FramePoint3D();
      virtualRepellentPoints.get(stepNumber).getPosition(currentVRPPosition);
      double deltaTDSend = (1.0 - this.alphaDSini);

      FramePoint3D startOfStepDesiredCPPosition = new FramePoint3D();
      FrameVector3D startOfStepCPVelocity = new FrameVector3D();
      calculateCornerPoint(stepNumber, startOfStepDesiredCPPosition, startOfStepCPVelocity);
      this.startOfStepDesiredCPPosition.set(startOfStepDesiredCPPosition);

      FrameVector3D offset = new FrameVector3D();
      offset.sub(startOfStepDesiredCPPosition, currentVRPPosition);
      capturePointEoSDSToPack.scaleAdd(Math.exp(deltaTDSend * doubleSupportDuration * omega0), offset, currentVRPPosition);
   }

   void calculateEndOfStepCPDoubleSupportVelocity(int stepNumber, FrameVector3D capturePointEoSDSVelocityToPack)
   {
      FramePoint3D startOfTrajectoryPoint = new FramePoint3D();
      FramePoint3D endOfTrajectoryPoint = new FramePoint3D();

      calculateEndOfStepCPDoubleSupport(stepNumber + 1, endOfTrajectoryPoint);
      calculateInitialCPDoubleSupport(stepNumber + 1, startOfTrajectoryPoint);

      calculateDesiredCPVelocitySS(0.0, startOfTrajectoryPoint, endOfTrajectoryPoint, capturePointEoSDSVelocityToPack);

   }

   void calculateInitialCPDoubleSupport(int stepNumber, FramePoint3D capturePointIniDSToPack)
   {
      FramePoint3D previousVRPPosition = new FramePoint3D();
      virtualRepellentPoints.get(stepNumber - 1).getPosition(previousVRPPosition);
      double deltaTDini = this.alphaDSini;

      FramePoint3D startOfStepDesiredCPPosition = new FramePoint3D();
      FrameVector3D startOfStepCPVelocity = new FrameVector3D();
      calculateCornerPoint(stepNumber, startOfStepDesiredCPPosition, startOfStepCPVelocity);
      this.startOfStepDesiredCPPosition.set(startOfStepDesiredCPPosition);

      FrameVector3D offset = new FrameVector3D();
      offset.sub(startOfStepDesiredCPPosition, previousVRPPosition);
      capturePointIniDSToPack.scaleAdd(Math.exp(-1.0 * deltaTDini * doubleSupportDuration * omega0), offset, previousVRPPosition);
   }

   void calculateInitialCPDoubleSupportVelocity(int stepNumber, FrameVector3D capturePointIniDSVelocityToPack)
   {
      FramePoint3D startOfTrajectoryPoint = new FramePoint3D();
      FramePoint3D endOfTrajectoryPoint = new FramePoint3D();

      calculateEndOfStepCPDoubleSupport(stepNumber, endOfTrajectoryPoint);
      calculateInitialCPDoubleSupport(stepNumber, startOfTrajectoryPoint);

      calculateDesiredCPVelocitySS(swingDuration, startOfTrajectoryPoint, endOfTrajectoryPoint, capturePointIniDSVelocityToPack);
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
