package us.ihmc.robotWalkerFive;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CapturePointTrajectory
{

   //   private ArrayList<Footstep> plannedFootSteps;
   private final int nStepsToPlan;
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

   public CapturePointTrajectory(ArrayList<Footstep> plannedFootSteps, double swingDuration)
   {
      //      this.plannedFootSteps = plannedFootSteps;
      this.swingDuration = swingDuration;
      this.nStepsToPlan = 3;

   }

   void initialize(ArrayList<Footstep> plannedFootSteps, double swingDuration, double transferDuration)
   {
      //      this.plannedFootSteps = plannedFootSteps;
      this.swingDuration = swingDuration;
      this.doubleSupportDuration = transferDuration;
      this.virtualRepellentPoints = plannedFootSteps; //.forEach(t -> t.addOffset(new FrameVector3D(0.0,0.0,CENTER_OF_MASS_HEIGHT)));
   }

   void compute(double time, FramePoint3D desiredCapturePointPositionToPack, FrameVector3D desiredCaptureVelocityToPack)
   {

      FramePoint3D capturePointIniDS = new FramePoint3D();
      FramePoint3D capturePointEoDSTprevious = new FramePoint3D();
      FrameVector3D capturePointIniDSVelocity = new FrameVector3D();
      FrameVector3D capturePointEoDSVelocity = new FrameVector3D();

      calculateEndOfStepCPDoubleSupport(0, alphaDSini, capturePointEoDSTprevious, capturePointEoDSVelocity);
      calculateInitialCPDoubleSupport(1, alphaDSini, capturePointIniDS, capturePointIniDSVelocity);

      calculateDesiredCPPositionDS(time, 1, capturePointIniDS, capturePointEoDSTprevious, desiredCapturePointPositionToPack);
      calculateDesiredCPVelocityDS(time, 1, capturePointIniDS, capturePointEoDSTprevious, desiredCaptureVelocityToPack);

      this.capturePointEoDST.set(capturePointEoDSTprevious);
      this.capturePointIniDS.set(capturePointIniDS);

   }

   void calculateDesiredCPPositionDS(double time,
                                     int stepNumber,
                                     FramePoint3D capturePointIniDS,
                                     FramePoint3D capturePointEoDSTpreviousStep,
                                     FramePoint3D desiredCPPositiontoPack)
   {

      FrameVector3D offset = new FrameVector3D();
      double deltaT = time - swingDuration;
      offset.sub(capturePointIniDS, capturePointEoDSTpreviousStep);
      desiredCPPositiontoPack.scaleAdd(Math.exp(deltaT * omega0), offset, capturePointEoDSTpreviousStep);

   }
   
   void calculateDesiredCPVelocityDS(double time,
                                     int stepNumber,
                                     FramePoint3D capturePointIniDS,
                                     FramePoint3D capturePointEoDSTpreviousStep,
                                     FrameVector3D desiredCPVelocitytoPack)
   {
      FrameVector3D offset = new FrameVector3D();
      double deltaT = time - swingDuration;
      offset.sub(capturePointIniDS, capturePointEoDSTpreviousStep);

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

   void calculateCapturePointDoubleSupportPhase(int stepNumber, double timeIn, FramePoint3D desiredCPPositionTopack, FrameVector3D desiredCPVelocityToPack)
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

   void getPolynomParameters(int stepNumber, FrameVector3D a0, FrameVector3D a1, FrameVector3D a2, FrameVector3D a3)
   {

      // polynomial parameter matrix P
      double invertedTs = 1.0 / doubleSupportDuration;

      double p11 = (2.0 * invertedTs * invertedTs * invertedTs);
      double p12 = (invertedTs * invertedTs);
      double p13 = (-2.0 * invertedTs * invertedTs * invertedTs);
      double p14 = (invertedTs * invertedTs);
      
      double p21 = (-3.0 * invertedTs * invertedTs);
      double p22 = (-2.0 * invertedTs);
      double p23 = (3.0 * invertedTs * invertedTs);
      double p24 = (-1.0 * invertedTs);

      // coefficents
      FramePoint3D capturePointIniDS = new FramePoint3D();
      FramePoint3D capturePointEoSDS = new FramePoint3D();
      FrameVector3D capturePointIniDSVelocity = new FrameVector3D();
      FrameVector3D capturePointEoDSVelocity = new FrameVector3D();

      calculateInitialCPDoubleSupport(stepNumber, alphaDSini, capturePointIniDS, capturePointIniDSVelocity);
      calculateEndOfStepCPDoubleSupport(stepNumber, alphaDSini, capturePointEoSDS, capturePointEoDSVelocity);

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

   void calculateEndOfStepCPDoubleSupport(int stepNumber, double alphaDS, FramePoint3D capturePointEoSDSToPack, FrameVector3D capturePointEoSDSVelocityToPack)
   {
      FramePoint3D currentVRPPosition = new FramePoint3D();
      virtualRepellentPoints.get(stepNumber).getPosition(currentVRPPosition);
      double deltaTDSend = (1.0 - alphaDS);

      FramePoint3D startOfStepDesiredCPPosition = new FramePoint3D();
      FrameVector3D startOfStepCPVelocity = new FrameVector3D();
      calculateCornerPoint(stepNumber, startOfStepDesiredCPPosition, startOfStepCPVelocity);
      this.startOfStepDesiredCPPosition.set(startOfStepDesiredCPPosition);

      FrameVector3D offset = new FrameVector3D();
      offset.sub(startOfStepDesiredCPPosition, currentVRPPosition);
      capturePointEoSDSToPack.scaleAdd(Math.exp(deltaTDSend * doubleSupportDuration * omega0), offset, currentVRPPosition);
      
//      TODO calculateDesiredCPVelocityDS(0.0, stepNumber+1, capturePointIniDS, capturePointEoSDSToPack, capturePointEoSDSVelocityToPack);

      
      capturePointEoSDSVelocityToPack.set(offset);
      capturePointEoSDSVelocityToPack.scale(deltaTDSend * omega0 * Math.exp(deltaTDSend * doubleSupportDuration * omega0));

   }

   void calculateInitialCPDoubleSupport(int stepNumber, double alphaDS, FramePoint3D capturePointIniDSToPack, FrameVector3D capturePointIniDSVelocityToPack)
   {
      FramePoint3D previousVRPPosition = new FramePoint3D();
      virtualRepellentPoints.get(stepNumber - 1).getPosition(previousVRPPosition);
      double deltaTDini = alphaDS;

      FramePoint3D startOfStepDesiredCPPosition = new FramePoint3D();
      FrameVector3D startOfStepCPVelocity = new FrameVector3D();
      calculateCornerPoint(stepNumber, startOfStepDesiredCPPosition, startOfStepCPVelocity);
      this.startOfStepDesiredCPPosition.set(startOfStepDesiredCPPosition);

      FrameVector3D offset = new FrameVector3D();
      offset.sub(startOfStepDesiredCPPosition, previousVRPPosition);
      capturePointIniDSToPack.scaleAdd(Math.exp(-1.0 * deltaTDini * doubleSupportDuration * omega0), offset, previousVRPPosition);
      capturePointIniDSVelocityToPack.set(offset);
      capturePointIniDSVelocityToPack.scale(-1.0 * deltaTDini * omega0 * Math.exp(-1.0 * deltaTDini * doubleSupportDuration * omega0));
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
