package us.ihmc.robotWalkerFive;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

/**
 * This class allows to calculate a capture point trajectory based on planned footsteps. The method
 * is based on Englsberger et al. (www.doi.org/10.1109/TRO.2015.2405592)
 */
public class CapturePointTrajectory
{

   private static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();

   /** The number of steps to be considered when planning the capture point trajectory. */
   private final int nStepsToPlan;

   /** Parameter for inverted pendulum model: Math.sqrt(9.81 / CENTER_OF_MASS_HEIGHT) */
   private double omega0;

   /** Parameter for double support duration according to Englsberger et al. */
   private final double alphaDSini = 0.5;

   /** Duration of leg swing. */
   private double swingDuration;

   /** Duration of double support. */
   private double doubleSupportDuration;

   /** List of planned footsteps (= virtual repellent points). */
   private ArrayList<Footstep> virtualRepellentPoints;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoGraphicDefinition graphicsGroup;

   private YoFramePoint3D capturePointIniDS = new YoFramePoint3D("capturePointIniDS", WORLD_FRAME, registry);
   private YoFramePoint3D capturePointEoDST = new YoFramePoint3D("capturePointEoDST", WORLD_FRAME, registry);
   private YoFramePoint3D startOfStepDesiredCPPosition = new YoFramePoint3D("startOfStepDesiredCPPosition", WORLD_FRAME, registry);
   private YoFramePoint3D previousVRPPosition = new YoFramePoint3D("previousVRPPosition", WORLD_FRAME, registry);
   private YoFramePoint3D currentVRPPosition = new YoFramePoint3D("currentVRPPosition", WORLD_FRAME, registry);
   private YoFrameVector3D capturePointEoDSVelocity = new YoFrameVector3D("capturePointEoDSVelocity", WORLD_FRAME, registry);
   private YoFrameVector3D capturePointIniDSVelocity = new YoFrameVector3D("capturePointInitDSVelocity", WORLD_FRAME, registry);

   private int index;
   private final ArrayList<YoFramePoint3D> yoGraphicPositions = new ArrayList<>();
   private final YoInteger numberOfControlTicksPerVizUpdate = new YoInteger("numberOfControlTicksPerVizUpdate", registry);

   public CapturePointTrajectory(double omega0, YoRegistry parentRegistry, int nStepsToPlan)
   {
      this.nStepsToPlan = nStepsToPlan;
      this.graphicsGroup = createVisualization();
      this.omega0 = omega0;

      parentRegistry.addChild(registry);
   }

   void initialize(ArrayList<Footstep> plannedFootSteps, double swingDuration, double transferDuration)
   {
      this.swingDuration = swingDuration;
      this.doubleSupportDuration = transferDuration;
      this.virtualRepellentPoints = plannedFootSteps;

      // Create "bag-of-balls" to visualize the capture point trajectory
      index = 0;
      numberOfControlTicksPerVizUpdate.set(300);
   }

   /**
    * The {@code computeDesiredCapturePointSingleSupport} calculates the desired capture point position
    * and velocity for a time point in single support. Calculations are based on the paper of
    * Englsberger et al. (www.doi.org/10.1109/TRO.2015.2405592)
    * 
    * @param stepNumber                        number of current step in the step plan
    * @param timeIn                            refers to the current time in single support
    * @param desiredCapturePointPositionToPack holds the calculated capture point position for the time
    *                                          (modified)
    * @param desiredCaptureVelocityToPack      holds the calculated capture point velocity for the time
    *                                          (modified)
    */
   void calculateDesiredCapturePointSingleSupport(int stepNumber,
                                                  double timeIn,
                                                  FramePoint3DBasics desiredCapturePointPositionToPack,
                                                  FrameVector3DBasics desiredCaptureVelocityToPack)
   {
      FramePoint3D capturePointEoDSTpreviousStep = new FramePoint3D();
      FramePoint3D capturePointIniDSnextStep = new FramePoint3D();

      // end of double support for this step 
      calculateEndOfStepDoubleSupportCapturePointPosition(stepNumber, capturePointEoDSTpreviousStep);
      // start of double support for next step 
      calculateInitialDoubleSupportCapturePointPosition(stepNumber + 1, capturePointIniDSnextStep);

      // calculate time dependent position and velocity of CP
      calculateDesiredCapturePointPositionSingleSupport(timeIn, capturePointEoDSTpreviousStep, capturePointIniDSnextStep, desiredCapturePointPositionToPack);
      calculateDesiredCapturePointVelocitySingleSupport(timeIn, capturePointEoDSTpreviousStep, capturePointIniDSnextStep, desiredCaptureVelocityToPack);

      // visualization
      this.capturePointEoDST.set(capturePointEoDSTpreviousStep);
      this.capturePointIniDS.set(capturePointIniDSnextStep);
   }

   /**
    * The {@code computeDesiredCapturePointDoubleSupport} calculates the desired capture point position
    * and velocity for a time point in double support. Calculations are based on the paper of
    * Englsberger et al. (www.doi.org/10.1109/TRO.2015.2405592)
    * 
    * @param stepNumber                        number of current step in the step plan
    * @param timeIn                            refers to the current time in double support
    * @param desiredCapturePointPositionToPack holds the calculated capture point position for the time
    *                                          (modified)
    * @param desiredCaptureVelocityToPack      holds the calculated capture point velocity for the time
    *                                          (modified)
    */
   void calculateDesiredCapturePointDoubleSupport(int stepNumber,
                                                  double timeIn,
                                                  FramePoint3DBasics desiredCapturePointPositionToPack,
                                                  FrameVector3DBasics desiredCaptureVelocityToPack)
   {
      // TODO check this
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

      desiredCapturePointPositionToPack.set(desiredCPPosition);
      desiredCaptureVelocityToPack.set(desiredCPVelocity);
   }

   /**
    * The {@code calculateDesiredCPPositionSingleSupport} calculates the desired capture point position
    * in single support. Calculations are based on the paper of Englsberger et al.
    * (www.doi.org/10.1109/TRO.2015.2405592)
    * 
    * @param timeIn                  refers to the current time in the single support phase
    * @param capturePointStart       refers to the desired capture point position at the start of the
    *                                single support phase
    * @param capturePointEnd         refers to the desired capture point position at the end of the
    *                                single support phase
    * @param desiredCPPositiontoPack holds the calculated capture point position for the time in the
    *                                single support phase (modified)
    */
   void calculateDesiredCapturePointPositionSingleSupport(double timeIn,
                                                          FramePoint3DReadOnly capturePointPositionStartSS,
                                                          FramePoint3DReadOnly capturePointPositionEndSS,
                                                          FramePoint3DBasics desiredCPPositiontoPack)
   {
      FrameVector3D offset = new FrameVector3D();
      offset.sub(capturePointPositionEndSS, capturePointPositionStartSS);
      desiredCPPositiontoPack.scaleAdd(Math.exp((timeIn - swingDuration) * omega0), offset, capturePointPositionStartSS);
   }

   /**
    * The {@code calculateDesiredCPVelocitySingleSupport} calculates the desired capture point velocity
    * in single support. Calculations are based on the paper of Englsberger et al.
    * (www.doi.org/10.1109/TRO.2015.2405592)
    * 
    * @param timeIn                  refers to the current time in the single support phase
    * @param capturePointStart       refers to the desired capture point position at the start of the
    *                                single support phase
    * @param capturePointEnd         refers to the desired capture point position at the end of the
    *                                single support phase
    * @param desiredCPVelocitytoPack holds the calculated capture point velocity for the time in the
    *                                single support phase (modified)
    */
   void calculateDesiredCapturePointVelocitySingleSupport(double time,
                                                          FramePoint3DReadOnly capturePointPositionStartSingleSupport,
                                                          FramePoint3DReadOnly capturePointPositionEndSingleSupport,
                                                          FrameVector3DBasics desiredCPVelocitytoPack)
   {
      FrameVector3D offset = new FrameVector3D();
      offset.sub(capturePointPositionEndSingleSupport, capturePointPositionStartSingleSupport);
      desiredCPVelocitytoPack.set(offset);
      desiredCPVelocitytoPack.scale(omega0 * Math.exp((time - swingDuration) * omega0));
   }

   /**
    * The {@code calculateCornerPoint} calculates the corner points for the capture point trajectory
    * based on the planned footsteps (= virtual repellent points). Calculations are based on the paper
    * of Englsberger et al. (www.doi.org/10.1109/TRO.2015.2405592)
    * 
    * @param stepNumber              number of current step in the step plan
    * @param desiredCPPositiontoPack holds the calculated capture point corner position (modified)
    * @param desiredCPVelocitytoPack holds the calculated capture point corner velocity (modified)
    */
   void calculateCornerPoint(int stepNumber, FramePoint3DBasics desiredCPPositiontoPack, FrameVector3DBasics desiredCPVelocitytoPack)
   {
      FramePoint3D endOfStepDesiredCapturePointPosition = new FramePoint3D();
      FramePoint3D virtualRepellentPointPosition = new FramePoint3D();
      virtualRepellentPoints.get(stepNumber).getPosition(virtualRepellentPointPosition);

      if (stepNumber == nStepsToPlan)
      {
         endOfStepDesiredCapturePointPosition.set(virtualRepellentPointPosition);
      }
      else
      {
         FrameVector3D endOfStepCPVelocity = new FrameVector3D();
         calculateCornerPoint(stepNumber + 1, endOfStepDesiredCapturePointPosition, endOfStepCPVelocity);
      }

      FrameVector3D offset = new FrameVector3D();
      double deltaT = 0.0 - swingDuration;
      offset.sub(endOfStepDesiredCapturePointPosition, virtualRepellentPointPosition);
      desiredCPPositiontoPack.scaleAdd(Math.exp(deltaT * omega0), offset, virtualRepellentPointPosition);

      desiredCPVelocitytoPack.set(offset);
      desiredCPVelocitytoPack.scale(omega0 * Math.exp(deltaT * omega0));

      // visualize the capture point corner points 
      if (index >= yoGraphicPositions.size())
      {
         index = 0;
      }
      yoGraphicPositions.get(index).set(desiredCPPositiontoPack);
      yoGraphicPositions.get(index).setZ(0.0);
      ;
      index++;
   }

   /**
    * The {@code getPolynomParameters} calculates the polynomial parameters for the polynomial
    * interpolation of the capture point trajectory (for not instantaneous transition). Calculations
    * are based on the paper of Englsberger et al. (www.doi.org/10.1109/TRO.2015.2405592)
    * 
    * @param stepNumber number of current step in the step plan
    * @param a0         holds the calculated polynomial parameter a0
    * @param a1         holds the calculated polynomial parameter a1
    * @param a2         holds the calculated polynomial parameter a2
    * @param a3         holds the calculated polynomial parameter a3
    */
   void getPolynomParameters(int stepNumber, FrameVector3DBasics a0, FrameVector3DBasics a1, FrameVector3DBasics a2, FrameVector3DBasics a3)
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
      FramePoint3D capturePointIniDSnext = new FramePoint3D();
      FramePoint3D capturePointEoSDS = new FramePoint3D();
      FramePoint3D capturePointEoSDSprevious = new FramePoint3D();
      FrameVector3D capturePointIniDSVelocity = new FrameVector3D();
      FrameVector3D capturePointEoDSVelocity = new FrameVector3D();

      calculateInitialDoubleSupportCapturePointPosition(stepNumber + 1, capturePointIniDSnext);
      calculateEndOfStepDoubleSupportCapturePointPosition(stepNumber - 1, capturePointEoSDSprevious);

      // positions
      calculateInitialDoubleSupportCapturePointPosition(stepNumber, capturePointIniDS);
      calculateEndOfStepDoubleSupportCapturePointPosition(stepNumber, capturePointEoSDS);

      // velocity for start of double support = velocity of end of previous single support
      calculateDesiredCapturePointVelocitySingleSupport(swingDuration, capturePointEoSDSprevious, capturePointIniDS, capturePointIniDSVelocity);

      // velocity for end of double support = velocity of start of next single support phase
      calculateDesiredCapturePointVelocitySingleSupport(0.0, capturePointEoSDS, capturePointIniDSnext, capturePointEoDSVelocity);

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

      // TODO check why end of double support velocity seems not to be considered for polynomial
      this.startOfStepDesiredCPPosition.set(capturePointIniDS);
      this.capturePointEoDST.set(capturePointEoSDS);

      this.capturePointEoDSVelocity.set(capturePointEoDSVelocity);
      this.capturePointEoDSVelocity.normalize();
      this.capturePointEoDSVelocity.scale(0.1);

      this.capturePointIniDSVelocity.set(capturePointIniDSVelocity);
      this.capturePointIniDSVelocity.normalize();
      this.capturePointIniDSVelocity.scale(0.1);

   }

   /**
    * The {@code calculateEndOfStepDoubleSupportCapturePointPosition} calculates the desired capture
    * point position at the end of the double support phase. Calculations are based on the paper of
    * Englsberger et al. (www.doi.org/10.1109/TRO.2015.2405592)
    * 
    * @param stepNumber              number of current step in the step plan
    * @param capturePointEoSDSToPack holds the calculated capture point position at the end of the
    *                                double support phase (modified)
    */
   void calculateEndOfStepDoubleSupportCapturePointPosition(int stepNumber, FramePoint3DBasics capturePointEoSDSToPack)
   {
      FramePoint3D currentVRPPosition = new FramePoint3D();
      virtualRepellentPoints.get(stepNumber).getPosition(currentVRPPosition);
      double deltaTDSend = (1.0 - this.alphaDSini);

      FramePoint3D startOfStepDesiredCPPosition = new FramePoint3D();
      FrameVector3D startOfStepCPVelocity = new FrameVector3D();
      calculateCornerPoint(stepNumber, startOfStepDesiredCPPosition, startOfStepCPVelocity);

      FrameVector3D offset = new FrameVector3D();
      offset.sub(startOfStepDesiredCPPosition, currentVRPPosition);
      capturePointEoSDSToPack.scaleAdd(Math.exp(deltaTDSend * doubleSupportDuration * omega0), offset, currentVRPPosition);

      this.startOfStepDesiredCPPosition.set(startOfStepDesiredCPPosition);
      this.capturePointEoDST.set(capturePointEoSDSToPack);
   }

   /**
    * The {@code calculateInitialDoubleSupportCapturePointPosition} calculates the capture point
    * position at the start of the double support phase. Calculations are based on the paper of
    * Englsberger et al. (www.doi.org/10.1109/TRO.2015.2405592)
    * 
    * @param stepNumber              number of current step in the step plan
    * @param capturePointIniDSToPack holds the calculated capture point position at the start of the
    *                                double support phase (modified)
    */
   void calculateInitialDoubleSupportCapturePointPosition(int stepNumber, FramePoint3DBasics capturePointIniDSToPack)
   {
      FramePoint3D previousVRPPosition = new FramePoint3D();
      virtualRepellentPoints.get(stepNumber - 1).getPosition(previousVRPPosition);
      double deltaTDini = this.alphaDSini;

      FramePoint3D startOfStepDesiredCPPosition = new FramePoint3D();
      FrameVector3D startOfStepCPVelocity = new FrameVector3D();
      calculateCornerPoint(stepNumber, startOfStepDesiredCPPosition, startOfStepCPVelocity);

      FrameVector3D offset = new FrameVector3D();
      offset.sub(startOfStepDesiredCPPosition, previousVRPPosition);
      capturePointIniDSToPack.scaleAdd(Math.exp(-1.0 * deltaTDini * doubleSupportDuration * omega0), offset, previousVRPPosition);

      this.startOfStepDesiredCPPosition.set(startOfStepDesiredCPPosition);
      this.capturePointIniDS.set(capturePointIniDSToPack);
   }

   /**
    * The {@code createVisualization} creates visualizations for the planned capture point trajectory.
    */
   public YoGraphicDefinition createVisualization()
   {
      YoGraphicGroupDefinition graphicsGroup = new YoGraphicGroupDefinition("CapturePointTrajectoryPlanner");

      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("capturePointIniDS", capturePointIniDS, 0.009, ColorDefinitions.Green()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("capturePointEoDST", capturePointEoDST, 0.009, ColorDefinitions.Yellow()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("startOfStepDesiredCPPosition",
                                                                            startOfStepDesiredCPPosition,
                                                                            0.009,
                                                                            ColorDefinitions.Crimson()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("previousVRPPosition", previousVRPPosition, 0.009, ColorDefinitions.LightPink()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D("currentVRPPosition", currentVRPPosition, 0.009, ColorDefinitions.HotPink()));
      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D("endofDSVelocity",
                                                                            capturePointEoDST,
                                                                            capturePointEoDSVelocity,
                                                                            0.5,
                                                                            ColorDefinitions.Yellow()));
      //TODO 
      //      graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D("startofDSVelocity",
      //                                                                            capturePointIniDS,
      //                                                                            capturePointIniDSVelocity,
      //                                                                            0.5,
      //                                                                            ColorDefinitions.Green()));

      // visualize the capture point corner point on the path
      String name = "capturePointCornerPoints";
      YoRegistry cpCornerPoints = new YoRegistry("capturePointCornerPoints");

      int numberOfBalls = nStepsToPlan;
      double ballSize = 0.008;
      for (int i = 0; i <= numberOfBalls; i++)
      {
         YoFramePoint3D yoFramePoint = new YoFramePoint3D(name + i, "", WORLD_FRAME, cpCornerPoints);
         yoFramePoint.setToNaN();
         yoGraphicPositions.add(yoFramePoint);

         graphicsGroup.addChild(YoGraphicDefinitionFactory.newYoGraphicPoint3D(name + i, yoFramePoint, ballSize, ColorDefinitions.DarkViolet()));
      }
      registry.addChild(cpCornerPoints);

      return graphicsGroup;
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public YoGraphicDefinition getYoGraphicDefinition()
   {
      return graphicsGroup;
   }
}
