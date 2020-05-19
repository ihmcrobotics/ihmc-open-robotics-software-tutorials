package us.ihmc.robotWalkerFour;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactablePlaneBody;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.simulatedSensors.InverseDynamicsJointsFromSCSRobotGenerator;
import us.ihmc.sensorProcessing.simulatedSensors.SCSToInverseDynamicsJointMap;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * In this class, we use a copy of the M2 robot model. M2 was a bipedal robot developed at the MIT
 * Leg Lab: <a href="http://www.ai.mit.edu/projects/leglab/robots/robots.html">website</a>.
 */
public class RobotWalkerFour
{
   private static final ReferenceFrame WORLD_FRAME = ReferenceFrame.getWorldFrame();
   /**
    * This is the M2Robot definition that we will as a the simulated robot here.
    */
   private final M2Robot simulatedM2Robot = new M2Robot();
   /**
    * As in the previous examples, we need this generator to create an inverse dynamics robot model
    * that the whole-body controller core can work with.
    */
   private final InverseDynamicsJointsFromSCSRobotGenerator inverseDynamicsRobot;
   /**
    * A mapping to easily retrieve joints in the inverse dynamics robot given a joint from the
    * simulated robot.
    */
   private final SCSToInverseDynamicsJointMap jointMap;
   /**
    * In this example, the center of mass reference frame will become handy as it allows to control
    * the robot's center of mass position.
    */
   private final ReferenceFrame centerOfMassFrame;
   /**
    * We will define additional reference frames located at the center of the foot soles. This will
    * allow to use them as reference when positioning the center of mass and will also be used as
    * control frames during the swing phase.
    */
   private final SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
   /**
    * A {@code ContactablePlaneBody} allows to attach to a rigid-body a list of points that can be
    * used to distribute the ground reaction forces on the foot when it is in support.
    */
   private final SideDependentList<ContactablePlaneBody> footContactableBodies = new SideDependentList<>();

   public RobotWalkerFour()
   {
      inverseDynamicsRobot = new InverseDynamicsJointsFromSCSRobotGenerator(simulatedM2Robot);
      jointMap = inverseDynamicsRobot.getSCSToInverseDynamicsJointMap();
      centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", WORLD_FRAME, inverseDynamicsRobot.getElevator());

      for (RobotSide robotSide : RobotSide.values)
      {
         double footWidth = M2Robot.FOOT_WIDTH;
         double footLength = M2Robot.FOOT_LENGTH;
         double footBack = M2Robot.FOOT_BACK;

         RigidBody foot = getFoot(robotSide);

         /*
          * Let's first create a reference frame located at the bottom of the foot.
          */
         ReferenceFrame frameAfterAnkleJoint = foot.getParentJoint().getFrameAfterJoint();
         String soleFrameName = robotSide.getCamelCaseName() + "SoleFrame";
         RigidBodyTransform transformToAnkle = new RigidBodyTransform();
         transformToAnkle.setTranslationZ(-M2Robot.FOOT_HEIGHT); // Offset to be at the bottom of the foot.
         transformToAnkle.setTranslationX(-footBack + 0.5 * footLength); // Offset to center the frame in the middle of the sole.
         ReferenceFrame soleFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent(soleFrameName, frameAfterAnkleJoint, transformToAnkle);
         soleFrames.put(robotSide, soleFrame);

         /*
          * Now we define the corner points of the foot. They can be used to apply forces on the
          * ground.
          */
         List<Point2D> contactPoints = new ArrayList<>();
         contactPoints.add(new Point2D(0.5 * footLength, -0.5 * footWidth));
         contactPoints.add(new Point2D(0.5 * footLength, 0.5 * footWidth));
         contactPoints.add(new Point2D(-0.5 * footLength, -0.5 * footWidth));
         contactPoints.add(new Point2D(-0.5 * footLength, 0.5 * footWidth));
         ContactablePlaneBody footContactableBody = new ListOfPointsContactablePlaneBody(foot, soleFrame, contactPoints);
         footContactableBodies.put(robotSide, footContactableBody);
      }
   }

   /**
    * Updates the state of the inverse dynamics robot model based on the state of the simulated
    * robot.
    */
   public void updateInverseDynamicsRobotState()
   {
      inverseDynamicsRobot.updateInverseDynamicsRobotModelFromRobot(true);
      centerOfMassFrame.update();
   }

   /**
    * Gets the variable holding the current simulation time.
    * 
    * @return the yo-time.
    */
   public YoDouble getYoTime()
   {
      return simulatedM2Robot.getYoTime();
   }

   /**
    * Gets the robot that will be used for the simulation.
    * 
    * @return the simulated robot.
    */
   public Robot getSimulatedRobot()
   {
      return simulatedM2Robot;
   }

   /**
    * Gets the root body of this robot.
    * <p>
    * The elevator is a massless, sizeless rigid-body fixed in world to which the first joint of the
    * robot is attached. The name comes from the use of this rigid-body to add the gravity effect to
    * the robot by making it accelerate like an elevator when it starts moving. However, this
    * elevator is always fixed in world with no velocity.
    * </p>
    * 
    * @return the elevator.
    */
   public RigidBody getElevator()
   {
      return inverseDynamicsRobot.getElevator();
   }

   /**
    * Gets the reference frame which is origin is located at the robot's center of mass.
    * <p>
    * Its axes are aligned with the world frame.
    * </p>
    * 
    * @return the center of mass reference frame.
    */
   public ReferenceFrame getCenterOfMassFrame()
   {
      return centerOfMassFrame;
   }

   /**
    * Gets the floating joint of this robot.
    * <p>
    * The floating joint is the 6 degrees of freedom joint that represent the unactuated virtual
    * connection between the robot and the world.
    * </p>
    * 
    * @return the robot floating joint.
    */
   public FloatingInverseDynamicsJoint getRootJoint()
   {
      return jointMap.getInverseDynamicsSixDoFJoint(simulatedM2Robot.getFloatingJoint());
   }

   /**
    * Gets the pelvis.
    * 
    * @return the pelvis.
    */
   public RigidBody getPelvis()
   {
      return getRootJoint().getSuccessor();
   }

   /**
    * Gets the left or right foot given the {@code RobotSide}.
    * 
    * @param robotSide whether this method should return the left or right foot.
    * @return the foot.
    */
   public RigidBody getFoot(RobotSide robotSide)
   {
      return jointMap.getRigidBody(simulatedM2Robot.getFootParentJoint(robotSide));
   }

   /**
    * Gets the foot with its contact points.
    * 
    * @param robotSide whether this method should return the left or right contactable foot.
    * @return the contactable foot.
    */
   public ContactablePlaneBody getFootContactableBody(RobotSide robotSide)
   {
      return footContactableBodies.get(robotSide);
   }

   /**
    * Gets the sole frame of one of the feet.
    * 
    * @param robotSide whether this method should return the left or right sole frame.
    * @return the sole frame.
    */
   public ReferenceFrame getSoleFrame(RobotSide robotSide)
   {
      return soleFrames.get(robotSide);
   }

   /**
    * Finds the corresponding simulated joint to the given {@code inverseDynamicsJoint} and set its
    * desired torque.
    * 
    * @param inverseDynamicsJoint the joint of interest.
    * @param desiredEffort the new effort value.
    */
   public void setDesiredEffort(OneDoFJoint inverseDynamicsJoint, double desiredEffort)
   {
      jointMap.getSimulatedOneDegreeOfFreedomJoint(inverseDynamicsJoint).setTau(desiredEffort);
   }
}
