package it.iit.iCub.animationTests;

import java.util.Random;

import org.junit.Ignore;
import org.junit.Test;

import it.iit.iCub.flatGroundWalking.ICubFlatGroundWalkingTest;
import it.iit.iCub.testTools.ICubTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.MessageOfMessages;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.yoVariables.variable.YoDouble;

public class ICubWholeBodyMotionTests extends ICubTest
{
   private static final Random random = new Random(42L);

   // Ignore this test for now since it is mostly used for tuning and debugging.
   @Ignore
   @Test
   public void testWholeBodyMotions() throws SimulationExceededMaximumTimeException
   {
      simulate(0.5);

      SimulationConstructionSet scs = getTestHelper().getSimulationConstructionSet();
      TrackingObserver trackingObserver = new TrackingObserver(scs);
      getTestHelper().addRobotControllerOnControllerThread(trackingObserver);

      double trajectoryTime = 10.0;
      FullHumanoidRobotModel fullRobotModel = getTestHelper().getControllerFullRobotModel();

      MessageOfMessages motion = new MessageOfMessages();
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      motion.addPacket(ICubFlatGroundWalkingTest.createWalkingMessage(trajectoryTime, fullRobotModel, walkingControllerParameters));
      motion.addPacket(createRandomChestTrajectory(trajectoryTime));
      motion.addPacket(createRandomArmTrajectory(trajectoryTime, RobotSide.RIGHT, fullRobotModel));
      motion.addPacket(createRandomArmTrajectory(trajectoryTime, RobotSide.LEFT, fullRobotModel));
      motion.addPacket(createRandomPelvisOrientation(trajectoryTime));

      sendPacket(motion);
      simulate(trajectoryTime + 0.5);

      trackingObserver.print();
   }

   private ChestTrajectoryMessage createRandomChestTrajectory(double trajectoryTime)
   {
      double minMaxAngle = Math.toRadians(20.0);
      double minMaxAngularVelocity = minMaxAngle / 5.0;

      int points = 5 + random.nextInt(5);
      ChestTrajectoryMessage message = new ChestTrajectoryMessage(points);

      for (int i = 0; i < points; i++)
      {
         double time = trajectoryTime * (i + 1) / points;
         Quaternion orientation = EuclidCoreRandomTools.generateRandomQuaternion(random, minMaxAngle);
         Vector3D angularVelocity = EuclidCoreRandomTools.generateRandomVector3D(random, minMaxAngularVelocity, minMaxAngularVelocity);

         if (i == points - 1)
         {
            angularVelocity.setToZero();
         }

         message.setTrajectoryPoint(i, time, orientation, angularVelocity, ReferenceFrame.getWorldFrame());
      }

      return message;
   }

   private ArmTrajectoryMessage createRandomArmTrajectory(double trajectoryTime, RobotSide robotSide, FullHumanoidRobotModel fullRobotModel)
   {
      int points = 5 + random.nextInt(5);
      double maxJointVelocity = 5.0;

      RigidBody chest = fullRobotModel.getChest();
      RigidBody hand = fullRobotModel.getHand(robotSide);
      OneDoFJoint[] armJoints = ScrewTools.createOneDoFJointPath(chest, hand);
      int numberOfJoints = ScrewTools.computeDegreesOfFreedom(armJoints);
      ArmTrajectoryMessage message = new ArmTrajectoryMessage(robotSide, numberOfJoints, points);

      for (int i = 0; i < points; i++)
      {
         double time = trajectoryTime * (i + 1) / points;
         for (int jointIdx = 0; jointIdx < numberOfJoints; jointIdx++)
         {
            OneDoFJoint joint = armJoints[jointIdx];
            double lowerLimit = Double.isFinite(joint.getJointLimitLower()) ? joint.getJointLimitLower() : -Math.PI / 2.0;
            double upperLimit = Double.isFinite(joint.getJointLimitUpper()) ? joint.getJointLimitUpper() : Math.PI / 2.0;
            double position = RandomNumbers.nextDouble(random, lowerLimit, upperLimit);
            double velocity = RandomNumbers.nextDouble(random, maxJointVelocity);

            if (i == points - 1)
            {
               velocity = 0.0;
            }

            message.setTrajectoryPoint(jointIdx, i, time, position, velocity);
         }
      }

      return message;
   }

   private PelvisOrientationTrajectoryMessage createRandomPelvisOrientation(double trajectoryTime)
   {
      double minMaxAngle = Math.toRadians(10.0);
      double minMaxAngularVelocity = minMaxAngle / 5.0;

      int points = 5 + random.nextInt(5);
      PelvisOrientationTrajectoryMessage message = new PelvisOrientationTrajectoryMessage(points);
      message.setEnableUserPelvisControlDuringWalking(true);

      for (int i = 0; i < points; i++)
      {
         double time = trajectoryTime * (i + 1) / points;
         Quaternion orientation = EuclidCoreRandomTools.generateRandomQuaternion(random, minMaxAngle);
         Vector3D angularVelocity = EuclidCoreRandomTools.generateRandomVector3D(random, minMaxAngularVelocity, minMaxAngularVelocity);

         if (i == points - 1)
         {
            angularVelocity.setToZero();
         }

         message.setTrajectoryPoint(i, time, orientation, angularVelocity, ReferenceFrame.getWorldFrame());
      }

      return message;
   }

   private class TrackingObserver extends SimpleRobotController
   {
      private final YoDouble icpErrorX;
      private final YoDouble icpErrorY;

      private int calls = 0;
      private double errorXAverage;
      private double errorYAverage;

      public TrackingObserver(SimulationConstructionSet scs)
      {
         icpErrorX = (YoDouble) scs.getVariable("icpErrorX");
         icpErrorY = (YoDouble) scs.getVariable("icpErrorY");
      }

      @Override
      public void doControl()
      {
         errorXAverage = (calls * errorXAverage + icpErrorX.getDoubleValue()) / (calls + 1.0);
         errorYAverage = (calls * errorYAverage + icpErrorY.getDoubleValue()) / (calls + 1.0);
         calls++;
      }

      public void print()
      {
         PrintTools.info("Average ICP tracking error:\nxErr = " + errorXAverage + "\nyErr = " + errorYAverage);
      }
   }
}
