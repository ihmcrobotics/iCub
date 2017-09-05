package it.iit.iCub.flatGroundWalking;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import it.iit.iCub.testTools.ICubTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ICubFlatGroundWalkingTest extends ICubTest
{
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test
   public void testStanding() throws SimulationExceededMaximumTimeException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);
      assertTrue(success);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test
   public void testSteppingInPlace() throws SimulationExceededMaximumTimeException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      referenceFrames.updateFrames();

      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      for (RobotSide robotSide : RobotSide.values)
      {
         MovingReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         FrameOrientation orientation = new FrameOrientation(soleFrame);
         FramePoint3D location = new FramePoint3D(soleFrame);
         orientation.changeFrame(ReferenceFrame.getWorldFrame());
         location.changeFrame(ReferenceFrame.getWorldFrame());
         FootstepDataMessage footstep = new FootstepDataMessage(robotSide, location.getPoint(), orientation.getQuaternion());
         footsteps.add(footstep);
      }

      drcSimulationTestHelper.send(footsteps);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);
      assertTrue(success);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test
   public void testWalkingForward() throws SimulationExceededMaximumTimeException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkingTime = 10.0;
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      FootstepDataListMessage footsteps = createWalkingMessage(walkingTime, fullRobotModel);

      drcSimulationTestHelper.send(footsteps);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(walkingTime + 0.5);
      assertTrue(success);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test
   public void testFastWalkingForward() throws SimulationExceededMaximumTimeException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      double walkingTime = 10.0;
      double swingTime = 0.4;
      double transferTime = 0.05;
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      FootstepDataListMessage footsteps = createWalkingMessage(swingTime, transferTime, walkingTime, fullRobotModel);

      drcSimulationTestHelper.send(footsteps);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(walkingTime + 0.5);
      assertTrue(success);
   }

   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test
   public void testStandingOnOneFoot() throws SimulationExceededMaximumTimeException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      referenceFrames.updateFrames();

      RobotSide robotSide = RobotSide.LEFT;
      MovingReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
      FrameOrientation orientation = new FrameOrientation(soleFrame);
      FramePoint3D location = new FramePoint3D(soleFrame);
      location.setZ(0.1);
      orientation.changeFrame(ReferenceFrame.getWorldFrame());
      location.changeFrame(ReferenceFrame.getWorldFrame());
      FootTrajectoryMessage message = new FootTrajectoryMessage(robotSide, 1.0, location.getPoint(), orientation.getQuaternion());

      drcSimulationTestHelper.send(message);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);
      assertTrue(success);
   }

   public static FootstepDataListMessage createWalkingMessage(double time, FullHumanoidRobotModel fullRobotModel)
   {
      double swingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      double transferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      return createWalkingMessage(swingTime, transferTime, time, fullRobotModel);
   }

   public static FootstepDataListMessage createWalkingMessage(double swingTime, double transferTime, double time, FullHumanoidRobotModel fullRobotModel)
   {
      double initialTransferTime = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      FootstepDataListMessage message = new FootstepDataListMessage(swingTime, transferTime);

      double stepLength = 0.2;
      double stepWidth = 0.15;

      int steps = (int) Math.floor((time - initialTransferTime) / (swingTime + transferTime));
      for (int i = 0; i < steps; i++)
      {
         RobotSide side = RobotSide.values[i % 2];
         MovingReferenceFrame stanceSoleFrame = fullRobotModel.getSoleFrame(side.getOppositeSide());
         FramePoint3D position = new FramePoint3D(stanceSoleFrame);
         FrameOrientation orientation = new FrameOrientation(stanceSoleFrame);

         position.setX(stepLength * (i + 1));
         position.setY(side.negateIfRightSide(stepWidth));

         position.changeFrame(ReferenceFrame.getWorldFrame());
         orientation.changeFrame(ReferenceFrame.getWorldFrame());
         FootstepDataMessage footstep = new FootstepDataMessage(side, position.getPoint(), orientation.getQuaternion());
         message.add(footstep);
      }

      return message;
   }
}
