package it.iit.iCub.flatGroundWalking;

import org.junit.Test;

import it.iit.iCub.testTools.ICubTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.Packet;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.MessageOfMessages;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ICubFlatGroundWalkingTest extends ICubTest
{
   @Test
   public void testStanding()
   {
      simulate(3.0);
   }

   @Test
   public void testSteppingInPlace() throws SimulationExceededMaximumTimeException
   {
      simulate(0.5);

      FullHumanoidRobotModel fullRobotModel = getTestHelper().getControllerFullRobotModel();
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

      sendPacket(footsteps);
      simulate(5.0);
   }

   @Test
   public void testWalkingForward() throws SimulationExceededMaximumTimeException
   {
      simulate(0.5);

      double walkingTime = 10.0;
      FullHumanoidRobotModel fullRobotModel = getTestHelper().getControllerFullRobotModel();
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      FootstepDataListMessage footsteps = createWalkingMessage(walkingTime, fullRobotModel, walkingControllerParameters);

      sendPacket(footsteps);
      simulate(walkingTime + 0.5);
   }

   @Test
   public void testFastWalkingForward() throws SimulationExceededMaximumTimeException
   {
      simulate(0.5);

      double walkingTime = 10.0;
      double swingTime = 0.4;
      double transferTime = 0.05;
      FullHumanoidRobotModel fullRobotModel = getTestHelper().getControllerFullRobotModel();
      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      FootstepDataListMessage footsteps = createWalkingMessage(swingTime, transferTime, walkingTime, fullRobotModel, walkingControllerParameters);

      sendPacket(footsteps);
      simulate(walkingTime + 0.5);
   }

   @Test
   public void testStandingOnOneFoot() throws SimulationExceededMaximumTimeException
   {
      simulate(0.5);

      FullHumanoidRobotModel fullRobotModel = getTestHelper().getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      referenceFrames.updateFrames();

      RobotSide robotSide = RobotSide.LEFT;
      MovingReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
      FrameOrientation orientation = new FrameOrientation(soleFrame);
      FramePoint3D location = new FramePoint3D(soleFrame);
      location.setZ(0.1);
      orientation.changeFrame(ReferenceFrame.getWorldFrame());
      location.changeFrame(ReferenceFrame.getWorldFrame());
      FootTrajectoryMessage footMessage = new FootTrajectoryMessage(robotSide, 1.0, location.getPoint(), orientation.getQuaternion());

      Quaternion chestOrientation = new Quaternion();
      chestOrientation.appendRollRotation(Math.toRadians(30.0));
      chestOrientation.appendPitchRotation(Math.PI / 6.0);
      long pelvisZUpId = CommonReferenceFrameIds.PELVIS_ZUP_FRAME.getHashId();
      ChestTrajectoryMessage chestMessage = new ChestTrajectoryMessage(1.0, chestOrientation, pelvisZUpId);

      Packet<?> message = new MessageOfMessages(footMessage, chestMessage);
      sendPacket(message);
      simulate(3.0);
   }

   public static FootstepDataListMessage createWalkingMessage(double time, FullHumanoidRobotModel fullRobotModel,
                                                              WalkingControllerParameters walkingControllerParameters)
   {
      double swingTime = walkingControllerParameters.getDefaultSwingTime();
      double transferTime = walkingControllerParameters.getDefaultTransferTime();
      return createWalkingMessage(swingTime, transferTime, time, fullRobotModel, walkingControllerParameters);
   }

   public static FootstepDataListMessage createWalkingMessage(double swingTime, double transferTime, double time, FullHumanoidRobotModel fullRobotModel,
                                                              WalkingControllerParameters walkingControllerParameters)
   {
      double initialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
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
