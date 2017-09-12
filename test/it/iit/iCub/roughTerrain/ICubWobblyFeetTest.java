package it.iit.iCub.roughTerrain;

import org.junit.Ignore;
import org.junit.Test;

import it.iit.iCub.IcubRobotModel;
import it.iit.iCub.testTools.ICubTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.WobblySimulationContactPoints;

public class ICubWobblyFeetTest extends ICubTest
{
   /**
    * The four contact points on the foot are modified such that they are not in a plane
    * anymore: two diagonal points are elevated such that the robot foot will not be able
    * to stand flat
    */
   private static final double zWobble = 0.005;

   @Ignore
   @Test
   public void testTurningWithWobblyFeet() throws SimulationExceededMaximumTimeException
   {
      simulate(0.5);

      FullHumanoidRobotModel fullRobotModel = getTestHelper().getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      MovingReferenceFrame midFootFrame = referenceFrames.getMidFootZUpGroundFrame();

      double stepWidth = 0.15;
      double finalYaw = Math.toRadians(90.0);
      double yawPerStep = Math.toRadians(10.0);
      int steps = (int) Math.ceil(finalYaw / yawPerStep);

      FootstepDataListMessage message = new FootstepDataListMessage();
      for (int i = 0; i < steps ; i++)
      {
         RobotSide side = RobotSide.values[i % 2];
         PoseReferenceFrame rotatedFrame = new PoseReferenceFrame("RotatedMidFootFrame", midFootFrame);
         rotatedFrame.setOrientationAndUpdate(new AxisAngle(yawPerStep * i, 0.0, 0.0));

         FramePoint3D position = new FramePoint3D(rotatedFrame);
         FrameOrientation orientation = new FrameOrientation(rotatedFrame);
         position.setY(side.negateIfRightSide(stepWidth / 2.0));

         position.changeFrame(ReferenceFrame.getWorldFrame());
         orientation.changeFrame(ReferenceFrame.getWorldFrame());

         FootstepDataMessage footstep = new FootstepDataMessage(side, position.getPoint(), orientation.getQuaternion());
         message.add(footstep);
      }

      sendPacket(message);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double swingTime = walkingControllerParameters.getDefaultSwingTime();
      double transferTime = walkingControllerParameters.getDefaultTransferTime();
      double initialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      double walkingTime = initialTransferTime + steps * (swingTime + transferTime);

      simulate(walkingTime + 0.5);
   }

   @Test
   public void testWalkingWithWobblyFeet() throws SimulationExceededMaximumTimeException
   {
      simulate(0.5);

      FullHumanoidRobotModel fullRobotModel = getTestHelper().getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      MovingReferenceFrame midFootFrame = referenceFrames.getMidFootZUpGroundFrame();

      double stepLength = 0.2;
      double stepWidth = 0.15;
      double distance = 2.0;

      FootstepDataListMessage message = new FootstepDataListMessage();
      double lastX = 0.0;
      RobotSide side = RobotSide.LEFT;
      int stepIdx = 0;

      while (lastX < distance)
      {
         side = side.getOppositeSide();
         FramePoint3D position = new FramePoint3D(midFootFrame);
         FrameOrientation orientation = new FrameOrientation(midFootFrame);

         position.setX(stepLength * (++stepIdx));
         position.setY(side.negateIfRightSide(stepWidth / 2.0));

         position.changeFrame(ReferenceFrame.getWorldFrame());
         orientation.changeFrame(ReferenceFrame.getWorldFrame());

         lastX = position.getX();

         FootstepDataMessage footstep = new FootstepDataMessage(side, position.getPoint(), orientation.getQuaternion());
         message.add(footstep);
      }

      sendPacket(message);

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double swingTime = walkingControllerParameters.getDefaultSwingTime();
      double transferTime = walkingControllerParameters.getDefaultTransferTime();
      double initialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      double walkingTime = initialTransferTime + stepIdx * (swingTime + transferTime);

      simulate(walkingTime + 0.5);
   }

   @Override
   public IcubRobotModel createRobotModel()
   {
      FootContactPoints wobblyContacts = new WobblySimulationContactPoints(zWobble);
      return new IcubRobotModel(removeJointLimits(), wobblyContacts);
   }
}
