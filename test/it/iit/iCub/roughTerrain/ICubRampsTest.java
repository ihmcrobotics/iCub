package it.iit.iCub.roughTerrain;

import org.junit.Test;

import it.iit.iCub.testTools.ICubTest;
import it.iit.iCub.testTools.TestingEnvironment;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class ICubRampsTest extends ICubTest
{
   @Test
   public void testWalkingUpRamp() throws SimulationExceededMaximumTimeException
   {
      simulate(0.5);

      FullHumanoidRobotModel fullRobotModel = getTestHelper().getControllerFullRobotModel();
      double stepLength = 0.1;
      double stepWidth = 0.15;
      double distance = 2.5;

      FootstepDataListMessage message = new FootstepDataListMessage();
      double lastX = 0.0;
      RobotSide side = RobotSide.LEFT;
      int stepIdx = 0;

      while (lastX < distance)
      {
         side = side.getOppositeSide();
         MovingReferenceFrame stanceSoleFrame = fullRobotModel.getSoleFrame(side.getOppositeSide());

         FramePoint3D position = new FramePoint3D(stanceSoleFrame);
         position.setX(stepLength * (++stepIdx));
         position.setY(side.negateIfRightSide(stepWidth));
         position.changeFrame(ReferenceFrame.getWorldFrame());
         lastX = position.getX();
         position.setZ(RampEnvironment.getHeight(lastX));

         FrameOrientation orientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
         orientation.appendPitchRotation(RampEnvironment.getPitch(lastX));

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

   @Test
   public void testWalkingDownRamp() throws SimulationExceededMaximumTimeException
   {
      simulate(0.5);

      FullHumanoidRobotModel fullRobotModel = getTestHelper().getControllerFullRobotModel();
      double stepLength = 0.1;
      double stepWidth = 0.15;
      double goal = 0.0;

      FootstepDataListMessage message = new FootstepDataListMessage();
      double lastX = 2.5;
      RobotSide side = RobotSide.LEFT;
      int stepIdx = 0;

      while (lastX > goal)
      {
         side = side.getOppositeSide();
         MovingReferenceFrame stanceSoleFrame = fullRobotModel.getSoleFrame(side.getOppositeSide());

         FramePoint3D position = new FramePoint3D(stanceSoleFrame);
         position.setX(stepLength * (++stepIdx));
         position.setY(side.negateIfRightSide(stepWidth));
         position.changeFrame(ReferenceFrame.getWorldFrame());
         lastX = position.getX();
         position.setZ(RampEnvironment.getHeight(lastX));

         FrameOrientation orientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
         orientation.appendYawRotation(Math.PI);
         orientation.appendPitchRotation(-RampEnvironment.getPitch(lastX));

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
   public Point3D getCameraPosion()
   {
      return new Point3D(-6.0, 4.0, 2.0);
   }

   @Override
   public BoundingBox3D getFinalBoundingBox()
   {
      if ("testWalkingUpRamp".equals(name.getMethodName()))
      {
         Point3DReadOnly min = new Point3D(RampEnvironment.rampEnd, -0.5, RampEnvironment.rampHeight);
         Point3DReadOnly max = new Point3D(RampEnvironment.rampEnd + 1.0, 0.5, RampEnvironment.rampHeight + 1.0);
         return new BoundingBox3D(min, max);
      }
      else if ("testWalkingDownRamp".equals(name.getMethodName()))
      {
         Point3DReadOnly min = new Point3D(-0.5, -0.5, 0.0);
         Point3DReadOnly max = new Point3D(0.5, 0.5, 1.0);
         return new BoundingBox3D(min, max);
      }

      /**
       * For these tests it is common for the robot to get stuck. The test will not fail because the
       * robot has not fallen. To make sure the robot walked where expected define a bounding box that
       * the robot needs to be in after the test is done.
       */
      throw new RuntimeException("Implement a final bounding box for your test.");
   }

   @Override
   public DRCStartingLocation getStartingLocation()
   {
      if ("testWalkingDownRamp".equals(name.getMethodName()))
      {
         return new DRCStartingLocation()
         {
            @Override
            public OffsetAndYawRobotInitialSetup getStartingLocationOffset()
            {
               return new OffsetAndYawRobotInitialSetup(RampEnvironment.rampHeight, new Vector3D(2.5, 0.0, 0.0), Math.PI);
            }
         };
      }

      return super.getStartingLocation();
   }

   @Override
   public CommonAvatarEnvironmentInterface getEnvironment()
   {
      return new RampEnvironment();
   }

   public static class RampEnvironment extends TestingEnvironment
   {
      public static final double rampStart = 0.5;
      public static final double rampEnd = 2.0;
      public static final double rampHeight = 0.2;

      public RampEnvironment()
      {
         terrain.addBox(-0.5, -0.5, 3.0, 0.5, -0.01, 0.0);
         terrain.addRamp(rampStart, -0.5, rampEnd, 0.5, rampHeight, YoAppearance.Gray());
         terrain.addBox(rampEnd, -0.5, 3.0, 0.5, 0.0, rampHeight);
      }

      public static double getHeight(double x)
      {
         if (x <= rampStart)
         {
            return 0.0;
         }

         if (x > rampEnd)
         {
            return rampHeight;
         }

         double percentInRamp = (x - rampStart) / (rampEnd - rampStart);
         return percentInRamp * rampHeight;
      }

      public static double getPitch(double x)
      {
         if (x <= rampStart || x > rampEnd)
         {
            return 0.0;
         }

         return -Math.atan2(rampHeight, (rampEnd - rampStart));
      }

   }
}
