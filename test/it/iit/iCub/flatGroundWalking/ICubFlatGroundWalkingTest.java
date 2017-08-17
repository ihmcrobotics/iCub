package it.iit.iCub.flatGroundWalking;

import static org.junit.Assert.*;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import it.iit.iCub.IcubRobotModel;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ICubFlatGroundWalkingTest
{
   private static final DRCStartingLocation startingLocation = DRCObstacleCourseStartingLocation.DEFAULT;
   private static final IcubRobotModel robotModel = new IcubRobotModel();

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

   static
   {
      simulationTestingParameters.setUsePefectSensors(true);
   }

   private DRCSimulationTestHelper drcSimulationTestHelper;

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

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      referenceFrames.updateFrames();

      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      for (RobotSide robotSide : RobotSide.values)
      {
         MovingReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         FrameOrientation orientation = new FrameOrientation(soleFrame);
         FramePoint3D location = new FramePoint3D(soleFrame);
         location.setX(0.2);
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
      FootTrajectoryMessage message = new FootTrajectoryMessage(robotSide , 1.0, location.getPoint(), orientation.getQuaternion());

      drcSimulationTestHelper.send(message);
      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0);
      assertTrue(success);
   }

   @Before
   public void showMemoryUsageBeforeTest() throws SimulationExceededMaximumTimeException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, "Test", startingLocation, simulationTestingParameters, robotModel);
      OffsetAndYawRobotInitialSetup startingLocationOffset = startingLocation.getStartingLocationOffset();
      Point3D cameraFocus = new Point3D(startingLocationOffset.getAdditionalOffset());
      cameraFocus.addZ(0.4);
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotationYawAndZeroTranslation(startingLocationOffset.getYaw());
      Point3D cameraPosition = new Point3D(5.0, 3.0, 1.0);
      transform.transform(cameraPosition);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFocus, cameraPosition);
      ThreadTools.sleep(1000);
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
