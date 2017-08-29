package it.iit.iCub.roughTerrain;

import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.Test;

import it.iit.iCub.testTools.ICubTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class ICubStepUpTest extends ICubTest
{
   private static final double step1Height = 0.05;
   private static final double step2Height = 0.1;
   private static final double step3Height = 0.15;

   @Test
   public void testWalkingOnBoxes() throws SimulationExceededMaximumTimeException
   {
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double stepLength = 0.2;
      double stepWidth = 0.15;
      int steps = 20;

      FootstepDataListMessage message = new FootstepDataListMessage();
      for (int i = 0; i < steps ; i++)
      {
         RobotSide side = RobotSide.values[i % 2];
         MovingReferenceFrame stanceSoleFrame = fullRobotModel.getSoleFrame(side.getOppositeSide());
         FramePoint3D position = new FramePoint3D(stanceSoleFrame);
         FrameOrientation orientation = new FrameOrientation(stanceSoleFrame);

         position.setX(stepLength * (i + 1));
         position.setY(side.negateIfRightSide(stepWidth));

         position.changeFrame(ReferenceFrame.getWorldFrame());
         orientation.changeFrame(ReferenceFrame.getWorldFrame());

         if (MathTools.intervalContains(position.getX(), 0.5, 1.1))
         {
            position.setZ(step1Height);
         }
         else if (MathTools.intervalContains(position.getX(), 2.5, 3.1))
         {
            position.setZ(step2Height);
         }
         else if (MathTools.intervalContains(position.getX(), 4.5, 5.1))
         {
            position.setZ(step3Height);
         }

         FootstepDataMessage footstep = new FootstepDataMessage(side, position.getPoint(), orientation.getQuaternion());
         message.add(footstep);
      }

      drcSimulationTestHelper.send(message);

      double swingTime = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      double transferTime = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double initialTransferTime = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double walkingTime = initialTransferTime + steps * (swingTime + transferTime);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(walkingTime + 0.5);
      assertTrue(success);
   }

   @Override
   public CommonAvatarEnvironmentInterface getEnvironment()
   {
      return new TestingEnvironment();
   }

   public class TestingEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D terrain;

      public TestingEnvironment()
      {
         terrain = new CombinedTerrainObject3D(getClass().getSimpleName());
         terrain.addBox(-1.0, -0.5, 10.0, 0.5, -0.01, 0.0);
         terrain.addBox(0.5, -0.5, 1.1, 0.5, 0.0, step1Height);
         terrain.addBox(2.5, -0.5, 3.1, 0.5, 0.0, step2Height);
         terrain.addBox(4.5, -0.5, 5.1, 0.5, 0.0, step3Height);
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return terrain;
      }

      @Override
      public List<? extends Robot> getEnvironmentRobots()
      {
         return null;
      }

      @Override
      public void createAndSetContactControllerToARobot()
      {
      }

      @Override
      public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
      {
      }

      @Override
      public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
      {
      }
   }
}
