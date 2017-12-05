package it.iit.iCub.flatGroundWalking;

import org.junit.Test;

import it.iit.iCub.IcubRobotModel;
import it.iit.iCub.parameters.IcubICPOptimizationParameters;
import it.iit.iCub.parameters.IcubWalkingControllerParameters;
import it.iit.iCub.testTools.ICubTest;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.simulationToolkit.controllers.PushRobotController;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.yoVariables.variable.YoEnum;

public class ICubPushRecoveryTest extends ICubTest
{
   @Override
   public boolean createPushController()
   {
      return true;
   }

   /**
    * Create a robot model that has push recovery enabled:</br>
    * {@inheritDoc}
    */
   @Override
   public IcubRobotModel createRobotModel()
   {
      return new IcubRobotModel(removeJointLimits())
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new IcubWalkingControllerParameters(getJointMap())
            {
               @Override
               public boolean useOptimizationBasedICPController()
               {
                  return true;
               }

               @Override
               public ICPOptimizationParameters getICPOptimizationParameters()
               {
                  return new IcubICPOptimizationParameters();
               }
            };
         }
      };
   }

   @Test
   public void testPushForward() throws SimulationExceededMaximumTimeException
   {
      PushRobotController pushController = getPushRobotController();
      SwingStartCondition condition = new SwingStartCondition(getTestHelper().getSimulationConstructionSet());

      simulate(0.5);

      FootstepDataListMessage footsteps = createStepsInPlace();

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double swingTime = walkingControllerParameters.getDefaultSwingTime();
      double transferTime = walkingControllerParameters.getDefaultTransferTime();
      double initialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      double walkingTime = initialTransferTime + (swingTime + transferTime) * footsteps.getDataList().size();

      double magnitude = 70.0;
      condition.set(RobotSide.LEFT);
      pushController.applyForceDelayed(condition, swingTime / 4.0, new Vector3D(1.0, 0.0, 0.0), magnitude, 0.2);

      sendPacket(footsteps);
      simulate(walkingTime + 0.5);
   }

   @Test
   public void testPushSideway() throws SimulationExceededMaximumTimeException
   {
      PushRobotController pushController = getPushRobotController();
      SwingStartCondition condition = new SwingStartCondition(getTestHelper().getSimulationConstructionSet());

      simulate(0.5);

      FootstepDataListMessage footsteps = createStepsInPlace();

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double swingTime = walkingControllerParameters.getDefaultSwingTime();
      double transferTime = walkingControllerParameters.getDefaultTransferTime();
      double initialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      double walkingTime = initialTransferTime + (swingTime + transferTime) * footsteps.getDataList().size();

      double magnitude = 40.0;
      condition.set(RobotSide.LEFT);
      pushController.applyForceDelayed(condition, swingTime / 4.0, new Vector3D(0.0, 1.0, 0.0), magnitude, 0.2);

      sendPacket(footsteps);
      simulate(walkingTime + 0.5);
   }

   @Test
   public void testPushBackward() throws SimulationExceededMaximumTimeException
   {
      PushRobotController pushController = getPushRobotController();
      SwingStartCondition condition = new SwingStartCondition(getTestHelper().getSimulationConstructionSet());

      simulate(0.5);

      FootstepDataListMessage footsteps = createStepsInPlace();

      WalkingControllerParameters walkingControllerParameters = getRobotModel().getWalkingControllerParameters();
      double swingTime = walkingControllerParameters.getDefaultSwingTime();
      double transferTime = walkingControllerParameters.getDefaultTransferTime();
      double initialTransferTime = walkingControllerParameters.getDefaultInitialTransferTime();
      double walkingTime = initialTransferTime + (swingTime + transferTime) * footsteps.getDataList().size();

      double magnitude = 60.0;
      condition.set(RobotSide.LEFT);
      pushController.applyForceDelayed(condition, swingTime / 4.0, new Vector3D(-1.0, 0.0, 0.0), magnitude, 0.2);

      sendPacket(footsteps);
      simulate(walkingTime + 0.5);
   }

   private FootstepDataListMessage createStepsInPlace()
   {
      FullHumanoidRobotModel fullRobotModel = getTestHelper().getControllerFullRobotModel();
      HumanoidReferenceFrames referenceFrames = new HumanoidReferenceFrames(fullRobotModel);
      referenceFrames.updateFrames();

      FootstepDataListMessage footsteps = new FootstepDataListMessage();
      footsteps.setOffsetFootstepsWithExecutionError(true);
      for (RobotSide robotSide : RobotSide.values)
      {
         MovingReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
         FrameQuaternion orientation = new FrameQuaternion(soleFrame);
         FramePoint3D location = new FramePoint3D(soleFrame);
         orientation.changeFrame(ReferenceFrame.getWorldFrame());
         location.changeFrame(ReferenceFrame.getWorldFrame());
         FootstepDataMessage footstep = new FootstepDataMessage(robotSide, location.getPoint(), orientation.getQuaternion());
         footsteps.add(footstep);
      }

      return footsteps;
   }

   private class SwingStartCondition implements StateTransitionCondition
   {
      SideDependentList<YoEnum<ConstraintType>> footStates = new SideDependentList<>();
      private RobotSide robotSide = null;

      @SuppressWarnings("unchecked")
      public SwingStartCondition(SimulationConstructionSet scs)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            String prefix = robotSide.getLowerCaseName();
            footStates.put(robotSide, (YoEnum<ConstraintType>) scs.getVariable(prefix + "FootState"));
         }
      }

      public void set(RobotSide robotSide)
      {
         this.robotSide = robotSide;
      }

      @Override
      public boolean checkCondition()
      {
         if (robotSide == null)
         {
            return false;
         }

         boolean inSwing = footStates.get(robotSide).getEnumValue() == ConstraintType.SWING;

         if (inSwing)
         {
            robotSide = null;
            return true;
         }

         return false;
      }
   }
}
