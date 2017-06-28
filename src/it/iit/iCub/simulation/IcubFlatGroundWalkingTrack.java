package it.iit.iCub.simulation;

import it.iit.iCub.IcubRobotModel;
import us.ihmc.avatar.DRCFlatGroundWalkingTrack;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.HeadingAndVelocityEvaluationScriptParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WalkingProvider;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class IcubFlatGroundWalkingTrack
{
   public static void main(String[] args)
   {
      DRCRobotModel robotModel = new IcubRobotModel();
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);

      final double groundHeight = 0.0;
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);
      scsInitialSetup.setUsePerfectSensors(true);

      double initialYaw = 0.0;
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);

      boolean useVelocityAndHeadingScript = true;
      boolean cheatWithGroundHeightAtForFootstep = false;

      HeadingAndVelocityEvaluationScriptParameters walkingScriptParameters = new HeadingAndVelocityEvaluationScriptParameters();
      walkingScriptParameters.setAcceleration(0.15);
      walkingScriptParameters.setCruiseVelocity(0.15);
      walkingScriptParameters.setHeadingDot(0.10);
      walkingScriptParameters.setMaxHeadingDot(0.15);
      walkingScriptParameters.setMaxVelocity(0.15);
      walkingScriptParameters.setSideStepVelocity(0.09);

      new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup, useVelocityAndHeadingScript, cheatWithGroundHeightAtForFootstep,
                                    robotModel, WalkingProvider.VELOCITY_HEADING_COMPONENT, walkingScriptParameters);
   }

}
