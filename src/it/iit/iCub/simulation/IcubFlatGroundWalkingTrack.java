package it.iit.iCub.simulation;

import it.iit.iCub.IcubRobotModel;
import us.ihmc.darpaRoboticsChallenge.DRCFlatGroundWalkingTrack;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCSCSInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class IcubFlatGroundWalkingTrack
{
   public static void main(String[] args)
   {
      DRCRobotModel robotModel = new IcubRobotModel(false, false);
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(true, false);      
      
      final double groundHeight = 0.0;
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);
      
      double initialYaw = 0.0;
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);

      boolean useVelocityAndHeadingScript = true;
      boolean cheatWithGroundHeightAtForFootstep = false;

      new DRCFlatGroundWalkingTrack(robotInitialSetup, guiInitialSetup, scsInitialSetup, useVelocityAndHeadingScript,
                                    cheatWithGroundHeightAtForFootstep, robotModel);
   }

}
