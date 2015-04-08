package it.iit.iCub.parameters;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;

public class IcubCapturePointPlannerParameters implements CapturePointPlannerParameters
{
   private boolean runningOnRealRobot;

   public IcubCapturePointPlannerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   @Override
   public double getDoubleSupportInitialTransferDuration()
   {
      return runningOnRealRobot ? 2.0 : 1.0;
   }

   @Override
   public double getDoubleSupportDuration()
   {
      return runningOnRealRobot ? 1.5 : 0.25;
   }

   @Override
   public double getAdditionalTimeForSingleSupport()
   {
      return 0.1;
   }

   @Override
   public double getSingleSupportDuration()
   {
      return runningOnRealRobot ? 1.5 : 0.7;
   }

   @Override
   public int getNumberOfFootstepsToConsider()
   {
      return 3;
   }

   @Override
   public int getNumberOfCoefficientsForDoubleSupportPolynomialTrajectory()
   {
      return 5;
   }

   @Override
   public int getNumberOfFootstepsToStop()
   {
      return 2;
   }

   @Override
   public double getIsDoneTimeThreshold()
   {
      return -1e-4;
   }

   @Override
   public double getDoubleSupportSplitFraction()
   {
      return 0.5;
   }

   @Override
   public double getFreezeTimeFactor()
   {
      return 0.9;
   }

   @Override
   public double getMaxInstantaneousCapturePointErrorForStartingSwing()
   {
      return 0.02;
   }

   @Override
   public boolean getDoTimeFreezing()
   {
      return true;
   }

   @Override
   public boolean getDoFootSlipCompensation()
   {
      return true;
   }

   @Override
   public double getAlphaDeltaFootPositionForFootslipCompensation()
   {
      return 0.65;
   }

   @Override
   public double getReferenceCMPInsideOffset()
   {
      return 0.006;
   }

   @Override
   public double getReferenceCMPForwardOffset()
   {
      return 0.0;
   }

   @Override
   public double getMaxAllowedErrorWithoutPartialTimeFreeze()
   {
      return 0.03;
   }

   @Override
   public boolean useTerribleHackToReduceICPVelocityAtTheEndOfTransfer()
   {
      return true;
   }

   @Override
   public boolean useNewICPPlanner()
   {
      return false;
   }

   @Override
   public boolean useTwoCMPsPerSupport()
   {
      return false;
   }

   @Override
   public double getTimeSpentOnExitCMPInPercentOfStepTime()
   {
      return 0.50;
   }

   @Override
   public double getMaxReferenceCMPForwardOffset()
   {
      return 0.05;
   }

   @Override
   public double getMinReferenceCMPForwardOffset()
   {
      return -0.02;
   }

   @Override
   public double getSafeDistanceForSupportEdges()
   {
      return 0.03;
   }
}
