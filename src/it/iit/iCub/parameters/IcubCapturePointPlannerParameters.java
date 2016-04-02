package it.iit.iCub.parameters;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;

/** {@inheritDoc} */
public class IcubCapturePointPlannerParameters extends CapturePointPlannerParameters
{
   private final boolean runningOnRealRobot;

   public IcubCapturePointPlannerParameters(boolean runningOnRealRobot)
   {
      this.runningOnRealRobot = runningOnRealRobot;
   }

   /** {@inheritDoc} */
   @Override
   public double getDoubleSupportInitialTransferDuration()
   {
      return runningOnRealRobot ? 2.0 : 1.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getEntryCMPInsideOffset()
   {
      return 0.006;
   }

   /** {@inheritDoc} */
   @Override
   public double getExitCMPInsideOffset()
   {
      return 0.006;
   }

   /** {@inheritDoc} */
   @Override
   public double getEntryCMPForwardOffset()
   {
      return 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getExitCMPForwardOffset()
   {
      return 0.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxExitCMPForwardOffset()
   {
      return 0.05;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinExitCMPForwardOffset()
   {
      return -0.02;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useTwoCMPsPerSupport()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxEntryCMPForwardOffset()
   {
      return 0.05;
   }

   /** {@inheritDoc} */
   @Override
   public double getMinEntryCMPForwardOffset()
   {
      return -0.02;
   }
}
