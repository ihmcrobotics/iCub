package it.iit.iCub.parameters;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;

public class IcubICPOptimizationParameters extends ICPOptimizationParameters
{
   @Override
   public int numberOfFootstepsToConsider()
   {
      return 1;
   }

   @Override
   public double getForwardFootstepWeight()
   {
      return 15.0;
   }

   @Override
   public double getLateralFootstepWeight()
   {
      return 15.0;
   }

   @Override
   public double getFootstepRegularizationWeight()
   {
      return 0.005;
   }

   @Override
   public double getFeedbackForwardWeight()
   {
      return 0.5;
   }

   @Override
   public double getFeedbackLateralWeight()
   {
      return 0.5;
   }

   @Override
   public double getFeedbackRegularizationWeight()
   {
      return 0.00005;
   }

   @Override
   public double getFeedbackParallelGain()
   {
      return 2.5;
   }

   @Override
   public double getFeedbackOrthogonalGain()
   {
      return 1.5;
   }

   @Override
   public double getDynamicsObjectiveWeight()
   {
      return 1000.0;
   }

   @Override
   public double getDynamicsObjectiveDoubleSupportWeightModifier()
   {
      return 1.0;
   }

   @Override
   public double getAngularMomentumMinimizationWeight()
   {
      return 50.0;
   }

   @Override
   public boolean scaleStepRegularizationWeightWithTime()
   {
      return false;
   }

   @Override
   public boolean scaleFeedbackWeightWithGain()
   {
      return true;
   }

   @Override
   public boolean useFeedbackRegularization()
   {
      return true;
   }

   @Override
   public boolean useStepAdjustment()
   {
      return true;
   }

   @Override
   public boolean useAngularMomentum()
   {
      return false;
   }

   @Override
   public boolean useFootstepRegularization()
   {
      return true;
   }

   @Override
   public double getMinimumFootstepWeight()
   {
      return 0.01;
   }

   @Override
   public double getMinimumFeedbackWeight()
   {
      return 0.01;
   }

   @Override
   public double getMinimumTimeRemaining()
   {
      return 0.01;
   }

   @Override
   public double getAdjustmentDeadband()
   {
      return 0.0;
   }
}
