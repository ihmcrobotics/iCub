package it.iit.iCub.parameters;

import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;

public class IcubICPOptimizationParameters extends ICPOptimizationParameters
{
    private final boolean runningOnRealRobot;
    private final boolean useAngularMomentum = false;
    private final boolean useStepAdjustment = false;

    public IcubICPOptimizationParameters(boolean runningOnRealRobot)
    {
        this.runningOnRealRobot = runningOnRealRobot;
    }

//   @Override
//   public boolean useSimpleOptimization()
//   {
//      return true;
//   }

   @Override
   public double getDynamicsObjectiveDoubleSupportWeightModifier()
   {
      if (useAngularMomentum)
         return runningOnRealRobot ? 50.0 : 100.0;
      else if (useStepAdjustment)
         return runningOnRealRobot ? 1.0 : 4.0;
      else
         return 1.0;
   }

    @Override
    public double getDynamicsObjectiveWeight()
    {
        if (runningOnRealRobot)
            return 10000.0;
        else if (useAngularMomentum)
            return 100000.0;
        else if (useStepAdjustment)
            return 1000.0;
        else
            return 10000.0;
        //return runningOnRealRobot ? 10000.0 : (useAngularMomentum ? 100000.0 : 1000.0);
    }

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

//   @Override
//   public double getDynamicRelaxationWeight()
//   {
//      return 1000.0;
//   }
//
//   @Override
//   public double getDynamicRelaxationDoubleSupportWeightModifier()
//   {
//      return 1.0;
//   }

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

//   @Override
//   public boolean scaleUpcomingStepWeights()
//   {
//      return true;
//   }

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

//   @Override
//   public boolean useTimingOptimization()
//   {
//      return false;
//   }

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
