package it.iit.iCub.parameters;

import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;

public class IcubSwingTrajectoryParameters extends SwingTrajectoryParameters
{
   private final double min_mechanical_leg_length;// = IcubRobotModel.SCALE_FACTOR * 0.20; // TODO tune

   public IcubSwingTrajectoryParameters(double modelScale)
   {
      min_mechanical_leg_length = modelScale * 0.20;
   }

   @Override
   public boolean doToeTouchdownIfPossible()
   {
      return false;
   }

   @Override
   public double getToeTouchdownAngle()
   {
      return Math.toRadians(20.0);
   }

   @Override
   public boolean doHeelTouchdownIfPossible()
   {
      return false;
   }

   @Override
   public double getHeelTouchdownAngle()
   {
      return Math.toRadians(-20.0);
   }

   @Override
   public double getMinMechanicalLegLength()
   {
      return min_mechanical_leg_length;
   }

   @Override
   public double getDesiredTouchdownHeightOffset()
   {
      return 0;
   }

   @Override
   public double getDesiredTouchdownVelocity()
   {
      return -0.3;
   }

   @Override
   public double getDesiredTouchdownAcceleration()
   {
      return 0;
   }

}
