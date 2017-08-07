package it.iit.iCub.parameters;

import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.UIParameters;

public class ICubUIParameters implements UIParameters
{
   private final double pelvis_pitch_upper_limit = 1.46608;
   private final double pelvis_pitch_lower_limit = -0.383972;

   private final IcubPhysicalProperties physicalProperties;

   public ICubUIParameters(IcubPhysicalProperties physicalProperties)
   {
      this.physicalProperties = physicalProperties;
   }

   @Override
   public double getAnkleHeight()
   {
      return Math.abs(physicalProperties.getSoleToAnkleFrameTransform(RobotSide.LEFT).getTranslationZ());
   }

   @Override
   public double getSpineYawLimit()
   {
      return 0.959931;
   }

   @Override
   public double getSpineRollLimit()
   {
      return 0;
   }

   @Override
   public double getSpinePitchUpperLimit()
   {
      return pelvis_pitch_upper_limit;
   }

   @Override
   public double getSpinePitchLowerLimit()
   {
      return pelvis_pitch_lower_limit;
   }

   @Override
   public boolean isSpinePitchReversed()
   {
      return false;
   }

   @Override
   public double getSideLengthOfBoundingBoxForFootstepHeight()
   {
      return (1 + 0.3) * 2 * Math.sqrt(physicalProperties.getFootForward() * physicalProperties.getFootForward()
            + 0.25 * physicalProperties.getFootWidth() * physicalProperties.getFootWidth());
   }

   @Override
   public double pelvisToAnkleThresholdForWalking()
   {
      return 0;
   }
}
