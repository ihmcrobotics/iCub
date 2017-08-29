package it.iit.iCub.parameters;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;

public class ICubSteppingParameters implements SteppingParameters
{
   private final IcubJointMap jointMap;

   public ICubSteppingParameters(IcubJointMap jointMap)
   {
      this.jointMap = jointMap;
   }

   @Override
   public double getFootForwardOffset()
   {
      return jointMap.getPhysicalProperties().getFootForward();
   }

   @Override
   public double getFootBackwardOffset()
   {
      return jointMap.getPhysicalProperties().getFootBack();
   }

   @Override
   public double getInPlaceWidth()
   {
      return 0.2;
   }

   @Override
   public double getDesiredStepForward()
   {
      return 0.15;
   }

   @Override
   public double getMaxStepLength()
   {
      return 0.2;
   }

   @Override
   public double getDefaultStepLength()
   {
      return 0.1;
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.2;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.3;
   }

   @Override
   public double getStepPitch()
   {
      return 0.0;
   }

   @Override
   public double getMaxStepUp()
   {
      return 0.05;
   }

   @Override
   public double getMaxStepDown()
   {
      return 0.05;
   }

   @Override
   public double getMaxSwingHeightFromStanceFoot()
   {
      return 0.15;
   }

   @Override
   public double getMinSwingHeightFromStanceFoot()
   {
      return 0.05;
   }

   @Override
   public double getMaxAngleTurnOutwards()
   {
      return 0.0;
   }

   @Override
   public double getMaxAngleTurnInwards()
   {
      return 0.0;
   }

   @Override
   public double getMinAreaPercentForValidFootstep()
   {
      return 0.5;
   }

   @Override
   public double getDangerAreaPercentForValidFootstep()
   {
      return 0.75;
   }

   @Override
   public double getFootWidth()
   {
      return jointMap.getPhysicalProperties().getFootWidth();
   }

   @Override
   public double getToeWidth()
   {
      return jointMap.getPhysicalProperties().getToeWidth();
   }

   @Override
   public double getFootLength()
   {
      return jointMap.getPhysicalProperties().getFootLength();
   }

   @Override
   public double getActualFootWidth()
   {
      return getFootWidth();
   }

   @Override
   public double getActualFootLength()
   {
      return getFootLength();
   }
}
