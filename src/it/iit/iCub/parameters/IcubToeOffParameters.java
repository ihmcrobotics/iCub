package it.iit.iCub.parameters;

import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;

public class IcubToeOffParameters extends ToeOffParameters
{
   private final IcubJointMap jointMap;

   public IcubToeOffParameters(IcubJointMap jointMap)
   {
      this.jointMap = jointMap;
   }

   @Override
   public boolean doToeOffIfPossible()
   {
      return true;
   }

   @Override
   public boolean doToeOffIfPossibleInSingleSupport()
   {
      return true;
   }

   @Override
   public boolean checkECMPLocationToTriggerToeOff()
   {
      return false;
   }

   @Override
   public double getICPPercentOfStanceForSSToeOff()
   {
      return 1.0;
   }

   @Override
   public boolean useToeOffLineContactInSwing()
   {
      return true;
   }

   @Override
   public double getMinStepLengthForToeOff()
   {
      return jointMap.getPhysicalProperties().getFootLength();
   }

   @Override
   public double getMaximumToeOffAngle()
   {
      return Math.toRadians(45.0);
   }

   @Override
   public boolean doToeOffWhenHittingAnkleLimit()
   {
      return true;
   }

}
