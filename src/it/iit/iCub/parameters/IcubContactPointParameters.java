package it.iit.iCub.parameters;

import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class IcubContactPointParameters extends RobotContactPointParameters
{
   public IcubContactPointParameters(IcubJointMap jointMap)
   {
      super(jointMap, jointMap.getPhysicalProperties().getFootWidth(), jointMap.getPhysicalProperties().getFootLength(), jointMap.getPhysicalProperties().getSoleToAnkleFrameTransforms());
      createDefaultFootContactPoints();
   }

   @Override
   public void setupGroundContactModelParameters(LinearGroundContactModel linearGroundContactModel)
   {
      linearGroundContactModel.setZStiffness(500.0);
      linearGroundContactModel.setZDamping(150.0);
      linearGroundContactModel.setXYStiffness(5000.0);
      linearGroundContactModel.setXYDamping(200.0);
   }
}
