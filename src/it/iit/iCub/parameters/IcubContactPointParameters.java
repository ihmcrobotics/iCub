package it.iit.iCub.parameters;

import static it.iit.iCub.parameters.IcubPhysicalProperties.*;

import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.RobotContactPointParameters;

public class IcubContactPointParameters extends RobotContactPointParameters
{
   public IcubContactPointParameters(DRCRobotJointMap jointMap)
   {
      super(jointMap, footWidth, footLength, IcubPhysicalProperties.soleToAnkleFrameTransforms);
      createDefaultControllerFootContactPoints();
      createDefaultSimulationFootContactPoints();
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
