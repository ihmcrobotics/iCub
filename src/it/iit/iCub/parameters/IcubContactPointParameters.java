package it.iit.iCub.parameters;

import static it.iit.iCub.parameters.IcubPhysicalProperties.footLength;
import static it.iit.iCub.parameters.IcubPhysicalProperties.footWidth;

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
}
