package it.iit.iCub.hardwareDrivers;

import gnu.trove.map.TIntObjectMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import it.iit.iCub.parameters.IcubOrderedJointMap;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

import java.util.List;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class IcubIndexToJointMapTools
{
   public static TIntObjectMap<OneDoFJoint> createMapping(List<OneDoFJoint> oneDoFJointList)
   {
      TIntObjectHashMap<OneDoFJoint> map = new TIntObjectHashMap<>(IcubOrderedJointMap.numberOfJoints);

      for (OneDoFJoint oneDoFJoint : oneDoFJointList)
      {
         String[] jointNames = IcubOrderedJointMap.jointNames;
         for (int i = 0; i < jointNames.length; i++)
         {
            if(jointNames[i].equals(oneDoFJoint.getName()))
            {
               System.out.println("creating mapping for " + jointNames[i]);
               map.put(i, oneDoFJoint);
            }
         }
      }

      return map;
   }
}
