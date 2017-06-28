package it.iit.iCub.parameters;

import us.ihmc.avatar.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class IcubPhysicalProperties implements DRCRobotPhysicalProperties
{
   public static final double footsizeReduction = 0.0;

   private final double ankleHeight;
   private final double footLength;
   private final double footBack;
   private final double footForward;
   private final double footWidth;
   private final double toeWidth;
   private final double thighLength;
   private final double shinLength;
   private final double pelvisToFoot;

   private final SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms = new SideDependentList<>();
   private final SideDependentList<RigidBodyTransform> handControlFrameToWristTransforms = new SideDependentList<RigidBodyTransform>(new RigidBodyTransform(), new RigidBodyTransform());

   public IcubPhysicalProperties()
   {
      ankleHeight = 0.042;
      footLength = 0.15;
      footBack = 0.045;
      footForward = footLength - footBack;
      footWidth = 0.07;
      toeWidth = 0.0;
      thighLength = 0.213;
      shinLength = 0.226;
      pelvisToFoot = 0.481;

      for (RobotSide side : RobotSide.values)
      {
         RigidBodyTransform soleToAnkleFrame = new RigidBodyTransform();
         soleToAnkleFrame.setTranslation(new Vector3D(side.negateIfLeftSide(0.0333) + 0.02, side.negateIfLeftSide(-0.005), -0.07));
         soleToAnkleFrameTransforms.put(side, soleToAnkleFrame);
      }
   }

   @Override
   public double getAnkleHeight()
   {
      return ankleHeight;
   }

   public RigidBodyTransform getSoleToAnkleFrameTransform(RobotSide side)
   {
      return soleToAnkleFrameTransforms.get(side);
   }

   public double getFootLength()
   {
      return footLength;
   }

   public double getFootBack()
   {
      return footBack;
   }

   public double getFootForward()
   {
      return footForward;
   }

   public double getFootWidth()
   {
      return footWidth;
   }

   public double getToeWidth()
   {
      return toeWidth;
   }

   public double getThighLength()
   {
      return thighLength;
   }

   public double getShinLength()
   {
      return shinLength;
   }

   public double getPelvisToFoot()
   {
      return pelvisToFoot;
   }

   public RigidBodyTransform getHandControlFrameToWristTransform(RobotSide robotSide)
   {
      return handControlFrameToWristTransforms.get(robotSide);
   }

   public SideDependentList<RigidBodyTransform> getSoleToAnkleFrameTransforms()
   {
      return soleToAnkleFrameTransforms;
   }
}