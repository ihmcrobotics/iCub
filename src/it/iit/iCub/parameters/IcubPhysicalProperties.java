package it.iit.iCub.parameters;

import us.ihmc.avatar.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class IcubPhysicalProperties implements DRCRobotPhysicalProperties
{
   public static final double footsizeReduction = 0.0;
   
   private final double scale;
   private final double massScalePower;
   
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

   public IcubPhysicalProperties(double scale, double massScalePower)
   {
      this.scale = scale;
      this.massScalePower = massScalePower;
      ankleHeight = scale * 0.042;
      footLength = scale * (0.18 - footsizeReduction); // new foot, is bigger than 3D model         
      footBack = scale * 0.045;
      footForward = footLength - footBack;
      footWidth = scale * (0.08 - footsizeReduction); // new foot, is bigger than 3D model          
      toeWidth = scale * 0.08;
      thighLength = scale * 0.213;
      shinLength = scale * 0.226;
      pelvisToFoot = scale * 0.481;

      for (RobotSide side : RobotSide.values)
      {
         RigidBodyTransform soleToAnkleFrame = new RigidBodyTransform();
         soleToAnkleFrame.setRotationEulerAndZeroTranslation(new Vector3D(0.0, 0.0, 0.0)); // need to check this
         soleToAnkleFrame.setTranslation(new Vector3D(footLength / 2.0 - footBack, 0.0, -ankleHeight));
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

   public double getModelScale()
   {
      return scale;
   }
   
   public double getMassScalePower()
   {
      return massScalePower;
   }

}