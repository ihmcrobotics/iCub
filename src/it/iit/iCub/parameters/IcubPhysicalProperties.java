package it.iit.iCub.parameters;

import javax.vecmath.Vector3d;

import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class IcubPhysicalProperties extends DRCRobotPhysicalProperties
{
	public static final double footsizeReduction = 0.0;
	   
	public static final double ankleHeight = 0.042;
	public static final double footLength = 0.18 - footsizeReduction; // new foot, is bigger than 3D model
	public static final double footBack = 0.045;
	public static final double footForward = footLength - footBack;
	public static final double footWidth = 0.08 - footsizeReduction; // new foot, is bigger than 3D model
	public static final double toeWidth = 0.08;
	public static final double thighLength = 0.213;
	public static final double shinLength = 0.226;
	public static final double pelvisToFoot = 0.481;

    public static final SideDependentList<RigidBodyTransform> soleToAnkleFrameTransforms = new SideDependentList<>();
    public static final SideDependentList<RigidBodyTransform> handControlFrameToWristTransforms = new SideDependentList<RigidBodyTransform>(new RigidBodyTransform(), new RigidBodyTransform());

    static
    {
       for (RobotSide side : RobotSide.values)
       {
          RigidBodyTransform soleToAnkleFrame = new RigidBodyTransform();
          soleToAnkleFrame.setEuler(new Vector3d(0.0, 0.0, 0.0)); // need to check this
          soleToAnkleFrame.setTranslation(new Vector3d(footLength / 2.0 - footBack, 0.0, -IcubPhysicalProperties.ankleHeight));
          soleToAnkleFrameTransforms.put(side, soleToAnkleFrame);
       }
    }
    
	@Override
	public double getAnkleHeight() 
	{
		return ankleHeight;
	}

	public static RigidBodyTransform getAnkleToSoleFrameTransform(RobotSide side) 
	{
		return soleToAnkleFrameTransforms.get(side);
	}
}