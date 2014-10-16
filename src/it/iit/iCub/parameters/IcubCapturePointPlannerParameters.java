package it.iit.iCub.parameters;

import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;

public class IcubCapturePointPlannerParameters implements CapturePointPlannerParameters
{
	private boolean runningOnRealRobot;

	public IcubCapturePointPlannerParameters(boolean runningOnRealRobot)
	{
		this.runningOnRealRobot = runningOnRealRobot;
	}
	
	@Override
	public double getDoubleSupportInitialTransferDuration()
	{
		return runningOnRealRobot ? 2.0 : 1.0; 
	}

	@Override
	public double getDoubleSupportDuration()
	{
		return runningOnRealRobot ? 1.5 : 0.5;
	}

	@Override
	public double getSingleSupportDuration()
	{
		return runningOnRealRobot ? 1.5 : 0.7;
	}

	@Override
	public int getNumberOfFootstepsToConsider()
	{
		return 3;
	}

	@Override
	public int getNumberOfCoefficientsForDoubleSupportPolynomialTrajectory()
	{
		return 5;
	}

	@Override
	public int getNumberOfFootstepsToStop()
	{
		return 2;
	}

	@Override
	public double getIsDoneTimeThreshold()
	{
		return -1e-4;
	}

	@Override
	public double getDoubleSupportSplitFraction() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getFreezeTimeFactor() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public double getMaxInstantaneousCapturePointErrorForStartingSwing() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public boolean getDoTimeFreezing() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean getDoFootSlipCompensation() {
		// TODO Auto-generated method stub
		return false;
	}

}
