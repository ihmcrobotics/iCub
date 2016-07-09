package it.iit.iCub.parameters;

import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing;
import us.ihmc.sensorProcessing.sensorProcessors.SensorProcessing.SensorType;
import us.ihmc.sensorProcessing.simulatedSensors.SensorNoiseParameters;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotSide.SideDependentList;

public class IcubStateEstimatorParameters implements StateEstimatorParameters
{
   private final boolean runningOnRealRobot;

   private final double estimatorDT;

   private final double jointVelocitySlopTimeForBacklashCompensation;

   private final double defaultFilterBreakFrequency;

   // private final SensorNoiseParameters sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorJerryTuning();
   // private SensorNoiseParameters sensorNoiseParameters = DRCSimulatedSensorNoiseParameters.createNoiseParametersForEstimatorJerryTuningSeptember2013();
   private SensorNoiseParameters sensorNoiseParameters = null;

   public IcubStateEstimatorParameters(boolean runningOnRealRobot, double estimatorDT)
   {
      this.runningOnRealRobot = runningOnRealRobot;

      this.estimatorDT = estimatorDT;

      defaultFilterBreakFrequency = runningOnRealRobot ? 16.0 : Double.POSITIVE_INFINITY;

      jointVelocitySlopTimeForBacklashCompensation = 0.03;
   }

   public void configureSensorProcessing(SensorProcessing sensorProcessing)
   {
      YoVariableRegistry registry = sensorProcessing.getYoVariableRegistry();

      DoubleYoVariable jointPositionAlphaFilter = sensorProcessing.createAlphaFilter("jointPositionAlphaFilter", defaultFilterBreakFrequency);
      DoubleYoVariable jointVelocityAlphaFilter = sensorProcessing.createAlphaFilter("jointVelocityAlphaFilter", defaultFilterBreakFrequency);
      DoubleYoVariable jointVelocitySlopTime = new DoubleYoVariable("jointBacklashSlopTime", registry);
      jointVelocitySlopTime.set(jointVelocitySlopTimeForBacklashCompensation);

      DoubleYoVariable orientationAlphaFilter = sensorProcessing.createAlphaFilter("orientationAlphaFilter", defaultFilterBreakFrequency);
      DoubleYoVariable angularVelocityAlphaFilter = sensorProcessing.createAlphaFilter("angularVelocityAlphaFilter", defaultFilterBreakFrequency);
      DoubleYoVariable linearAccelerationAlphaFilter = sensorProcessing.createAlphaFilter("linearAccelerationAlphaFilter", defaultFilterBreakFrequency);

      sensorProcessing.addSensorAlphaFilter(jointPositionAlphaFilter, false, SensorType.JOINT_POSITION);

      sensorProcessing.addSensorAlphaFilter(jointVelocityAlphaFilter, false, SensorType.JOINT_VELOCITY);
      sensorProcessing.computeJointAccelerationFromFiniteDifference(jointVelocityAlphaFilter, false);

      sensorProcessing.addSensorAlphaFilter(orientationAlphaFilter, false, SensorType.IMU_ORIENTATION);
      sensorProcessing.addSensorAlphaFilter(angularVelocityAlphaFilter, false, SensorType.IMU_ANGULAR_VELOCITY);
      sensorProcessing.addSensorAlphaFilter(linearAccelerationAlphaFilter, false, SensorType.IMU_LINEAR_ACCELERATION);
   }

   @Override
   public SensorNoiseParameters getSensorNoiseParameters()
   {
      return sensorNoiseParameters;
   }

   @Override
   public double getEstimatorDT()
   {
      return estimatorDT;
   }

   @Override
   public boolean isRunningOnRealRobot()
   {
      return runningOnRealRobot;
   }

   @Override
   public double getKinematicsPelvisPositionFilterFreqInHertz()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getKinematicsPelvisLinearVelocityFilterFreqInHertz()
   {
      return 16.0;
   }

   @Override
   public double getCoPFilterFreqInHertz()
   {
      return 4.0;
   }

   @Override
   public boolean enableIMUBiasCompensation()
   {
      return false;
   }

   @Override
   public boolean enableIMUYawDriftCompensation()
   {
      return false;
   }

   @Override
   public double getIMUBiasFilterFreqInHertz()
   {
      return 6.0e-3;
   }

   @Override
   public double getIMUYawDriftFilterFreqInHertz()
   {
      return 1.0e-3;
   }

   @Override
   public double getIMUBiasVelocityThreshold()
   {
      return 0.015;
   }

   @Override
   public boolean useAccelerometerForEstimation()
   {
      return true;
   }

   @Override
   public boolean cancelGravityFromAccelerationMeasurement()
   {
      return true;
   }

   @Override
   public double getPelvisPositionFusingFrequency()
   {
      return 11.7893; // alpha = 0.8 with dt = 0.003
   }

   @Override
   public double getPelvisLinearVelocityFusingFrequency()
   {
      return 0.4261; // alpha = 0.992 with dt = 0.003
   }

   @Override
   public double getPelvisVelocityBacklashSlopTime()
   {
      return jointVelocitySlopTimeForBacklashCompensation;
   }

   @Override
   public double getDelayTimeForTrustingFoot()
   {
      return 0.02;
   }

   @Override
   public double getForceInPercentOfWeightThresholdToTrustFoot()
   {
      return 0.3;
   }

   @Override
   public boolean trustCoPAsNonSlippingContactPoint()
   {
      return true;
   }

   @Override
   public boolean useControllerDesiredCenterOfPressure()
   {
      return false;
   }

   @Override
   public boolean useTwistForPelvisLinearStateEstimation()
   {
      return true;
   }

   @Override
   public double getPelvisLinearVelocityAlphaNewTwist()
   {
      return 0.15;
   }

   @Override
   public boolean createFusedIMUSensor()
   {
      return false;
   }

   @Override
   public double getContactThresholdForce()
   {
      return 10.0;
   }

   @Override
   public double getFootSwitchCoPThresholdFraction()
   {
      return 0.02;
   }

   @Override
   public ImmutablePair<String, String> getIMUsForSpineJointVelocityEstimation()
   {
      // TODO For Valkyrie. Probably have to make more generic.
      return null;
   }

   @Override
   public boolean useIMUsForSpineJointVelocityEstimation()
   {
      // TODO For Valkyrie. Probably have to make more generic.
      return false;
   }

   @Override
   public double getAlphaIMUsForSpineJointVelocityEstimation()
   {
      // TODO For Valkyrie. Probably have to make more generic.
      return 0;
   }

   @Override
   public double getContactThresholdHeight()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public FootSwitchType getFootSwitchType()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public boolean requestWristForceSensorCalibrationAtStart()
   {
      return false;
   }

   @Override
   public SideDependentList<String> getWristForceSensorNames()
   {
      return null;
   }

   @Override
   public boolean requestFootForceSensorCalibrationAtStart()
   {
      return false;
   }

   @Override
   public SideDependentList<String> getFootForceSensorNames()
   {
      return null;
   }

   @Override
   public boolean getPelvisLinearStateUpdaterTrustImuWhenNoFeetAreInContact()
   {
      return false;
   }
}
