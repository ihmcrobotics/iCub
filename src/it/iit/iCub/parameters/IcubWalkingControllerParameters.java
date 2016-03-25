package it.iit.iCub.parameters;

import java.util.LinkedHashMap;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootSE3Gains;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class IcubWalkingControllerParameters implements WalkingControllerParameters
{
   private final boolean runningOnRealRobot;
   private final SideDependentList<RigidBodyTransform> handPosesWithRespectToChestFrame = new SideDependentList<RigidBodyTransform>();

   // Limits
   private final double neck_pitch_upper_limit = 0.523599;
   private final double neck_pitch_lower_limit = -0.698132;
   private final double head_yaw_limit = 0.959931;
   private final double head_roll_limit = 1.0472; // take the smaller
   private final double pelvis_pitch_upper_limit = 1.46608;
   private final double pelvis_pitch_lower_limit = -0.383972;

   private final double min_leg_length_before_collapsing_single_support = 0.25; //TODO tune
   private final double min_mechanical_leg_length = 0.20; // TODO tune

   private final DRCRobotJointMap jointMap;

   public IcubWalkingControllerParameters(DRCRobotJointMap jointMap)
   {
      this(jointMap, false);
   }

   public IcubWalkingControllerParameters(DRCRobotJointMap jointMap, boolean runningOnRealRobot) // TODO: are wrong
   {
      this.runningOnRealRobot = runningOnRealRobot;
      this.jointMap = jointMap;

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform transform = new RigidBodyTransform();

         double x = 0.10;
         double y = robotSide.negateIfRightSide(0.15); //0.30);
         double z = -0.20;
         transform.setTranslation(new Vector3d(x, y, z));

         Matrix3d rotation = new Matrix3d();
         double yaw = 0.0;//robotSide.negateIfRightSide(-1.7);
         double pitch = 0.7;
         double roll = 0.0;//robotSide.negateIfRightSide(-0.8);
         RotationTools.convertYawPitchRollToMatrix(yaw, pitch, roll, rotation);
         transform.setRotation(rotation);

         handPosesWithRespectToChestFrame.put(robotSide, transform);
      }
   }

   @Override
   public double getOmega0()
   {
      // TODO probably need to be tuned.
      return 3.4;
   }

   @Override
   public double getTimeToGetPreparedForLocomotion()
   {
      return 0.0;
   }

   @Override
   public boolean doToeOffIfPossible()
   {
      return true;
   }

   @Override
   public boolean doToeOffIfPossibleInSingleSupport()
   {
      return false;
   }

   @Override
   public boolean checkECMPLocationToTriggerToeOff()
   {
      return true;
   }

   @Override
   public double getMinStepLengthForToeOff()
   {
      return getFootLength();
   }

   @Override
   public double getMaximumToeOffAngle()
   {
      return Math.toRadians(45.0);
   }

   @Override
   public boolean doToeTouchdownIfPossible()
   {
      return false;
   }

   @Override
   public double getToeTouchdownAngle()
   {
      return Math.toRadians(20.0);
   }

   @Override
   public boolean doHeelTouchdownIfPossible()
   {
      return false;
   }

   @Override
   public double getHeelTouchdownAngle()
   {
      return Math.toRadians(-20.0);
   }

   @Override
   public boolean allowShrinkingSingleSupportFootPolygon()
   {
      return false;
   }

   @Override
   public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
   {
      return false;
   }

   @Override
   public boolean allowAutomaticManipulationAbort()
   {
      return false;
   }

   @Override
   public double getICPErrorThresholdToSpeedUpSwing()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getMinimumSwingTimeForDisturbanceRecovery()
   {
      return getDefaultSwingTime();
   }

   @Override
   public boolean isNeckPositionControlled()
   {
      return false;
   }

   @Override
   public String[] getDefaultHeadOrientationControlJointNames()
   {
      return new String[] { jointMap.getNeckJointName(NeckJointName.LOWER_NECK_PITCH), jointMap.getNeckJointName(NeckJointName.NECK_ROLL),
            jointMap.getNeckJointName(NeckJointName.NECK_YAW) };
   }

   @Override
   public String[] getDefaultChestOrientationControlJointNames()
   {
      String[] defaultChestOrientationControlJointNames = new String[] { jointMap.getSpineJointName(SpineJointName.SPINE_YAW),
            jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), jointMap.getSpineJointName(SpineJointName.SPINE_ROLL) };

      return defaultChestOrientationControlJointNames;
   }

   //TODO need to better tune this
   // USE THESE FOR Real Robot and sims when controlling pelvis height instead of CoM.
   private final double minimumHeightAboveGround = 0.4 + 0.03;
   private double nominalHeightAboveGround = 0.49 + 0.03;
   private final double maximumHeightAboveGround = 0.65 + 0.03;

   @Override
   public double minimumHeightAboveAnkle()
   {
      return minimumHeightAboveGround;
   }

   @Override
   public double nominalHeightAboveAnkle()
   {
      return nominalHeightAboveGround;
   }

   @Override
   public double maximumHeightAboveAnkle()
   {
      return maximumHeightAboveGround;
   }

   @Override
   public double defaultOffsetHeightAboveAnkle()
   {
      return 0.0;
   }

   public void setNominalHeightAboveAnkle(double nominalHeightAboveAnkle)
   {
      this.nominalHeightAboveGround = nominalHeightAboveAnkle;
   }

   @Override
   public double getNeckPitchUpperLimit()
   {
      return neck_pitch_upper_limit;
   }

   @Override
   public double getNeckPitchLowerLimit()
   {
      return neck_pitch_lower_limit;
   }

   @Override
   public double getHeadYawLimit()
   {
      return head_yaw_limit;
   }

   @Override
   public double getHeadRollLimit()
   {
      return head_roll_limit;
   }

   @Override
   public double getFootForwardOffset()
   {
      return IcubPhysicalProperties.footForward;
   }

   @Override
   public double getFootBackwardOffset()
   {
      return IcubPhysicalProperties.footBack;
   }

   @Override
   public double getAnkleHeight()
   {
      return IcubPhysicalProperties.ankleHeight;
   }

   @Override
   public double getLegLength()
   {
      return IcubPhysicalProperties.shinLength + IcubPhysicalProperties.thighLength;
   }

   @Override
   public double getMinLegLengthBeforeCollapsingSingleSupport()
   {
      return min_leg_length_before_collapsing_single_support;
   }

   @Override
   public double getInPlaceWidth()
   {
      return 0.15; //
   }

   @Override
   public double getDesiredStepForward()
   {
      return 0.15; //0.35;
   }

   @Override
   public double getMaxStepLength()
   {
      return 0.2; //0.5; //0.35;
   }

   @Override
   public double getDefaultStepLength()
   {
      return .18;
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.2;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.3; //0.4;
   }

   @Override
   public double getStepPitch()
   {
      return 0.0;
   }

   @Override
   public double getMaxStepUp()
   {
      return 0.05;
   }

   @Override
   public double getMaxStepDown()
   {
      return 0.05;
   }

   @Override
   public double getMaxSwingHeightFromStanceFoot()
   {
      return 0.2;
   }

   @Override
   public double getMaxAngleTurnOutwards()
   {
      return 0.0;
   }

   @Override
   public double getMaxAngleTurnInwards()
   {
      return 0.0;
   }

   @Override
   public double getMinAreaPercentForValidFootstep()
   {
      return 0.5;
   }

   @Override
   public double getDangerAreaPercentForValidFootstep()
   {
      return 0.75;
   }

   @Override
   public ICPControlGains getICPControlGains()
   {
      ICPControlGains gains = new ICPControlGains();

      double kp = runningOnRealRobot ? 1.0 : 1.0;
      double ki = runningOnRealRobot ? 4.0 : 4.0;
      double kiBleedOff = 0.9;

      gains.setKpParallelToMotion(kp);
      gains.setKpOrthogonalToMotion(kp);
      gains.setKi(ki);
      gains.setKiBleedOff(kiBleedOff);

      return gains;
   }

   @Override
   public YoPDGains createPelvisICPBasedXYControlGains(YoVariableRegistry registry)
   {
      YoPDGains gains = new YoPDGains("PelvisXY", registry);

      gains.setKp(4.0);
      gains.setKd(runningOnRealRobot ? 0.5 : 1.2);

      return gains;
   }

   @Override
   public YoOrientationPIDGainsInterface createPelvisOrientationControlGains(YoVariableRegistry registry)
   {
      YoSymmetricSE3PIDGains gains = new YoSymmetricSE3PIDGains("PelvisOrientation", registry);

      double kp = 80.0;
      double zeta = runningOnRealRobot ? 0.25 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 12.0 : 36.0;
      double maxJerk = runningOnRealRobot ? 180.0 : 540.0;

      gains.setProportionalGain(kp);
      gains.setDampingRatio(zeta);
      gains.setIntegralGain(ki);
      gains.setMaximumIntegralError(maxIntegralError);
      gains.setMaximumAcceleration(maxAccel);
      gains.setMaximumJerk(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public YoOrientationPIDGainsInterface createHeadOrientationControlGains(YoVariableRegistry registry)
   {
      YoSymmetricSE3PIDGains gains = new YoSymmetricSE3PIDGains("HeadOrientation", registry);

      double kp = 40.0;
      double zeta = runningOnRealRobot ? 0.4 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 6.0 : 36.0;
      double maxJerk = runningOnRealRobot ? 60.0 : 540.0;

      gains.setProportionalGain(kp);
      gains.setDampingRatio(zeta);
      gains.setIntegralGain(ki);
      gains.setMaximumIntegralError(maxIntegralError);
      gains.setMaximumAcceleration(maxAccel);
      gains.setMaximumJerk(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public YoPIDGains createHeadJointspaceControlGains(YoVariableRegistry registry)
   {
      YoPIDGains gains = new YoPIDGains("HeadJointspace", registry);

      double kp = 40.0;
      double zeta = runningOnRealRobot ? 0.4 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 6.0 : 36.0;
      double maxJerk = runningOnRealRobot ? 60.0 : 540.0;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setKi(ki);
      gains.setMaximumIntegralError(maxIntegralError);
      gains.setMaximumAcceleration(maxAccel);
      gains.setMaximumJerk(maxJerk);
      gains.createDerivativeGainUpdater(true);
      
      return gains;
   }

   @Override
   public double getTrajectoryTimeHeadOrientation()
   {
      return 3.0;
   }

   @Override
   public double[] getInitialHeadYawPitchRoll()
   {
      return new double[] { 0.0, 0.0, 0.0 };
   }

   @Override
   public YoOrientationPIDGainsInterface createChestControlGains(YoVariableRegistry registry)
   {
      YoSymmetricSE3PIDGains gains = new YoSymmetricSE3PIDGains("ChestOrientation", registry);

      double kp = 80.0;
      double zeta = runningOnRealRobot ? 0.25 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 6.0 : 36.0;
      double maxJerk = runningOnRealRobot ? 60.0 : 540.0;

      gains.setProportionalGain(kp);
      gains.setDampingRatio(zeta);
      gains.setIntegralGain(ki);
      gains.setMaximumIntegralError(maxIntegralError);
      gains.setMaximumAcceleration(maxAccel);
      gains.setMaximumJerk(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public YoSE3PIDGainsInterface createSwingFootControlGains(YoVariableRegistry registry)
   {
      YoFootSE3Gains gains = new YoFootSE3Gains("SwingFoot", registry);

      double kpXY = 100.0;
      double kpZ = runningOnRealRobot ? 200.0 : 200.0;
      double zetaXYZ = runningOnRealRobot ? 0.3 : 0.7;
      double kpXYOrientation = runningOnRealRobot ? 300.0 : 300.0;
      double kpZOrientation = runningOnRealRobot ? 40.0 : 200.0;
      double zetaOrientation = runningOnRealRobot ? 0.3 : 0.7;
      double maxPositionAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxPositionJerk = runningOnRealRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxOrientationAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxOrientationJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      gains.setPositionProportionalGains(kpXY, kpZ);
      gains.setPositionDampingRatio(zetaXYZ);
      gains.setPositionMaxAccelerationAndJerk(maxPositionAcceleration, maxPositionJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatio(zetaOrientation);
      gains.setOrientationMaxAccelerationAndJerk(maxOrientationAcceleration, maxOrientationJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public double getSwingHeightMaxForPushRecoveryTrajectory()
   {
      return 0.05;
   }

   public double getSwingMaxHeightForPushRecoveryTrajectory()
   {
      return 0.05;
   }

   @Override
   public boolean doPrepareManipulationForLocomotion()
   {
      return true;
   }

   @Override
   public double getDefaultTransferTime()
   {
      return runningOnRealRobot ? 1.5 : 0.35;
   }

   @Override
   public double getDefaultSwingTime()
   {
      return runningOnRealRobot ? 1.5 : 1.0;
   }

   @Override
   public double getSpineYawLimit()
   {
      return 0;
   }

   @Override
   public double getSpineRollLimit()
   {
      return 0;
   }

   @Override
   public double getSpinePitchUpperLimit()
   {
      return pelvis_pitch_upper_limit;
   }

   @Override
   public double getSpinePitchLowerLimit()
   {
      return pelvis_pitch_lower_limit;
   }

   @Override
   public boolean isSpinePitchReversed()
   {
      return false;
   }

   @Override
   public double getFootWidth()
   {
      return IcubPhysicalProperties.footWidth;
   }

   @Override
   public double getToeWidth()
   {
      return IcubPhysicalProperties.toeWidth;
   }

   @Override
   public double getFootLength()
   {
      return IcubPhysicalProperties.footLength;
   }

   @Override
   public double getActualFootWidth()
   {
      return getFootWidth();
   }

   @Override
   public double getActualFootLength()
   {
      return getFootLength();
   }

   @Override
   public double getFootstepArea()
   {
      return (getToeWidth() + getFootWidth()) * getFootLength() / 2.0;
   }

   @Override
   public double getFoot_start_toetaper_from_back()
   {
      return 0.0;
   }

   @Override
   public double getSideLengthOfBoundingBoxForFootstepHeight()
   {
      return (1 + 0.3) * 2 * Math.sqrt(getFootForwardOffset() * getFootForwardOffset() + 0.25 * getFootWidth() * getFootWidth());
   }

   @Override
   public SideDependentList<RigidBodyTransform> getDesiredHandPosesWithRespectToChestFrame()
   {
      return handPosesWithRespectToChestFrame;
   }

   @Override
   public double getDesiredTouchdownHeightOffset()
   {
      return 0;
   }

   @Override
   public double getDesiredTouchdownVelocity()
   {
      return -0.3;
   }

   @Override
   public double getDesiredTouchdownAcceleration()
   {
      return 0;
   }

   @Override
   public double getContactThresholdForce()
   {
      return runningOnRealRobot ? 30.0 : 5.0;
   }

   @Override
   public double getSecondContactThresholdForceIgnoringCoP()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getCoPThresholdFraction()
   {
      return 0.02;
   }

   @Override
   public String[] getJointsToIgnoreInController()
   {
      return null;
   }

   @Override
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      MomentumOptimizationSettings momentumOptimizationSettings = new MomentumOptimizationSettings();
      return momentumOptimizationSettings;
   }

   @Override
   public YoPDGains createCoMHeightControlGains(YoVariableRegistry registry)
   {
      YoPDGains gains = new YoPDGains("CoMHeight", registry);

      double kp = runningOnRealRobot ? 40.0 : 50.0;
      double zeta = runningOnRealRobot ? 0.4 : 1.0;
      double maxAcceleration = 0.5 * 9.81;
      double maxJerk = maxAcceleration / 0.05;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setMaximumAcceleration(maxAcceleration);
      gains.setMaximumJerk(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public boolean getCoMHeightDriftCompensation()
   {
      return false;
   }

   @Override
   public YoSE3PIDGainsInterface createHoldPositionFootControlGains(YoVariableRegistry registry)
   {
      YoFootSE3Gains gains = new YoFootSE3Gains("HoldFoot", registry);

      double kpXY = 100.0;
      double kpZ = 0.0;
      double zetaXYZ = runningOnRealRobot ? 0.2 : 1.0;
      double kpXYOrientation = runningOnRealRobot ? 40.0 : 100.0;
      double kpZOrientation = runningOnRealRobot ? 40.0 : 100.0;
      double zetaOrientation = runningOnRealRobot ? 0.2 : 1.0;
      double maxLinearAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = runningOnRealRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      gains.setPositionProportionalGains(kpXY, kpZ);
      gains.setPositionDampingRatio(zetaXYZ);
      gains.setPositionMaxAccelerationAndJerk(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatio(zetaOrientation);
      gains.setOrientationMaxAccelerationAndJerk(maxAngularAcceleration, maxAngularJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public YoSE3PIDGainsInterface createToeOffFootControlGains(YoVariableRegistry registry)
   {
      YoFootSE3Gains gains = new YoFootSE3Gains("ToeOffFoot", registry);

      double kpXY = 100.0;
      double kpZ = 0.0;
      double zetaXYZ = runningOnRealRobot ? 0.4 : 0.4;
      double kpXYOrientation = runningOnRealRobot ? 200.0 : 200.0;
      double kpZOrientation = runningOnRealRobot ? 200.0 : 200.0;
      double zetaOrientation = runningOnRealRobot ? 0.4 : 0.4;
      double maxLinearAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = runningOnRealRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      gains.setPositionProportionalGains(kpXY, kpZ);
      gains.setPositionDampingRatio(zetaXYZ);
      gains.setPositionMaxAccelerationAndJerk(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatio(zetaOrientation);
      gains.setOrientationMaxAccelerationAndJerk(maxAngularAcceleration, maxAngularJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public YoSE3PIDGainsInterface createEdgeTouchdownFootControlGains(YoVariableRegistry registry)
   {
      YoFootSE3Gains gains = new YoFootSE3Gains("EdgeTouchdownFoot", registry);

      double kp = 0.0;
      double zetaXYZ = runningOnRealRobot ? 0.0 : 0.0;
      double kpXYOrientation = runningOnRealRobot ? 40.0 : 300.0;
      double kpZOrientation = runningOnRealRobot ? 40.0 : 300.0;
      double zetaOrientation = runningOnRealRobot ? 0.4 : 0.4;
      double maxLinearAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = runningOnRealRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      gains.setPositionProportionalGains(kp, kp);
      gains.setPositionDampingRatio(zetaXYZ);
      gains.setPositionMaxAccelerationAndJerk(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatio(zetaOrientation);
      gains.setOrientationMaxAccelerationAndJerk(maxAngularAcceleration, maxAngularJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public boolean doToeOffWhenHittingAnkleLimit()
   {
      return false;
   }

   @Override
   public double getMinMechanicalLegLength()
   {
      return min_mechanical_leg_length;
   }

   @Override
   public boolean doFancyOnToesControl()
   {
      return false;
   }

   @Override
   public YoPDGains createUnconstrainedJointsControlGains(YoVariableRegistry registry)
   {
      YoPDGains gains = new YoPDGains("UnconstrainedJoints", registry);

      double kp = runningOnRealRobot ? 80.0 : 100.0;
      double zeta = runningOnRealRobot ? 0.6 : 0.8;
      double maxAcceleration = runningOnRealRobot ? 18.0 : 18.0;
      double maxJerk = runningOnRealRobot ? 270.0 : 270.0;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setMaximumAcceleration(maxAcceleration);
      gains.setMaximumJerk(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public FootSwitchType getFootSwitchType()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public double getContactThresholdHeight()
   {
      // TODO Auto-generated method stub
      return 0;
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportX()
   {
      return 0.035;
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportY()
   {
      return 0.035;
   }

   @Override
   public boolean finishSingleSupportWhenICPPlannerIsDone()
   {
      return false;
   }

   @Override
   public double pelvisToAnkleThresholdForWalking()
   {
      return 0;
   }
   
   @Override
   public boolean controlHeadAndHandsWithSliders()
   {
      return false;
   }

   @Override
   public SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> getSliderBoardControlledFingerJointsWithLimits()
   {
      return new SideDependentList<LinkedHashMap<String, ImmutablePair<Double,Double>>>();
   }

   @Override
   public LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits()
   {
      return new LinkedHashMap<NeckJointName, ImmutablePair<Double,Double>>();
   }

   /** {@inheritDoc} */
   @Override
   public double getHighCoPDampingDurationToPreventFootShakies()
   {
      return -1.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getCoPErrorThresholdForHighCoPDamping()
   {
      return Double.POSITIVE_INFINITY;
   }
}
