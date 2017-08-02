package it.iit.iCub.parameters;

import static us.ihmc.robotics.partNames.SpineJointName.SPINE_PITCH;
import static us.ihmc.robotics.partNames.SpineJointName.SPINE_ROLL;
import static us.ihmc.robotics.partNames.SpineJointName.SPINE_YAW;

import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootSE3Gains;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoPositionPIDGainsInterface;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.controllers.YoSymmetricSE3PIDGains;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class IcubWalkingControllerParameters extends WalkingControllerParameters
{
   private final boolean runningOnRealRobot;

   // Limits
   private final double pelvis_pitch_upper_limit = 1.46608;
   private final double pelvis_pitch_lower_limit = -0.383972;

   //TODO need to better tune this
   // USE THESE FOR Real Robot and sims when controlling playback height instead of CoM.
   private final double minimumHeightAboveGround; //= IcubRobotModel.SCALE_FACTOR * (0.4 - 0.02);// + 0.03;
   private double nominalHeightAboveGround; //= IcubRobotModel.SCALE_FACTOR * (0.49);// + 0.03;
   private final double maximumHeightAboveGround;// = IcubRobotModel.SCALE_FACTOR * (0.52);// + 0.03;

   private final IcubJointMap jointMap;

   private final ToeOffParameters toeOffParameters;
   private final SwingTrajectoryParameters swingTrajectoryParameters;

   public IcubWalkingControllerParameters(IcubJointMap jointMap)
   {
      this(jointMap, false);
   }

   public IcubWalkingControllerParameters(IcubJointMap jointMap, boolean runningOnRealRobot) // TODO: are wrong
   {
      this.runningOnRealRobot = runningOnRealRobot;
      this.jointMap = jointMap;

      minimumHeightAboveGround = jointMap.getModelScale() * (0.5);
      nominalHeightAboveGround = jointMap.getModelScale() * (0.55);
      maximumHeightAboveGround = jointMap.getModelScale() * (0.6);

      toeOffParameters = new IcubToeOffParameters(jointMap);
      swingTrajectoryParameters = new IcubSwingTrajectoryParameters(jointMap.getModelScale());
   }

   @Override
   public double getOmega0()
   {
      return 4.7;
   }

   @Override
   public double getTimeToGetPreparedForLocomotion()
   {
      return 0.0;
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
   public String[] getDefaultChestOrientationControlJointNames()
   {
      String[] defaultChestOrientationControlJointNames = new String[] {jointMap.getSpineJointName(SpineJointName.SPINE_YAW),
            jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), jointMap.getSpineJointName(SpineJointName.SPINE_ROLL)};

      return defaultChestOrientationControlJointNames;
   }

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
   public double getFootForwardOffset()
   {
      return jointMap.getPhysicalProperties().getFootForward();
   }

   @Override
   public double getFootBackwardOffset()
   {
      return jointMap.getPhysicalProperties().getFootBack();
   }

   @Override
   public double getAnkleHeight()
   {
      return jointMap.getPhysicalProperties().getAnkleHeight();
   }

   @Override
   public double getLegLength()
   {
      return jointMap.getPhysicalProperties().getShinLength() + jointMap.getPhysicalProperties().getThighLength();
   }

   @Override
   public double getInPlaceWidth()
   {
      return 0.2; //
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
      return 0.1;
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
      return 0.15;
   }

   @Override
   public double getMinSwingHeightFromStanceFoot()
   {
      return 0.05;
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
   public ICPControlGains createICPControlGains(YoVariableRegistry registry)
   {
      ICPControlGains gains = new ICPControlGains("", registry);

      double kpParallel = 2.5;
      double kpOrthogonal = 1.5;
      double ki = 0.0;
      double kiBleedOff = 0.0;

      gains.setKpParallelToMotion(kpParallel);
      gains.setKpOrthogonalToMotion(kpOrthogonal);
      gains.setKi(ki);
      gains.setKiBleedOff(kiBleedOff);

      //      boolean runningOnRealRobot = target == DRCRobotModel.RobotTarget.REAL_ROBOT;
      //      if (runningOnRealRobot) gains.setFeedbackPartMaxRate(1.0);
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
      gains.setMaximumFeedback(maxAccel);
      gains.setMaximumFeedbackRate(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   private YoOrientationPIDGainsInterface createHeadOrientationControlGains(YoVariableRegistry registry)
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
      gains.setMaximumFeedback(maxAccel);
      gains.setMaximumFeedbackRate(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   private YoPIDGains createHeadJointspaceControlGains(YoVariableRegistry registry)
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
      gains.setMaximumFeedback(maxAccel);
      gains.setMaximumFeedbackRate(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
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
      gains.setMaximumFeedback(maxAccel);
      gains.setMaximumFeedbackRate(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   private YoPIDGains createSpineControlGains(YoVariableRegistry registry)
   {
      double kp = 250.0;
      double zeta = 0.6;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      YoPIDGains spineGains = new YoPIDGains("SpineJointspace", registry);
      spineGains.setKp(kp);
      spineGains.setZeta(zeta);
      spineGains.setKi(ki);
      spineGains.setMaximumIntegralError(maxIntegralError);
      spineGains.setMaximumFeedback(maxAccel);
      spineGains.setMaximumFeedbackRate(maxJerk);
      spineGains.createDerivativeGainUpdater(true);

      return spineGains;
   }

   private YoPIDGains createArmControlGains(YoVariableRegistry registry)
   {
      YoPIDGains armGains = new YoPIDGains("ArmJointspace", registry);

      double kp = runningOnRealRobot ? 60.0 : 80.0;
      double zeta = runningOnRealRobot ? 0.4 : 0.6;
      double ki = runningOnRealRobot ? 0.0 : 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 200.0 : Double.POSITIVE_INFINITY;

      armGains.setKp(kp);
      armGains.setZeta(zeta);
      armGains.setKi(ki);
      armGains.setMaximumIntegralError(maxIntegralError);
      armGains.setMaximumFeedback(maxAccel);
      armGains.setMaximumFeedbackRate(maxJerk);
      armGains.createDerivativeGainUpdater(true);

      return armGains;
   }

   private YoOrientationPIDGainsInterface createHandOrientationControlGains(YoVariableRegistry registry)
   {
      YoSymmetricSE3PIDGains orientationGains = new YoSymmetricSE3PIDGains("HandOrientation", registry);

      double kp = 100.0;
      double zeta = runningOnRealRobot ? 0.6 : 1.0;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      orientationGains.setProportionalGain(kp);
      orientationGains.setDampingRatio(zeta);
      orientationGains.setIntegralGain(ki);
      orientationGains.setMaximumIntegralError(maxIntegralError);
      orientationGains.setMaximumFeedback(maxAccel);
      orientationGains.setMaximumFeedbackRate(maxJerk);
      orientationGains.createDerivativeGainUpdater(true);

      return orientationGains;
   }

   private YoPositionPIDGainsInterface createHandPositionControlGains(YoVariableRegistry registry)
   {
      YoSymmetricSE3PIDGains positionGains = new YoSymmetricSE3PIDGains("HandPosition", registry);

      double kp = 100.0;
      double zeta = runningOnRealRobot ? 0.6 : 1.0;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      positionGains.setProportionalGain(kp);
      positionGains.setDampingRatio(zeta);
      positionGains.setIntegralGain(ki);
      positionGains.setMaximumIntegralError(maxIntegralError);
      positionGains.setMaximumFeedback(maxAccel);
      positionGains.setMaximumFeedbackRate(maxJerk);
      positionGains.createDerivativeGainUpdater(true);

      return positionGains;
   }

   private Map<String, YoPIDGains> jointspaceGains = null;

   /** {@inheritDoc} */
   @Override
   public Map<String, YoPIDGains> getOrCreateJointSpaceControlGains(YoVariableRegistry registry)
   {
      if (jointspaceGains != null)
         return jointspaceGains;

      jointspaceGains = new HashMap<>();

      YoPIDGains spineGains = createSpineControlGains(registry);
      for (SpineJointName name : jointMap.getSpineJointNames())
         jointspaceGains.put(jointMap.getSpineJointName(name), spineGains);

      YoPIDGains headGains = createHeadJointspaceControlGains(registry);
      for (NeckJointName name : jointMap.getNeckJointNames())
         jointspaceGains.put(jointMap.getNeckJointName(name), headGains);

      YoPIDGains armGains = createArmControlGains(registry);
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ArmJointName name : jointMap.getArmJointNames())
            jointspaceGains.put(jointMap.getArmJointName(robotSide, name), armGains);
      }

      return jointspaceGains;
   }

   private Map<String, YoOrientationPIDGainsInterface> taskspaceAngularGains = null;

   /** {@inheritDoc} */
   @Override
   public Map<String, YoOrientationPIDGainsInterface> getOrCreateTaskspaceOrientationControlGains(YoVariableRegistry registry)
   {
      if (taskspaceAngularGains != null)
         return taskspaceAngularGains;

      taskspaceAngularGains = new HashMap<>();

      YoOrientationPIDGainsInterface chestAngularGains = createChestControlGains(registry);
      taskspaceAngularGains.put(jointMap.getChestName(), chestAngularGains);

      YoOrientationPIDGainsInterface headAngularGains = createHeadOrientationControlGains(registry);
      taskspaceAngularGains.put(jointMap.getHeadName(), headAngularGains);

      YoOrientationPIDGainsInterface handAngularGains = createHandOrientationControlGains(registry);
      for (RobotSide robotSide : RobotSide.values)
         taskspaceAngularGains.put(jointMap.getHandName(robotSide), handAngularGains);

      return taskspaceAngularGains;
   }

   private Map<String, YoPositionPIDGainsInterface> taskspaceLinearGains = null;

   /** {@inheritDoc} */
   @Override
   public Map<String, YoPositionPIDGainsInterface> getOrCreateTaskspacePositionControlGains(YoVariableRegistry registry)
   {
      if (taskspaceLinearGains != null)
         return taskspaceLinearGains;

      taskspaceLinearGains = new HashMap<>();

      YoPositionPIDGainsInterface handLinearGains = createHandPositionControlGains(registry);
      for (RobotSide robotSide : RobotSide.values)
         taskspaceLinearGains.put(jointMap.getHandName(robotSide), handLinearGains);

      return taskspaceLinearGains;
   }

   private TObjectDoubleHashMap<String> jointHomeConfiguration = null;

   /** {@inheritDoc} */
   @Override
   public TObjectDoubleHashMap<String> getOrCreateJointHomeConfiguration()
   {
      if (jointHomeConfiguration != null)
         return jointHomeConfiguration;

      jointHomeConfiguration = new TObjectDoubleHashMap<String>();

      jointHomeConfiguration.put(jointMap.getSpineJointName(SPINE_PITCH), 0.5);
      jointHomeConfiguration.put(jointMap.getSpineJointName(SPINE_ROLL), 0.0);
      jointHomeConfiguration.put(jointMap.getSpineJointName(SPINE_YAW), 0.0);

      for (NeckJointName name : jointMap.getNeckJointNames())
         jointHomeConfiguration.put(jointMap.getNeckJointName(name), 0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_PITCH), -0.3);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), 0.18);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), 0.0);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), 1.0);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_YAW), 0.0);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), 0.0);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_YAW), 0.0);
      }

      return jointHomeConfiguration;
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
      gains.setPositionMaxFeedbackAndFeedbackRate(maxPositionAcceleration, maxPositionJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatio(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxOrientationAcceleration, maxOrientationJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public double getSwingHeightMaxForPushRecoveryTrajectory()
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
      return runningOnRealRobot ? 1.5 : 0.25;
   }

   @Override
   public double getDefaultSwingTime()
   {
      return runningOnRealRobot ? 1.5 : 0.6;
   }

   /** @inheritDoc */
   @Override
   public double getDefaultInitialTransferTime()
   {
      return runningOnRealRobot ? 2.0 : 1.0;
   }

   @Override
   public double getSpineYawLimit()
   {
      return 0.959931;
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
      return jointMap.getPhysicalProperties().getFootWidth();
   }

   @Override
   public double getToeWidth()
   {
      return jointMap.getPhysicalProperties().getToeWidth();
   }

   @Override
   public double getFootLength()
   {
      return jointMap.getPhysicalProperties().getFootLength();
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
      return null;
   }

   @Override
   public double getContactThresholdForce()
   {
      return runningOnRealRobot ? 30.0 : 5.0;
   }

   @Override
   public double getSecondContactThresholdForceIgnoringCoP()
   {
      return 20.0;
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
      return new IcubMomentumOptimizationSettings(jointMap);
   }

   @Override
   public ICPAngularMomentumModifierParameters getICPAngularMomentumModifierParameters()
   {
      return null;
   }

   @Override
   public YoPDGains createCoMHeightControlGains(YoVariableRegistry registry)
   {
      YoPDGains gains = new YoPDGains("CoMHeight", registry);

      double kp = runningOnRealRobot ? 40.0 : 200.0;
      double zeta = runningOnRealRobot ? 0.4 : 1.0;
      double maxAcceleration = 0.5 * 9.81;
      double maxJerk = maxAcceleration / 0.05;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setMaximumFeedback(maxAcceleration);
      gains.setMaximumFeedbackRate(maxJerk);
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
      gains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatio(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);
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
      gains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatio(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);
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
      gains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatio(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
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
      gains.setMaximumFeedback(maxAcceleration);
      gains.setMaximumFeedbackRate(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   @Override
   public FootSwitchType getFootSwitchType()
   {
      return FootSwitchType.WrenchBased;
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
      return new SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>>();
   }

   @Override
   public LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits()
   {
      return new LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>>();
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

   @Override
   public boolean useOptimizationBasedICPController()
   {
      return false;
   }

   @Override
   public void useInverseDynamicsControlCore()
   {
      // once another mode is implemented, use this to change the default gains for inverse dynamics
   }

   @Override
   public void useVirtualModelControlCore()
   {
      // once another mode is implemented, use this to change the default gains for virtual model control
   }

   @Override
   public ToeOffParameters getToeOffParameters()
   {
      return toeOffParameters;
   }

   @Override
   public SwingTrajectoryParameters getSwingTrajectoryParameters()
   {
      return swingTrajectoryParameters;
   }
}
