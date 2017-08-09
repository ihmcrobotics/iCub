package it.iit.iCub.parameters;

import static us.ihmc.robotics.partNames.SpineJointName.SPINE_PITCH;
import static us.ihmc.robotics.partNames.SpineJointName.SPINE_ROLL;
import static us.ihmc.robotics.partNames.SpineJointName.SPINE_YAW;

import java.util.HashMap;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootSE3Gains;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.geometry.Pose3D;
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
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class IcubWalkingControllerParameters extends WalkingControllerParameters
{
   private final boolean runningOnRealRobot;

   //TODO need to better tune this
   // USE THESE FOR Real Robot and sims when controlling playback height instead of CoM.
   private final double minimumHeightAboveGround; //= IcubRobotModel.SCALE_FACTOR * (0.4 - 0.02);// + 0.03;
   private double nominalHeightAboveGround; //= IcubRobotModel.SCALE_FACTOR * (0.49);// + 0.03;
   private final double maximumHeightAboveGround;// = IcubRobotModel.SCALE_FACTOR * (0.52);// + 0.03;

   private final IcubJointMap jointMap;

   private final ToeOffParameters toeOffParameters;
   private final SwingTrajectoryParameters swingTrajectoryParameters;
   private final ICubSteppingParameters steppingParameters;

   public IcubWalkingControllerParameters(IcubJointMap jointMap)
   {
      this(jointMap, false);
   }

   public IcubWalkingControllerParameters(IcubJointMap jointMap, boolean runningOnRealRobot) // TODO: are wrong
   {
      this.runningOnRealRobot = runningOnRealRobot;
      this.jointMap = jointMap;

      minimumHeightAboveGround = jointMap.getModelScale() * (0.5);
      nominalHeightAboveGround = jointMap.getModelScale() * (0.525);
      maximumHeightAboveGround = jointMap.getModelScale() * (0.55);

      toeOffParameters = new IcubToeOffParameters(jointMap);
      swingTrajectoryParameters = new IcubSwingTrajectoryParameters(jointMap.getModelScale());
      steppingParameters = new ICubSteppingParameters(jointMap);
   }

   @Override
   public double getOmega0()
   {
      return 4.7;
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
   public double getMaximumLegLengthForSingularityAvoidance()
   {
      return jointMap.getPhysicalProperties().getShinLength() + jointMap.getPhysicalProperties().getThighLength();
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

   private YoOrientationPIDGainsInterface createPelvisOrientationControlGains(YoVariableRegistry registry)
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

   private YoOrientationPIDGainsInterface createChestControlGains(YoVariableRegistry registry)
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

      YoOrientationPIDGainsInterface pelvisAngularGains = createPelvisOrientationControlGains(registry);
      taskspaceAngularGains.put(jointMap.getPelvisName(), pelvisAngularGains);

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

   /** {@inheritDoc} */
   @Override
   public RigidBodyControlMode getDefaultControlModeForRigidBody(String bodyName)
   {
      if (bodyName.equals(jointMap.getChestName()))
      {
         return RigidBodyControlMode.TASKSPACE;
      }
      else
      {
         return RigidBodyControlMode.JOINTSPACE;
      }
   }

   private TObjectDoubleHashMap<String> jointHomeConfiguration = null;

   /** {@inheritDoc} */
   @Override
   public TObjectDoubleHashMap<String> getOrCreateJointHomeConfiguration()
   {
      if (jointHomeConfiguration != null)
         return jointHomeConfiguration;

      jointHomeConfiguration = new TObjectDoubleHashMap<String>();

      jointHomeConfiguration.put(jointMap.getSpineJointName(SPINE_PITCH), 0.0);
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

   private Map<String, Pose3D> bodyHomeConfiguration = null;

   /** {@inheritDoc} */
   @Override
   public Map<String, Pose3D> getOrCreateBodyHomeConfiguration()
   {
      if (bodyHomeConfiguration != null)
         return bodyHomeConfiguration;

      bodyHomeConfiguration = new HashMap<String, Pose3D>();

      Pose3D homeChestPoseInPelvisZUpFrame = new Pose3D();
      homeChestPoseInPelvisZUpFrame.appendPitchRotation(Math.PI / 8.0);
      bodyHomeConfiguration.put(jointMap.getChestName(), homeChestPoseInPelvisZUpFrame);

      return bodyHomeConfiguration;
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
   public boolean doPrepareManipulationForLocomotion()
   {
      return true;
   }

   @Override
   public double getDefaultTransferTime()
   {
      return runningOnRealRobot ? 1.5 : 0.1;
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
   public ToeOffParameters getToeOffParameters()
   {
      return toeOffParameters;
   }

   @Override
   public SwingTrajectoryParameters getSwingTrajectoryParameters()
   {
      return swingTrajectoryParameters;
   }

   @Override
   public ICPOptimizationParameters getICPOptimizationParameters()
   {
      return null;
   }

   @Override
   public SteppingParameters getSteppingParameters()
   {
      return steppingParameters;
   }
}
