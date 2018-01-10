package it.iit.iCub.parameters;

import static us.ihmc.robotics.partNames.SpineJointName.SPINE_PITCH;
import static us.ihmc.robotics.partNames.SpineJointName.SPINE_ROLL;
import static us.ihmc.robotics.partNames.SpineJointName.SPINE_YAW;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.ImmutableTriple;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDGains;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.avatar.drcRobot.RobotTarget;

public class IcubWalkingControllerParameters extends WalkingControllerParameters
{
   private final IcubJointMap jointMap;
   private final RobotTarget target;
   private final double massScale;
   private final boolean runningOnRealRobot;
   private final IcubMomentumOptimizationSettings momentumOptimizationSettings;

   private final ToeOffParameters toeOffParameters;
   private final SwingTrajectoryParameters swingTrajectoryParameters;
   private final IcubSteppingParameters steppingParameters;

   public IcubWalkingControllerParameters(RobotTarget target, IcubJointMap jointMap, IcubContactPointParameters contactPointParameters)
   {
      this.jointMap = jointMap;
      this.target = target;
      this.massScale = Math.pow(jointMap.getModelScale(), jointMap.getMassScalePower());
      runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      momentumOptimizationSettings = new IcubMomentumOptimizationSettings(jointMap, contactPointParameters.getNumberOfContactableBodies());

      toeOffParameters = new IcubToeOffParameters(jointMap);
      swingTrajectoryParameters = new IcubSwingTrajectoryParameters();
      steppingParameters = new IcubSteppingParameters(jointMap);
   }

   @Override
   public double getOmega0()
   {
      return 4.4;
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
      return 0.4;
   }

   @Override
   public double nominalHeightAboveAnkle()
   {
      return 0.5;
   }

   @Override
   public double maximumHeightAboveAnkle()
   {
      return 0.55;
   }

   @Override
   public double defaultOffsetHeightAboveAnkle()
   {
      return 0.0;
   }

   @Override
   public double getMaximumLegLengthForSingularityAvoidance()
   {
      IcubPhysicalProperties physicalProperties = jointMap.getPhysicalProperties();
      return physicalProperties.getShinLength() + physicalProperties.getThighLength();
   }

   @Override
   public ICPControlGains createICPControlGains()
   {
      ICPControlGains gains = new ICPControlGains();

      double kpParallel = 2.5;
      double kpOrthogonal = 1.5;

      gains.setKpParallelToMotion(kpParallel);
      gains.setKpOrthogonalToMotion(kpOrthogonal);

      return gains;
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PIDGainsReadOnly>> getJointSpaceControlGains()
   {
      List<String> spineNames = new ArrayList<>();
      List<String> neckNames = new ArrayList<>();
      List<String> armNames = new ArrayList<>();

      Arrays.stream(jointMap.getSpineJointNames()).forEach(n -> spineNames.add(jointMap.getSpineJointName(n)));
      Arrays.stream(jointMap.getNeckJointNames()).forEach(n -> neckNames.add(jointMap.getNeckJointName(n)));
      for (RobotSide side : RobotSide.values)
      {
         Arrays.stream(jointMap.getArmJointNames()).forEach(n -> armNames.add(jointMap.getArmJointName(side, n)));
      }

      PIDGains spineGains = createSpineControlGains();
      PIDGains neckGains = createNeckControlGains();
      PIDGains armGains = createArmControlGains();

      List<GroupParameter<PIDGainsReadOnly>> jointspaceGains = new ArrayList<>();
      jointspaceGains.add(new GroupParameter<>("_SpineJointGains", spineGains, spineNames));
      jointspaceGains.add(new GroupParameter<>("_NeckJointGains", neckGains, neckNames));
      jointspaceGains.add(new GroupParameter<>("_ArmJointGains", armGains, armNames));

      return jointspaceGains;
   }

   private PIDGains createSpineControlGains()
   {
      PIDGains spineGains = new PIDGains();

      double kp = 100.0;
      double zeta = 1.0;
      double maxAccel = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      spineGains.setKp(kp);
      spineGains.setZeta(zeta);
      spineGains.setMaximumFeedback(maxAccel);
      spineGains.setMaximumFeedbackRate(maxJerk);

      return spineGains;
   }

   private PIDGains createNeckControlGains()
   {
      PIDGains gains = new PIDGains();

      double kp = 100.0;
      double zeta = 1.0;
      double maxAccel = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setMaximumFeedback(maxAccel);
      gains.setMaximumFeedbackRate(maxJerk);

      return gains;
   }

   private PIDGains createArmControlGains()
   {
      PIDGains armGains = new PIDGains();

      double kp = 100.0;
      double zeta = 1.0;
      double maxAccel = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      armGains.setKp(kp);
      armGains.setZeta(zeta);
      armGains.setMaximumFeedback(maxAccel);
      armGains.setMaximumFeedbackRate(maxJerk);

      return armGains;
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PID3DGainsReadOnly>> getTaskspaceOrientationControlGains()
   {
      List<GroupParameter<PID3DGainsReadOnly>> taskspaceAngularGains = new ArrayList<>();

      PID3DGains chestAngularGains = createChestOrientationControlGains();
      List<String> chestGainBodies = new ArrayList<>();
      chestGainBodies.add(jointMap.getChestName());
      taskspaceAngularGains.add(new GroupParameter<>("Chest", chestAngularGains, chestGainBodies));

      PID3DGains headAngularGains = createHeadOrientationControlGains();
      List<String> headGainBodies = new ArrayList<>();
      headGainBodies.add(jointMap.getHeadName());
      taskspaceAngularGains.add(new GroupParameter<>("Head", headAngularGains, headGainBodies));

      PID3DGains handAngularGains = createHandOrientationControlGains();
      List<String> handGainBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         handGainBodies.add(jointMap.getHandName(robotSide));
      }
      taskspaceAngularGains.add(new GroupParameter<>("Hand", handAngularGains, handGainBodies));

      PID3DGains pelvisAngularGains = createPelvisOrientationControlGains();
      List<String> pelvisGainBodies = new ArrayList<>();
      pelvisGainBodies.add(jointMap.getPelvisName());
      taskspaceAngularGains.add(new GroupParameter<>("Pelvis", pelvisAngularGains, pelvisGainBodies));

      return taskspaceAngularGains;
   }

   private PID3DGains createPelvisOrientationControlGains()
   {
      double kp = 100.0;
      double zeta = 1.0;
      double maxAccel = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XYZ, false);
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   private PID3DGains createHeadOrientationControlGains()
   {
      double kp = 100.0;
      double zeta = 1.0;
      double maxAccel = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XYZ, false);
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   private PID3DGains createChestOrientationControlGains()
   {
      double kp = 100.0;
      double zeta = 1.0;
      double maxAccel = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XYZ, false);
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   private PID3DGains createHandOrientationControlGains()
   {
      double kp = 100.0;
      double zeta = 1.0;
      double maxAccel = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XYZ, false);
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PID3DGainsReadOnly>> getTaskspacePositionControlGains()
   {
      List<GroupParameter<PID3DGainsReadOnly>> taskspaceLinearGains = new ArrayList<>();

      PID3DGains handLinearGains = createHandPositionControlGains();
      List<String> handGainBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         handGainBodies.add(jointMap.getHandName(robotSide));
      }
      taskspaceLinearGains.add(new GroupParameter<>("Hand", handLinearGains, handGainBodies));

      return taskspaceLinearGains;
   }

   private PID3DGains createHandPositionControlGains()
   {
      double kp = 100.0;
      double zeta = 1.0;
      double maxAccel = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XYZ, false);
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   /** {@inheritDoc} */
   @Override
   public Map<String, RigidBodyControlMode> getDefaultControlModesForRigidBodies()
   {
      Map<String, RigidBodyControlMode> defaultControlModes = new HashMap<>();
      defaultControlModes.put(jointMap.getChestName(), RigidBodyControlMode.TASKSPACE);
      return defaultControlModes;
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
      homeChestPoseInPelvisZUpFrame.appendPitchRotation(Math.PI / 6.0);
      bodyHomeConfiguration.put(jointMap.getChestName(), homeChestPoseInPelvisZUpFrame);

      return bodyHomeConfiguration;
   }

   @Override
   public boolean doPrepareManipulationForLocomotion()
   {
      return false;
   }

   @Override
   public boolean doPreparePelvisForLocomotion()
   {
      return false;
   }

   @Override
   public double getDefaultTransferTime()
   {
      return 0.1;
   }

   @Override
   public double getDefaultSwingTime()
   {
      return 0.6;
   }

   @Override
   public double getDefaultInitialTransferTime()
   {
      return 1.0;
   }

   @Override
   public double getContactThresholdForce()
   {
      return 5.0;
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
      return momentumOptimizationSettings;
   }

   @Override
   public ICPAngularMomentumModifierParameters getICPAngularMomentumModifierParameters()
   {
      return null;
   }

   @Override
   public PDGains getCoMHeightControlGains()
   {
      PDGains gains = new PDGains();

      double kp = 100.0;
      double zeta = 1.0;
      double maxAcceleration = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setMaximumFeedback(maxAcceleration);
      gains.setMaximumFeedbackRate(maxJerk);

      return gains;
   }

   @Override
   public PIDSE3Gains getSwingFootControlGains()
   {
      double kpXY = 100.0;
      double kpZ = 200.0;
      double zetaXYZ = 1.0;
      double kpXYOrientation = 200.0;
      double kpZOrientation = 200.0;
      double zetaOrientation = 1.0;
      double maxPositionAcceleration = Double.POSITIVE_INFINITY;
      double maxPositionJerk = Double.POSITIVE_INFINITY;
      double maxOrientationAcceleration = Double.POSITIVE_INFINITY;
      double maxOrientationJerk = Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains(GainCoupling.XY, false);
      gains.setPositionProportionalGains(kpXY, kpXY, kpZ);
      gains.setPositionDampingRatios(zetaXYZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxPositionAcceleration, maxPositionJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxOrientationAcceleration, maxOrientationJerk);

      return gains;
   }

   @Override
   public PIDSE3Gains getHoldPositionFootControlGains()
   {
      double kpXY = 100.0;
      double kpZ = 0.0;
      double zetaXYZ = 1.0;
      double kpXYOrientation = 100.0;
      double kpZOrientation = 100.0;
      double zetaOrientation = 1.0;
      double maxLinearAcceleration = Double.POSITIVE_INFINITY;
      double maxLinearJerk = Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = Double.POSITIVE_INFINITY;
      double maxAngularJerk = Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains(GainCoupling.XY, false);
      gains.setPositionProportionalGains(kpXY, kpXY, kpZ);
      gains.setPositionDampingRatios(zetaXYZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);

      return gains;
   }

   @Override
   public PIDSE3Gains getToeOffFootControlGains()
   {
      double kpXY = 100.0;
      double kpZ = 0.0;
      double zetaXYZ = 1.0;
      double kpXYOrientation = 100.0;
      double kpZOrientation = 100.0;
      double zetaOrientation = 1.0;
      double maxLinearAcceleration = Double.POSITIVE_INFINITY;
      double maxLinearJerk = Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = Double.POSITIVE_INFINITY;
      double maxAngularJerk = Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains(GainCoupling.XY, false);
      gains.setPositionProportionalGains(kpXY, kpXY, kpZ);
      gains.setPositionDampingRatios(zetaXYZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);

      return gains;
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportX()
   {
      return 0.01;
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportY()
   {
      return 0.01;
   }

   @Override
   public boolean finishSingleSupportWhenICPPlannerIsDone()
   {
      return false;
   }

   @Override
   public double getHighCoPDampingDurationToPreventFootShakies()
   {
      return -1.0;
   }

   @Override
   public double getCoPErrorThresholdForHighCoPDamping()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public boolean useOptimizationBasedICPController()
   {
      if(runningOnRealRobot)
      {
         return false;
      }
      else
      {
         return true;
      }
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
      return new IcubICPOptimizationParameters(runningOnRealRobot);
   }

   @Override
   public SteppingParameters getSteppingParameters()
   {
      return steppingParameters;
   }
}
