package it.iit.iCub;

import it.iit.iCub.configuration.IcubConfigurationRoot;
import it.iit.iCub.parameters.IcubArmControllerParameters;
import it.iit.iCub.parameters.IcubCapturePointPlannerParameters;
import it.iit.iCub.parameters.IcubJointMap;
import it.iit.iCub.parameters.IcubPhysicalProperties;
import it.iit.iCub.parameters.IcubSensorInformation;
import it.iit.iCub.parameters.IcubStateEstimatorParameters;
import it.iit.iCub.parameters.IcubWalkingControllerParameters;
import it.iit.iCub.sensors.IcubSensorSuiteManager;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.SdfLoader.JaxbSDFLoader;
import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFJointNameMap;
import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.HeightCalculatorParameters;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.darpaRoboticsChallenge.DRCRobotSDFLoader;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.handControl.HandCommandManager;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.time.AlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.graphics3DAdapter.jme.util.JMEGeometryUtils;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.pathGeneration.footstepPlanner.FootstepPlanningParameterization;
import us.ihmc.robotDataCommunication.logger.LogSettings;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationconstructionset.physics.ScsCollisionConfigure;
import us.ihmc.simulationconstructionset.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.robotController.OutputProcessor;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.NeckJointName;
import us.ihmc.utilities.math.TimeTools;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.DRCRobotContactPointParameters;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.WholeBodyIkSolver;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;
import us.ihmc.wholeBodyController.parameters.DefaultArmConfigurations;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

public class IcubRobotModel implements DRCRobotModel
{
   private static final long ESTIMATOR_DT_IN_NS = 1000000;
   private static final double ESTIMATOR_DT = TimeTools.nanoSecondstoSeconds(ESTIMATOR_DT_IN_NS);
   private static final double CONTROL_DT = 0.006;
   private static final double SIMULATE_DT = 0.0001;

   private final ArmControllerParameters armControllerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final StateEstimatorParameters stateEstimatorParamaters;
   private final DRCRobotPhysicalProperties physicalProperties;
   private final DRCRobotSensorInformation sensorInformation;
   private final DRCRobotJointMap jointMap;
   private final String robotName = "ICUB";
   private final SideDependentList<Transform> offsetHandFromWrist = new SideDependentList<Transform>();
   private final CapturePointPlannerParameters capturePointPlannerParameters;

   private final String[] resourceDirectories = { "", "models/", "models/conf/", "models/meshes/", "models/meshes/visual/", "models/meshes/collision/" };

   private final JaxbSDFLoader loader;
   
   private final boolean runningOnRealRobot;
   
   private boolean enableJointDamping = true;

   @Override
   public WholeBodyIkSolver createWholeBodyIkSolver()  
   {
      return null;
   }

   public IcubRobotModel(boolean runningOnRealRobot, boolean headless)
   {
      this.runningOnRealRobot = runningOnRealRobot;
      jointMap = new IcubJointMap();
      physicalProperties = new IcubPhysicalProperties();
      sensorInformation = new IcubSensorInformation();

      if (headless)
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(new String[] {}, getSdfFileAsStream(), true);
      }
      else
      {
         this.loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), getSdfFileAsStream(), false);
      }

      for (String forceSensorNames : IcubSensorInformation.forceSensorNames)
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         if (forceSensorNames.equals("l_ankle_roll"))
         {
            transform.set(IcubSensorInformation.transformFromMeasurementToAnkleZUpFrames.get(RobotSide.LEFT));
         }
         else if (forceSensorNames.equals("r_ankle_roll"))
         {
            transform.set(IcubSensorInformation.transformFromMeasurementToAnkleZUpFrames.get(RobotSide.RIGHT));
         }

         loader.addForceSensor(jointMap, forceSensorNames, forceSensorNames, transform);
      }

      armControllerParameters = new IcubArmControllerParameters(runningOnRealRobot);
      walkingControllerParameters = new IcubWalkingControllerParameters(jointMap, runningOnRealRobot);
      stateEstimatorParamaters = new IcubStateEstimatorParameters(runningOnRealRobot, getEstimatorDT());
      capturePointPlannerParameters = new IcubCapturePointPlannerParameters(runningOnRealRobot);
   }

   @Override
   public ArmControllerParameters getArmControllerParameters()
   {
      return armControllerParameters;
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public FootstepPlanningParameterization getFootstepParameters()
   {
      return null;
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      return stateEstimatorParamaters;
   }

   @Override
   public DRCRobotPhysicalProperties getPhysicalProperties()
   {
      return physicalProperties;
   }

   @Override
   public DRCRobotJointMap getJointMap()
   {
      return jointMap;
   }

   public DRCHandType getHandType()
   {
      return null;
   }

   @Override
   public Transform getJmeTransformWristToHand(RobotSide side)
   {
      createTransforms();
      return offsetHandFromWrist.get(side);
   }
   
   @Override
   public RigidBodyTransform getTransform3dWristToHand(RobotSide side)
   {
      return JMEGeometryUtils.transformFromJMECoordinatesToZup( getJmeTransformWristToHand(side));
   }


   private String getSdfFile()
   {
      return IcubConfigurationRoot.SDF_FILE;
   }

   private String[] getResourceDirectories()
   {
      return resourceDirectories;
   }

   private InputStream getSdfFileAsStream()
   {
      return getClass().getClassLoader().getResourceAsStream(getSdfFile());
   }

   private void createTransforms() // need to check this
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         Vector3f centerOfHandToWristTranslation = new Vector3f();
         float[] angles = new float[3];
         centerOfHandToWristTranslation = new Vector3f(0.0625f, 0.0f, robotSide.negateIfLeftSide(0.016f));
         angles[0] = 0.0f;
         angles[1] = 0.0f;
         angles[2] = robotSide == RobotSide.LEFT ? 0.0f : (float) Math.PI;

         Quaternion centerOfHandToWristRotation = new Quaternion(angles);
         offsetHandFromWrist.set(robotSide, new Transform(centerOfHandToWristTranslation, centerOfHandToWristRotation));
      }
   }

   @Override
   public String toString()
   {
      return robotName;
   }

   @Override
   public DRCRobotInitialSetup<SDFRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new IcubInitialSetup(groundHeight, initialYaw);
   }

   @Override
   public WalkingControllerParameters getMultiContactControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public ScsCollisionConfigure getPhysicsConfigure(SDFRobot sdfRobot)
   {
      return null;
   }

   @Override
   public DRCRobotContactPointParameters getContactPointParameters()
   {
      return jointMap.getContactPointParameters();
   }

   @Override
   public void setJointDamping(SDFRobot simulatedRobot)
   {
      System.err.println("Joint Damping not setup for iCub. IcubRobotModel setJointDamping!");
   }
   
   @Override
   public void setEnableJointDamping(boolean enableJointDamping)
   {
      this.enableJointDamping  = enableJointDamping;
   }

   @Override
   public boolean getEnableJointDamping()
   {
      return enableJointDamping;
   }

   @Override
   public HandModel getHandModel()
   {
      return null;
   }

   @Override
   public WalkingControllerParameters getDrivingControllerParameters()
   {
      return getWalkingControllerParameters();
   }

   @Override
   public DRCRobotSensorInformation getSensorInformation()
   {
      return sensorInformation;
   }

   @Override
   public SDFFullRobotModel createFullRobotModel()
   {
      return loader.createFullRobotModel(getJointMap(), sensorInformation.getSensorFramesToTrack());
   }

   @Override
   public SDFRobot createSdfRobot(boolean createCollisionMeshes)
   { 
      boolean useCollisionMeshes = false;
      boolean enableTorqueVelocityLimits = false;
      SDFJointNameMap jointMap = getJointMap();
      boolean enableJointDamping = getEnableJointDamping();
      return loader.createRobot(jointMap.getModelName(), jointMap, useCollisionMeshes, enableTorqueVelocityLimits, enableJointDamping);
   }

   @Override
   public double getSimulateDT()
   {
      return SIMULATE_DT;
   }

   @Override
   public double getEstimatorDT()
   {
      return ESTIMATOR_DT;
   }

   @Override
   public double getControllerDT()
   {
      return CONTROL_DT;
   }

   @Override
   public GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      return loader.getGeneralizedSDFRobotModel(getJointMap().getModelName());
   }

   @Override
   public PPSTimestampOffsetProvider getPPSTimestampOffsetProvider()
   {
      return new AlwaysZeroOffsetPPSTimestampOffsetProvider();
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager()
   {
      return new IcubSensorSuiteManager(this, getPPSTimestampOffsetProvider(), sensorInformation, jointMap, runningOnRealRobot);
   }

   @Override
   public SideDependentList<HandCommandManager> createHandCommandManager()
   {
      return null;
   }

   @Override
   public CapturePointPlannerParameters getCapturePointPlannerParameters()
   {
      return capturePointPlannerParameters;
   }

   @Override
   public MultiThreadedRobotControlElement createSimulatedHandController(SDFRobot simulatedRobot, ThreadDataSynchronizerInterface threadDataSynchronizer,
         GlobalDataProducer globalDataProducer)
   {
      return null;
   }

   @Override
   public DRCHandType getDRCHandType()
   {
      return null;
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return new SDFLogModelProvider(jointMap.getModelName(), getSdfFileAsStream(), getResourceDirectories());
   }

   @Override
   public OutputProcessor getOutputProcessor(FullRobotModel controllerFullRobotModel)
   {
      return null;
   }
   
   @Override
   public LogSettings getLogSettings()
   {
      return LogSettings.SIMULATION;
   }

   @Override
   public DefaultArmConfigurations getDefaultArmConfigurations()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public Pair<Class<?>, String[]> getOperatorInterfaceStarter()
   {
      return null;
   }

   @Override
   public Class<?> getSpectatorInterfaceClass()
   {
      
      
      return null;
   }

   @Override
   public HeightCalculatorParameters getHeightCalculatorParameters()
   {
      return null;
   }

   @Override public String getSimpleRobotName()
   {
      return "iCub";
   }

   @Override
   public CollisionBoxProvider getCollisionBoxProvider()
   {
      return null;
   }
   
   @Override
   public LinkedHashMap<NeckJointName, Pair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits()
   {
      return walkingControllerParameters.getSliderBoardControlledNeckJointsWithLimits();
   }
   
   @Override
   public SideDependentList<LinkedHashMap<String,Pair<Double,Double>>> getSliderBoardControlledFingerJointsWithLimits()
   {
      return walkingControllerParameters.getSliderBoardControlledFingerJointsWithLimits();
   }
}
