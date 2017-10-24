package it.iit.iCub;

import java.io.InputStream;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import com.jme3.math.Quaternion;
import com.jme3.math.Transform;
import com.jme3.math.Vector3f;

import it.iit.iCub.parameters.IcubCapturePointPlannerParameters;
import it.iit.iCub.parameters.IcubContactPointParameters;
import it.iit.iCub.parameters.IcubInitialSetup;
import it.iit.iCub.parameters.IcubJointMap;
import it.iit.iCub.parameters.IcubPhysicalProperties;
import it.iit.iCub.parameters.IcubSensorInformation;
import it.iit.iCub.parameters.IcubStateEstimatorParameters;
import it.iit.iCub.parameters.IcubWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.time.DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.sensors.DRCSensorSuiteManager;
import us.ihmc.commonWalkingControlModules.configurations.ICPWithTimeFreezingPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.modelFileLoaders.SdfLoader.DRCRobotSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.GeneralizedSDFRobotModel;
import us.ihmc.modelFileLoaders.SdfLoader.JaxbSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.RobotDescriptionFromSDFLoader;
import us.ihmc.modelFileLoaders.SdfLoader.SDFContactSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFDescriptionMutator;
import us.ihmc.modelFileLoaders.SdfLoader.SDFForceSensor;
import us.ihmc.modelFileLoaders.SdfLoader.SDFJointHolder;
import us.ihmc.modelFileLoaders.SdfLoader.SDFLinkHolder;
import us.ihmc.modelFileLoaders.SdfLoader.xmlDescription.SDFSensor;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.multicastLogDataProtocol.modelLoaders.SDFLogModelProvider;
import us.ihmc.robotDataLogger.logger.LogSettings;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFromDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.simulationConstructionSetTools.robotController.MultiThreadedRobotControlElement;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.FootContactPoints;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.wholeBodyController.UIParameters;
import us.ihmc.wholeBodyController.concurrent.ThreadDataSynchronizerInterface;

public class IcubRobotModel implements DRCRobotModel, SDFDescriptionMutator
{
   private static final double ESTIMATOR_DT = 0.001;
   private static final double CONTROL_DT = 0.004;
   private static final double SIMULATE_DT = 0.0001;

   private final boolean removeLimits;
   private final boolean enableDamping = false;
   private final boolean increaseInertias = true;

   private final WalkingControllerParameters walkingControllerParameters;
   private final StateEstimatorParameters stateEstimatorParamaters;
   private final IcubPhysicalProperties physicalProperties;
   private final DRCRobotSensorInformation sensorInformation;
   private final IcubJointMap jointMap;
   private final RobotContactPointParameters contactPointParameters;
   private final SideDependentList<Transform> offsetHandFromWrist = new SideDependentList<Transform>();
   private final ICPWithTimeFreezingPlannerParameters capturePointPlannerParameters;

   private final String[] resourceDirectories = {"", "models/", "models/conf/", "models/meshes/", "models/meshes/visual/", "models/meshes/collision/"};

   private final JaxbSDFLoader loader;

   private final RobotDescription robotDescription;

   public IcubRobotModel()
   {
      this(false, null);
   }

   public IcubRobotModel(boolean removeLimits)
   {
      this(removeLimits, null);
   }

   public IcubRobotModel(boolean removeLimits, FootContactPoints contactPoints)
   {
      this.removeLimits = removeLimits;

      physicalProperties = new IcubPhysicalProperties();
      jointMap = new IcubJointMap(physicalProperties);
      contactPointParameters = new IcubContactPointParameters(jointMap, contactPoints);
      sensorInformation = new IcubSensorInformation();

      this.loader = DRCRobotSDFLoader.loadDRCRobot(getResourceDirectories(), getSdfFileAsStream(), this);

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

      walkingControllerParameters = new IcubWalkingControllerParameters(jointMap);
      stateEstimatorParamaters = new IcubStateEstimatorParameters(getEstimatorDT());
      capturePointPlannerParameters = new IcubCapturePointPlannerParameters();
      robotDescription = createRobotDescription();
   }

   private RobotDescription createRobotDescription()
   {
      boolean useCollisionMeshes = false;
      GeneralizedSDFRobotModel generalizedSDFRobotModel = getGeneralizedRobotModel();
      RobotDescriptionFromSDFLoader descriptionLoader = new RobotDescriptionFromSDFLoader();
      RobotDescription robotDescription = descriptionLoader.loadRobotDescriptionFromSDF(generalizedSDFRobotModel, jointMap, contactPointParameters,
                                                                                        useCollisionMeshes);
      return robotDescription;
   }

   @Override
   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }

   @Override
   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return walkingControllerParameters;
   }

   @Override
   public StateEstimatorParameters getStateEstimatorParameters()
   {
      return stateEstimatorParamaters;
   }

   @Override
   public IcubJointMap getJointMap()
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

   private String getSdfFile()
   {
      return "models/iCubGenova04.sdf";
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
      return jointMap.getModelName();
   }

   @Override
   public DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> getDefaultRobotInitialSetup(double groundHeight, double initialYaw)
   {
      return new IcubInitialSetup(groundHeight, initialYaw);
   }

   @Override
   public RobotContactPointParameters getContactPointParameters()
   {
      return contactPointParameters;
   }

   @Override
   public HandModel getHandModel()
   {
      return null;
   }

   @Override
   public DRCRobotSensorInformation getSensorInformation()
   {
      return sensorInformation;
   }

   @Override
   public FullHumanoidRobotModel createFullRobotModel()
   {
      return new FullHumanoidRobotModelFromDescription(robotDescription, jointMap, sensorInformation.getSensorFramesToTrack());
   }

   @Override
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes)
   {
      return createHumanoidFloatingRootJointRobot(createCollisionMeshes, enableDamping);
   }

   @Override
   public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes, boolean enableJointDamping)
   {
      boolean enableTorqueVelocityLimits = false;
      return new HumanoidFloatingRootJointRobot(robotDescription, jointMap, enableJointDamping, enableTorqueVelocityLimits);
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

   private GeneralizedSDFRobotModel getGeneralizedRobotModel()
   {
      return loader.getGeneralizedSDFRobotModel(getJointMap().getModelName());
   }

   @Override
   public DRCROSPPSTimestampOffsetProvider getPPSTimestampOffsetProvider()
   {
      return new DRCROSAlwaysZeroOffsetPPSTimestampOffsetProvider();
   }

   @Override
   public DRCSensorSuiteManager getSensorSuiteManager()
   {
      return null;
   }

   @Override
   public ICPWithTimeFreezingPlannerParameters getCapturePointPlannerParameters()
   {
      return capturePointPlannerParameters;
   }

   @Override
   public UIParameters getUIParameters()
   {
      return null;
   }

   @Override
   public MultiThreadedRobotControlElement createSimulatedHandController(FloatingRootJointRobot simulatedRobot,
                                                                         ThreadDataSynchronizerInterface threadDataSynchronizer,
                                                                         HumanoidGlobalDataProducer globalDataProducer,
                                                                         CloseableAndDisposableRegistry closeableAndDisposableRegistry)
   {
      return null;
   }

   @Override
   public LogModelProvider getLogModelProvider()
   {
      return new SDFLogModelProvider(jointMap.getModelName(), getSdfFileAsStream(), getResourceDirectories());
   }

   @Override
   public LogSettings getLogSettings()
   {
      return LogSettings.SIMULATION;
   }

   @Override
   public String getSimpleRobotName()
   {
      return jointMap.getModelName();
   }

   @Override
   public CollisionBoxProvider getCollisionBoxProvider()
   {
      return null;
   }

   @Override
   public double getStandPrepAngle(String jointName)
   {
      System.err.println("Need to add access to stand prep joint angles.");
      return 0;
   }

   @Override
   public void mutateJointForModel(GeneralizedSDFRobotModel model, SDFJointHolder jointHolder)
   {
      // remove limits
      if (removeLimits)
      {
         jointHolder.setLimits(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
      }
   }

   @Override
   public void mutateLinkForModel(GeneralizedSDFRobotModel model, SDFLinkHolder linkHolder)
   {
      // For simulation stability define a minimum determinant of the inertia matrix of a link.
      if (increaseInertias)
      {
         double minDet = 1.0e-5;
         Matrix3D inertia = linkHolder.getInertia();
         DenseMatrix64F inertiaMatrix = new DenseMatrix64F(3, 3);
         inertia.get(inertiaMatrix);
         double det = CommonOps.det(inertiaMatrix);
         if (det < minDet)
         {
            double scale = Math.pow(minDet / det, 1.0 / 3.0);
            inertia.scale(scale);
         }
      }
   }

   @Override
   public void mutateSensorForModel(GeneralizedSDFRobotModel model, SDFSensor sensor)
   {
   }

   @Override
   public void mutateForceSensorForModel(GeneralizedSDFRobotModel model, SDFForceSensor forceSensor)
   {
   }

   @Override
   public void mutateContactSensorForModel(GeneralizedSDFRobotModel model, SDFContactSensor contactSensor)
   {
   }

   @Override
   public void mutateModelWithAdditions(GeneralizedSDFRobotModel model)
   {
   }
}
