package it.iit.iCub.hardwareDrivers.sensorReader;

import java.util.List;

import it.iit.iCub.hardwareDrivers.IcubIndexToJointMapTools;
import it.iit.iCub.messages.it.iit.yarp.RobotFeedback;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReader;
import us.ihmc.sensorProcessing.simulatedSensors.SensorReaderFactory;
import us.ihmc.sensorProcessing.simulatedSensors.StateEstimatorSensorDefinitions;
import us.ihmc.sensorProcessing.stateEstimation.StateEstimatorParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class IcubSensorReaderFactory implements SensorReaderFactory
{
   private final StateEstimatorParameters stateEstimatorParameters;
   private final DRCRobotSensorInformation sensorInformation;
   private StateEstimatorSensorDefinitions stateEstimatorSensorDefinitions;

   private SensorReader sensorReader;

   public IcubSensorReaderFactory(DRCRobotModel robotModel)
   {
      this.stateEstimatorParameters = robotModel.getStateEstimatorParameters();
      this.sensorInformation = robotModel.getSensorInformation();
   }

   @Override
   public void build(FloatingInverseDynamicsJoint rootJoint, IMUDefinition[] imuDefinitions, ForceSensorDefinition[] forceSensorDefinitions,
                     ContactSensorHolder contactSensorHolder, RawJointSensorDataHolderMap rawJointSensorDataHolderMap,
                     JointDesiredOutputList estimatorDesiredJointDataHolder, YoVariableRegistry parentRegistry)
   {
      stateEstimatorSensorDefinitions = new StateEstimatorSensorDefinitions();

      for (InverseDynamicsJoint inverseDynamicsJoint : ScrewTools.computeSubtreeJoints(rootJoint.getSuccessor()))
      {
         if (inverseDynamicsJoint instanceof OneDoFJoint)
         {
            OneDoFJoint oneDoFJoint = (OneDoFJoint) inverseDynamicsJoint;
            stateEstimatorSensorDefinitions.addJointSensorDefinition(oneDoFJoint);
         }
      }

      for (IMUDefinition imuDefinition : imuDefinitions)
      {
         stateEstimatorSensorDefinitions.addIMUSensorDefinition(imuDefinition);
      }

      for (ForceSensorDefinition forceSensorDefinition : forceSensorDefinitions)
      {
         stateEstimatorSensorDefinitions.addForceSensorDefinition(forceSensorDefinition);
      }

      YoVariableRegistry sensorReaderRegistry = new YoVariableRegistry("iCubSensorReader");
      RobotFeedback iCubRobotFeeback = new RobotFeedback();
      List<OneDoFJoint> jointList = stateEstimatorSensorDefinitions.getJointSensorDefinitions();

      sensorReader = new IcubSensorReader(iCubRobotFeeback, IcubIndexToJointMapTools.createMapping(jointList), stateEstimatorSensorDefinitions, rawJointSensorDataHolderMap,
                                          sensorInformation, stateEstimatorParameters, sensorReaderRegistry);

      parentRegistry.addChild(sensorReaderRegistry);
   }

   @Override
   public SensorReader getSensorReader()
   {
      return sensorReader;
   }

   @Override
   public StateEstimatorSensorDefinitions getStateEstimatorSensorDefinitions()
   {
      return stateEstimatorSensorDefinitions;
   }

   @Override
   public boolean useStateEstimator()
   {
      return false;
   }
}
