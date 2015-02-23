package it.iit.iCub.sensors;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.communication.subscribers.RobotDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SCSPointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.ihmcPerception.depthData.DepthDataFilter;
import us.ihmc.ihmcPerception.depthData.RobotBoundingBoxes;
import us.ihmc.ihmcPerception.depthData.RobotDepthDataFilter;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class IcubSensorSuiteManager implements DRCSensorSuiteManager
{
   private final KryoLocalPacketCommunicator sensorSuitePacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),PacketDestination.SENSOR_MANAGER.ordinal(), "ICub_SENSOR_MANAGER");
   private final AtomicSettableTimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   
   private RobotPoseBuffer robotPoseBuffer;
   private final SDFFullRobotModel sdfFullRobotModel;
   private final DepthDataFilter lidarDataFilter;
   private final RobotDataReceiver drcRobotDataReceiver;
   private final RobotBoundingBoxes robotBoundingBoxes;
   private final boolean useSimulatedSensors;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final DRCRobotSensorInformation sensorInformation;

   public IcubSensorSuiteManager(PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation, SDFFullRobotModel sdfFullRobotModel, DRCRobotJointMap jointMap, boolean useSimulatedSensors)
   {
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.sensorInformation = sensorInformation;
      this.sdfFullRobotModel = sdfFullRobotModel;
      this.drcRobotDataReceiver = new RobotDataReceiver(sdfFullRobotModel, null, true);
      this.robotBoundingBoxes = new RobotBoundingBoxes(drcRobotDataReceiver, DRCHandType.NONE, sdfFullRobotModel);
      this.lidarDataFilter = new RobotDepthDataFilter(robotBoundingBoxes, sdfFullRobotModel, jointMap.getContactPointParameters().getFootContactPoints());
      this.useSimulatedSensors = useSimulatedSensors;
   }
   
   @Override
   public void initializeSimulatedSensors(PacketCommunicator scsSensorsPacketCommunicator)
   {
      robotPoseBuffer = new RobotPoseBuffer(sensorSuitePacketCommunicator, 10000, timestampProvider);
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, drcRobotDataReceiver);
      new SCSCameraDataReceiver(robotPoseBuffer, scsSensorsPacketCommunicator, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider);
      new SCSPointCloudDataReceiver(robotPoseBuffer, scsSensorsPacketCommunicator, sdfFullRobotModel, sensorInformation, ppsTimestampOffsetProvider, lidarDataFilter);
   }

   @Override
   public void initializePhysicalSensors(URI sensorURI)
   {
      
   }

   @Override
   public PacketCommunicator getProcessedSensorsCommunicator()
   {
      return sensorSuitePacketCommunicator;
   }

//   @Override
//   public PacketCommunicator createSensorModule(URI sensorURI)
//   {
//      KryoLocalPacketCommunicator sensorSuitePacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),PacketDestination.SENSOR_MANAGER.ordinal(), "STEPPR_SENSOR_MANAGER");
//      if(useSimulatedSensors)
//      {
//         initializeSimulatedSensors(sensorSuitePacketCommunicator);
//      } else {
//         initializePhysicalSensors(sensorSuitePacketCommunicator, sensorURI);
//      }
//      
//      return sensorSuitePacketCommunicator;
//   }

}
