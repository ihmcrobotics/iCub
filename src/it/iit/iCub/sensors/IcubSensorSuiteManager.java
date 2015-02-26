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
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.PointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SCSCheatingPointCloudLidarReceiver;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.ihmcPerception.depthData.RobotBoundingBoxes;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class IcubSensorSuiteManager implements DRCSensorSuiteManager
{
   private final KryoLocalPacketCommunicator sensorSuitePacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),PacketDestination.SENSOR_MANAGER.ordinal(), "ICub_SENSOR_MANAGER");
   private final AtomicSettableTimestampProvider timestampProvider = new AtomicSettableTimestampProvider();
   
   private final RobotPoseBuffer robotPoseBuffer;
   private final RobotDataReceiver drcRobotDataReceiver;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final PointCloudDataReceiver pointCloudDataReceiver;
   private final RobotBoundingBoxes robotBoundingBoxes;

   public IcubSensorSuiteManager(PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation, SDFFullRobotModel sdfFullRobotModel, DRCRobotJointMap jointMap, boolean useSimulatedSensors)
   {
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.drcRobotDataReceiver = new RobotDataReceiver(sdfFullRobotModel, null, true);
      robotBoundingBoxes = new RobotBoundingBoxes(drcRobotDataReceiver, DRCHandType.NONE, sdfFullRobotModel);
      this.robotPoseBuffer = new RobotPoseBuffer(sensorSuitePacketCommunicator, 10000, timestampProvider);
      this.pointCloudDataReceiver = new PointCloudDataReceiver(sdfFullRobotModel, jointMap, robotPoseBuffer, sensorSuitePacketCommunicator);
   }
   
   @Override
   public void initializeSimulatedSensors(PacketCommunicator scsSensorsPacketCommunicator)
   {
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, drcRobotDataReceiver);
      new SCSCameraDataReceiver(robotPoseBuffer, scsSensorsPacketCommunicator, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider);
      new SCSCheatingPointCloudLidarReceiver(robotBoundingBoxes, scsSensorsPacketCommunicator, pointCloudDataReceiver);
      pointCloudDataReceiver.start();
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
