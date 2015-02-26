package it.iit.iCub.sensors;

import java.net.URI;

import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.communication.kryo.IHMCCommunicationKryoNetClassList;
import us.ihmc.communication.packetCommunicator.KryoLocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.PacketDestination;
import us.ihmc.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.camera.SCSCameraDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.PointCloudDataReceiver;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData.SCSCheatingPointCloudLidarReceiver;
import us.ihmc.darpaRoboticsChallenge.sensors.DRCSensorSuiteManager;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class IcubSensorSuiteManager implements DRCSensorSuiteManager
{
   private final KryoLocalPacketCommunicator sensorSuitePacketCommunicator = new KryoLocalPacketCommunicator(new IHMCCommunicationKryoNetClassList(),
         PacketDestination.SENSOR_MANAGER.ordinal(), "ICub_SENSOR_MANAGER");

   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer = new RobotConfigurationDataBuffer();
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final PointCloudDataReceiver pointCloudDataReceiver;
   private final SDFFullRobotModelFactory modelFactory;
   private final DRCRobotSensorInformation sensorInformation;
   
   public IcubSensorSuiteManager(SDFFullRobotModelFactory modelFactory, PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotSensorInformation sensorInformation,
         DRCRobotJointMap jointMap, boolean useSimulatedSensors)
   {
      this.modelFactory = modelFactory;
      this.sensorInformation = sensorInformation;
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.pointCloudDataReceiver = new PointCloudDataReceiver(modelFactory, ppsTimestampOffsetProvider, jointMap, robotConfigurationDataBuffer,
            sensorSuitePacketCommunicator);
   }

   @Override
   public void initializeSimulatedSensors(PacketCommunicator scsSensorsPacketCommunicator)
   {
      sensorSuitePacketCommunicator.attachListener(RobotConfigurationData.class, robotConfigurationDataBuffer);
      new SCSCameraDataReceiver(modelFactory, sensorInformation.getCameraParameters(0).getSensorNameInSdf(), robotConfigurationDataBuffer, scsSensorsPacketCommunicator, sensorSuitePacketCommunicator, ppsTimestampOffsetProvider);
      new SCSCheatingPointCloudLidarReceiver(sensorInformation.getLidarParameters(0).getSensorNameInSdf(), scsSensorsPacketCommunicator, pointCloudDataReceiver);
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
